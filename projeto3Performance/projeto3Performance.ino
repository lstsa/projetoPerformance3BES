#define FFT_SPEED_OVER_PRECISION

#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

#include <OneWire.h>
#include <DallasTemperature.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

#include <arduinoFFT.h>

// ===== WIFI =====
const char* ssid = "WIFI-ESP";
const char* password = "onze";

WebServer server(80);
Preferences preferences;

// ===== TEMP =====
#define ONE_WIRE_BUS 4
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// ===== ACCEL =====
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified();

// ===== FFT =====
#define SAMPLES 64
#define SAMPLING_FREQUENCY 100

double vReal[SAMPLES];
double vImag[SAMPLES];

ArduinoFFT<double> FFT = ArduinoFFT<double>();

unsigned long lastSampleMicros = 0;
int sampleIndex = 0;
bool fftReady = false;

// ===== TEMP CONTROL =====
unsigned long ultimoTempRequest = 0;
bool aguardandoTemp = false;
float tempAtual = 0;

// ===== VIB =====
float vibAtual = 0;

// ===== FLASH (BUFFER CIRCULAR) =====
#define MAX 288
float vibDados[MAX];
float tempDados[MAX];
int indice = 0;
bool cheio = false;

// ===== TEMPO =====
unsigned long ultimoTempoMedia = 0;
const unsigned long intervaloMedia = 300000; // 5 min

// ===== GRAVIDADE =====
float alpha = 0.9;
float gravX = 0, gravY = 0, gravZ = 0;

void removerGravidade(float x, float y, float z,
                      float &linX, float &linY, float &linZ) {
  gravX = alpha * gravX + (1 - alpha) * x;
  gravY = alpha * gravY + (1 - alpha) * y;
  gravZ = alpha * gravZ + (1 - alpha) * z;

  linX = x - gravX;
  linY = y - gravY;
  linZ = z - gravZ;
}

// ===== FLASH =====
void salvarFlash() {
  preferences.begin("app", false);
  preferences.putBytes("vib", vibDados, sizeof(vibDados));
  preferences.putBytes("temp", tempDados, sizeof(tempDados));
  preferences.putInt("indice", indice);
  preferences.putBool("cheio", cheio);
  preferences.end();
}

void carregarFlash() {
  preferences.begin("app", true);

  if (preferences.isKey("vib")) {
    preferences.getBytes("vib", vibDados, sizeof(vibDados));
    preferences.getBytes("temp", tempDados, sizeof(tempDados));
    indice = preferences.getInt("indice", 0);
    cheio = preferences.getBool("cheio", false);
  }

  preferences.end();
}

// ===== FFT NÃO BLOQUEANTE =====
void coletarAmostraFFT() {
  unsigned long periodo = 1000000 / SAMPLING_FREQUENCY;

  if (micros() - lastSampleMicros >= periodo) {
    lastSampleMicros = micros();

    sensors_event_t event;
    accel.getEvent(&event);

    float linX, linY, linZ;
    removerGravidade(event.acceleration.x,
                     event.acceleration.y,
                     event.acceleration.z,
                     linX, linY, linZ);

    vReal[sampleIndex] = sqrt(linX*linX + linY*linY + linZ*linZ);
    vImag[sampleIndex] = 0;

    sampleIndex++;

    if (sampleIndex >= SAMPLES) {
      sampleIndex = 0;
      fftReady = true;
    }
  }
}

void processarFFT() {
  FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.complexToMagnitude(vReal, vImag, SAMPLES);

  double pico = 0;
  int idx = 0;

  for (int i = 1; i < SAMPLES / 2; i++) {
    if (vReal[i] > pico) {
      pico = vReal[i];
      idx = i;
    }
  }

  double freq = (idx * SAMPLING_FREQUENCY) / SAMPLES;

  if (freq > 0) {
    vibAtual = (pico / (2 * PI * freq)) * 1000;
  }

  fftReady = false;
}

// ===== TEMP NÃO BLOQUEANTE =====
void atualizarTemperatura() {
  if (!aguardandoTemp && millis() - ultimoTempRequest >= 1000) {
    sensors.requestTemperatures();
    aguardandoTemp = true;
    ultimoTempRequest = millis();
  }

  if (aguardandoTemp && millis() - ultimoTempRequest >= 750) {
    float t = sensors.getTempCByIndex(0);
    if (t != -127) tempAtual = t;
    aguardandoTemp = false;
  }
}

// ===== BUFFER =====
void adicionarDado(float vib, float temp) {
  vibDados[indice] = vib;
  tempDados[indice] = temp;

  indice++;
  if (indice >= MAX) {
    indice = 0;
    cheio = true;
  }
}

// ===== CSV =====
void handleCSV() {
  String csv = "Vibracao(mm/s),Temperatura\n";

  if (!cheio) {
    for (int i = 0; i < indice; i++) {
      csv += String(vibDados[i]) + "," + String(tempDados[i]) + "\n";
    }
  } else {
    for (int i = indice; i < MAX; i++) {
      csv += String(vibDados[i]) + "," + String(tempDados[i]) + "\n";
    }
    for (int i = 0; i < indice; i++) {
      csv += String(vibDados[i]) + "," + String(tempDados[i]) + "\n";
    }
  }

  server.send(200, "text/csv", csv);
}

// ===== JSON =====
String gerarJSON() {
  String json = "{";

  json += "\"vibAtual\":" + String(vibAtual) + ",";
  json += "\"tempAtual\":" + String(tempAtual) + ",";

  json += "\"vibHist\":[";
  bool primeiro = true;

  auto add = [&](float v){
    if(!primeiro) json += ",";
    json += String(v);
    primeiro = false;
  };

  if (!cheio) {
    for (int i = 0; i < indice; i++) add(vibDados[i]);
  } else {
    for (int i = indice; i < MAX; i++) add(vibDados[i]);
    for (int i = 0; i < indice; i++) add(vibDados[i]);
  }

  json += "],\"tempHist\":[";
  primeiro = true;

  if (!cheio) {
    for (int i = 0; i < indice; i++) add(tempDados[i]);
  } else {
    for (int i = indice; i < MAX; i++) add(tempDados[i]);
    for (int i = 0; i < indice; i++) add(tempDados[i]);
  }

  json += "]}";
  return json;
}

// ===== HTML =====
String gerarHTML() {
  return R"rawliteral(
<!DOCTYPE html>
<html>
<body>

<h3>Vibração</h3>
<div style="overflow-x:auto">
<canvas id="vib" height="150"></canvas>
</div>

<h3>Temperatura</h3>
<div style="overflow-x:auto">
<canvas id="temp" height="150"></canvas>
</div>

<h3>Histórico</h3>
<div style="overflow-x:auto">
<canvas id="hist" height="150"></canvas>
</div>

<br>
<a href="/csv">📥 Baixar CSV</a>

<script>
let vib=[], temp=[];
let vibHist=[], tempHist=[];

function draw(ctx,c,data,color){
 let w=Math.max(300,data.length*5);
 c.width=w;

 ctx.clearRect(0,0,w,c.height);

 let max=Math.max(...data,1);
 let min=Math.min(...data,0);

 ctx.fillText(max.toFixed(1),2,10);
 ctx.fillText(min.toFixed(1),2,c.height-2);

 ctx.beginPath();
 ctx.strokeStyle=color;

 for(let i=0;i<data.length;i++){
  let x=i*5;
  let y=c.height-((data[i]-min)/(max-min||1))*c.height;
  i?ctx.lineTo(x,y):ctx.moveTo(x,y);
 }
 ctx.stroke();
}

let cv=document.getElementById("vib");
let ct=document.getElementById("temp");
let ch=document.getElementById("hist");

let xv=cv.getContext("2d");
let xt=ct.getContext("2d");
let xh=ch.getContext("2d");

async function loop(){
 let r=await fetch('/data');
 let d=await r.json();

 vib.push(d.vibAtual);
 temp.push(d.tempAtual);

 vibHist=d.vibHist;
 tempHist=d.tempHist;

 draw(xv,cv,vib,"red");
 draw(xt,ct,temp,"blue");
 draw(xh,ch,vibHist,"green");

 setTimeout(loop,300);
}

loop();
</script>

</body>
</html>
)rawliteral";
}

// ===== SETUP =====
void setup() {
  Serial.begin(115200);

  sensors.begin();
  sensors.setWaitForConversion(false);

  accel.begin();

  carregarFlash();

  WiFi.softAP(ssid, password);

  server.on("/", [](){ server.send(200,"text/html",gerarHTML()); });
  server.on("/data", [](){ server.send(200,"application/json",gerarJSON()); });
  server.on("/csv", handleCSV);

  server.begin();
}

// ===== LOOP =====
void loop() {
  server.handleClient();

  coletarAmostraFFT();

  if (fftReady) processarFFT();

  atualizarTemperatura();

  if (millis() - ultimoTempoMedia >= intervaloMedia) {
    ultimoTempoMedia = millis();

    adicionarDado(vibAtual, tempAtual);
    salvarFlash(); // 🔥 salva sempre
  }
}
