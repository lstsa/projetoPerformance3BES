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

#include <LittleFS.h>

// ===== WIFI CONFIG =====
bool modoAP = false;

String ssidSalvo = "";
String senhaSalva = "";

unsigned long wifiStart = 0;
const unsigned long wifiTimeout = 10000;

bool reiniciar = false;
unsigned long tempoReinicio = 0;

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

// ===== FLASH =====
#define MAX 288
float vibDados[MAX];
float tempDados[MAX];
int indice = 0;
bool cheio = false;

// ===== TEMPO =====
unsigned long ultimoTempoMedia = 0;
const unsigned long intervaloMedia = 300000;

// ===== GRAVIDADE =====
float alpha = 0.9;
float gravX = 0, gravY = 0, gravZ = 0;

// ================= WIFI =================
void salvarWiFi(String ssid, String senha){
  preferences.begin("wifi", false);
  preferences.putString("ssid", ssid);
  preferences.putString("senha", senha);
  preferences.end();
}

void carregarWiFi(){
  preferences.begin("wifi", true);
  ssidSalvo = preferences.getString("ssid", "");
  senhaSalva = preferences.getString("senha", "");
  preferences.end();
}

void iniciarAP(){
  WiFi.mode(WIFI_AP);
  WiFi.softAP("ESP-CONFIG", "12345678");
  modoAP = true;

  Serial.println("=== MODO AP ATIVO ===");
  Serial.println("Rede: ESP-CONFIG");

  Serial.println("Modo AP ativo");
  Serial.print("Acesse: http://");
  Serial.println(WiFi.softAPIP());
}

void iniciarSTA(){
  WiFi.mode(WIFI_STA);

  Serial.println("Tentando conectar no WiFi...");
  Serial.println("SSID: " + ssidSalvo);
  
  WiFi.begin(ssidSalvo.c_str(), senhaSalva.c_str());
  wifiStart = millis();
}

void gerenciarWiFi(){

  if(modoAP) return;

  wl_status_t status = WiFi.status();

  static wl_status_t ultimoStatus = WL_IDLE_STATUS;

  if(status != ultimoStatus){
    Serial.print("Status WiFi mudou: ");

    switch(status){
      case WL_NO_SSID_AVAIL: Serial.println("SSID não encontrado"); break;
      case WL_CONNECT_FAILED: Serial.println("Falha na senha"); break;
      case WL_DISCONNECTED: Serial.println("Desconectado"); break;
      case WL_CONNECTED: Serial.println("Conectado"); break;
      default: Serial.println(status); break;
    }

    ultimoStatus = status;
  }

  if(status == WL_CONNECTED){
    static bool conectado = false;
    if(!conectado){
      Serial.println("=== CONECTADO COM SUCESSO ===");
      Serial.print("IP: ");
      Serial.println(WiFi.localIP());
      conectado = true;
    }
    return;
  }

  if(millis() - wifiStart > wifiTimeout){
    Serial.println("=== FALHA AO CONECTAR ===");
    Serial.println("Tempo excedido (timeout)");

    Serial.println("Voltando para modo AP...");

    iniciarAP();
  }
}

void handleSalvar(){
  String s = server.arg("s");
  String p = server.arg("p");

  Serial.println("=== NOVA CONFIG WIFI ===");
  Serial.println("SSID: " + s);
  Serial.println("Senha: " + p);

  salvarWiFi(s,p);

  Serial.println("Credenciais salvas!");

  server.send(200,"text/html","Salvo! Reiniciando...");

  reiniciar = true;
  tempoReinicio = millis();
}

void scanWiFi(){
  Serial.println("Escaneando redes...");

  int n = WiFi.scanNetworks();

  if(n == 0){
    Serial.println("Nenhuma rede encontrada");
  } else {
    for(int i=0;i<n;i++){
      Serial.print(i+1);
      Serial.print(": ");
      Serial.println(WiFi.SSID(i));
    }
  }
}
// ================= CORE =================
void removerGravidade(float x, float y, float z,
                      float &linX, float &linY, float &linZ) {
  gravX = alpha * gravX + (1 - alpha) * x;
  gravY = alpha * gravY + (1 - alpha) * y;
  gravZ = alpha * gravZ + (1 - alpha) * z;

  linX = x - gravX;
  linY = y - gravY;
  linZ = z - gravZ;
}

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

// ===== FFT =====
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

  if (freq > 1) {
    vibAtual = (pico / (2 * PI * freq)) * 1000;
  }

  fftReady = false;
}

// ===== TEMP =====
void atualizarTemperatura() {
  if (!aguardandoTemp && millis() - ultimoTempRequest >= 1000) {
    sensors.requestTemperatures();
    aguardandoTemp = true;
    ultimoTempRequest = millis();
  }

  if (aguardandoTemp && millis() - ultimoTempRequest >= 750) {
    float t = sensors.getTempCByIndex(0);
    if (t != -127 && t > -50 && t < 150) {
      tempAtual = t;
    }
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

// ===== SETUP =====
void setup() {

  LittleFS.begin();
  Serial.begin(115200);

  sensors.begin();
  sensors.setWaitForConversion(false);

  accel.begin();

  carregarFlash();
  carregarWiFi();

  scanWiFi();

  if(ssidSalvo != ""){
    iniciarSTA();
  } else {
    iniciarAP();
  }

  server.on("/salvar", handleSalvar);
  server.on("/", [](){ server.send(200,"text/html",gerarHTML()); });
  server.on("/data", [](){ server.send(200,"application/json",gerarJSON()); });
  server.on("/csv", handleCSV);

  server.begin();
  Serial.println("=== SISTEMA INICIADO ===");
  Serial.println("SSID salvo: " + ssidSalvo);
}

// ===== LOOP =====
void loop() {
  server.handleClient();

  gerenciarWiFi();

  coletarAmostraFFT();

  if (fftReady) processarFFT();

  atualizarTemperatura();

  if (millis() - ultimoTempoMedia >= intervaloMedia) {
    ultimoTempoMedia = millis();
    adicionarDado(vibAtual, tempAtual);
    salvarFlash();
  }

  if(reiniciar && millis() - tempoReinicio > 2000){
    ESP.restart();
  }
}
