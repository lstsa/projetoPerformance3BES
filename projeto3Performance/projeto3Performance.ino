// ===== DEFINES =====
#define FFT_SPEED_OVER_PRECISION
#define LED_PIN 2
#define ONE_WIRE_BUS 4
#define SAMPLES 64
#define SAMPLING_FREQUENCY 100
#define MAX_HISTORICO 288


// ===== INCLUDES =====
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <arduinoFFT.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"



// ===== ESTRUTURA DE DADOS =====
typedef struct {
  float vibracao;
  float temperatura;
  unsigned long timestamp;
} DadoSensor;


// ===== FILAS =====
QueueHandle_t filaProcessamento;
QueueHandle_t filaLeitura;


// ===== MUTEXES =====
SemaphoreHandle_t fsMutex;


// ===== SERVIDOR =====
AsyncWebServer server(80);


// ===== SENSORES =====
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified();


// ===== FFT =====
double vReal[SAMPLES];
double vImag[SAMPLES];
ArduinoFFT<double> FFT = ArduinoFFT<double>();
int sampleIndex = 0;
bool fftReady = false;


// ===== TIMER DE INTERRUPÇÃO =====
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool coletarAmostra = false;


// ===== GRAVIDADE =====
float alpha = 0.9;
float gravX = 0, gravY = 0, gravZ = 0;


// ===== TEMPERATURA =====
unsigned long ultimoTempRequest = 0;
bool aguardandoTemp = false;


// ===== BUFFERS HTTP =====
static char jsonBuffer[128];
static char csvBuffer[8192];


// ===== WIFI CONFIG =====
String ssidSalvo = "";
String senhaSalva = "";
unsigned long wifiStart = 0;
const unsigned long wifiTimeout = 10000;


// ===== ESTADO WIFI =====
enum WifiState { AP_MODE, CONNECTING, CONNECTED };
WifiState wifiState = AP_MODE;


// ===== INTERRUPÇÃO DE HARDWARE =====
void IRAM_ATTR onTimer(){
  portENTER_CRITICAL_ISR(&timerMux);
  coletarAmostra = true;
  portEXIT_CRITICAL_ISR(&timerMux);
}


// ===== WIFI =====
void salvarWiFi(String ssid, String senha){
  if(xSemaphoreTake(fsMutex, pdMS_TO_TICKS(500))){
    File f = LittleFS.open("/config.txt", "w");
    if(f){
      f.println(ssid);
      f.println(senha);
      f.close();
    }
    xSemaphoreGive(fsMutex);
  }
}

void carregarWiFi(){
  if(xSemaphoreTake(fsMutex, pdMS_TO_TICKS(500))){
    File f = LittleFS.open("/config.txt", "r");
    if(f){
      ssidSalvo  = f.readStringUntil('\n'); ssidSalvo.trim();
      senhaSalva = f.readStringUntil('\n'); senhaSalva.trim();
      f.close();
    }
    xSemaphoreGive(fsMutex);
  }
}

void iniciarAP(){
  WiFi.mode(WIFI_AP);
  WiFi.softAP("ESP-CONFIG", "12345678");
  wifiState = AP_MODE;
  Serial.println("=== MODO AP ATIVO ===");
  Serial.println(WiFi.softAPIP());
}

void iniciarSTA(){
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(true);
  if(senhaSalva == ""){
    WiFi.begin(ssidSalvo.c_str());
  } else {
    WiFi.begin(ssidSalvo.c_str(), senhaSalva.c_str());
  }
  wifiStart = millis();
  wifiState = CONNECTING;
  Serial.println("=== CONECTANDO ===");
  Serial.println(ssidSalvo);
}

void gerenciarWiFi(){
  wl_status_t status = WiFi.status();

  switch(wifiState){

    case AP_MODE:
      break;

    case CONNECTING:
      if(status == WL_CONNECTED){
        wifiState = CONNECTED;
        Serial.println("=== WIFI CONECTADO ===");
        Serial.println(WiFi.localIP());
      } else if(millis() - wifiStart > wifiTimeout){
        Serial.println("=== FALHA AO CONECTAR ===");
        WiFi.disconnect();
        iniciarAP();
      }
      break;

    case CONNECTED:
      if(status != WL_CONNECTED){
        static unsigned long inicioQueda = 0;
        if(inicioQueda == 0){
          inicioQueda = millis();
          Serial.println("WiFi instavel, aguardando...");
        } else if(millis() - inicioQueda > 5000){
          inicioQueda = 0;
          Serial.println("=== WIFI DESCONECTADO ===");
          iniciarAP();
        }
      } else {
        static unsigned long inicioQueda = 0;
        inicioQueda = 0;
      }
      break;
  }
}

void handleSalvar(AsyncWebServerRequest *request){
  String s = request->hasParam("s") ? request->getParam("s")->value() : "";
  String p = request->hasParam("p") ? request->getParam("p")->value() : "";

  if(s == ""){
    request->send(400, "text/plain", "SSID invalido");
    return;
  }

  salvarWiFi(s, p);
  ssidSalvo  = s;
  senhaSalva = p;

  request->send(200, "text/html", "Salvo! Reconectando...");
  iniciarSTA();
}


// ===== SENSORES =====
void removerGravidade(float x, float y, float z,
                      float &linX, float &linY, float &linZ){
  gravX = alpha * gravX + (1 - alpha) * x;
  gravY = alpha * gravY + (1 - alpha) * y;
  gravZ = alpha * gravZ + (1 - alpha) * z;
  linX = x - gravX;
  linY = y - gravY;
  linZ = z - gravZ;
}

void coletarAmostraFFT(){
  portENTER_CRITICAL(&timerMux);
  bool deve = coletarAmostra;
  coletarAmostra = false;
  portEXIT_CRITICAL(&timerMux);

  if(!deve) return;

  sensors_event_t event;
  if(!accel.getEvent(&event)) return;

  float linX, linY, linZ;
  removerGravidade(event.acceleration.x,
                   event.acceleration.y,
                   event.acceleration.z,
                   linX, linY, linZ);

  vReal[sampleIndex] = sqrt(linX*linX + linY*linY + linZ*linZ);
  vImag[sampleIndex] = 0;
  sampleIndex++;

  if(sampleIndex >= SAMPLES){
    sampleIndex = 0;
    fftReady = true;
  }
}

void processarFFT(float tempAtual){
  FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.complexToMagnitude(vReal, vImag, SAMPLES);

  double pico = 0;
  int idx = 0;

  for(int i = 1; i < SAMPLES / 2; i++){
    if(vReal[i] > pico){
      pico = vReal[i];
      idx = i;
    }
  }

  double freq = (idx * SAMPLING_FREQUENCY) / (double)SAMPLES;
  if(freq <= 1){
    fftReady = false;
    return;
  }

  DadoSensor dado;
  dado.vibracao   = (pico / (2 * PI * freq)) * 1000;
  dado.temperatura = tempAtual;
  dado.timestamp   = millis();

  xQueueSend(filaProcessamento, &dado, 0);
  xQueueOverwrite(filaLeitura, &dado);

  fftReady = false;
}

void atualizarTemperatura(float &tempAtual){
  if(!aguardandoTemp && millis() - ultimoTempRequest >= 1000){
    sensors.requestTemperatures();
    aguardandoTemp = true;
    ultimoTempRequest = millis();
  }

  if(aguardandoTemp && millis() - ultimoTempRequest >= 750){
    float t = sensors.getTempCByIndex(0);
    if(t != -127 && t > -50 && t < 150){
      tempAtual = t;
    }
    aguardandoTemp = false;
  }
}


// ===== ARMAZENAMENTO =====
void salvarDado(DadoSensor &dado){
  if(xSemaphoreTake(fsMutex, pdMS_TO_TICKS(500))){

    int linhas = 0;
    File f = LittleFS.open("/dados.csv", "r");
    if(f){
      while(f.available()){
        if(f.read() == '\n') linhas++;
      }
      f.close();
    }

    if(linhas >= MAX_HISTORICO){
      File fLeitura = LittleFS.open("/dados.csv", "r");
      File fTemp    = LittleFS.open("/temp.csv", "w");
      if(fLeitura && fTemp){
        fLeitura.readStringUntil('\n'); // descarta linha mais antiga
        while(fLeitura.available()){
          fTemp.write(fLeitura.read());
        }
      }
      if(fLeitura) fLeitura.close();
      if(fTemp)    fTemp.close();
      LittleFS.remove("/dados.csv");
      LittleFS.rename("/temp.csv", "/dados.csv");
    }

    File fa = LittleFS.open("/dados.csv", "a");
    if(fa){
      fa.printf("%.2f,%.2f,%lu\n",
                dado.vibracao,
                dado.temperatura,
                dado.timestamp);
      fa.close();
    }

    xSemaphoreGive(fsMutex);
  }
}

String gerarCSV(){
  memset(csvBuffer, 0, sizeof(csvBuffer));
  int len = 0;

  len += snprintf(csvBuffer + len, sizeof(csvBuffer) - len,
                  "Vibracao(mm/s),Temperatura,Timestamp\n");

  if(xSemaphoreTake(fsMutex, pdMS_TO_TICKS(500))){
    File f = LittleFS.open("/dados.csv", "r");
    if(f){
      while(f.available() && len < (int)sizeof(csvBuffer) - 32){
        csvBuffer[len++] = (char)f.read();
      }
      f.close();
    }
    xSemaphoreGive(fsMutex);
  }

  return String(csvBuffer);
}

String gerarJSON(){
  DadoSensor dado = {0.0f, 0.0f, 0};
  xQueuePeek(filaLeitura, &dado, 0);

  snprintf(jsonBuffer, sizeof(jsonBuffer),
    "{\"vibAtual\":%.2f,\"tempAtual\":%.2f}",
    dado.vibracao, dado.temperatura);

  return String(jsonBuffer);
}


// ===== LED =====
void atualizarLED(){
  static unsigned long lastBlink = 0;
  static bool estado = false;

  switch(wifiState){
    case AP_MODE:
      if(millis() - lastBlink > 1000){
        estado = !estado;
        digitalWrite(LED_PIN, estado);
        lastBlink = millis();
      }
      break;
    case CONNECTING:
      if(millis() - lastBlink > 200){
        estado = !estado;
        digitalWrite(LED_PIN, estado);
        lastBlink = millis();
      }
      break;
    case CONNECTED:
      digitalWrite(LED_PIN, HIGH);
      break;
  }
}

// ===== TASKS =====
void taskComunicacao(void *pvParameters){
  while(true){
    gerenciarWiFi();
    atualizarLED();
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void taskSensores(void *pvParameters){
  float tempLocal = 0.0f;

  while(true){
    coletarAmostraFFT();

    if(fftReady) processarFFT(tempLocal);

    atualizarTemperatura(tempLocal);

    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

void taskArmazenamento(void *pvParameters){
  DadoSensor dado;
  DadoSensor acumulado = {0.0f, 0.0f, 0};
  int contagem = 0;
  unsigned long ultimoSalvo = millis();

  while(true){
    if(xQueueReceive(filaProcessamento, &dado, pdMS_TO_TICKS(1000))){
      acumulado.vibracao    += dado.vibracao;
      acumulado.temperatura += dado.temperatura;
      acumulado.timestamp    = dado.timestamp;
      contagem++;
    }

    if(millis() - ultimoSalvo >= 300000 && contagem > 0){
      DadoSensor media;
      media.vibracao    = acumulado.vibracao    / contagem;
      media.temperatura = acumulado.temperatura / contagem;
      media.timestamp   = acumulado.timestamp;

      salvarDado(media);

      acumulado = {0.0f, 0.0f, 0};
      contagem  = 0;
      ultimoSalvo = millis();
    }

    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}


// ===== SETUP =====
void setup(){
  Serial.begin(115200);
  delay(500);

  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  pinMode(LED_PIN, OUTPUT);

  // recursos compartilhados
  filaProcessamento = xQueueCreate(20, sizeof(DadoSensor));
  filaLeitura       = xQueueCreate(1,  sizeof(DadoSensor));
  fsMutex           = xSemaphoreCreateMutex();

  if(!LittleFS.begin(true)){
    Serial.println("ERRO: LittleFS falhou");
  } else {
    Serial.println("LittleFS OK");
  }

  sensors.begin();
  sensors.setWaitForConversion(false);

  if(!accel.begin()){
    Serial.println("ERRO: ADXL345 nao encontrado");
    while(true){ vTaskDelay(100 / portTICK_PERIOD_MS); }
  }
  Serial.println("ADXL345 OK");

// timer de interrupção — 100Hz
  timer = timerBegin(100);                    // frequência diretamente em Hz
  timerAttachInterrupt(timer, &onTimer);      // sem o terceiro argumento
  timerAlarm(timer, 1000000 / 100, true, 0); // 100Hz = alarme a cada 10ms

  carregarWiFi();

  if(ssidSalvo != ""){
    iniciarSTA();
  } else {
    iniciarAP();
  }

  server.on("/salvar", HTTP_GET, handleSalvar);
  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
  server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "application/json", gerarJSON());
  });
  server.on("/csv", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/csv", gerarCSV());
  });
  server.begin();

  Serial.println("=== SISTEMA INICIADO ===");

  xTaskCreatePinnedToCore(taskComunicacao,   "Comunicacao",   4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(taskSensores,      "Sensores",      6144, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(taskArmazenamento, "Armazenamento", 4096, NULL, 1, NULL, 1);
}

// ===== LOOP =====
void loop(){}
