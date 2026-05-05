#define FFT_SPEED_OVER_PRECISION


// ==== mutex ====

SemaphoreHandle_t dataMutex;

//===== maquina wifiState ======

enum WifiState {
  AP_MODE,
  CONNECTING,
  CONNECTED
};

WifiState wifiState = AP_MODE;

// ======= bibliotecas ======

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <Preferences.h>
#define LED_PIN 2

#include <OneWire.h>
#include <DallasTemperature.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

#include <arduinoFFT.h>

#include <LittleFS.h>

#include "soc/soc.h"           // ADICIONE no topo do arquivo, junto dos outros includes
#include "soc/rtc_cntl_reg.h"  // ADICIONE no topo do arquivo, junto dos outros includes

// ===== WIFI CONFIG =====
bool modoAP = false;

String ssidSalvo = "";
String senhaSalva = "";

unsigned long wifiStart = 0;
const unsigned long wifiTimeout = 10000;

bool reiniciar = false;
unsigned long tempoReinicio = 0;

AsyncWebServer server(80);
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
static char csvBuffer[4096];
static char jsonBuffer[256];

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
  wifiState = AP_MODE;

  Serial.println("\n=== MODO AP ATIVO ===");
  Serial.print("IP AP: ");
  Serial.println(WiFi.softAPIP());
}

void iniciarSTA(){
  WiFi.mode(WIFI_STA);
  if(senhaSalva == ""){
    WiFi.begin(ssidSalvo.c_str());
  } else {
    WiFi.begin(ssidSalvo.c_str(), senhaSalva.c_str());
  }

  wifiStart = millis();
  modoAP = false;
  wifiState = CONNECTING;

  Serial.println("\n=== CONECTANDO WIFI ===");
  Serial.println("SSID: " + ssidSalvo);
}
void gerenciarWiFi(){

  wl_status_t status = WiFi.status();

  switch(wifiState){

    // ================= AP =================
    case AP_MODE:
      break;

    // ================= CONECTANDO =================
    case CONNECTING:

      if(status == WL_CONNECTED){
        wifiState = CONNECTED;
        modoAP = false;

        Serial.println("\n=== WIFI CONECTADO ===");
        Serial.print("IP: ");
        Serial.println(WiFi.localIP());

      } else if(millis() - wifiStart > wifiTimeout){

        Serial.println("\n=== FALHA AO CONECTAR ===");
        WiFi.disconnect();
        iniciarAP();
      }

      break;

    // ================= CONECTADO =================
    case CONNECTED:

      if(status != WL_CONNECTED){
        Serial.println("\n=== WIFI DESCONECTADO ===");
        iniciarAP();
      }

      break;
  }
}

void handleSalvar(AsyncWebServerRequest *request){
  String s = request->hasParam("s") ? request->getParam("s")->value() : "";
  String p = request->hasParam("p") ? request->getParam("p")->value() : "";

  if(s == ""){
    request->send(400, "text/plain", "SSID inválido");
    return;
  }

  Serial.println("=== NOVA CONFIG WIFI ===");
  Serial.println("SSID: " + s);
  Serial.println("Senha: " + p);

  salvarWiFi(s,p);

  request->send(200,"text/html","Salvo! Reiniciando...");

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
    if(xSemaphoreTake(dataMutex, portMAX_DELAY)){
      vibAtual = (pico / (2 * PI * freq)) * 1000;
      xSemaphoreGive(dataMutex);
    }
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
      if(xSemaphoreTake(dataMutex, portMAX_DELAY)){
        tempAtual = t;
        xSemaphoreGive(dataMutex);
      }
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
String gerarCSV(){
  int len = 0;
  memset(csvBuffer, 0, sizeof(csvBuffer));
  len += snprintf(csvBuffer + len, sizeof(csvBuffer) - len,  // CORRIGIDO
                  "Vibracao(mm/s),Temperatura\n");

  float vibLocal[MAX];
  float tempLocal[MAX];
  int indiceLocal;
  bool cheioLocal;

  if(xSemaphoreTake(dataMutex, portMAX_DELAY)){
    memcpy(vibLocal, vibDados, sizeof(vibDados));
    memcpy(tempLocal, tempDados, sizeof(tempDados));
    indiceLocal = indice;
    cheioLocal = cheio;
    xSemaphoreGive(dataMutex);
  }

  for(int i = 0; i < indiceLocal; i++){
    len += snprintf(csvBuffer + len, sizeof(csvBuffer) - len,  // CORRIGIDO
                    "%.2f,%.2f\n",
                    vibLocal[i], tempLocal[i]);
  }

  return String(csvBuffer);
}


// ===== JSON =====
String gerarJSON() {
  float vib, temp;

  if(xSemaphoreTake(dataMutex, portMAX_DELAY)){
    vib = vibAtual;
    temp = tempAtual;
    xSemaphoreGive(dataMutex);
  }

  snprintf(jsonBuffer, sizeof(jsonBuffer),
    "{\"vibAtual\":%.2f,\"tempAtual\":%.2f}",
    vib, temp
  );

  return String(jsonBuffer);
}

//====== led ======

void atualizarLED() {
  static unsigned long lastBlink = 0;
  static bool estado = false;

  switch(wifiState){

    case AP_MODE:
      if (millis() - lastBlink > 1000) {
        estado = !estado;
        digitalWrite(LED_PIN, estado);
        lastBlink = millis();
      }
      break;

    case CONNECTING:
      if (millis() - lastBlink > 200) {
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

//====== tasks =====

  void gravarHeartbeat(const char* origem){
    preferences.begin("diag", false);
    preferences.putString("hb", origem);
    preferences.end();
  }
  
void taskWiFi(void *pvParameters){
  static unsigned long lastHeap = 0;

  while(true){
    gerenciarWiFi();
    atualizarLED();

    if(millis() - lastHeap > 5000){
      lastHeap = millis();
      debugHeap();
      Serial.print("Stack WiFi: ");                        // ADICIONADO
      Serial.println(uxTaskGetStackHighWaterMark(NULL));   // ADICIONADO
      gravarHeartbeat("taskWiFi");  // ADICIONADO
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void taskSensores(void *pvParameters){
  static unsigned long lastStack = 0;                      // ADICIONADO

  while(true){
    coletarAmostraFFT();

    if (fftReady) processarFFT();

    atualizarTemperatura();

    if(millis() - lastStack > 5000){                       // ADICIONADO
      lastStack = millis();                                // ADICIONADO
      Serial.print("Stack Sensores: ");                   // ADICIONADO
      Serial.println(uxTaskGetStackHighWaterMark(NULL));   // ADICIONADO
      gravarHeartbeat("taskSensores");  // ADICIONADO
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);  // era 5, agora 10
  }
}

void taskArmazenamento(void *pvParameters){
  static unsigned long lastStack = 0;                      // ADICIONADO

  while(true){
    if (millis() - ultimoTempoMedia >= intervaloMedia) {
      ultimoTempoMedia = millis();

      float vib, temp;

      if(xSemaphoreTake(dataMutex, portMAX_DELAY)){
        vib = vibAtual;
        temp = tempAtual;
        adicionarDado(vib, temp);
        xSemaphoreGive(dataMutex);
      }

      salvarFlash();
    }

    if(millis() - lastStack > 5000){                       // ADICIONADO
      lastStack = millis();                                // ADICIONADO
      Serial.print("Stack Armazenamento: ");              // ADICIONADO
      Serial.println(uxTaskGetStackHighWaterMark(NULL));   // ADICIONADO
      gravarHeartbeat("taskArmazenamento");  // ADICIONADO
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
//=== debug heap ===
void debugHeap(){
  Serial.print("Heap livre: ");
  Serial.println(ESP.getFreeHeap());
}

void setup() {
  Serial.begin(115200);
  delay(500);

  // lê e salva ANTES de qualquer outro preferences.begin()
  esp_reset_reason_t motivo = esp_reset_reason();

  preferences.begin("diag", false);
  String ultimoCrash = preferences.getString("crash", "nenhum");
  String ultimoHeartbeat = preferences.getString("hb", "nenhum");  // ADICIONADO
  Serial.print("Ultimo crash salvo: ");
  Serial.println(ultimoCrash);
  Serial.print("Ultimo heartbeat: ");                               // ADICIONADO
  Serial.println(ultimoHeartbeat);                                  // ADICIONADO

  // salva o motivo atual para o próximo boot
  switch(motivo){
    case ESP_RST_POWERON:   preferences.putString("crash", "Power on");        break;
    case ESP_RST_SW:        preferences.putString("crash", "Software");        break;
    case ESP_RST_PANIC:     preferences.putString("crash", "Panic/Exception"); break;
    case ESP_RST_INT_WDT:   preferences.putString("crash", "WDT Interrupcao");break;
    case ESP_RST_TASK_WDT:  preferences.putString("crash", "WDT Task");       break;
    case ESP_RST_WDT:       preferences.putString("crash", "WDT Outro");      break;
    case ESP_RST_BROWNOUT:  preferences.putString("crash", "Brownout");       break;
    default:                preferences.putString("crash", "Desconhecido");   break;
  }
  preferences.end(); // fecha ANTES de qualquer outro begin()

  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  dataMutex = xSemaphoreCreateMutex();

  pinMode(LED_PIN, OUTPUT);

  if(!LittleFS.begin(true)){
    Serial.println("Erro ao montar LittleFS");
  } else {
    Serial.println("LittleFS OK");
  }

  sensors.begin();
  sensors.setWaitForConversion(false);

  accel.begin();

  carregarFlash();
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
  Serial.println("SSID salvo: " + ssidSalvo);


  xTaskCreatePinnedToCore(taskWiFi, "Task WiFi", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(taskSensores, "Task Sensores", 6144, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(taskArmazenamento, "Task Armazenamento", 4096, NULL, 1, NULL, 1);
}

// ===== LOOP =====
void loop() {
  if(reiniciar && millis() - tempoReinicio > 1000){
    ESP.restart();
  }
}
