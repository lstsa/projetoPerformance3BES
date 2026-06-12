// ===== DEFINES =====
#define FFT_SPEED_OVER_PRECISION
#define LED_PIN 2
#define ONE_WIRE_BUS 4
#define SAMPLES 256
// frequência configurável (padrão 100Hz, mínimo 10, máximo 200)
int samplingFrequency = 800;
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
#include <LiquidCrystal_I2C.h>

// ===== PINOS LEDs RGB =====
#define LED_WIFI_R 25
#define LED_WIFI_G 26
#define LED_WIFI_B 27
#define LED_VIB_R  32
#define LED_VIB_G  33
#define LED_VIB_B  14

// ===== LCD =====
LiquidCrystal_I2C lcd(0x27, 16, 2);
bool lcdDisponivel = false;


// ===== ESTRUTURA DE DADOS =====
typedef struct {
  float vibracao;
  float temperatura;
  unsigned long timestamp;
} DadoSensor;

typedef struct {
  float vibMedia;
  float tempMedia;
  unsigned long timestamp;
} DadoHistorico;

#define MAX_HISTORICO_RAM 288
DadoHistorico historicoRAM[MAX_HISTORICO_RAM];
int historicoCount = 0;      // quantos slots preenchidos
int historicoInicio = 0;     // índice do mais antigo (buffer circular)
SemaphoreHandle_t historicoMutex;

// ===== CONFIGURAÇÃO DE MÁQUINA =====
typedef struct {
  int grupo;           // 1, 2, 3 ou 4
  float limAB;         // limite zona A/B
  float limBC;         // limite zona B/C
  float limCD;         // limite zona C/D
} ConfigMaquina;

// limites ISO 10816-3 por grupo
const ConfigMaquina GRUPOS_ISO[4] = {
  {1, 0.71f, 1.80f,  4.50f},
  {2, 1.12f, 2.80f,  7.10f},
  {3, 1.80f, 4.50f, 11.20f},
  {4, 2.80f, 7.10f, 18.00f}
};

ConfigMaquina configAtual = GRUPOS_ISO[0]; // padrão: Grupo I

// ===== MÉTRICAS DE TEMPO DE EXECUÇÃO =====
volatile unsigned long tempoFFT          = 0;
volatile unsigned long tempoArmazenamento = 0;
volatile unsigned long tempoJSON         = 0;
volatile unsigned long tempoTemperatura  = 0;
volatile unsigned long tempoWifi         = 0;

// ===== LOGS =====
#define MAX_LOGS 100
#define MAX_LOG_MSG 80

typedef struct {
  unsigned long timestamp;
  char nivel;    // 'I' info, 'W' warning, 'E' erro
  char msg[MAX_LOG_MSG];
} EntradaLog;

EntradaLog bufferLog[MAX_LOGS];
int logCount   = 0;
int logInicio  = 0;
SemaphoreHandle_t logMutex;

void registrarLog(char nivel, const char* msg){
  if(xSemaphoreTake(logMutex, pdMS_TO_TICKS(100))){
    int idx = (logInicio + logCount) % MAX_LOGS;
    bufferLog[idx].timestamp = millis();
    bufferLog[idx].nivel     = nivel;
    strncpy(bufferLog[idx].msg, msg, MAX_LOG_MSG - 1);
    bufferLog[idx].msg[MAX_LOG_MSG - 1] = '\0';

    if(logCount < MAX_LOGS){
      logCount++;
    } else {
      logInicio = (logInicio + 1) % MAX_LOGS;
    }
    xSemaphoreGive(logMutex);
  }
}


// ===== FILAS =====
QueueHandle_t filaProcessamento;
QueueHandle_t filaLeitura;


// ===== MUTEXES =====
SemaphoreHandle_t fsMutex;
SemaphoreHandle_t configMutex;


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
static char jsonBuffer[256];
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

void salvarConfigMaquina(int grupo){
  if(xSemaphoreTake(fsMutex, pdMS_TO_TICKS(500))){
    File f = LittleFS.open("/maquina.txt", "w");
    if(f){
      f.println(grupo);
      f.close();
    }
    xSemaphoreGive(fsMutex);
  }
}

void carregarConfigMaquina(){
  if(xSemaphoreTake(fsMutex, pdMS_TO_TICKS(500))){
    File f = LittleFS.open("/maquina.txt", "r");
    if(f){
      int g = f.readStringUntil('\n').toInt();
      if(g >= 1 && g <= 4){
        if(xSemaphoreTake(configMutex, pdMS_TO_TICKS(500))){
          configAtual = GRUPOS_ISO[g - 1];
          xSemaphoreGive(configMutex);
        }
        char logMsg[MAX_LOG_MSG];
        snprintf(logMsg, sizeof(logMsg), "Grupo de maquina carregado: %d", g);
        registrarLog('I', logMsg);
      }
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
  registrarLog('I', "Modo AP ativo: 192.168.4.1");
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
  char logMsg[MAX_LOG_MSG];
  snprintf(logMsg, sizeof(logMsg), "Conectando ao Wi-Fi: %s", ssidSalvo.c_str());
  registrarLog('I', logMsg);
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
        char logMsg[MAX_LOG_MSG];
        snprintf(logMsg, sizeof(logMsg), "Wi-Fi conectado. IP: %s", WiFi.localIP().toString().c_str());
        registrarLog('I', logMsg);
      } else if(millis() - wifiStart > wifiTimeout){
        Serial.println("=== FALHA AO CONECTAR ===");
        WiFi.disconnect();
        registrarLog('E', "Falha ao conectar ao Wi-Fi. Voltando ao modo AP.");
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
          registrarLog('W', "Wi-Fi desconectado. Voltando ao modo AP.");
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

  unsigned long t0 = micros();

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
  tempoWifi = micros() - t0;
}

void processarFFT(float tempAtual){
  unsigned long t0 = micros();
  double somaQuad = 0;
  for(int i = 0; i < SAMPLES; i++){
    somaQuad += vReal[i] * vReal[i];
  }
  double acelRMS = sqrt(somaQuad / SAMPLES); // m/s²

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

  double freq = (idx * samplingFrequency) / (double)SAMPLES;
  if(freq <= 1){
    fftReady = false;
    return;
  }

  // integração espectral: soma contribuição de cada bin de frequência
  double somaVelQuad = 0;
  for(int i = 1; i < SAMPLES / 2; i++){
    double fi = (i * samplingFrequency) / (double)SAMPLES;
    if(fi < 1.0) continue; // ignora DC e frequências muito baixas
    double velBin = (vReal[i] / (2.0 * PI * fi)) * 1000.0 / (SAMPLES / 2.0);
    somaVelQuad += velBin * velBin;
  }
  double velRMS = sqrt(somaVelQuad);

  DadoSensor dado;
  dado.vibracao    = velRMS;   
  dado.temperatura = tempAtual;
  dado.timestamp   = millis();

  xQueueSend(filaProcessamento, &dado, 0);
  xQueueOverwrite(filaLeitura, &dado);
  
  tempoFFT = micros() - t0;
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
void salvarDado(DadoHistorico &dado){
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
        fLeitura.readStringUntil('\n');
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
                dado.vibMedia,
                dado.tempMedia,
                dado.timestamp);
      fa.close();
    }

    registrarLog('I', "Historico salvo na flash.");
    xSemaphoreGive(fsMutex);
  }
}

String gerarCSV(){
  memset(csvBuffer, 0, sizeof(csvBuffer));
  int len = 0;

  // cabeçalho do relatório
  len += snprintf(csvBuffer + len, sizeof(csvBuffer) - len,
    "# Relatorio ESP Monitor — ISO 10816\n"
    "# Intervalo de amostragem: 5 minutos\n"
    "# Maximo de registros: 288 (24 horas)\n"
    "#\n"
    "Timestamp(ms),Vibracao_media(mm/s RMS),Temperatura_media(C)\n");

  float vibMax  = -1e9, vibMin  =  1e9, vibSoma  = 0;
  float tempMax = -1e9, tempMin =  1e9, tempSoma = 0;
  int   total   = 0;

  if(xSemaphoreTake(historicoMutex, pdMS_TO_TICKS(500))){
    for(int i = 0; i < historicoCount; i++){
      int idx = (historicoInicio + i) % MAX_HISTORICO_RAM;
      DadoHistorico &h = historicoRAM[idx];

      len += snprintf(csvBuffer + len, sizeof(csvBuffer) - len,
                "%.2f,%.2f,%lu\n",
                h.vibMedia, h.tempMedia, h.timestamp);

      // acumula estatísticas
      if(h.vibMedia  > vibMax)  vibMax  = h.vibMedia;
      if(h.vibMedia  < vibMin)  vibMin  = h.vibMedia;
      if(h.tempMedia > tempMax) tempMax = h.tempMedia;
      if(h.tempMedia < tempMin) tempMin = h.tempMedia;
      vibSoma  += h.vibMedia;
      tempSoma += h.tempMedia;
      total++;
    }
    xSemaphoreGive(historicoMutex);
  }

  // bloco de estatísticas no rodapé
  if(total > 0){
    len += snprintf(csvBuffer + len, sizeof(csvBuffer) - len,
      "#\n"
      "# === ESTATISTICAS (ultimas 24h) ===\n"
      "# Vibração maxima: %.2f mm/s RMS\n"
      "# Vibração minima: %.2f mm/s RMS\n"
      "# Vibração media:  %.2f mm/s RMS\n"
      "# Temperatura maxima: %.1f C\n"
      "# Temperatura minima: %.1f C\n"
      "# Temperatura media:  %.1f C\n"
      "# Total de registros: %d\n",
      vibMax, vibMin, vibSoma / total,
      tempMax, tempMin, tempSoma / total,
      total);
  }

  return String(csvBuffer);
}

String gerarJSON(){
  unsigned long t0 = micros();
  DadoSensor dado = {0.0f, 0.0f, 0};
  xQueuePeek(filaLeitura, &dado, 0);

  ConfigMaquina cfg;
  if(xSemaphoreTake(configMutex, pdMS_TO_TICKS(50))){
    cfg = configAtual;
    xSemaphoreGive(configMutex);
  } else {
    cfg = GRUPOS_ISO[0]; // fallback Grupo I
  }

  const char* zona = "A";
  if(dado.vibracao >= cfg.limCD)      zona = "D";
  else if(dado.vibracao >= cfg.limBC) zona = "C";
  else if(dado.vibracao >= cfg.limAB) zona = "B";

  snprintf(jsonBuffer, sizeof(jsonBuffer),
    "{\"vibAtual\":%.2f,\"tempAtual\":%.2f,"
    "\"zona\":\"%s\",\"grupo\":%d,"
    "\"limAB\":%.2f,\"limBC\":%.2f,\"limCD\":%.2f}",
    dado.vibracao, dado.temperatura,
    zona, cfg.grupo,
    cfg.limAB, cfg.limBC, cfg.limCD);

  tempoJSON = micros() - t0;
  return String(jsonBuffer);
}

// buffer separado para não conflitar com jsonBuffer
static char jsonHistBuffer[12288];

String gerarJSONHistorico(){
  unsigned long t0 = micros();
  memset(jsonHistBuffer, 0, sizeof(jsonHistBuffer));
  int len = 0;

  len += snprintf(jsonHistBuffer + len, sizeof(jsonHistBuffer) - len, "{\"vib\":[");

  if(xSemaphoreTake(historicoMutex, pdMS_TO_TICKS(500))){
    for(int i = 0; i < historicoCount; i++){
      int idx = (historicoInicio + i) % MAX_HISTORICO_RAM;
      len += snprintf(jsonHistBuffer + len, sizeof(jsonHistBuffer) - len,
                      "%.2f%s", historicoRAM[idx].vibMedia,
                      i < historicoCount - 1 ? "," : "");
    }

    len += snprintf(jsonHistBuffer + len, sizeof(jsonHistBuffer) - len, "],\"temp\":[");

    for(int i = 0; i < historicoCount; i++){
      int idx = (historicoInicio + i) % MAX_HISTORICO_RAM;
      len += snprintf(jsonHistBuffer + len, sizeof(jsonHistBuffer) - len,
                      "%.2f%s", historicoRAM[idx].tempMedia,
                      i < historicoCount - 1 ? "," : "");
    }

    len += snprintf(jsonHistBuffer + len, sizeof(jsonHistBuffer) - len, "]}");
    xSemaphoreGive(historicoMutex);
  }

  tempoTemperatura = micros() - t0;
  return String(jsonHistBuffer);
}


static char jsonLogsBuffer[12288];

String gerarJSONLogs(){
  memset(jsonLogsBuffer, 0, sizeof(jsonLogsBuffer));
  int len = 0;

  len += snprintf(jsonLogsBuffer + len, sizeof(jsonLogsBuffer) - len, "[");

  if(xSemaphoreTake(logMutex, pdMS_TO_TICKS(500))){
    for(int i = 0; i < logCount; i++){
      int idx = (logInicio + i) % MAX_LOGS;
      char nivel[2] = { bufferLog[idx].nivel, '\0' };
      len += snprintf(jsonLogsBuffer + len, sizeof(jsonLogsBuffer) - len,
        "{\"t\":%lu,\"n\":\"%s\",\"m\":\"%s\"}%s",
        bufferLog[idx].timestamp,
        nivel,
        bufferLog[idx].msg,
        i < logCount - 1 ? "," : "");
    }
    xSemaphoreGive(logMutex);
  }

  len += snprintf(jsonLogsBuffer + len, sizeof(jsonLogsBuffer) - len, "]");
  return String(jsonLogsBuffer);
}

static char jsonPerfBuffer[2048];

String gerarJSONPerformance(){
  memset(jsonPerfBuffer, 0, sizeof(jsonPerfBuffer));

  // heap
  uint32_t heapLivre  = ESP.getFreeHeap();
  uint32_t heapTotal  = ESP.getHeapSize();
  uint32_t heapUsado  = heapTotal - heapLivre;

  // flash
  uint32_t flashTotal = ESP.getFlashChipSize();
  uint32_t sketchUsado = ESP.getSketchSize();

  // uptime
  unsigned long uptime = millis() / 1000;

  // wifi rssi
  int rssi = (wifiState == CONNECTED) ? WiFi.RSSI() : 0;

  snprintf(jsonPerfBuffer, sizeof(jsonPerfBuffer),
    "{"
    "\"heapLivre\":%lu,"
    "\"heapTotal\":%lu,"
    "\"heapUsado\":%lu,"
    "\"flashTotal\":%lu,"
    "\"sketchUsado\":%lu,"
    "\"uptime\":%lu,"
    "\"rssi\":%d,"
    "\"wifiState\":\"%s\","
    "\"tempoFFT\":%lu,"
    "\"tempoArmazenamento\":%lu,"
    "\"tempoJSON\":%lu,"
    "\"tempoTemperatura\":%lu,"
    "\"tempoWifi\":%lu"
    "}",
    heapLivre, heapTotal, heapUsado,
    flashTotal, sketchUsado,
    uptime, rssi,
    wifiState == CONNECTED ? "Conectado" : wifiState == CONNECTING ? "Conectando" : "AP",
    tempoFFT, tempoArmazenamento, tempoJSON,
    tempoTemperatura, tempoWifi
  );

  return String(jsonPerfBuffer);
}

void carregarHistorico(){
  if(xSemaphoreTake(fsMutex, pdMS_TO_TICKS(500))){
    File f = LittleFS.open("/dados.csv", "r");
    if(!f){
      Serial.println("Sem historico salvo.");
      xSemaphoreGive(fsMutex);
      return;
    }

    historicoCount  = 0;
    historicoInicio = 0;

    while(f.available() && historicoCount < MAX_HISTORICO_RAM){
      String linha = f.readStringUntil('\n');
      linha.trim();

      // ignora linhas de comentário e cabeçalho
      if(linha.startsWith("#") || linha.startsWith("T")) continue;
      if(linha.length() == 0) continue;

      // parse: timestamp,vibMedia,tempMedia
      int c1 = linha.indexOf(',');
      int c2 = linha.indexOf(',', c1 + 1);
      if(c1 < 0 || c2 < 0) continue;

      DadoHistorico h;
      h.vibMedia  = linha.substring(0, c1).toFloat();
      h.tempMedia = linha.substring(c1 + 1, c2).toFloat();
      h.timestamp = strtoul(linha.substring(c2 + 1).c_str(), NULL, 10);

      historicoRAM[historicoCount] = h;
      historicoCount++;
    }

    f.close();
    xSemaphoreGive(fsMutex);

    Serial.printf("Historico carregado: %d registros\n", historicoCount);
    char logMsg[MAX_LOG_MSG];
    snprintf(logMsg, sizeof(logMsg), "Historico carregado: %d registros.", historicoCount);
    registrarLog('I', logMsg);
  }
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

// catodo comum: 0 = apagado, 255 = máximo brilho
void setLED(int pinR, int pinG, int pinB, int r, int g, int b){
  analogWrite(pinR, 255 - r);
  analogWrite(pinG, 255 - g);
  analogWrite(pinB, 255 - b);
}

void atualizarLEDWifi(){
  static unsigned long lastBlink = 0;
  static bool estado = false;

  switch(wifiState){
    case AP_MODE:
      // amarelo fixo
      setLED(LED_WIFI_R, LED_WIFI_G, LED_WIFI_B, 255, 180, 0);
      break;
    case CONNECTING:
      // azul piscando
      if(millis() - lastBlink > 300){
        estado = !estado;
        lastBlink = millis();
      }
      setLED(LED_WIFI_R, LED_WIFI_G, LED_WIFI_B,
             0, 0, estado ? 255 : 0);
      break;
    case CONNECTED:
      // verde fixo
      setLED(LED_WIFI_R, LED_WIFI_G, LED_WIFI_B, 0, 255, 0);
      break;
  }
}

void atualizarLEDVibracao(float vRMS){
  ConfigMaquina cfg;
  if(xSemaphoreTake(configMutex, pdMS_TO_TICKS(50))){
    cfg = configAtual;
    xSemaphoreGive(configMutex);
  } else {
    return;
  }
  if(vRMS < cfg.limAB){
    // verde
    setLED(LED_VIB_R, LED_VIB_G, LED_VIB_B, 0, 200, 0);
  } else if(vRMS < cfg.limBC){
    // amarelo
    setLED(LED_VIB_R, LED_VIB_G, LED_VIB_B, 200, 200, 0);
  } else if(vRMS < cfg.limCD){
    // laranja
    setLED(LED_VIB_R, LED_VIB_G, LED_VIB_B, 220, 100, 0);
  } else {
    // vermelho
    setLED(LED_VIB_R, LED_VIB_G, LED_VIB_B, 220, 0, 0);
  }
}

void atualizarLCD(){
  if(!lcdDisponivel) return;
  static unsigned long ultimoLCD = 0;
  if(millis() - ultimoLCD < 5000) return; // atualiza a cada 5s
  ultimoLCD = millis();

  lcd.clear();
  lcd.setCursor(0, 0);

  switch(wifiState){
    case AP_MODE:
      lcd.print("AP: 192.168.4.1");
      lcd.setCursor(0, 1);
      lcd.print("Rede: ESP-CONFIG");
      break;
    case CONNECTING:
      lcd.print("Conectando...");
      lcd.setCursor(0, 1);
      lcd.print(ssidSalvo.substring(0, 16));
      break;
    case CONNECTED:
      lcd.print("IP:");
      lcd.print(WiFi.localIP().toString());
      lcd.setCursor(0, 1);
      lcd.print("WiFi: OK");
      break;
  }
}

// ===== TASKS =====
void taskComunicacao(void *pvParameters){
  while(true){
    gerenciarWiFi();
    atualizarLED();
    atualizarLEDWifi();

    // vibração atual para o LED
    DadoSensor dadoAtual = {0.0f, 0.0f, 0};
    xQueuePeek(filaLeitura, &dadoAtual, 0);
    atualizarLEDVibracao(dadoAtual.vibracao);

    atualizarLCD();
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

void inserirHistoricoRAM(DadoHistorico &h){
  if(xSemaphoreTake(historicoMutex, pdMS_TO_TICKS(500))){
    int idx = (historicoInicio + historicoCount) % MAX_HISTORICO_RAM;
    historicoRAM[idx] = h;

    if(historicoCount < MAX_HISTORICO_RAM){
      historicoCount++;
    } else {
      // buffer cheio — avança o inicio (descarta o mais antigo)
      historicoInicio = (historicoInicio + 1) % MAX_HISTORICO_RAM;
    }
    xSemaphoreGive(historicoMutex);
  }
}

void taskArmazenamento(void *pvParameters){
  DadoSensor dado;
  float acumVib  = 0.0f;
  float acumTemp = 0.0f;
  int   contagem = 0;
  unsigned long ultimoSalvo = millis();

  while(true){
    if(xQueueReceive(filaProcessamento, &dado, pdMS_TO_TICKS(1000))){
      acumVib  += dado.vibracao;
      acumTemp += dado.temperatura;
      contagem++;
    }

    if(millis() - ultimoSalvo >= 300000 && contagem > 0){
      unsigned long t0 = micros();
      DadoHistorico media;
      media.vibMedia  = acumVib  / contagem;
      media.tempMedia = acumTemp / contagem;
      media.timestamp = millis();

      // salva na flash
      salvarDado(media);

      // salva no array circular em RAM
      inserirHistoricoRAM(media);
      tempoArmazenamento = micros() - t0;

      acumVib  = 0.0f;
      acumTemp = 0.0f;
      contagem = 0;
      ultimoSalvo = millis();
    }

    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}


// ===== SETUP =====
void setup(){
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\n=== BOOT ===");
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_WIFI_R, OUTPUT); pinMode(LED_WIFI_G, OUTPUT); pinMode(LED_WIFI_B, OUTPUT);
  pinMode(LED_VIB_R,  OUTPUT); pinMode(LED_VIB_G,  OUTPUT); pinMode(LED_VIB_B,  OUTPUT);

  // anodo comum: garante LEDs apagados no boot (LOW acenderia)
  setLED(LED_WIFI_R, LED_WIFI_G, LED_WIFI_B, 0, 0, 0);
  setLED(LED_VIB_R,  LED_VIB_G,  LED_VIB_B,  0, 0, 0);

  // recursos compartilhados
  filaProcessamento = xQueueCreate(20, sizeof(DadoSensor));
  filaLeitura       = xQueueCreate(1,  sizeof(DadoSensor));
  fsMutex           = xSemaphoreCreateMutex();
  historicoMutex    = xSemaphoreCreateMutex();
  logMutex          = xSemaphoreCreateMutex();
  configMutex       = xSemaphoreCreateMutex();

  Wire.begin(21, 22, 400000); // SDA, SCL, 400kHz Fast Mode
  Wire.beginTransmission(0x27);
  bool lcdPresente = (Wire.endTransmission() == 0);
  if(lcdPresente){
    lcd.init();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("Iniciando...");
    registrarLog('I', "LCD encontrado e iniciado.");
  } else {
    registrarLog('W', "LCD nao encontrado. Continuando sem display.");
  }
  lcdDisponivel = lcdPresente;

  if(!LittleFS.begin(true)){
    Serial.println("ERRO: LittleFS falhou");
  } else {
    Serial.println("LittleFS OK");
    carregarHistorico();
    carregarConfigMaquina();

    // carrega frequência salva
    if(xSemaphoreTake(fsMutex, pdMS_TO_TICKS(500))){
      File ff = LittleFS.open("/freq.txt", "r");
      if(ff){
        int f = ff.readStringUntil('\n').toInt();
        if(f >= 10 && f <= 2000){
          samplingFrequency = f;
          timerAlarm(timer, 1000000 / f, true, 0);
          char logMsg[MAX_LOG_MSG];
          snprintf(logMsg, sizeof(logMsg), "Frequencia carregada: %dHz.", f);
          registrarLog('I', logMsg);
        }
        ff.close();
      }
      xSemaphoreGive(fsMutex);
    }

    // estima tempo desligado pelo último timestamp do histórico
    if(historicoCount > 0){
      int ultimoIdx = (historicoInicio + historicoCount - 1) % MAX_HISTORICO_RAM;
      unsigned long ultimoTs = historicoRAM[ultimoIdx].timestamp;
      unsigned long tempoDesligado = ultimoTs / 1000; // converte ms para segundos aproximado
      char logMsg[MAX_LOG_MSG];
      snprintf(logMsg, sizeof(logMsg),
        "REINICIO detectado. Ultimo registro: %lus atras (estimado).",
        tempoDesligado);
      registrarLog('W', logMsg);
    } else {
      registrarLog('I', "Primeira inicializacao. Sem historico anterior.");
    }
  }

  sensors.begin();
  sensors.setWaitForConversion(false);

  if(!accel.begin()){
    Serial.println("ERRO: ADXL345 nao encontrado");
    while(true){ vTaskDelay(100 / portTICK_PERIOD_MS); }
  }
  Serial.println("ADXL345 OK");
  registrarLog('I', "Sistema iniciado. ADXL345 OK.");

// timer de interrupção
  timer = timerBegin(1000000);                    
  timerAttachInterrupt(timer, &onTimer);      
  timerAlarm(timer, 1000000 / samplingFrequency, true, 0); 

  carregarWiFi();

  if(ssidSalvo != ""){
    iniciarSTA();
  } else {
    iniciarAP();
  }

  server.on("/data", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "application/json", gerarJSON());
  });
  server.on("/historico", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "application/json", gerarJSONHistorico());
  });
  server.on("/csv", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "text/csv", gerarCSV());
  });
  server.on("/salvar", HTTP_GET, handleSalvar);
  server.on("/performance", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "application/json", gerarJSONPerformance());
  });
  server.on("/logs", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(200, "application/json", gerarJSONLogs());
  });
  server.on("/logs/limpar", HTTP_GET, [](AsyncWebServerRequest *request){
    if(xSemaphoreTake(logMutex, pdMS_TO_TICKS(500))){
      logCount  = 0;
      logInicio = 0;
      xSemaphoreGive(logMutex);
    }
    request->send(200, "text/plain", "OK");
  });
  server.on("/config", HTTP_GET, [](AsyncWebServerRequest *request){
    if(request->hasParam("freq")){
      int f = request->getParam("freq")->value().toInt();
      if(f >= 10 && f <= 2000){
        samplingFrequency = f;
        timerAlarm(timer, 1000000 / f, true, 0);

        // salva na flash
        if(xSemaphoreTake(fsMutex, pdMS_TO_TICKS(500))){
          File ff = LittleFS.open("/freq.txt", "w");
          if(ff){ ff.println(f); ff.close(); }
          xSemaphoreGive(fsMutex);
        }

        char logMsg[MAX_LOG_MSG];
        snprintf(logMsg, sizeof(logMsg), "Frequencia alterada para %dHz.", f);
        registrarLog('I', logMsg);
        request->send(200, "text/plain", "OK");
      } else {
        request->send(400, "text/plain", "Frequencia invalida (10-2000Hz)");
      }
    } else {
      char buf[64];
      snprintf(buf, sizeof(buf), "{\"freq\":%d}", samplingFrequency);
      request->send(200, "application/json", buf);
    }
  });
  server.on("/maquina", HTTP_GET, [](AsyncWebServerRequest *request){
    if(request->hasParam("grupo")){
      int g = request->getParam("grupo")->value().toInt();
      if(g >= 1 && g <= 4){
        if(xSemaphoreTake(configMutex, pdMS_TO_TICKS(500))){
          configAtual = GRUPOS_ISO[g - 1];
          xSemaphoreGive(configMutex);
        }
        salvarConfigMaquina(g);
        char logMsg[MAX_LOG_MSG];
        snprintf(logMsg, sizeof(logMsg),
          "Grupo de maquina alterado para: %d (A/B=%.2f B/C=%.2f C/D=%.2f)",
          g, configAtual.limAB, configAtual.limBC, configAtual.limCD);
        registrarLog('I', logMsg);
        request->send(200, "text/plain", "OK");
      } else {
        request->send(400, "text/plain", "Grupo invalido (1-4)");
      }
    } else {
      char buf[128];
      snprintf(buf, sizeof(buf),
        "{\"grupo\":%d,\"limAB\":%.2f,\"limBC\":%.2f,\"limCD\":%.2f}",
        configAtual.grupo,
        configAtual.limAB, configAtual.limBC, configAtual.limCD);
      request->send(200, "application/json", buf);
    }
  });
  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
  server.begin();

  Serial.println("=== SISTEMA INICIADO ===");

  xTaskCreatePinnedToCore(taskComunicacao,   "Comunicacao",   4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(taskSensores, "Sensores", 8192, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(taskArmazenamento, "Armazenamento", 4096, NULL, 1, NULL, 1);
}

// ===== LOOP =====
void loop(){}
