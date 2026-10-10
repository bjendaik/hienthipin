#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Update.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <DNSServer.h>
#include <NimBLEDevice.h>

#if defined(CONFIG_IDF_TARGET_ESP32S3) || \
    defined(CONFIG_IDF_TARGET_ESP32S2) || \
    defined(CONFIG_IDF_TARGET_ESP32C3) || \
    defined(CONFIG_IDF_TARGET_ESP32C6) || \
    defined(CONFIG_IDF_TARGET_ESP32H2)
  #define HAS_DAC 0
  #define CHIP_NAME "ESP32-S3/S2/C3/C6/H2"
#else
  #define HAS_DAC 1
  #define CHIP_NAME "ESP32 Classic"
#endif

#define LOG_LEVEL 1
#define LOG_INFO(...)  do { if (LOG_LEVEL >= 1) MySerial.printf(__VA_ARGS__); } while(0)
#define LOG_DEBUG(...) do { if (LOG_LEVEL >= 2) MySerial.printf(__VA_ARGS__); } while(0)

#define FW_VERSION      "3.0"
#define PRESET_URL      "https://raw.githubusercontent.com/bjendaik/hienthipin/refs/heads/main/data.json"
#define UPDATE_JSON_URL "https://raw.githubusercontent.com/bjendaik/hienthipin/refs/heads/main/update.json"

#define LED_PIN        2
#define LED_PWM_FREQ   5000
#define LED_PWM_RES    8
enum LedMode { LED_MODE_CONFIG_BLINK, LED_MODE_SOLID, LED_MODE_BREATHE };
LedMode currentLedMode = LED_MODE_SOLID;

#define OUT_MODE_DAC  0
#define OUT_MODE_PWM  1
int outMode = OUT_MODE_DAC;
int dacPin = 25;
int pwmPin = 14;
int pwmFreq = 5000;
int pwmRes = 10;

#define AP_TIMEOUT_MS     60000UL
#define AP_CHECK_INTERVAL 1000UL
bool apActive = false;
unsigned long apLastClientTime = 0;
unsigned long apStartedTime = 0;
uint8_t apLastClientCount = 0;
unsigned long apLastCheck = 0;
unsigned long lastHttpRequestTime = 0;
#define TOUCH_HTTP() do { lastHttpRequestTime = millis(); } while(0)
#define HTTP_KEEPALIVE_MS 30000UL
volatile bool otaInProgress = false;

DNSServer dnsServer;
const byte DNS_PORT = 53;
bool dnsServerStarted = false;

#define STA_TIMEOUT_MS  15000UL
bool          staConnected  = false;
bool          staEnabled    = false;
String        staSSID       = "";
String        staPass       = "";
bool          staAttempting = false;
unsigned long staAttemptStart = 0;

String fwDate  = "";
String fwNotes = "";
bool autoUpdateEnabled    = false;
bool autoUpdateDone       = false;
bool autoUpdateInProgress = false;
unsigned long lastAutoCheck = 0;

bool   needAutoConnect = false;
String bmsDeviceName   = "";
String pendingScanName = "";

String remoteVersion = "";
String remoteFwUrl   = "";
String remoteNotes   = "";
String remoteDate    = "";
bool   remoteChecked = false;
bool   remoteNewer   = false;

volatile unsigned long loopCounter = 0;
float cpuLoadAvg = 0;
float cpuLoad0   = 0;
float cpuLoad1   = 0;

#define SERIAL_BUF_SIZE 4096
class MySerialClass : public Print {
public:
    char buf[SERIAL_BUF_SIZE];
    volatile size_t head = 0;
    volatile size_t tail = 0;
    SemaphoreHandle_t mtx = nullptr;
    void begin(unsigned long baud) { Serial.begin(baud); mtx = xSemaphoreCreateMutex(); }
    void lock()   { if (mtx) xSemaphoreTake(mtx, portMAX_DELAY); }
    void unlock() { if (mtx) xSemaphoreGive(mtx); }
    size_t write(uint8_t c) override {
        lock(); Serial.write(c);
        size_t next = (head + 1) % SERIAL_BUF_SIZE;
        if (next != tail) { buf[head] = c; head = next; }
        else { tail = (tail + 1) % SERIAL_BUF_SIZE; buf[head] = c; head = next; }
        unlock(); return 1;
    }
    size_t write(const uint8_t* data, size_t len) override {
        lock(); Serial.write(data, len);
        for (size_t i = 0; i < len; i++) {
            size_t next = (head + 1) % SERIAL_BUF_SIZE;
            if (next != tail) { buf[head] = data[i]; head = next; }
            else { tail = (tail + 1) % SERIAL_BUF_SIZE; buf[head] = data[i]; head = next; }
        }
        unlock(); return len;
    }
    String readAll() {
        String out = ""; lock();
        while (tail != head) { out += buf[tail]; tail = (tail + 1) % SERIAL_BUF_SIZE; }
        unlock(); return out;
    }
};
MySerialClass MySerial;

TaskHandle_t bleTaskHandle = nullptr;
SemaphoreHandle_t stateMutex = nullptr;

bool bleAutoScan = true;
bool bleEnabled  = true;
volatile bool bleShuttingDown = false;

float minVoltage = 2.07f;
float maxVoltage = 2.35f;

uint8_t currentSoc = 0;
int currentOut = 0;
bool manualSimMode = false;
uint8_t realBmsSoc = 0;
uint8_t savedSoc = 0;
unsigned long bmsSocTimestamp = 0;

float bmsTotalVoltage = 0.0f;
float bmsCells[16] = {0};
float bmsCurrent = 0.0f;
float bmsPower   = 0.0f;
uint8_t bmsCellCount = 0;
bool bmsHasCurrent = false;
bool bmsHasPower   = false;
bool bmsHasCells   = false;
bool bmsHasTotalV  = false;
String bmsRawString = "Đang chờ dữ liệu...";

WebServer server(80);
Preferences preferences;
String targetMac = "";

String cachedJsonData = "";
unsigned long cachedJsonTime = 0;
#define JSON_CACHE_MS 800
unsigned long lastHexUpdateMs = 0;
volatile bool httpBusy = false;

#define BATT_TYPE_LEAD  0
#define BATT_TYPE_LFP   1
int batteryType = BATT_TYPE_LEAD;
int batteryCells = 4;

#define LFP_V_EMPTY   3.00f
#define LFP_V_FULL    3.45f
#define LFP_V_MAX     3.65f
#define LFP_V_NOM     3.20f
#define LEAD_V_EMPTY  10.5f
#define LEAD_V_FULL   13.8f
#define LEAD_V_MAX    15.0f
#define LEAD_V_NOM    12.0f

bool  acquyMode = false;
int   acquyAdcPin = 32;
float acquyR1 = 100.0f;
float acquyR2 = 4.7f;
float acquyCalibGain   = 1.0f;
float acquyCalibOffset = 0.0f;
float acquyMeasuredV = 0.0f;
float acquyAdcRawV = 0.0f;
uint8_t acquySoc = 0;
bool  acquyOverVoltage = false;
unsigned long lastAdcRead = 0;
float acquyRawV     = 0.0f;
float acquyStableV = 0.0f;
uint8_t acquySocRaw    = 0;
uint8_t acquySocStable = 0;

#define ACQY_MEDIAN_SIZE     5
#define ACQY_EMA_ALPHA       0.25f
#define ACQY_STABLE_DELTA    0.30f
#define ACQY_STABLE_TIME_MS  2500UL
#define ACQY_OUTPUT_DELAY_MS 800UL
#define ACQY_OUTPUT_HYST     1
float acquyMedianBuf[ACQY_MEDIAN_SIZE] = {0};
uint8_t acquyMedianIdx = 0;
bool    acquyMedianFilled = false;
float acquyEmaV = 0.0f;
bool    acquyEmaInit = false;
float acquyLastStableV = 0.0f;
unsigned long acquyStableStart = 0;
bool    acquyStable = false;
unsigned long acquyLastOutputChange = 0;

#define MAX_VMAP_ENTRIES 20
struct VMapEntry { uint8_t startPct; uint8_t endPct; float voltage; };
VMapEntry vmap[MAX_VMAP_ENTRIES];
uint8_t vmapCount = 0;
bool    vmapEnabled = false;

#define MAX_REMOTE_PRESETS 30
#define MAX_ENTRIES_PER_PRESET 20
struct RemotePreset {
    String name; String subtitle; String ranges;
    uint8_t count; VMapEntry entries[MAX_ENTRIES_PER_PRESET];
};
RemotePreset remotePresets[MAX_REMOTE_PRESETS];
int remotePresetCount = 0;
String presetLastUpdated = "";
bool   presetInfoLoaded = false;
unsigned long lastPresetSync = 0;
String activePresetName = "";

NimBLEClient* pClient = nullptr;
NimBLERemoteCharacteristic* pChar = nullptr;
NimBLERemoteCharacteristic* pWriteChar = nullptr;
volatile bool connected = false;
volatile bool doConnect = false;
volatile bool doDisconnect = false;
volatile bool bleInitialized = false;
volatile bool isScanning = false;
volatile bool doScan = false;
volatile bool scanReady = false;
String scanResultHtml = "";

NimBLEUUID serviceUUID("0000FFE0-0000-1000-8000-00805F9B34FB");
NimBLEUUID charUUID("0000FFE1-0000-1000-8000-00805F9B34FB");

uint8_t loginCmd[] = { 0xA5, 0x0B, 0x00, 0x58, 0x58, 0x19, 0x0A, 0x1E, 0x0E, 0x28, 0x0D };
uint8_t requestSocCmd[] = { 0xA5, 0x40, 0x90, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7D };
uint8_t jkDeviceInfoCmd[] = { 0xAA,0x55,0x90,0xEB,0x96,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x10 };
uint8_t jkCellInfoNewCmd[] = { 0xAA,0x55,0x90,0xEB,0x96,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x11 };
uint8_t jkCellInfoOldCmd[] = { 0xAA,0x55,0x90,0xEB,0x97,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x11 };

volatile uint8_t bmsProtocol = 255;
volatile bool    probeSent = false;
unsigned long    probeStartMs = 0;
volatile uint8_t jkVersion = 0;
volatile bool    jkVersionProbed = false;
unsigned long    jkDetectTime = 0;

#define JK_FRAME_SIZE 350
uint8_t jkFrameBuf[JK_FRAME_SIZE];
size_t  jkFrameLen = 0;
unsigned long jkFrameStartMs = 0;

inline void lockState()   { if (stateMutex) xSemaphoreTake(stateMutex, portMAX_DELAY); }
inline void unlockState() { if (stateMutex) xSemaphoreGive(stateMutex); }

String parseHttpDate(const String& s);
bool fetchPresetInfo(const String& url);
bool fetchPresetsFromUrl(const String& url);
bool fetchUpdateInfo();
void bleInit();
void bleDeinit();
void autoUpdateTask();
void applyOutput(uint8_t soc);
void apStart();
bool parseJkFrame(uint8_t* data, size_t len);
uint8_t detectJkVersion(const char* model, size_t len);

void getBatteryLimits(float &vEmpty, float &vFull, float &vMaxWarn) {
    if (batteryType == BATT_TYPE_LFP) {
        vEmpty = LFP_V_EMPTY * batteryCells;
        vFull  = LFP_V_FULL  * batteryCells;
        vMaxWarn = LFP_V_MAX * batteryCells;
    } else {
        vEmpty = LEAD_V_EMPTY * batteryCells;
        vFull  = LEAD_V_FULL  * batteryCells;
        vMaxWarn = LEAD_V_MAX * batteryCells;
    }
}

float calcAdcMaxVoltage() {
    if (acquyR2 < 0.01f) return 999.0f;
    float ratio = acquyR2 / (acquyR1 + acquyR2);
    float vFull, vEmpty, vMaxWarn;
    getBatteryLimits(vEmpty, vFull, vMaxWarn);
    return vMaxWarn * ratio;
}

int calcSocFromV(float vBat) {
    float vEmpty, vFull, vMaxWarn;
    getBatteryLimits(vEmpty, vFull, vMaxWarn);
    if (batteryType == BATT_TYPE_LEAD) {
        int soc = (int)round(((vBat - vEmpty) / (vFull - vEmpty)) * 100.0f);
        return constrain(soc, 0, 100);
    } else {
        float vMid = vEmpty + (vFull - vEmpty) * 0.55f;
        if (vBat <= vEmpty) return 0;
        if (vBat >= vFull)  return 100;
        int soc;
        if (vBat < vMid) { float t = (vBat - vEmpty) / (vMid - vEmpty); soc = (int)round(t * 50.0f); }
        else { float t = (vBat - vMid) / (vFull - vMid); soc = 50 + (int)round(t * 50.0f); }
        return constrain(soc, 0, 100);
    }
}

void ledSetup() { ledcDetach(LED_PIN); pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, HIGH); }

void ledSetMode(LedMode mode) {
    currentLedMode = mode;
    ledcDetach(LED_PIN);
    pinMode(LED_PIN, OUTPUT);
    if (mode == LED_MODE_SOLID) digitalWrite(LED_PIN, HIGH);
    else if (mode == LED_MODE_CONFIG_BLINK) digitalWrite(LED_PIN, LOW);
    else { ledcAttach(LED_PIN, LED_PWM_FREQ, LED_PWM_RES); ledcWrite(LED_PIN, 20); }
}

void ledUpdate() {
    static unsigned long lastUpdate = 0;
    if (millis() - lastUpdate < 50) return;
    lastUpdate = millis();
    static unsigned long lastToggle = 0;
    static bool toggleState = false;
    static int breatheVal = 20;
    static int breatheDir = 1;
    switch (currentLedMode) {
        case LED_MODE_CONFIG_BLINK:
            if (millis() - lastToggle >= 300) { lastToggle = millis(); toggleState = !toggleState; digitalWrite(LED_PIN, toggleState ? HIGH : LOW); }
            break;
        case LED_MODE_SOLID: digitalWrite(LED_PIN, HIGH); break;
        case LED_MODE_BREATHE:
            breatheVal += breatheDir * 15;
            if (breatheVal >= 255) { breatheVal = 255; breatheDir = -1; }
            if (breatheVal <= 10)  { breatheVal = 10;  breatheDir = 1; }
            ledcWrite(LED_PIN, breatheVal);
            break;
    }
}

void measureCpuLoad() {
    unsigned long now = millis();
    static unsigned long lastCheck = 0;
    static unsigned long lastLoop = 0;
    static float loadSmooth = 0;
    if (now - lastCheck < 1000) return;
    unsigned long elapsed = now - lastCheck;
    unsigned long loops = loopCounter - lastLoop;
    lastCheck = now; lastLoop = loopCounter;
    float maxLoops = elapsed / 5.0f;
    if (maxLoops < 1) maxLoops = 1;
    float load = 100.0f - (loops / maxLoops * 100.0f);
    if (load < 0) load = 0; if (load > 100) load = 100;
    loadSmooth = loadSmooth * 0.5f + load * 0.5f;
    cpuLoadAvg = loadSmooth; cpuLoad0 = cpuLoadAvg; cpuLoad1 = cpuLoadAvg;
}

void staStart() {
    if (staSSID == "") return;
    staAttempting = true;
    staAttemptStart = millis();
    LOG_INFO("📶 STA: kết nối '%s'... (tối đa %lus)\n", staSSID.c_str(), STA_TIMEOUT_MS / 1000);
    if (apActive) WiFi.mode(WIFI_AP_STA); else WiFi.mode(WIFI_STA);
    WiFi.begin(staSSID.c_str(), staPass.c_str());
}

void staStop() {
    WiFi.disconnect(false, false);
    staConnected = false; staAttempting = false;
    if (apActive) WiFi.mode(WIFI_AP_STA); else WiFi.mode(WIFI_OFF);
    LOG_INFO("📶 STA: đã ngắt\n");
}

void staCheck() {
    if (!staEnabled || staSSID == "") return;
    wl_status_t st = WiFi.status();
    if (st == WL_CONNECTED) {
        if (!staConnected) { staConnected = true; staAttempting = false; LOG_INFO("✅ STA kết nối! IP: %s\n", WiFi.localIP().toString().c_str()); }
        return;
    }
    if (staConnected) {
        staConnected = false; staAttempting = false;
        LOG_INFO("⚠️ STA mất kết nối\n");
        if (!apActive) { LOG_INFO("🔁 STA mất → bật lại AP\n"); apStart(); ledSetMode(LED_MODE_CONFIG_BLINK); }
        return;
    }
    if (staAttempting) {
        if (millis() - staAttemptStart > STA_TIMEOUT_MS) {
            staAttempting = false; staEnabled = false; staConnected = false;
            LOG_INFO("⛔ STA: hết %lus → TẮT STA\n", STA_TIMEOUT_MS / 1000);
            preferences.begin("bms_config", false);
            preferences.putBool("sta_en", false);
            preferences.end();
            if (apActive) WiFi.mode(WIFI_AP); else { WiFi.disconnect(false, false); WiFi.mode(WIFI_OFF); }
        }
        return;
    }
}

void apStart() {
    if (apActive) return;
    if (staEnabled && staSSID != "") WiFi.mode(WIFI_AP_STA); else WiFi.mode(WIFI_AP);
    WiFi.setSleep(false);
    WiFi.softAP("BMS_Config", "12345678", 1);
    WiFi.setTxPower(WIFI_POWER_19_5dBm);
    apActive = true;
    apStartedTime = millis();
    apLastClientTime = 0; apLastClientCount = 0; apLastCheck = millis();
    LOG_INFO("📡 AP bật: BMS_Config/12345678 | IP: %s\n", WiFi.softAPIP().toString().c_str());
    if (!dnsServerStarted) { dnsServer.start(DNS_PORT, "*", WiFi.softAPIP()); dnsServerStarted = true; }
}

void apStop() {
    if (!apActive) return;
    if (!staConnected) { LOG_INFO("🛡️ Bỏ qua tắt AP (STA chưa kết nối)\n"); apLastClientTime = millis(); return; }
    LOG_INFO("📴 AP tắt (STA có IP: %s)\n", WiFi.localIP().toString().c_str());
    if (dnsServerStarted) { dnsServer.stop(); dnsServerStarted = false; }
    WiFi.softAPdisconnect(true); delay(50);
    WiFi.mode(WIFI_STA); delay(50);
    apActive = false;
    if (!acquyMode && connected) ledSetMode(LED_MODE_BREATHE); else ledSetMode(LED_MODE_SOLID);
}

void apWatchdog() {
    if (!apActive) return;
    if (otaInProgress) return;
    if (millis() - apLastCheck < AP_CHECK_INTERVAL) return;
    apLastCheck = millis();
    uint8_t cc = WiFi.softAPgetStationNum();
    if (cc > 0) { if (apLastClientCount == 0) LOG_INFO("👤 Client AP: %u\n", cc); apLastClientTime = millis(); apLastClientCount = cc; return; }
    if (lastHttpRequestTime > 0) {
        if (millis() - lastHttpRequestTime < HTTP_KEEPALIVE_MS) { if (apLastClientCount == 0) LOG_INFO("👤 Client qua STA\n"); apLastClientTime = millis(); apLastClientCount = 1; return; }
    }
    if (apLastClientCount > 0) { LOG_INFO("👋 Client cuối ngắt → đếm %lus\n", AP_TIMEOUT_MS / 1000); apLastClientCount = 0; apLastClientTime = millis(); return; }
    if (!staConnected) {
        unsigned long ref = (apLastClientTime > 0) ? apLastClientTime : apStartedTime;
        if (millis() - ref >= AP_TIMEOUT_MS) {
            static unsigned long lastWarn = 0;
            if (millis() - lastWarn > 60000) { lastWarn = millis(); LOG_INFO("🛡️ AP vẫn bật (STA chưa kết nối)\n"); }
            apLastClientTime = millis();
        }
        return;
    }
    unsigned long ref = (apLastClientTime > 0) ? apLastClientTime : apStartedTime;
    if (millis() - ref >= AP_TIMEOUT_MS) apStop();
}

float getMappedVoltage(uint8_t soc) {
    if (!vmapEnabled || vmapCount == 0) return -1.0f;
    for (uint8_t i = 0; i < vmapCount; i++) {
        if (soc >= vmap[i].startPct && soc <= vmap[i].endPct) return vmap[i].voltage;
    }
    if (soc < vmap[0].startPct) return vmap[0].voltage;
    return vmap[vmapCount - 1].voltage;
}

void applyOutput(uint8_t soc) {
    soc = constrain(soc, 0, 100);
    int outValue = 0;
    float mappedV = getMappedVoltage(soc);
    if (outMode == OUT_MODE_DAC) {
#if HAS_DAC
        if (mappedV >= 0.0f) outValue = constrain((int)round((mappedV / 3.3f) * 255.0f), 0, 255);
        else { int minDac = constrain((int)round((minVoltage / 3.3f) * 255.0f), 0, 255); int maxDac = constrain((int)round((maxVoltage / 3.3f) * 255.0f), 0, 255); outValue = map(soc, 0, 100, minDac, maxDac); }
        dacWrite(dacPin, outValue);
#else
        int maxDuty = (1 << pwmRes) - 1;
        if (mappedV >= 0.0f) outValue = constrain((int)round((mappedV / 3.3f) * maxDuty), 0, maxDuty);
        else { int minD = constrain((int)round((minVoltage / 3.3f) * maxDuty), 0, maxDuty); int maxD = constrain((int)round((maxVoltage / 3.3f) * maxDuty), 0, maxDuty); outValue = map(soc, 0, 100, minD, maxD); }
        ledcWrite(pwmPin, outValue);
#endif
    } else {
        int maxDuty = (1 << pwmRes) - 1;
        if (mappedV >= 0.0f) outValue = constrain((int)round((mappedV / 3.3f) * maxDuty), 0, maxDuty);
        else { int minDuty = constrain((int)round((minVoltage / 3.3f) * maxDuty), 0, maxDuty); int maxDutyVal = constrain((int)round((maxVoltage / 3.3f) * maxDuty), 0, maxDuty); outValue = map(soc, 0, 100, minDuty, maxDutyVal); }
        ledcWrite(pwmPin, outValue);
    }
    lockState(); currentSoc = soc; currentOut = outValue; unlockState();
}

void setupPWM() { ledcAttach(pwmPin, pwmFreq, pwmRes); LOG_INFO("🔧 PWM: GPIO%d, %d Hz, %d-bit\n", pwmPin, pwmFreq, pwmRes); }
void disablePWM() { ledcWrite(pwmPin, 0); ledcDetach(pwmPin); }

float readAdcVolts(int pin) {
    uint32_t sum = 0;
    for (int i = 0; i < 32; i++) { sum += analogRead(pin); delayMicroseconds(50); }
    return ((float)sum / 32.0f / 4095.0f) * 3.3f;
}

void processAcquyReading() {
    if (!acquyMode) return;
    if (millis() - lastAdcRead < 500) return;
    lastAdcRead = millis();
    float vAdc = readAdcVolts(acquyAdcPin);
    acquyAdcRawV = vAdc;
    if (acquyR2 <= 0.01f) acquyR2 = 4.7f;
    float ratio = (acquyR1 + acquyR2) / acquyR2;
    float vBatRaw = vAdc * ratio * acquyCalibGain + acquyCalibOffset;
    acquyRawV = vBatRaw;
    acquyMedianBuf[acquyMedianIdx] = vBatRaw;
    acquyMedianIdx = (acquyMedianIdx + 1) % ACQY_MEDIAN_SIZE;
    if (acquyMedianIdx == 0) acquyMedianFilled = true;
    float sorted[ACQY_MEDIAN_SIZE];
    uint8_t nMedian = acquyMedianFilled ? ACQY_MEDIAN_SIZE : (acquyMedianIdx == 0 ? ACQY_MEDIAN_SIZE : acquyMedianIdx);
    if (nMedian < 1) nMedian = 1;
    for (uint8_t i = 0; i < nMedian; i++) sorted[i] = acquyMedianBuf[i];
    for (uint8_t i = 0; i < nMedian - 1; i++)
        for (uint8_t j = i + 1; j < nMedian; j++)
            if (sorted[j] < sorted[i]) { float t = sorted[i]; sorted[i] = sorted[j]; sorted[j] = t; }
    float vBatMedian = sorted[nMedian / 2];
    if (!acquyEmaInit) { acquyEmaV = vBatMedian; acquyEmaInit = true; }
    else acquyEmaV = acquyEmaV * (1.0f - ACQY_EMA_ALPHA) + vBatMedian * ACQY_EMA_ALPHA;
    acquyMeasuredV = acquyEmaV;
    float vEmpty, vFull, vMaxWarn;
    getBatteryLimits(vEmpty, vFull, vMaxWarn);
    acquyOverVoltage = (vBatRaw > vMaxWarn) || (vAdc >= 3.15f);
    int socRaw = calcSocFromV(vBatRaw);
    acquySocRaw = (uint8_t)socRaw;
    if (fabsf(vBatMedian - acquyLastStableV) < ACQY_STABLE_DELTA && acquyEmaInit) {
        if (acquyStableStart == 0) acquyStableStart = millis();
        if (millis() - acquyStableStart >= ACQY_STABLE_TIME_MS) { acquyStable = true; acquyLastStableV = vBatMedian; }
    } else { acquyStable = false; acquyStableStart = 0; }
    if (acquyStable) acquyStableV = vBatMedian;
    else if (acquyLastStableV > 0.1f) acquyStableV = acquyLastStableV;
    else acquyStableV = acquyEmaV;
    int socStable = calcSocFromV(acquyStableV);
    acquySocStable = (uint8_t)socStable;
    lockState();
    realBmsSoc = acquySocStable; acquySoc = acquySocStable;
    bmsTotalVoltage = acquyStableV; bmsHasTotalV = true;
    bmsSocTimestamp = millis();
    unlockState();
    if (!manualSimMode && acquyStable) {
        if (abs((int)socStable - (int)currentSoc) >= ACQY_OUTPUT_HYST) {
            if (millis() - acquyLastOutputChange > ACQY_OUTPUT_DELAY_MS) { applyOutput(socStable); acquyLastOutputChange = millis(); }
        }
    }
}

uint8_t detectJkVersion(const char* model, size_t len) {
    if (len < 3) return 1;
    char norm[32] = {0};
    size_t n = (len < 31) ? len : 31;
    for (size_t i = 0; i < n; i++) { char c = model[i]; if (c == '_') c = '-'; norm[i] = c; }
    if (strncmp(norm, "JK-PB2", 6) == 0) return 2;
    if (strncmp(norm, "JK-BD", 5) == 0)  return 2;
    if (strncmp(norm, "JK-HY", 5) == 0)  return 2;
    if (strncmp(norm, "JK-B2", 5) == 0)  return 1;
    return 1;
}

bool parseJkFrame(uint8_t* data, size_t len) {
    if (len >= 4 && data[0] == 0x55 && data[1] == 0xAA && data[2] == 0xEB && data[3] == 0x90) { jkFrameLen = 0; jkFrameStartMs = millis(); }
    if (jkFrameLen == 0 && !(len >= 4 && data[0] == 0x55 && data[1] == 0xAA)) return false;
    if (jkFrameLen > 0 && millis() - jkFrameStartMs > 1500) { LOG_INFO("⚠️ JK frame timeout (%u bytes) -> reset\n", jkFrameLen); jkFrameLen = 0; return false; }
    size_t toCopy = len;
    if (jkFrameLen + toCopy > JK_FRAME_SIZE) toCopy = JK_FRAME_SIZE - jkFrameLen;
    memcpy(jkFrameBuf + jkFrameLen, data, toCopy);
    jkFrameLen += toCopy;
    if (jkFrameLen < 260) return false;
    uint8_t* frame = jkFrameBuf;
    uint8_t frameType = frame[4];
    if (frameType == 0x03) {
        if (!jkVersionProbed) {
            char model[32] = {0};
            memcpy(model, frame + 6, 16); model[16] = 0;
            LOG_INFO("📋 JK raw model: '%s'\n", model);
            jkVersion = detectJkVersion(model, 16);
            jkVersionProbed = true;
            LOG_INFO("✅ JK protocol xác nhận: %s\n", jkVersion == 2 ? "JK02_32S" : "JK02_24S");
            preferences.begin("bms_config", false); preferences.putUChar("jk_ver", jkVersion); preferences.end();
        }
        if (jkFrameLen < 300) { jkFrameLen = 0; return false; }
    }
    if (frameType == 0x02 || frameType == 0x01) {
        size_t totalVOff  = (jkVersion == 2) ? 150 : 118;
        size_t currentOff = (jkVersion == 2) ? 158 : 126;
        size_t socOff     = (jkVersion == 2) ? 173 : 141;
        size_t minLen = socOff + 1;
        if (jkFrameLen < minLen) { jkFrameLen = 0; return false; }
        uint8_t soc = frame[socOff];
        float cells[16] = {0};
        uint8_t cellCount = 0;
        float cellSum = 0.0f;
        for (int i = 0; i < 16; i++) {
            uint16_t rawV = (uint16_t)(frame[6 + i * 2] | (frame[7 + i * 2] << 8));
            float v = rawV * 0.001f;
            if (v > 1.5f && v < 4.2f) { cells[i] = v; cellCount++; cellSum += v; }
        }
        uint32_t rawTotal = (uint32_t)frame[totalVOff] | ((uint32_t)frame[totalVOff + 1] << 8) | ((uint32_t)frame[totalVOff + 2] << 16) | ((uint32_t)frame[totalVOff + 3] << 24);
        float totalV = rawTotal * 0.001f;
        int32_t rawCur = (int32_t)frame[currentOff] | ((int32_t)frame[currentOff + 1] << 8) | ((int32_t)frame[currentOff + 2] << 16) | ((int32_t)frame[currentOff + 3] << 24);
        float current = rawCur * 0.001f;
        bool socValid = (soc <= 100);
        bool totalVValid = (totalV > 1.0f && totalV < (4.5f * 32.0f));
        bool cellsConsistent = true;
        if (cellCount >= 4 && totalVValid) { if (fabsf(cellSum - totalV) > totalV * 0.15f) cellsConsistent = false; }
        bool currentValid = (fabsf(current) < 500.0f);
        if (!socValid || !totalVValid || !cellsConsistent || !currentValid) {
            static unsigned long lastBadLog = 0;
            if (millis() - lastBadLog > 3000) { lastBadLog = millis(); LOG_INFO("⚠️ JK frame lỗi: soc=%d totV=%.2f cur=%.2f cells=%d sumCell=%.2f — BỎ QUA\n", soc, totalV, current, cellCount, cellSum); }
            jkFrameLen = 0;
            return false;
        }
        static unsigned long lastLog = 0;
        if (millis() - lastLog > 3000) {
            lastLog = millis();
            LOG_INFO("📥 JiKong[%s]: SOC=%d%% | V=%.2fV | I=%.2fA | P=%.1fW | Cells=%d\n", jkVersion == 2 ? "32S" : "24S", soc, totalV, current, totalV * current, cellCount);
        }
        lockState();
        realBmsSoc = soc;
        bmsTotalVoltage = totalV;
        bmsHasTotalV = true;
        bmsHasCurrent = true;
        bmsCurrent = current;
        bmsHasPower = true;
        bmsPower = totalV * current;
        bmsCellCount = cellCount;
        bmsHasCells = (cellCount > 0);
        for (int i = 0; i < 16; i++) bmsCells[i] = cells[i];
        bool manual = manualSimMode;
        unlockState();
        if (soc <= 100) {
            bmsSocTimestamp = millis();
            if (soc != savedSoc) { savedSoc = soc; preferences.begin("bms_config", false); preferences.putUInt("last_soc", savedSoc); preferences.end(); }
        }
        if (soc <= 100 && !manual) applyOutput(soc);
        jkFrameLen = 0;
        return true;
    }
    if (jkFrameLen >= JK_FRAME_SIZE) jkFrameLen = 0;
    return false;
}

class MyClientCallback : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient* pclient) override {
        connected = true; bmsProtocol = 255; probeSent = false; probeStartMs = 0;
        jkFrameLen = 0; jkDetectTime = 0;
        LOG_INFO(">>> BMS Đã kết nối!\n");
        if (!apActive && !acquyMode) ledSetMode(LED_MODE_BREATHE);
    }
    void onDisconnect(NimBLEClient* pclient, int reason) override {
        connected = false; pChar = nullptr; pWriteChar = nullptr;
        bmsSocTimestamp = 0; probeSent = false; probeStartMs = 0; jkFrameLen = 0; jkDetectTime = 0;
        LOG_INFO(">>> BMS Mất kết nối! Lý do: %d\n", reason);
        if (!apActive && !acquyMode) ledSetMode(LED_MODE_SOLID);
    }
};

class MyScanCallbacks : public NimBLEScanCallbacks {
    void onResult(const NimBLEAdvertisedDevice* adv) override {
        if (!bleInitialized || acquyMode) return;
        String foundMac = String(adv->getAddress().toString().c_str());
        foundMac.toUpperCase();
        String tgt;
        lockState(); tgt = targetMac; unlockState();
        if (tgt != "" && foundMac == tgt) {
            if (adv->haveName()) { lockState(); pendingScanName = String(adv->getName().c_str()); unlockState(); }
            doConnect = true;
        }
    }
};
static MyScanCallbacks scanCallbacksInstance;

void bleInit() {
    if (bleInitialized) return;
    LOG_INFO("🔔 Bật BLE\n");
    if (!NimBLEDevice::isInitialized()) NimBLEDevice::init("ESP32_BMS_Controller");
    NimBLEScan* pBLEScan = NimBLEDevice::getScan();
    pBLEScan->setScanCallbacks(&scanCallbacksInstance, false);
    pBLEScan->setActiveScan(true);
    pBLEScan->setInterval(160);
    pBLEScan->setWindow(60);
    bleInitialized = true;
    bleEnabled = true;
}

void bleDeinit() {
    if (!bleInitialized) return;
    LOG_INFO("🔕 Tắt BLE\n");
    bleShuttingDown = true;
    doScan = false; doConnect = false; doDisconnect = false;
    unsigned long t0 = millis();
    while (isScanning && millis() - t0 < 3000) delay(50);
    if (pClient && pClient->isConnected()) { pClient->disconnect(); delay(200); }
    connected = false; pChar = nullptr; pWriteChar = nullptr; pClient = nullptr;
    bleEnabled = false; bleInitialized = false;
    delay(50);
    NimBLEDevice::deinit(true);
    bleShuttingDown = false;
}

void notifyCallback(NimBLERemoteCharacteristic* c, uint8_t* data, size_t len, bool isNotify) {
    if (acquyMode) return;
    if (bmsProtocol == 255 && len >= 4) {
        if (data[0] == 0x55 && data[1] == 0xAA && data[2] == 0xEB && data[3] == 0x90) {
            bmsProtocol = 1; probeSent = true; jkDetectTime = millis();
            LOG_INFO("✅ Đã nhận diện: JiKong (JK-BMS)\n");
            preferences.begin("bms_config", false); preferences.putUChar("bms_proto", 1); preferences.end();
        } else if (data[0] == 0xA5 && data[1] == 0x7B) {
            bmsProtocol = 0; probeSent = true;
            LOG_INFO("✅ Đã nhận diện: BMS MOVE (JBD OEM)\n");
            preferences.begin("bms_config", false); preferences.putUChar("bms_proto", 0); preferences.end();
        }
    }
    NimBLERemoteCharacteristic* targetWrite = (pWriteChar != nullptr) ? pWriteChar : pChar;
    if (bmsProtocol == 255 && !probeSent && probeStartMs > 0 && millis() - probeStartMs > 2000) {
        if (targetWrite != nullptr) {
            LOG_INFO("🔍 Probe: thử JiKong...\n");
            targetWrite->writeValue(jkDeviceInfoCmd, sizeof(jkDeviceInfoCmd), false);
            probeSent = true; probeStartMs = millis();
        }
    }
    if (bmsProtocol == 255 && probeSent && probeStartMs > 0 && millis() - probeStartMs > 4000) {
        bmsProtocol = 254;
        LOG_INFO("⚠️ Không nhận diện được loại BMS\n");
    }
    if (millis() - lastHexUpdateMs > 2000) {
        lastHexUpdateMs = millis();
        static char hexBuf[600];
        int pos = snprintf(hexBuf, sizeof(hexBuf), "PKT(%u): ", (unsigned)len);
        for (int i = 0; i < (int)len && pos < (int)sizeof(hexBuf) - 4; i++)
            pos += snprintf(hexBuf + pos, sizeof(hexBuf) - pos, "%02X ", data[i]);
        lockState(); bmsRawString = String(hexBuf); unlockState();
    }
    if (bmsProtocol == 1) { parseJkFrame(data, len); return; }
    if (len >= 123 && data[0] == 0xA5 && data[1] == 0x7B) {
        uint8_t soc = data[11];
        float totalV = ((data[20] << 8) | data[19]) / 100.0f;
        bool hasCur = (len >= 25);
        float current = 0.0f;
        if (hasCur) { int16_t rawCurrent = (int16_t)((data[22] << 8) | data[21]); current = rawCurrent / 100.0f; }
        float power = totalV * current;
        float cells[16] = {0};
        uint8_t cellCount = 0;
        if (len >= 82) { for (int i = 0; i < 16; i++) { cells[i] = ((data[51 + i * 2] << 8) | data[50 + i * 2]) / 1000.0f; if (cells[i] > 0.5f) cellCount++; } }
        if (soc > 100 || totalV < 1.0f || totalV > 144.0f) return;
        static uint8_t lastLoggedSoc = 255;
        static unsigned long lastLogTime = 0;
        if (soc != lastLoggedSoc || millis() - lastLogTime > 60000) {
            lastLoggedSoc = soc; lastLogTime = millis();
            LOG_INFO("📥 BMS MOVE: SOC=%d%% | V=%.2fV | I=%.2fA | P=%.1fW | Cells=%d\n", soc, totalV, current, power, cellCount);
        }
        lockState();
        realBmsSoc = soc; bmsTotalVoltage = totalV; bmsHasTotalV = (totalV > 1.0f);
        bmsHasCurrent = hasCur; bmsCurrent = current;
        bmsHasPower = hasCur; bmsPower = power;
        bmsCellCount = cellCount; bmsHasCells = (cellCount > 0);
        if (len >= 82) for (int i = 0; i < 16; i++) bmsCells[i] = cells[i];
        bool manual = manualSimMode;
        unlockState();
        if (soc <= 100) {
            bmsSocTimestamp = millis();
            if (soc != savedSoc) { savedSoc = soc; preferences.begin("bms_config", false); preferences.putUInt("last_soc", savedSoc); preferences.end(); }
        }
        if (soc <= 100 && !manual) applyOutput(soc);
    }
}

bool connectToServer() {
    if (!bleEnabled || acquyMode) return false;
    String tgt;
    lockState(); tgt = targetMac; unlockState();
    if (tgt == "") return false;
    bmsProtocol = 255; probeSent = false; probeStartMs = 0; jkFrameLen = 0; jkDetectTime = 0;
    if (pClient == nullptr) {
        pClient = NimBLEDevice::createClient();
        pClient->setClientCallbacks(new MyClientCallback(), false);
        pClient->setConnectTimeout(5000);
    }
    if (pClient->isConnected()) { pClient->disconnect(); delay(200); }
    LOG_INFO("🔗 Kết nối tới %s...\n", tgt.c_str());
    NimBLEAddress targetAddr(std::string(tgt.c_str()), BLE_ADDR_RANDOM);
    bool connSuccess = pClient->connect(targetAddr, false);
    if (!connSuccess) {
        NimBLEAddress targetAddrPub(std::string(tgt.c_str()), BLE_ADDR_PUBLIC);
        connSuccess = pClient->connect(targetAddrPub, false);
    }
    if (!connSuccess || !pClient->isConnected()) { LOG_INFO("❌ Kết nối thất bại\n"); return false; }
    NimBLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (!pRemoteService) pRemoteService = pClient->getService("FFE0");
    if (!pRemoteService) { LOG_INFO("❌ Không tìm thấy Service BMS\n"); pClient->disconnect(); return false; }
    pChar = pRemoteService->getCharacteristic(charUUID);
    if (!pChar) pChar = pRemoteService->getCharacteristic("FFE1");
    if (!pChar) { LOG_INFO("❌ Không tìm thấy Characteristic\n"); pClient->disconnect(); return false; }
    pWriteChar = pRemoteService->getCharacteristic("FFE2");
    if (!pWriteChar || !pWriteChar->canWrite()) pWriteChar = pChar;
    if (pChar->canNotify()) pChar->subscribe(true, notifyCallback);
    delay(200);
    NimBLERemoteCharacteristic* targetWrite = (pWriteChar != nullptr) ? pWriteChar : pChar;
    LOG_INFO(">>> Bắt đầu probe protocol...\n");
    targetWrite->writeValue(loginCmd, sizeof(loginCmd), false);
    delay(80);
    targetWrite->writeValue(requestSocCmd, sizeof(requestSocCmd), false);
    delay(80);
    targetWrite->writeValue(jkDeviceInfoCmd, sizeof(jkDeviceInfoCmd), false);
    probeStartMs = millis();
    lockState();
    if (pendingScanName.length() > 0) { bmsDeviceName = pendingScanName; pendingScanName = ""; }
    else if (bmsDeviceName.length() == 0) bmsDeviceName = "BMS " + tgt;
    String nameToSave = bmsDeviceName;
    unlockState();
    preferences.begin("bms_config", false);
    preferences.putString("bms_name", nameToSave);
    preferences.end();
    return true;
}

void bleTask(void* param) {
    LOG_INFO("🔧 BLE Task Core %d\n", xPortGetCoreID());
    unsigned long lastScanAttempt = 0;
    unsigned long lastQueryTime = 0;
    for (;;) {
        if (acquyMode || otaInProgress) {
            if (isScanning) { NimBLEScan* pScan = NimBLEDevice::getScan(); if (pScan) pScan->stop(); isScanning = false; }
            if (pClient && pClient->isConnected()) pClient->disconnect();
            connected = false; pChar = nullptr; pWriteChar = nullptr;
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        if (!bleEnabled || bleShuttingDown) { vTaskDelay(pdMS_TO_TICKS(200)); continue; }
        if (doScan) {
            doScan = false;
            NimBLEScan* pScan = NimBLEDevice::getScan();
            if (isScanning) { pScan->stop(); isScanning = false; delay(100); }
            pScan->clearResults();
            pScan->setActiveScan(true);
            uint32_t scanDurMs = apActive ? 3000 : 5000;
            isScanning = true;
            NimBLEScanResults results = pScan->getResults(scanDurMs, false);
            isScanning = false;
            String html = "";
            int count = results.getCount();
            int bmsCount = 0;
            for (int i = 0; i < count; i++) {
                const NimBLEAdvertisedDevice* device = results.getDevice(i);
                if (!device) continue;
                bool isBms = false;
                if (device->isAdvertisingService(serviceUUID)) isBms = true;
                if (device->haveName()) {
                    String n = String(device->getName().c_str());
                    n.toUpperCase();
                    if (n.startsWith("JK_") || n.startsWith("JK-") || n.startsWith("JK02") || n.indexOf("BMS") >= 0) isBms = true;
                }
                if (!isBms) continue;
                String mac = String(device->getAddress().toString().c_str());
                mac.toUpperCase();
                String name = device->haveName() ? String(device->getName().c_str()) : "BMS";
                String safeName = name;
                safeName.replace("'", "\\'"); safeName.replace("\"", "&quot;");
                bmsCount++;
                html += "<li><div><strong>🔋 " + name + "</strong><br><small style='font-family:monospace'>" + mac + "</small></div>";
                html += "<button style='width:auto;' onclick=\"save('" + mac + "','" + safeName + "')\">Kết Nối</button></li>";
            }
            if (bmsCount == 0) html = "<li style='color:#888;'>Không tìm thấy BMS.</li>";
            pScan->clearResults();
            lockState(); scanResultHtml = html; scanReady = true; unlockState();
        }
        if (doDisconnect) {
            doDisconnect = false;
            if (pClient && pClient->isConnected()) { pClient->disconnect(); delay(200); }
            connected = false; pChar = nullptr; pWriteChar = nullptr; bmsSocTimestamp = 0;
        }
        if (doConnect) {
            doConnect = false;
            NimBLEScan* pScan = NimBLEDevice::getScan();
            if (isScanning) { pScan->stop(); isScanning = false; delay(100); }
            pScan->clearResults();
            connectToServer();
        }
        if (bleAutoScan && !otaInProgress && needAutoConnect) {
            String tgt;
            lockState(); tgt = targetMac; unlockState();
            if (tgt == "" && !connected && !doConnect && !doScan) {
                if (millis() - lastScanAttempt > 5000) {
                    lastScanAttempt = millis();
                    NimBLEScan* pScan = NimBLEDevice::getScan();
                    if (isScanning) { pScan->stop(); isScanning = false; delay(50); }
                    pScan->clearResults();
                    pScan->setActiveScan(true);
                    isScanning = true;
                    NimBLEScanResults results = pScan->getResults(3000, false);
                    isScanning = false;
                    int bmsCount = 0;
                    String foundMac = "", foundName = "";
                    for (int i = 0; i < results.getCount(); i++) {
                        const NimBLEAdvertisedDevice* dev = results.getDevice(i);
                        if (!dev) continue;
                        bool match = false;
                        if (dev->isAdvertisingService(serviceUUID)) match = true;
                        if (dev->haveName()) {
                            String n = String(dev->getName().c_str());
                            n.toUpperCase();
                            if (n.startsWith("JK_") || n.startsWith("JK-") || n.startsWith("JK02") || n.indexOf("BMS") >= 0) match = true;
                        }
                        if (!match) continue;
                        bmsCount++;
                        if (bmsCount == 1) {
                            foundMac = String(dev->getAddress().toString().c_str());
                            foundMac.toUpperCase();
                            foundName = dev->haveName() ? String(dev->getName().c_str()) : "BMS";
                        }
                    }
                    pScan->clearResults();
                    if (bmsCount == 1) {
                        LOG_INFO("✅ [AutoConnect] %s (%s)\n", foundName.c_str(), foundMac.c_str());
                        lockState(); targetMac = foundMac; bmsDeviceName = foundName; unlockState();
                        preferences.begin("bms_config", false);
                        preferences.putString("bms_mac", foundMac);
                        preferences.putString("bms_name", foundName);
                        preferences.putBool("need_auto", false);
                        preferences.putUChar("bms_proto", 255);
                        preferences.putUChar("jk_ver", 0);
                        preferences.end();
                        jkVersion = 0; jkVersionProbed = false; needAutoConnect = false; doConnect = true;
                    }
                }
            } else if (tgt != "") {
                needAutoConnect = false;
                preferences.begin("bms_config", false); preferences.putBool("need_auto", false); preferences.end();
            }
        }
        if (bleAutoScan && !otaInProgress && !needAutoConnect) {
            String tgt;
            lockState(); tgt = targetMac; unlockState();
            if (tgt != "" && !connected && !doConnect && !doScan) {
                if (millis() - lastScanAttempt > 5000) {
                    lastScanAttempt = millis();
                    NimBLEScan* pScan = NimBLEDevice::getScan();
                    if (isScanning) { pScan->stop(); isScanning = false; delay(50); }
                    pScan->clearResults();
                    pScan->setActiveScan(true);
                    isScanning = true;
                    pScan->getResults(2500, false);
                    isScanning = false;
                    pScan->clearResults();
                }
            }
        }
        if (connected && pChar != nullptr) {
            if (millis() - lastQueryTime > 2500) {
                lastQueryTime = millis();
                NimBLERemoteCharacteristic* targetWrite = (pWriteChar != nullptr) ? pWriteChar : pChar;
                if (bmsProtocol == 1) {
                    static uint8_t jkToggle = 0;
                    jkToggle++;
                    if (!jkVersionProbed) targetWrite->writeValue(jkDeviceInfoCmd, sizeof(jkDeviceInfoCmd), false);
                    else { if (jkToggle % 2 == 0) targetWrite->writeValue(jkCellInfoNewCmd, sizeof(jkCellInfoNewCmd), false); else targetWrite->writeValue(jkCellInfoOldCmd, sizeof(jkCellInfoOldCmd), false); }
                } else if (bmsProtocol == 0) targetWrite->writeValue(requestSocCmd, sizeof(requestSocCmd), false);
                else if (bmsProtocol == 255) targetWrite->writeValue(requestSocCmd, sizeof(requestSocCmd), false);
            }
        }
        if (httpBusy) vTaskDelay(pdMS_TO_TICKS(80)); else vTaskDelay(pdMS_TO_TICKS(50));
    }
}

int findJsonValue(const String& json, int start, const String& key) {
    int p = json.indexOf("\"" + key + "\"", start);
    if (p < 0) return -1;
    p = json.indexOf(':', p);
    if (p < 0) return -1;
    return p + 1;
}
String parseJsonString(const String& json, int start, const String& key) {
    int p = findJsonValue(json, start, key);
    if (p < 0) return "";
    p = json.indexOf('"', p);
    if (p < 0) return "";
    int e = json.indexOf('"', p + 1);
    if (e < 0) return "";
    return json.substring(p + 1, e);
}
float parseJsonNumber(const String& json, int start, const String& key) {
    int p = findJsonValue(json, start, key);
    if (p < 0) return 0;
    while (p < (int)json.length() && (json[p] == ' ' || json[p] == '\t')) p++;
    int e = p;
    while (e < (int)json.length() && (isdigit(json[e]) || json[e] == '.' || json[e] == '-' || json[e] == '+')) e++;
    return json.substring(p, e).toFloat();
}
String parseHttpDate(const String& s) {
    int comma = s.indexOf(',');
    if (comma < 0) return s;
    String rest = s.substring(comma + 1); rest.trim();
    int sp1 = rest.indexOf(' '); if (sp1 < 0) return s;
    String day = rest.substring(0, sp1);
    int sp2 = rest.indexOf(' ', sp1 + 1); if (sp2 < 0) return s;
    String mon = rest.substring(sp1 + 1, sp2);
    int sp3 = rest.indexOf(' ', sp2 + 1); if (sp3 < 0) return s;
    String year = rest.substring(sp2 + 1, sp3);
    String timePart = "";
    if (sp3 + 1 < (int)rest.length()) {
        int sp4 = rest.indexOf(' ', sp3 + 1);
        String t = (sp4 < 0) ? rest.substring(sp3 + 1) : rest.substring(sp3 + 1, sp4);
        int c1 = t.indexOf(':'); int c2 = t.indexOf(':', c1 + 1);
        if (c1 > 0 && c2 > c1) timePart = t.substring(0, c2); else timePart = t;
    }
    String monthNum = "??";
    if (mon == "Jan") monthNum = "01"; else if (mon == "Feb") monthNum = "02";
    else if (mon == "Mar") monthNum = "03"; else if (mon == "Apr") monthNum = "04";
    else if (mon == "May") monthNum = "05"; else if (mon == "Jun") monthNum = "06";
    else if (mon == "Jul") monthNum = "07"; else if (mon == "Aug") monthNum = "08";
    else if (mon == "Sep") monthNum = "09"; else if (mon == "Oct") monthNum = "10";
    else if (mon == "Nov") monthNum = "11"; else if (mon == "Dec") monthNum = "12";
    if (day.length() == 1) day = "0" + day;
    String result = day + "/" + monthNum + "/" + year;
    if (timePart.length() > 0) result += " " + timePart;
    return result;
}

bool fetchPresetInfo(const String& url) {
    if (url.length() == 0 || !url.startsWith("http")) return false;
    if (!staConnected && !apActive) return false;
    HTTPClient http; WiFiClient client; WiFiClientSecure clientSecure;
    if (url.startsWith("https://")) { clientSecure.setInsecure(); http.begin(clientSecure, url); }
    else http.begin(client, url);
    http.setTimeout(8000);
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    const char* headerKeys[] = { "Last-Modified", "X-Updated", "ETag" };
    http.collectHeaders(headerKeys, 3);
    int code = http.sendRequest("HEAD");
    if (code != 200) {
        http.end();
        if (url.startsWith("https://")) { clientSecure.setInsecure(); http.begin(clientSecure, url); } else http.begin(client, url);
        http.setTimeout(8000);
        http.addHeader("Range", "bytes=0-0");
        http.collectHeaders(headerKeys, 3);
        code = http.GET();
    }
    if (code != 200 && code != 206) { http.end(); presetInfoLoaded = true; return false; }
    String updated = http.header("X-Updated");
    if (updated.length() == 0) { String lm = http.header("Last-Modified"); if (lm.length() > 0) updated = parseHttpDate(lm); }
    if (updated.length() == 0) { String et = http.header("ETag"); if (et.length() > 0) updated = "ETag: " + et; }
    http.end();
    if (updated.length() > 0) { presetLastUpdated = updated; presetInfoLoaded = true; return true; }
    presetInfoLoaded = true;
    return false;
}

bool fetchPresetsFromUrl(const String& url) {
    if (url.length() == 0) return false;
    if (!staConnected && !apActive) return false;
    HTTPClient http; WiFiClient client; WiFiClientSecure clientSecure;
    if (url.startsWith("https://")) { clientSecure.setInsecure(); http.begin(clientSecure, url); } else http.begin(client, url);
    http.setTimeout(10000);
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    int code = http.GET();
    if (code != 200) { http.end(); return false; }
    String body = http.getString();
    http.end();
    if (body.length() < 20) return false;
    String jsonDate = parseJsonString(body, 0, "date");
    if (jsonDate.length() > 0) { presetLastUpdated = jsonDate; presetInfoLoaded = true; }
    int arrStart = body.indexOf("\"presets\"");
    if (arrStart < 0) arrStart = 0;
    int p = body.indexOf('[', arrStart);
    if (p < 0) return false;
    int newCount = 0; int pos = p + 1;
    while (newCount < MAX_REMOTE_PRESETS) {
        int objStart = body.indexOf('{', pos);
        if (objStart < 0) break;
        int objEnd = body.indexOf('}', objStart);
        if (objEnd < 0) break;
        int entriesPos = body.indexOf("\"entries\"", objStart);
        if (entriesPos < 0 || entriesPos > objEnd + 200) { pos = objEnd + 1; continue; }
        RemotePreset& rp = remotePresets[newCount];
        rp.name     = parseJsonString(body, objStart, "name");
        rp.subtitle = parseJsonString(body, objStart, "subtitle");
        rp.ranges   = parseJsonString(body, objStart, "ranges");
        if (rp.name.length() == 0) { pos = objEnd + 1; continue; }
        int entArr = body.indexOf('[', entriesPos);
        int entEnd = body.indexOf(']', entArr);
        if (entArr < 0 || entEnd < 0) { pos = objEnd + 1; continue; }
        int ec = 0, ep = entArr + 1;
        while (ec < MAX_ENTRIES_PER_PRESET) {
            int es = body.indexOf('{', ep);
            if (es < 0 || es > entEnd) break;
            int ee = body.indexOf('}', es);
            if (ee < 0 || ee > entEnd) break;
            String entryStr = body.substring(es, ee + 1);
            rp.entries[ec].startPct = (uint8_t)parseJsonNumber(entryStr, 0, "s");
            rp.entries[ec].endPct   = (uint8_t)parseJsonNumber(entryStr, 0, "e");
            rp.entries[ec].voltage  = parseJsonNumber(entryStr, 0, "v");
            ec++; ep = ee + 1;
        }
        rp.count = (uint8_t)ec;
        if (ec > 0) newCount++;
        pos = body.indexOf(']', entEnd);
        if (pos < 0) break;
        pos++;
    }
    remotePresetCount = newCount;
    if (newCount > 0) {
        preferences.begin("bms_presets", false);
        preferences.clear();
        preferences.putInt("count", remotePresetCount);
        for (int i = 0; i < remotePresetCount && i < MAX_REMOTE_PRESETS; i++) {
            String base = "p" + String(i) + "_";
            preferences.putString((base + "n").c_str(), remotePresets[i].name);
            preferences.putString((base + "s").c_str(), remotePresets[i].subtitle);
            preferences.putString((base + "r").c_str(), remotePresets[i].ranges);
            preferences.putUChar((base + "c").c_str(), remotePresets[i].count);
            for (int j = 0; j < remotePresets[i].count; j++) {
                String ek = base + "e" + String(j) + "_";
                preferences.putUChar((ek + "s").c_str(), remotePresets[i].entries[j].startPct);
                preferences.putUChar((ek + "e").c_str(), remotePresets[i].entries[j].endPct);
                preferences.putFloat((ek + "v").c_str(), remotePresets[i].entries[j].voltage);
            }
        }
        preferences.end();
    }
    return (newCount > 0);
}

bool fetchUpdateInfo() {
    if (!staConnected) return false;
    HTTPClient http; WiFiClient client; WiFiClientSecure clientSecure;
    String url = UPDATE_JSON_URL;
    if (url.startsWith("https://")) { clientSecure.setInsecure(); http.begin(clientSecure, url); } else http.begin(client, url);
    http.setTimeout(10000);
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    int code = http.GET();
    if (code != 200) { http.end(); return false; }
    String body = http.getString();
    http.end();
    if (body.length() < 20) return false;
    remoteVersion = parseJsonString(body, 0, "version");
    remoteFwUrl   = parseJsonString(body, 0, "url");
    remoteNotes   = parseJsonString(body, 0, "notes");
    remoteDate    = parseJsonString(body, 0, "date");
    remoteNotes.replace("\\n", "\n");
    remoteChecked = true;
    remoteNewer = (remoteVersion != String(FW_VERSION));
    return true;
}

void autoUpdateTask() {
    if (!autoUpdateEnabled || autoUpdateDone) return;
    if (autoUpdateInProgress || otaInProgress) return;
    if (!staConnected) return;
    unsigned long now = millis();
    if (lastAutoCheck == 0) { lastAutoCheck = now; return; }
    if (now - lastAutoCheck < 10000) return;
    autoUpdateDone = true;
    if (!fetchUpdateInfo() || !remoteNewer) return;
    if (remoteNotes.length() > 0) fwNotes = remoteNotes;
    if (remoteDate.length() > 0)  fwDate  = remoteDate;
    preferences.begin("bms_config", false);
    preferences.putString("fw_date", fwDate);
    preferences.putString("fw_notes", fwNotes);
    preferences.end();
    if (remoteFwUrl.length() == 0) return;
    autoUpdateInProgress = true;
    otaInProgress = true;
    doScan = false; doConnect = false; doDisconnect = false;
    if (pClient && pClient->isConnected()) { pClient->disconnect(); delay(500); }
    connected = false; pChar = nullptr; pWriteChar = nullptr;
    delay(1000);
    WiFi.setSleep(false);
    HTTPClient http; WiFiClient client; WiFiClientSecure clientSecure;
    if (remoteFwUrl.startsWith("https://")) { clientSecure.setInsecure(); http.begin(clientSecure, remoteFwUrl); }
    else http.begin(client, remoteFwUrl);
    http.setTimeout(60000);
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    int code = http.GET();
    if (code != 200) { http.end(); autoUpdateInProgress = false; otaInProgress = false; return; }
    int contentLen = http.getSize();
    if (contentLen <= 0) { http.end(); autoUpdateInProgress = false; otaInProgress = false; return; }
    if (!Update.begin(contentLen)) { http.end(); autoUpdateInProgress = false; otaInProgress = false; return; }
    WiFiClient* stream = http.getStreamPtr();
    uint8_t buff[1024];
    int written = 0;
    unsigned long lastDataTime = millis();
    while (written < contentLen) {
        if (!http.connected() && stream->available() == 0) break;
        if (millis() - lastDataTime > 30000) break;
        size_t avail = stream->available();
        if (avail) {
            size_t toRead = min((size_t)sizeof(buff), avail);
            if (written + (int)toRead > contentLen) toRead = contentLen - written;
            int r = stream->readBytes(buff, toRead);
            if (r > 0) {
                if (Update.write(buff, r) != (size_t)r) break;
                written += r;
                lastDataTime = millis();
            }
        } else delay(10);
    }
    http.end();
    if (written != contentLen) { Update.abort(); autoUpdateInProgress = false; otaInProgress = false; return; }
    if (Update.end(true)) { delay(2000); ESP.restart(); }
    autoUpdateInProgress = false; otaInProgress = false;
}

void handleChipInfo() {
    TOUCH_HTTP();
    uint32_t heapTotal = ESP.getHeapSize();
    uint32_t heapFree  = ESP.getFreeHeap();
    uint32_t heapUsed  = heapTotal - heapFree;
    uint32_t heapMin   = ESP.getMinFreeHeap();
    uint32_t psramTotal = ESP.getPsramSize();
    uint32_t psramFree  = ESP.getFreePsram();
    uint32_t flashSize  = ESP.getFlashChipSize();
    uint32_t sketchSize = ESP.getSketchSize();
    uint32_t sketchFree = ESP.getFreeSketchSpace();
    uint32_t cpuFreq    = ESP.getCpuFreqMHz();
    uint32_t chipRev    = ESP.getChipRevision();
    uint8_t  coreCount  = ESP.getChipCores();
    unsigned long uptime = millis() / 1000;
    String json = "{";
    json += "\"heapTotal\":" + String(heapTotal) + ",";
    json += "\"heapFree\":" + String(heapFree) + ",";
    json += "\"heapUsed\":" + String(heapUsed) + ",";
    json += "\"heapMin\":" + String(heapMin) + ",";
    json += "\"psramTotal\":" + String(psramTotal) + ",";
    json += "\"psramFree\":" + String(psramFree) + ",";
    json += "\"flashSize\":" + String(flashSize) + ",";
    json += "\"sketchSize\":" + String(sketchSize) + ",";
    json += "\"sketchFree\":" + String(sketchFree) + ",";
    json += "\"cpuFreq\":" + String(cpuFreq) + ",";
    json += "\"chipRev\":" + String(chipRev) + ",";
    json += "\"cores\":" + String(coreCount) + ",";
    json += "\"chipModel\":\"" + String(ESP.getChipModel()) + "\",";
    json += "\"cpuLoad\":" + String(cpuLoadAvg, 1) + ",";
    json += "\"cpuLoad0\":" + String(cpuLoad0, 1) + ",";
    json += "\"cpuLoad1\":" + String(cpuLoad1, 1) + ",";
    json += "\"uptime\":" + String(uptime);
    json += "}";
    server.sendHeader("Cache-Control", "no-cache");
    server.send(200, "application/json", json);
}

void handleData() {
    TOUCH_HTTP();
    if (cachedJsonData.length() > 0 && millis() - cachedJsonTime < JSON_CACHE_MS) {
        server.sendHeader("Cache-Control", "no-cache");
        server.sendHeader("Connection", "close");
        server.send(200, "application/json", cachedJsonData);
        return;
    }
    lockState();
    float voltage = 0.0f;
    if (outMode == OUT_MODE_DAC) {
#if HAS_DAC
        voltage = currentOut * 3.3f / 255.0f;
#endif
    } else {
        int maxDuty = (1 << pwmRes) - 1;
        voltage = currentOut * 3.3f / (float)maxDuty;
    }
    bool isFresh = false; unsigned long age = 0;
    if ((connected || acquyMode) && bmsSocTimestamp > 0) {
        age = millis() - bmsSocTimestamp;
        isFresh = (age < 5000);
    }
    float vE, vF, vW;
    getBatteryLimits(vE, vF, vW);
    String json = "{";
    json += "\"soc\":" + String(currentSoc) + ",";
    json += "\"outMode\":" + String(outMode) + ",";
    json += "\"dacPin\":" + String(dacPin) + ",";
    json += "\"pwmPin\":" + String(pwmPin) + ",";
    json += "\"pwmFreq\":" + String(pwmFreq) + ",";
    json += "\"pwmRes\":" + String(pwmRes) + ",";
    json += "\"hasDac\":" + String(HAS_DAC ? "true" : "false") + ",";
    json += "\"chipName\":\"" + String(CHIP_NAME) + "\",";
    json += "\"bleAutoScan\":" + String(bleAutoScan ? "true" : "false") + ",";
    json += "\"bleEnabled\":" + String(bleEnabled ? "true" : "false") + ",";
    json += "\"configMode\":" + String(apActive ? "true" : "false") + ",";
    json += "\"ledMode\":" + String((int)currentLedMode) + ",";
    json += "\"voltage\":" + String(voltage, 3) + ",";
    json += "\"minV\":" + String(minVoltage, 2) + ",";
    json += "\"maxV\":" + String(maxVoltage, 2) + ",";
    json += "\"connected\":" + String(connected ? "true" : "false") + ",";
    json += "\"manualSim\":" + String(manualSimMode ? "true" : "false") + ",";
    json += "\"realSoc\":" + String(realBmsSoc) + ",";
    json += "\"savedSoc\":" + String(savedSoc) + ",";
    json += "\"socFresh\":" + String(isFresh ? "true" : "false") + ",";
    json += "\"socAge\":" + String(age) + ",";
    json += "\"mac\":\"" + targetMac + "\",";
    String displayName = bmsDeviceName;
    if (displayName.length() == 0 && targetMac.length() > 0) displayName = "BMS " + targetMac;
    String safeBmsName = displayName;
    safeBmsName.replace("\\", "\\\\"); safeBmsName.replace("\"", "\\\"");
    json += "\"bmsName\":\"" + safeBmsName + "\",";
    json += "\"needAutoConnect\":" + String(needAutoConnect ? "true" : "false") + ",";
    json += "\"bmsProto\":" + String((int)bmsProtocol) + ",";
    json += "\"jkVer\":" + String((int)jkVersion) + ",";
    json += "\"acquyMode\":" + String(acquyMode ? "true" : "false") + ",";
    json += "\"aqPin\":" + String(acquyAdcPin) + ",";
    json += "\"aqR1\":" + String(acquyR1, 2) + ",";
    json += "\"aqR2\":" + String(acquyR2, 2) + ",";
    json += "\"aqGain\":" + String(acquyCalibGain, 4) + ",";
    json += "\"aqOffset\":" + String(acquyCalibOffset, 3) + ",";
    json += "\"aqMeasuredV\":" + String(acquyMeasuredV, 2) + ",";
    json += "\"aqAdcV\":" + String(acquyAdcRawV, 3) + ",";
    json += "\"aqOvp\":" + String(acquyOverVoltage ? "true" : "false") + ",";
    json += "\"aqRawV\":" + String(acquyRawV, 2) + ",";
    json += "\"aqStableV\":" + String(acquyStableV, 2) + ",";
    json += "\"aqSocRaw\":" + String(acquySocRaw) + ",";
    json += "\"aqSocStable\":" + String(acquySocStable) + ",";
    json += "\"aqIsStable\":" + String(acquyStable ? "true" : "false") + ",";
    json += "\"aqAdcMax\":" + String(calcAdcMaxVoltage(), 3) + ",";
    json += "\"battType\":" + String(batteryType) + ",";
    json += "\"battCells\":" + String(batteryCells) + ",";
    json += "\"battVEmpty\":" + String(vE, 2) + ",";
    json += "\"battVFull\":" + String(vF, 2) + ",";
    json += "\"battVMax\":" + String(vW, 2) + ",";
    json += "\"totalV\":" + String(bmsTotalVoltage, 2) + ",";
    json += "\"hasTotalV\":" + String(bmsHasTotalV ? "true" : "false") + ",";
    json += "\"current\":" + String(bmsCurrent, 2) + ",";
    json += "\"hasCurrent\":" + String(bmsHasCurrent ? "true" : "false") + ",";
    json += "\"power\":" + String(bmsPower, 1) + ",";
    json += "\"hasPower\":" + String(bmsHasPower ? "true" : "false") + ",";
    json += "\"cellCount\":" + String(bmsCellCount) + ",";
    json += "\"hasCells\":" + String(bmsHasCells ? "true" : "false") + ",";
    json += "\"cells\":[";
    for (int i = 0; i < 16; i++) { json += String(bmsCells[i], 3); if (i < 15) json += ","; }
    json += "]";
    json += ",\"vmapEnabled\":" + String(vmapEnabled ? "true" : "false");
    json += ",\"vmap\":[";
    for (uint8_t i = 0; i < vmapCount; i++) {
        json += "{\"s\":" + String(vmap[i].startPct) + ",\"e\":" + String(vmap[i].endPct) + ",\"v\":" + String(vmap[i].voltage, 2) + "}";
        if (i < vmapCount - 1) json += ",";
    }
    json += "]";
    json += ",\"staEnabled\":" + String(staEnabled ? "true" : "false");
    json += ",\"staConnected\":" + String(staConnected ? "true" : "false");
    json += ",\"staSSID\":\"" + staSSID + "\"";
    json += ",\"staPass\":\"" + staPass + "\"";
    if (staConnected) {
        json += ",\"staIP\":\"" + WiFi.localIP().toString() + "\"";
        json += ",\"staRSSI\":" + String(WiFi.RSSI());
    }
    json += ",\"presetUpdated\":\"" + presetLastUpdated + "\"";
    json += ",\"presetInfoLoaded\":" + String(presetInfoLoaded ? "true" : "false");
    String safeActiveName = activePresetName;
    safeActiveName.replace("\\", "\\\\"); safeActiveName.replace("\"", "\\\"");
    json += ",\"activePresetName\":\"" + safeActiveName + "\"";
    String safeFwNotes = fwNotes;
    safeFwNotes.replace("\"", "\\\""); safeFwNotes.replace("\n", "\\n");
    json += ",\"fwVersion\":\"" + String(FW_VERSION) + "\"";
    json += ",\"fwDate\":\"" + fwDate + "\"";
    json += ",\"fwNotes\":\"" + safeFwNotes + "\"";
    json += ",\"autoUpdate\":" + String(autoUpdateEnabled ? "true" : "false");
    json += ",\"autoUpdateDone\":" + String(autoUpdateDone ? "true" : "false");
    json += ",\"apActive\":" + String(apActive ? "true" : "false");
    if (apActive) {
        uint8_t cc = WiFi.softAPgetStationNum();
        unsigned long ref = (apLastClientTime > 0) ? apLastClientTime : apStartedTime;
        long remain = (long)AP_TIMEOUT_MS - (long)(millis() - ref);
        if (cc > 0) remain = -1;
        if (!staConnected) remain = -2;
        json += ",\"apClients\":" + String(cc);
        json += ",\"apRemain\":" + String(remain / 1000);
    }
    json += "}";
    unlockState();
    cachedJsonData = json;
    cachedJsonTime = millis();
    server.sendHeader("Cache-Control", "no-cache");
    server.sendHeader("Connection", "close");
    server.send(200, "application/json", json);
}

void handleSerial() {
    TOUCH_HTTP();
    String data = MySerial.readAll();
    server.sendHeader("Cache-Control", "no-cache");
    server.send(200, "text/plain; charset=utf-8", data);
}

void handleSim() {
    TOUCH_HTTP();
    if (server.hasArg("soc")) {
        uint8_t simSoc = (uint8_t)constrain(server.arg("soc").toInt(), 0, 100);
        lockState(); manualSimMode = true; unlockState();
        applyOutput(simSoc);
        server.send(200, "text/plain", "OK");
    } else if (server.hasArg("real")) {
        uint8_t realSoc, cached; bool conn;
        lockState();
        manualSimMode = false;
        realSoc = realBmsSoc; conn = connected || acquyMode; cached = savedSoc;
        unlockState();
        if (conn && realSoc > 0) applyOutput(realSoc);
        else if (cached > 0) applyOutput(cached);
        server.send(200, "text/plain", "OK");
    } else server.send(400, "text/plain", "Missing arg");
}

void handleSaveOutput() {
    TOUCH_HTTP();
    int newMode = server.hasArg("mode") ? server.arg("mode").toInt() : outMode;
    int newDacPin = server.hasArg("dacpin") ? server.arg("dacpin").toInt() : dacPin;
    int newPwmPin = server.hasArg("pwmpin") ? server.arg("pwmpin").toInt() : pwmPin;
    int newFreq = server.hasArg("freq") ? server.arg("freq").toInt() : pwmFreq;
    int newRes = server.hasArg("res") ? server.arg("res").toInt() : pwmRes;
#if !HAS_DAC
    if (newMode == OUT_MODE_DAC) { server.send(400, "text/plain", "No DAC"); return; }
#endif
    if (newMode == OUT_MODE_DAC && newDacPin != 25 && newDacPin != 26) { server.send(400, "text/plain", "DAC pin invalid"); return; }
    if (newMode == OUT_MODE_PWM) {
        if (newPwmPin < 0 || newPwmPin > 48) { server.send(400, "text/plain", "PWM pin invalid"); return; }
        if (newFreq < 1 || newFreq > 40000000) { server.send(400, "text/plain", "Freq invalid"); return; }
        if (newRes < 1 || newRes > 14) { server.send(400, "text/plain", "Res invalid"); return; }
    }
    if (outMode == OUT_MODE_DAC) {
#if HAS_DAC
        dacWrite(dacPin, 0); pinMode(dacPin, INPUT);
#endif
    } else disablePWM();
    outMode = newMode; dacPin = newDacPin; pwmPin = newPwmPin;
    pwmFreq = newFreq; pwmRes = newRes;
    if (outMode == OUT_MODE_DAC) {
#if HAS_DAC
        pinMode(dacPin, OUTPUT); dacWrite(dacPin, 0);
#endif
    } else { pinMode(pwmPin, OUTPUT); setupPWM(); }
    preferences.begin("bms_config", false);
    preferences.putInt("out_mode", outMode);
    preferences.putInt("dac_pin", dacPin);
    preferences.putInt("pwm_pin", pwmPin);
    preferences.putInt("pwm_freq", pwmFreq);
    preferences.putInt("pwm_res", pwmRes);
    preferences.end();
    applyOutput(currentSoc);
    server.send(200, "text/plain", "OK");
}

void handleSaveVMap() {
    TOUCH_HTTP();
    bool enabled = server.hasArg("enabled") && server.arg("enabled") == "1";
    int count = server.hasArg("count") ? server.arg("count").toInt() : 0;
    if (count < 0) count = 0;
    if (count > MAX_VMAP_ENTRIES) count = MAX_VMAP_ENTRIES;
    VMapEntry newMap[MAX_VMAP_ENTRIES];
    int newCount = 0;
    for (int i = 0; i < count; i++) {
        String ks = "e" + String(i) + "s"; String ke = "e" + String(i) + "e"; String kv = "e" + String(i) + "v";
        if (!server.hasArg(ks) || !server.hasArg(ke) || !server.hasArg(kv)) continue;
        int s = constrain(server.arg(ks).toInt(), 0, 100);
        int e = constrain(server.arg(ke).toInt(), 0, 100);
        float v = constrain(server.arg(kv).toFloat(), 0.0f, 3.3f);
        if (s > e) { int t = s; s = e; e = t; }
        newMap[newCount].startPct = (uint8_t)s; newMap[newCount].endPct = (uint8_t)e; newMap[newCount].voltage = v;
        newCount++;
    }
    for (int i = 0; i < newCount - 1; i++)
        for (int j = i + 1; j < newCount; j++)
            if (newMap[j].startPct < newMap[i].startPct) { VMapEntry tmp = newMap[i]; newMap[i] = newMap[j]; newMap[j] = tmp; }
    for (int i = 1; i < newCount; i++) {
        if (newMap[i].startPct > newMap[i - 1].endPct + 1) newMap[i].startPct = newMap[i - 1].endPct + 1;
        if (newMap[i].startPct <= newMap[i - 1].endPct) newMap[i].startPct = newMap[i - 1].endPct + 1;
    }
    lockState();
    vmapEnabled = enabled;
    vmapCount = (uint8_t)newCount;
    for (int i = 0; i < newCount; i++) vmap[i] = newMap[i];
    uint8_t soc = currentSoc;
    unlockState();
    activePresetName = "";
    preferences.begin("bms_config", false);
    preferences.putBool("vmap_en", vmapEnabled);
    preferences.putUChar("vmap_cnt", vmapCount);
    preferences.putString("active_preset", "");
    for (int i = 0; i < MAX_VMAP_ENTRIES; i++) {
        String base = "vm" + String(i) + "_";
        if (i < vmapCount) {
            preferences.putUChar((base + "s").c_str(), vmap[i].startPct);
            preferences.putUChar((base + "e").c_str(), vmap[i].endPct);
            preferences.putFloat((base + "v").c_str(), vmap[i].voltage);
        } else {
            preferences.remove((base + "s").c_str());
            preferences.remove((base + "e").c_str());
            preferences.remove((base + "v").c_str());
        }
    }
    preferences.end();
    applyOutput(soc);
    server.send(200, "text/plain", "OK");
}

void handlePresets() {
    TOUCH_HTTP();
    String json = "[";
    for (int i = 0; i < remotePresetCount; i++) {
        String name = remotePresets[i].name;
        String sub  = remotePresets[i].subtitle;
        String rng  = remotePresets[i].ranges;
        name.replace("\"", "\\\""); sub.replace("\"", "\\\""); rng.replace("\"", "\\\"");
        json += "{\"name\":\"" + name + "\"";
        json += ",\"subtitle\":\"" + sub + "\"";
        json += ",\"ranges\":\"" + rng + "\"";
        json += "}";
        if (i < remotePresetCount - 1) json += ",";
    }
    json += "]";
    server.sendHeader("Cache-Control", "no-cache");
    server.send(200, "application/json", json);
}

void handleApplyPreset() {
    TOUCH_HTTP();
    if (!server.hasArg("id")) { server.send(400, "text/plain", "Missing id"); return; }
    int id = server.arg("id").toInt();
    if (id < 0 || id >= remotePresetCount) { server.send(400, "text/plain", "Preset khong ton tai"); return; }
    RemotePreset& p = remotePresets[id];
    if (p.count == 0) { server.send(400, "text/plain", "Preset rong"); return; }
    lockState();
    vmapCount = p.count;
    for (uint8_t i = 0; i < p.count; i++) vmap[i] = p.entries[i];
    vmapEnabled = true;
    uint8_t soc = currentSoc;
    unlockState();
    activePresetName = p.name;
    preferences.begin("bms_config", false);
    preferences.putBool("vmap_en", true);
    preferences.putUChar("vmap_cnt", vmapCount);
    preferences.putString("active_preset", activePresetName);
    for (int i = 0; i < MAX_VMAP_ENTRIES; i++) {
        String base = "vm" + String(i) + "_";
        if (i < vmapCount) {
            preferences.putUChar((base + "s").c_str(), vmap[i].startPct);
            preferences.putUChar((base + "e").c_str(), vmap[i].endPct);
            preferences.putFloat((base + "v").c_str(), vmap[i].voltage);
        } else {
            preferences.remove((base + "s").c_str());
            preferences.remove((base + "e").c_str());
            preferences.remove((base + "v").c_str());
        }
    }
    preferences.end();
    applyOutput(soc);
    server.send(200, "text/plain", "OK");
}

void handleFetchPresets() {
    TOUCH_HTTP();
    presetInfoLoaded = false;
    presetLastUpdated = "";
    bool ok = fetchPresetsFromUrl(PRESET_URL);
    if (presetLastUpdated.length() == 0) fetchPresetInfo(PRESET_URL);
    if (ok) {
        lastPresetSync = millis();
        if (presetLastUpdated.length() == 0) presetLastUpdated = "Không rõ";
        String resp = "OK|" + String(remotePresetCount) + "|" + presetLastUpdated;
        server.send(200, "text/plain", resp);
    } else server.send(500, "text/plain", "Tai that bai");
}

void handleSaveWifi() {
    TOUCH_HTTP();
    if (!server.hasArg("ssid")) { server.send(400, "text/plain", "Missing ssid"); return; }
    String newSSID = server.arg("ssid");
    String newPass = server.hasArg("pass") ? server.arg("pass") : "";
    bool enable = server.hasArg("en") && server.arg("en") == "1";
    newSSID.trim();
    if (newSSID.length() == 0) { server.send(400, "text/plain", "SSID rong"); return; }
    if (newSSID.length() > 32) newSSID = newSSID.substring(0, 32);
    if (newPass.length() > 64) newPass = newPass.substring(0, 64);
    staSSID = newSSID; staPass = newPass; staEnabled = enable;
    preferences.begin("bms_config", false);
    preferences.putString("sta_ssid", staSSID);
    preferences.putString("sta_pass", staPass);
    preferences.putBool("sta_en", staEnabled);
    preferences.end();
    WiFi.disconnect(false, false);
    staConnected = false; staAttempting = false;
    if (staEnabled) {
        if (apActive) WiFi.mode(WIFI_AP_STA); else WiFi.mode(WIFI_STA);
        staStart();
    } else staStop();
    server.send(200, "text/plain", "OK");
}

void handleWifiScan() {
    TOUCH_HTTP();
    int n = WiFi.scanNetworks(false, true);
    String html = "";
    if (n <= 0) html = "<li style='color:#888;'>Không tìm thấy WiFi</li>";
    else {
        for (int i = 0; i < n; i++) {
            String ssid = WiFi.SSID(i);
            int rssi = WiFi.RSSI(i);
            String enc = (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? "🔓" : "🔒";
            String safeSsid = ssid;
            safeSsid.replace("'", "\\'");
            html += "<li><div><strong>" + enc + " " + ssid + "</strong><br><small>"
                    + String(rssi) + " dBm</small></div>"
                    "<button style='width:auto;' onclick=\"pickWifi('" + safeSsid + "')\">Chọn</button></li>";
        }
    }
    WiFi.scanDelete();
    server.send(200, "text/html", html);
}

void handleSave() {
    TOUCH_HTTP();
    if (!server.hasArg("mac")) { server.send(400, "text/plain", "Missing mac"); return; }
    String newMac = server.arg("mac");
    newMac.toUpperCase(); newMac.trim();
    if (newMac.length() != 17) { server.send(400, "text/plain", "MAC sai do dai"); return; }
    for (int i = 0; i < 17; i++) {
        char c = newMac[i];
        if (i % 3 == 2) { if (c != ':') { server.send(400, "text/plain", "Thieu :"); return; } }
        else { if (!isxdigit(c)) { server.send(400, "text/plain", "Ky tu khong hop le"); return; } }
    }
    String newName = server.hasArg("name") ? server.arg("name") : "";
    newName.trim();
    if (newName.length() > 40) newName = newName.substring(0, 40);
    lockState();
    targetMac = newMac;
    if (newName.length() > 0) bmsDeviceName = newName;
    else if (bmsDeviceName.length() == 0) bmsDeviceName = "BMS " + newMac;
    String nameToSave = bmsDeviceName;
    unlockState();
    preferences.begin("bms_config", false);
    preferences.putString("bms_mac", targetMac);
    preferences.putString("bms_name", nameToSave);
    preferences.putBool("need_auto", false);
    preferences.putUChar("bms_proto", 255);
    preferences.putUChar("jk_ver", 0);
    preferences.end();
    jkVersion = 0; jkVersionProbed = false; needAutoConnect = false;
    if (!acquyMode) { doDisconnect = true; doConnect = true; }
    server.send(200, "text/plain", "OK");
}

void handleClearMac() {
    TOUCH_HTTP();
    lockState(); targetMac = ""; bmsDeviceName = ""; unlockState();
    preferences.begin("bms_config", false);
    preferences.putString("bms_mac", "");
    preferences.putString("bms_name", "");
    preferences.putBool("need_auto", true);
    preferences.putUChar("bms_proto", 255);
    preferences.putUChar("jk_ver", 0);
    preferences.end();
    jkVersion = 0; jkVersionProbed = false;
    server.send(200, "text/plain", "OK");
}

void handleScan() {
    TOUCH_HTTP();
    if (acquyMode) { server.send(200, "text/html", "<li style='color:#888;'>Đang bật chế độ ẮC QUY. Hãy tắt để quét.</li>"); return; }
    lockState(); scanReady = false; scanResultHtml = ""; unlockState();
    doScan = true;
    unsigned long t0 = millis();
    while (millis() - t0 < 5000) {
        lockState(); bool ok = scanReady; unlockState();
        if (ok) break;
        delay(50);
    }
    String html;
    lockState();
    if (scanReady) { html = scanResultHtml; scanReady = false; }
    unlockState();
    if (html.length() > 0) server.send(200, "text/html", html);
    else server.send(200, "text/html", "<li style='color:#888;'>Không có kết quả</li>");
}

void handleToggleAcquy() {
    TOUCH_HTTP();
    if (!server.hasArg("en")) { server.send(400, "text/plain", "Missing arg"); return; }
    acquyMode = (server.arg("en") == "1");
    preferences.begin("bms_config", false);
    preferences.putBool("aq_en", acquyMode);
    preferences.end();
    if (acquyMode) {
        bleDeinit();
        acquyMedianFilled = false; acquyMedianIdx = 0;
        acquyEmaInit = false; acquyStable = false; acquyStableStart = 0;
        if (!apActive) ledSetMode(LED_MODE_SOLID);
    } else {
        bleInit();
        bleAutoScan = true;
        if (!apActive) ledSetMode(connected ? LED_MODE_BREATHE : LED_MODE_SOLID);
    }
    server.send(200, "text/plain", "OK");
}

void handleSaveAcquyCfg() {
    TOUCH_HTTP();
    if (server.hasArg("pin"))    acquyAdcPin = server.arg("pin").toInt();
    if (server.hasArg("r1"))     acquyR1 = server.arg("r1").toFloat();
    if (server.hasArg("r2"))     acquyR2 = server.arg("r2").toFloat();
    if (server.hasArg("btype"))  batteryType = server.arg("btype").toInt();
    if (server.hasArg("cells"))  batteryCells = server.arg("cells").toInt();
    if (server.hasArg("gain"))   acquyCalibGain = server.arg("gain").toFloat();
    if (server.hasArg("offset")) acquyCalibOffset = server.arg("offset").toFloat();
    if (batteryType != BATT_TYPE_LEAD && batteryType != BATT_TYPE_LFP) batteryType = BATT_TYPE_LEAD;
    if (batteryCells < 1) batteryCells = 1;
    if (batteryCells > 32) batteryCells = 32;
    if (acquyAdcPin < 0 || acquyAdcPin > 39) acquyAdcPin = 32;
    if (acquyR1 < 0.1f) acquyR1 = 100.0f;
    if (acquyR2 < 0.1f) acquyR2 = 4.7f;
    if (acquyCalibGain < 0.5f || acquyCalibGain > 2.0f) acquyCalibGain = 1.0f;
    if (acquyCalibOffset < -5.0f || acquyCalibOffset > 5.0f) acquyCalibOffset = 0.0f;
    {
        float vEmpty, vFull, vMaxWarn;
        getBatteryLimits(vEmpty, vFull, vMaxWarn);
        float ratio = acquyR2 / (acquyR1 + acquyR2);
        float vadcMax = vMaxWarn * ratio;
        if (vadcMax > 3.30f) LOG_INFO("⚠️ CẢNH BÁO: V_ADC max = %.3fV vượt 3.3V! R1=%.1fk R2=%.1fk\n", vadcMax, acquyR1, acquyR2);
        else LOG_INFO("✅ Phân áp OK: V_ADC max = %.3fV (chia %.2fx)\n", vadcMax, (acquyR1 + acquyR2) / acquyR2);
    }
    preferences.begin("bms_config", false);
    preferences.putInt("aq_pin", acquyAdcPin);
    preferences.putFloat("aq_r1", acquyR1);
    preferences.putFloat("aq_r2", acquyR2);
    preferences.putInt("batt_type", batteryType);
    preferences.putInt("batt_cells", batteryCells);
    preferences.putFloat("aq_gain", acquyCalibGain);
    preferences.putFloat("aq_off", acquyCalibOffset);
    preferences.end();
    pinMode(acquyAdcPin, INPUT);
    acquyMedianFilled = false; acquyMedianIdx = 0;
    acquyEmaInit = false; acquyStable = false; acquyStableStart = 0;
    LOG_INFO("💾 %s | %d cell | Gain=%.4f Offset=%.3fV\n",
             batteryType == BATT_TYPE_LFP ? "LFP" : "Chì",
             batteryCells, acquyCalibGain, acquyCalibOffset);
    server.send(200, "text/plain", "OK");
}

void handleSaveFwInfo() {
    TOUCH_HTTP();
    if (!server.hasArg("date") || !server.hasArg("notes")) { server.send(400, "text/plain", "Missing arg"); return; }
    fwDate  = server.arg("date");
    fwNotes = server.arg("notes");
    if (fwDate.length() > 32)  fwDate  = fwDate.substring(0, 32);
    if (fwNotes.length() > 500) fwNotes = fwNotes.substring(0, 500);
    preferences.begin("bms_config", false);
    preferences.putString("fw_date", fwDate);
    preferences.putString("fw_notes", fwNotes);
    preferences.end();
    server.send(200, "text/plain", "OK");
}

void handleGetNotes() {
    TOUCH_HTTP();
    String notes = remoteNotes;
    notes.replace("\n", "<br>");
    server.send(200, "text/html; charset=utf-8", notes);
}

void handleSaveAutoUpdate() {
    TOUCH_HTTP();
    if (!server.hasArg("en")) { server.send(400, "text/plain", "Missing arg"); return; }
    bool en = server.arg("en") == "1";
    bool wasEnabled = autoUpdateEnabled;
    autoUpdateEnabled = en;
    if (en && !wasEnabled) { autoUpdateDone = false; lastAutoCheck = 0; }
    preferences.begin("bms_config", false);
    preferences.putBool("auto_update", autoUpdateEnabled);
    preferences.end();
    server.send(200, "text/plain", "OK");
}

void handleUpdateDone() {
    server.sendHeader("Connection", "close");
    if (Update.hasError()) {
        String err = "Update failed. Error #";
        err += Update.errorString();
        server.send(500, "text/plain; charset=utf-8", err);
    } else {
        server.send(200, "text/plain; charset=utf-8", "OK");
        delay(500);
        ESP.restart();
    }
}

void handleUpdateUpload() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
        otaInProgress = true;
        apLastClientTime = millis();
        doScan = false; doConnect = false; doDisconnect = false;
        if (pClient && pClient->isConnected()) pClient->disconnect();
        connected = false; pChar = nullptr; pWriteChar = nullptr;
        if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial);
    } else if (upload.status == UPLOAD_FILE_WRITE) {
        if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) Update.printError(Serial);
    } else if (upload.status == UPLOAD_FILE_END) {
        if (!Update.end(true)) Update.printError(Serial);
        otaInProgress = false;
        apLastClientTime = millis();
    } else if (upload.status == UPLOAD_FILE_ABORTED) {
        Update.end();
        otaInProgress = false;
        apLastClientTime = millis();
    }
}

void handleCheckUpdate() {
    TOUCH_HTTP();
    if (!staConnected) { server.send(200, "text/plain", "NO_WIFI"); return; }
    bool ok = fetchUpdateInfo();
    if (!ok) { server.send(500, "text/plain", "FETCH_FAIL"); return; }
    if (remoteNewer) {
        bool updated = false;
        if (remoteNotes.length() > 0 && remoteNotes != fwNotes) { fwNotes = remoteNotes; updated = true; }
        if (remoteDate.length() > 0 && remoteDate != fwDate) { fwDate = remoteDate; updated = true; }
        if (updated) {
            preferences.begin("bms_config", false);
            preferences.putString("fw_date", fwDate);
            preferences.putString("fw_notes", fwNotes);
            preferences.end();
        }
        String resp = "NEW:";
        resp += remoteVersion + ":" + remoteDate + ":0:1:" + remoteFwUrl;
        server.send(200, "text/plain", resp);
        return;
    }
    bool synced = false;
    if (fwNotes.length() == 0 || fwDate.length() == 0) {
        if (remoteNotes.length() > 0) fwNotes = remoteNotes;
        if (remoteDate.length() > 0)  fwDate  = remoteDate;
        preferences.begin("bms_config", false);
        preferences.putString("fw_date", fwDate);
        preferences.putString("fw_notes", fwNotes);
        preferences.end();
        synced = true;
    }
    String resp = "LATEST:" + String(FW_VERSION);
    if (synced) resp += ":SYNCED";
    server.send(200, "text/plain", resp);
}

void handleOtaOnline() {
    TOUCH_HTTP();
    if (!staConnected) { server.send(400, "text/plain", "Can WiFi nha"); return; }
    if (remoteFwUrl.length() == 0) { server.send(400, "text/plain", "Chua co URL"); return; }
    otaInProgress = true;
    WiFi.setSleep(false);
    doScan = false; doConnect = false; doDisconnect = false;
    if (pClient && pClient->isConnected()) { pClient->disconnect(); delay(500); }
    connected = false; pChar = nullptr; pWriteChar = nullptr;
    delay(1000);
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain; charset=utf-8", "OK");
    delay(200);
    HTTPClient http; WiFiClient client; WiFiClientSecure clientSecure;
    if (remoteFwUrl.startsWith("https://")) { clientSecure.setInsecure(); http.begin(clientSecure, remoteFwUrl); }
    else http.begin(client, remoteFwUrl);
    http.setTimeout(60000);
    http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
    int code = http.GET();
    if (code != 200) { otaInProgress = false; http.end(); return; }
    int contentLen = http.getSize();
    if (contentLen <= 0) { otaInProgress = false; http.end(); return; }
    if (!Update.begin(contentLen)) { otaInProgress = false; http.end(); return; }
    WiFiClient* stream = http.getStreamPtr();
    uint8_t buff[1024];
    int written = 0;
    unsigned long lastDataTime = millis();
    while (written < contentLen) {
        if (!http.connected() && stream->available() == 0) break;
        if (millis() - lastDataTime > 30000) break;
        size_t avail = stream->available();
        if (avail) {
            size_t toRead = min((size_t)sizeof(buff), avail);
            if (written + (int)toRead > contentLen) toRead = contentLen - written;
            int r = stream->readBytes(buff, toRead);
            if (r > 0) {
                if (Update.write(buff, r) != (size_t)r) break;
                written += r;
                lastDataTime = millis();
            }
        } else delay(10);
    }
    http.end();
    if (written != contentLen) { Update.abort(); otaInProgress = false; return; }
    if (Update.end(true)) { delay(2000); ESP.restart(); }
    otaInProgress = false;
}

void handleCaptivePortal() {
    TOUCH_HTTP();
    server.sendHeader("Location", "http://192.168.4.1/", true);
    server.send(302, "text/plain", "");
}

void handleForceAp() {
    TOUCH_HTTP();
    if (!apActive) {
        apStart();
        ledSetMode(LED_MODE_CONFIG_BLINK);
        server.send(200, "text/plain", "Đã bật AP: BMS_Config/12345678");
    } else {
        server.send(200, "text/plain", "AP đang chạy sẵn");
    }
}
// =====================================================
// TRANG CHỦ HTML / UI
// =====================================================
void handleRoot() {
    TOUCH_HTTP();
    httpBusy = true;
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "text/html; charset=utf-8", "");

    server.sendContent(
        "<!DOCTYPE html><html lang='vi'><head><meta charset='utf-8'>"
        "<meta name='viewport' content='width=device-width,initial-scale=1'>"
        "<meta name='apple-mobile-web-app-capable' content='yes'>"
        "<meta name='mobile-web-app-capable' content='yes'>"
        "<title>BMS & ACQUY Controller</title><style>"
        "body{font-family:sans-serif;padding:15px;max-width:500px;margin:auto;background:#f0f2f5}"
        ".card{background:#fff;padding:15px;border-radius:10px;box-shadow:0 2px 5px rgba(0,0,0,.1);margin-bottom:15px}"
        "h2,h3{margin-top:0;color:#333;font-size:18px}"
        ".battery-bg{background:#e0e0e0;border-radius:8px;height:20px;overflow:hidden;margin-top:8px}"
        ".battery-bar{background:#28a745;height:100%;width:0;transition:width .3s}"
        "button{padding:10px 15px;width:100%;cursor:pointer;font-size:15px;background:#007bff;color:#fff;border:none;border-radius:5px;margin-top:10px}"
        ".btn-success{background:#28a745;font-weight:bold}"
        ".btn-danger{background:#dc3545;color:#fff;font-weight:bold}"
        ".btn-warn{background:#ff9800;color:#fff;font-weight:bold}"
        ".stat-box{background:#f8f9fa;border:1px solid #e9ecef;padding:10px;border-radius:8px;text-align:center}"
        ".stat-box div:first-child{font-size:13px;color:#6c757d;font-weight:bold}"
        ".stat-val{font-size:20px;font-weight:bold;color:#007bff;margin-top:5px}"
        "input[type=number],input[type=text],input[type=password],select{width:100%;padding:8px;border-radius:5px;border:1px solid #ccc;box-sizing:border-box}"
        "textarea{font-family:inherit}"
        "input[type=range]{width:100%;margin:12px 0}"
        ".input-row{display:flex;justify-content:space-between;align-items:center;margin-bottom:12px;gap:10px}"
        ".input-row label{flex:0 0 110px}"
        ".input-row input,.input-row select{flex:1}"
        ".input-with-icon{flex:1;position:relative;display:flex;align-items:center;min-width:0}"
        ".input-with-icon input{width:100%;padding-right:44px !important;margin:0}"
        ".input-with-icon .icon-btn{position:absolute;right:4px;top:50%;transform:translateY(-50%);width:36px;height:32px;margin:0;padding:0;background:transparent;color:#6c757d;border:none;border-radius:6px;cursor:pointer;font-size:18px;line-height:1;display:flex;align-items:center;justify-content:center;transition:background .2s}"
        ".input-with-icon .icon-btn:hover{background:#e9ecef}"
        ".input-with-icon .icon-btn:active{background:#dee2e6}"
        "#wifiList{max-height:0;overflow:hidden;transition:max-height .3s ease;padding:0;margin:0}"
        "#wifiList.open{max-height:220px;overflow-y:auto}"
        "ul{list-style:none;padding:0}li{background:#f8f9fa;margin:5px 0;padding:10px;border-radius:5px;display:flex;justify-content:space-between;align-items:center;border:1px solid #ddd}"
        ".mode-tabs{display:flex;gap:8px;margin-bottom:12px}"
        ".mode-tab{flex:1;padding:12px;text-align:center;border-radius:8px;cursor:pointer;border:2px solid #ccc;background:#f8f9fa;font-weight:bold}"
        ".mode-tab.active{border-color:#007bff;background:#e7f3ff;color:#007bff}"
        ".mode-tab.disabled{opacity:.4;cursor:not-allowed}"
        ".accordion{padding:0;overflow:hidden}"
        ".acc-head{display:flex;justify-content:space-between;align-items:center;padding:15px;cursor:pointer;user-select:none;margin:0;font-size:18px;font-weight:bold;transition:background .2s}"
        ".acc-head:hover{background:#f8f9fa}"
        ".acc-head .arrow{font-size:14px;transition:transform .3s;color:#888}"
        ".accordion.open .acc-head .arrow{transform:rotate(180deg)}"
        ".acc-body{max-height:0;overflow:hidden;transition:max-height .35s ease;padding:0 15px}"
        ".accordion.open .acc-body{max-height:9000px;padding:0 15px 15px 15px}"
        ".acc-head.green{color:#28a745;border-bottom:1px solid #e9ecef}"
        ".acc-head.blue{color:#007bff;border-bottom:1px solid #e9ecef}"
        ".acc-head.red{color:#dc3545;border-bottom:1px solid #e9ecef}"
        ".acc-head.dark{color:#444;border-bottom:1px solid #e9ecef}"
        ".vmap-row{display:flex;gap:6px;align-items:center;margin-bottom:8px}"
        ".vmap-row input{padding:6px;font-size:13px;text-align:center;min-width:0}"
        ".vmap-row input.pct{width:52px}"
        ".vmap-row input.volt{width:72px}"
        ".vmap-row button{width:auto;padding:6px 10px;margin:0;font-size:14px;background:#dc3545}"
        ".vmap-head{display:flex;gap:6px;font-size:11px;color:#6c757d;font-weight:bold;margin-bottom:6px;padding:0 2px}"
        ".vmap-head span{text-align:center}"
        ".vmap-head .h1{width:52px}.vmap-head .h2{width:14px}.vmap-head .h3{width:52px}"
        ".vmap-head .h4{width:72px}.vmap-head .h5{width:34px}"
        ".preset-btn{width:100%;padding:12px;margin-bottom:8px;background:#fff;color:#333;border:2px solid #6f42c1;border-radius:10px;text-align:left;cursor:pointer;font-size:15px;transition:all .2s;font-family:inherit}"
        ".preset-btn:hover{background:#f5f0ff;transform:translateY(-1px);box-shadow:0 3px 8px rgba(111,66,193,.25)}"
        ".preset-btn.active{background:#6f42c1;color:#fff;border-color:#28a745;box-shadow:0 0 0 3px rgba(40,167,69,.3)}"
        ".preset-btn .p-title{font-size:16px;font-weight:bold;color:#6f42c1;margin-bottom:2px}"
        ".preset-btn.active .p-title{color:#fff}"
        ".preset-btn .p-sub{font-size:11px;color:#6c757d;margin-bottom:6px}"
        ".preset-btn.active .p-sub{color:#e0d4f7}"
        ".preset-btn .p-ranges{font-size:11px;font-family:monospace;background:#f8f9fa;padding:5px 7px;border-radius:5px;color:#495057;line-height:1.5;word-break:break-word}"
        ".preset-btn.active .p-ranges{background:rgba(255,255,255,.15);color:#fff}"
        ".sub-tabs{display:flex;gap:6px;margin-bottom:12px;background:#f0f2f5;padding:4px;border-radius:8px}"
        ".sub-tab{flex:1;padding:9px;text-align:center;border-radius:6px;cursor:pointer;font-weight:bold;font-size:13px;color:#6c757d;transition:all .2s;user-select:none}"
        ".sub-tab.active{background:#fff;color:#6f42c1;box-shadow:0 1px 3px rgba(0,0,0,.1)}"
        "</style></head><body>"
    );

    server.sendContent(
        "<div class='card' style='border-top:4px solid #17a2b8'>"
        "<h2>MONITOR</h2>"
        "<div style='color:#6c757d;font-size:14px'>Trạng thái: <span id='conn-status' style='font-weight:bold;color:red'>Mất kết nối</span></div>"
        "<div style='color:#6c757d;font-size:12px;margin-top:4px'>AP: <span id='scan-mode' style='font-weight:bold'>--</span></div>"
        "<div style='color:#6c757d;font-size:12px'>LED: <span id='led-mode' style='font-weight:bold'>--</span></div>"
        "<div style='margin-top:10px;display:flex;align-items:center;gap:8px'>"
        "<strong>SOC:</strong><span id='soc-text' style='font-size:18px;font-weight:bold'>0%</span>"
        "<span id='soc-badge' style='font-size:11px;padding:3px 8px;border-radius:10px;background:#999;color:#fff;font-weight:bold'>--</span></div>"
        "<div class='battery-bg'><div id='soc-bar' class='battery-bar'></div></div>"
        "<div id='soc-warning' style='display:none;background:#fff3cd;color:#856404;padding:8px 12px;border-radius:5px;font-size:13px;margin-top:10px'></div>"
        "</div>"
    );

    server.sendContent(
        "<div class='card accordion' id='accScan' style='border:2px solid #007bff'>"
        "<div class='acc-head blue' onclick='toggleAcc(\"accScan\")'>"
        "<span>🔍 Nguồn Dữ Liệu: BMS / ẮC QUY</span><span class='arrow'>▼</span>"
        "</div>"
        "<div class='acc-body'>"

        "<div style='display:flex;justify-content:space-between;align-items:center;background:#fff3cd;"
        "border:1px solid #ffeeba;padding:12px;border-radius:8px;margin-top:12px'>"
        "<div>"
        "<strong style='color:#856404;font-size:15px'>🔋 Chế độ Đo ẮC QUY (Phân áp ADC)</strong><br>"
        "<small style='color:#6c757d;font-size:11px'>Bật chế độ này sẽ TẮT BLE để đọc ADC chân ESP32</small>"
        "</div>"
        "<label style='margin:0;display:flex;align-items:center;cursor:pointer'>"
        "<input type='checkbox' id='acquyToggle' style='width:22px;height:22px' onchange='toggleAcquy(this.checked)'>"
        "</label>"
        "</div>"

        "<div id='acquyConfigBox' style='display:none;background:#f8f9fa;border:1px solid #dee2e6;border-radius:8px;padding:12px;margin-top:10px'>"

        "<div id='ovpAlert' style='display:none;background:#f8d7da;color:#721c24;padding:8px;border-radius:5px;font-size:12px;font-weight:bold;margin-bottom:8px'>"
        "⚠️ CẢNH BÁO: QUÁ ÁP!</div>"

        "<div style='display:grid;grid-template-columns:1fr 1fr;gap:8px;margin-bottom:10px'>"
        "<div style='background:#fff;border:1px solid #dee2e6;border-radius:6px;padding:10px;text-align:center'>"
        "<div style='font-size:11px;color:#6c757d;font-weight:bold'>⚡ TỨC THỜI</div>"
        "<div style='font-size:18px;font-weight:bold;color:#ff9800;margin-top:4px' id='aqRawV'>-- V</div>"
        "<div style='font-size:11px;color:#ff9800;margin-top:2px' id='aqSocRaw'>--%</div>"
        "</div>"
        "<div style='background:#fff;border:1px solid #dee2e6;border-radius:6px;padding:10px;text-align:center'>"
        "<div style='font-size:11px;color:#6c757d;font-weight:bold'>📊 ỔN ĐỊNH</div>"
        "<div style='font-size:18px;font-weight:bold;color:#28a745;margin-top:4px' id='aqStableV'>-- V</div>"
        "<div style='font-size:11px;color:#28a745;margin-top:2px' id='aqSocStable'>--%</div>"
        "</div>"
        "</div>"

        "<div style='font-size:11px;color:#6c757d;margin-bottom:6px;text-align:center'>"
        "ADC: <span id='aqAdcV' style='font-family:monospace'>0.00V</span>"
        " | <b><span id='aqStableStatus'>--</span></b>"
        "</div>"
        "<div style='font-size:11px;color:#6c757d;margin-bottom:10px;text-align:center'>"
        "Hiệu chuẩn: Gain <b><span id='dispGain'>--</span></b>"
        " | Offset <b><span id='dispOffset'>--</span></b> V"
        "</div>"

        "<div class='input-row'><label><strong>Loại bình:</strong></label>"
        "<select id='battType' onchange='onBattTypeChange()'>"
        "<option value='0'>Ắc quy chì</option>"
        "<option value='1'>LFP (LiFePO4)</option>"
        "</select></div>"

        "<div id='leadOptions' style='display:block'>"
        "<div class='input-row'><label><strong>Số bình:</strong></label>"
        "<select id='battCells' onchange='updateBattInfo()'>"
        "<option value='1'>1 bình (12V)</option>"
        "<option value='2'>2 bình (24V)</option>"
        "<option value='3'>3 bình (36V)</option>"
        "<option value='4'>4 bình (48V)</option>"
        "<option value='5'>5 bình (60V)</option>"
        "<option value='6'>6 bình (72V)</option>"
        "</select></div>"
        "</div>"

        "<div id='lfpOptions' style='display:none'>"
        "<div class='input-row'><label><strong>Số cell:</strong></label>"
        "<input type='number' id='battCellsInput' min='1' max='32' value='16' oninput='updateBattInfo()'></div>"
        "<div style='font-size:11px;color:#6c757d;margin-top:-4px;margin-bottom:8px'>"
        "Mặc định: 3.00V – 3.45V/cell, OVP 3.65V/cell</div>"
        "</div>"

        "<div id='battInfoBox' style='background:#e7f3ff;border-left:3px solid #007bff;"
        "padding:8px 10px;border-radius:5px;font-size:11px;margin-bottom:10px;color:#004085'>"
        "<b>Ngưỡng:</b> <span id='battInfoText'>--</span>"
        "</div>"

        "<div class='input-row'><label><strong>Điện trở R1 (kΩ):</strong></label>"
        "<input type='number' id='aqR1' step='0.1' placeholder='Vd: 100' oninput='checkDivider()'></div>"
        "<div class='input-row'><label><strong>Điện trở R2 (kΩ):</strong></label>"
        "<input type='number' id='aqR2' step='0.1' placeholder='Vd: 4.7' oninput='checkDivider()'></div>"
        "<div id='dividerWarn' style='display:none;border-radius:5px;padding:10px;font-size:12px;margin-bottom:10px;line-height:1.6'></div>"

        "<div style='border-top:1px dashed #dee2e6;padding-top:10px;margin-top:10px'>"
        "<div style='font-size:12px;font-weight:bold;color:#007bff;margin-bottom:8px'>🎯 Hiệu chuẩn điện áp (2 điểm)</div>"
        "<div class='input-row'><label><strong>Hệ số nhân:</strong></label>"
        "<input type='number' id='aqGain' step='0.0001' min='0.5' max='2' placeholder='1.0000'></div>"
        "<div class='input-row'><label><strong>Bù lệch (V):</strong></label>"
        "<input type='number' id='aqOffset' step='0.01' min='-5' max='5' placeholder='0.00'></div>"

        "<div style='background:#e7f3ff;border-left:3px solid #007bff;padding:8px 10px;"
        "border-radius:5px;font-size:11px;margin-bottom:10px;color:#004085;line-height:1.6'>"
        "<b>🎯 Cách hiệu chuẩn:</b><br>"
        "• <b>1 điểm:</b> Gain = V_Thực / V_Đo<br>"
        "• <b>2 điểm:</b> Gain = (V₂_Thực − V₁_Thực) / (V₂_Đo − V₁_Đo)<br>"
        "&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;Offset = V₁_Thực − Gain × V₁_Đo"
        "</div>"

        "<div style='background:#fff3cd;border:1px solid #ffeeba;border-radius:5px;padding:10px;margin-bottom:10px'>"
        "<div style='font-size:12px;font-weight:bold;color:#856404;margin-bottom:8px'>⚡ Hiệu chuẩn nhanh (1 điểm)</div>"
        "<div class='input-row' style='margin:0 0 8px 0'>"
        "<label style='font-size:12px'><strong>V thực tế:</strong></label>"
        "<input type='number' id='calRefV' step='0.1' placeholder='Đo bằng VOM'></div>"
        "<button class='btn-warn' style='margin:0;font-size:13px;padding:8px' onclick='quickCalibrate()'>🎯 Tính Gain từ V_Đo → V_Thực</button>"
        "<div id='calMsg' style='font-size:11px;margin-top:6px'></div>"
        "</div>"
        "</div>"

        "<button class='btn-warn' style='margin-top:8px' onclick='saveAcquyCfg()'>💾 Lưu thông số Ắc quy</button>"
        "</div>"

        "<div id='bmsScanArea' style='margin-top:12px'>"
        "<div style='display:flex;justify-content:space-between;align-items:center;gap:8px'>"
        "<div style='flex:1;min-width:0'>"
        "<div style='font-size:13px'><strong>MAC hiện tại:</strong></div>"
        "<div id='curMacText' style='font-family:monospace;font-size:12px;color:#007bff;word-break:break-all;margin-top:2px'>"
        + (targetMac == "" ? String("Chưa có") : targetMac) + "</div>"
        "<div id='curBmsName' style='font-size:11px;color:#6c757d;margin-top:2px'>"
        + (bmsDeviceName.length() > 0 ? ("📛 " + bmsDeviceName) : "") + "</div>"
        "</div>"
        "<button id='btnClearMac' onclick='clearMac()' style='width:auto;margin:0;padding:8px 12px;font-size:12px;background:#dc3545'>🗑️ Xóa MAC</button>"
        "</div>"
        "<div id='autoConnectHint' style='display:none;background:#fff3cd;color:#856404;padding:8px 12px;border-radius:5px;font-size:12px;margin-top:10px'>"
        "⏳ Đang chờ BMS — sẽ tự kết nối khi chỉ tìm thấy 1 thiết bị</div>"
        "<div class='sub-tabs'>"
        "<div class='sub-tab active' id='subTabScan' onclick='switchMacTab(\"scan\")'>📡 Quét Tự Động</div>"
        "<div class='sub-tab' id='subTabManualMac' onclick='switchMacTab(\"manual\")'>⌨️ Nhập Thủ Công</div>"
        "</div>"
        "<div id='macSubScan'>"
        "<button onclick='scan()'>🔍 Bắt đầu quét Bluetooth</button>"
        "<div id='status' style='color:red;margin-top:10px'></div>"
        "<ul id='results'></ul>"
        "</div>"
        "<div id='macSubManual' style='display:none'>"
        "<div style='background:#f8f9fa;border:1px solid #dee2e6;border-radius:8px;padding:12px;margin-bottom:12px'>"
        "<div style='font-size:13px;font-weight:bold;color:#007bff;margin-bottom:8px'>⌨️ Nhập MAC thủ công</div>"
        "<div style='font-size:11px;color:#6c757d;margin-bottom:8px'>Định dạng: <code>AA:BB:CC:DD:EE:FF</code></div>"
        "<input type='text' id='manualMac' placeholder='AA:BB:CC:DD:EE:FF' maxlength='17' style='font-family:monospace;text-transform:uppercase;letter-spacing:1px'>"
        "<button class='btn-success' style='margin-top:8px' onclick='saveManualMac()'>💾 Lưu & Kết nối</button>"
        "<div id='manualMacMsg' style='font-size:12px;margin-top:6px'></div>"
        "</div>"
        "</div>"
        "</div>"

        "</div></div>"
    );

    server.sendContent(
        "<div class='card accordion' id='accBms' style='border:2px solid #17a2b8'>"
        "<div class='acc-head' id='accBmsHead' style='color:#17a2b8;border-bottom:1px solid #e9ecef' onclick='toggleAcc(\"accBms\")'>"
        "<span id='accBmsTitle'>🔋 Chi Tiết Điện Áp / BMS</span><span class='arrow'>▼</span>"
        "</div>"
        "<div class='acc-body'>"
        "<div id='bms-name-box' style='background:#e7f3ff;border-left:3px solid #17a2b8;padding:8px 12px;border-radius:5px;font-size:13px;margin:12px 0 8px 0'>"
        "📛 <b>Thiết bị:</b> <span id='bms-name-text'>--</span>"
        "<span id='bms-type-badge' style='display:none;font-size:10px;font-weight:bold;padding:2px 8px;border-radius:10px;margin-left:6px;vertical-align:middle'></span>"
        "</div>"
        "<div id='bms-empty' style='display:none;text-align:center;padding:20px;color:#888;font-size:13px'>📭 Chưa có dữ liệu chi tiết</div>"
        "<div id='bms-overview' style='display:grid;grid-template-columns:1fr 1fr;gap:8px;margin:12px 0'>"
        "<div class='stat-box' id='bms-box-totalv' style='display:none'>"
        "<div>Điện áp tổng</div>"
        "<div class='stat-val' id='bms-totalv' style='color:#007bff'>-- V</div></div>"
        "<div class='stat-box' id='aq-box-raw' style='display:none'>"
        "<div>⚡ Điện áp tức thời</div>"
        "<div class='stat-val' id='aq-totalv-raw' style='color:#ff9800'>-- V</div></div>"
        "<div class='stat-box' id='aq-box-stable' style='display:none'>"
        "<div>📊 Điện áp ổn định</div>"
        "<div class='stat-val' id='aq-totalv-stable' style='color:#28a745'>-- V</div></div>"
        "<div class='stat-box' id='bms-box-cellcount' style='display:none'>"
        "<div>Số cell</div>"
        "<div class='stat-val' id='bms-cellcount' style='color:#17a2b8'>--</div></div>"
        "<div class='stat-box' id='bms-box-current' style='display:none'>"
        "<div>Dòng điện</div>"
        "<div class='stat-val' id='bms-current' style='color:#ff9800'>-- A</div></div>"
        "<div class='stat-box' id='bms-box-power' style='display:none'>"
        "<div>Công suất</div>"
        "<div class='stat-val' id='bms-power' style='color:#6f42c1'>-- W</div></div>"
        "</div>"
        "<div id='aq-status-line' style='display:none;background:#f8f9fa;border:1px solid #dee2e6;"
        "border-radius:6px;padding:8px 12px;font-size:12px;color:#495057;margin-bottom:10px'>"
        "<div style='display:flex;justify-content:space-between;align-items:center;margin-bottom:6px'>"
        "<span>📌 <b>Chân ADC:</b> <span id='bms-aq-pin-label' style='font-family:monospace;color:#007bff'>GPIO--</span></span>"
        "<span><b style='font-family:monospace' id='bms-aq-adc-v'>0.000 V</b></span>"
        "</div>"
        "<div style='display:flex;justify-content:space-between;align-items:center;margin-bottom:6px'>"
        "<span>Trạng thái: <b><span id='bms-aq-status-text'>--</span></b></span>"
        "<span id='bms-aq-ovp-badge' style='display:none;background:#f8d7da;color:#721c24;"
        "padding:2px 8px;border-radius:10px;font-size:10px;font-weight:bold'>⚠️ QUÁ ÁP</span>"
        "</div>"
        "<div style='display:flex;justify-content:space-between;align-items:center;font-size:11px;color:#6c757d'>"
        "<span>Hiệu chuẩn: Gain <b style='color:#007bff'><span id='bms-aq-gain'>--</span></b></span>"
        "<span>Offset <b style='color:#007bff'><span id='bms-aq-offset'>--</span></b> V</span>"
        "</div>"
        "</div>"
        "<div id='bms-cellstats' style='display:none;background:#f8f9fa;border:1px solid #dee2e6;border-radius:8px;padding:10px;margin-bottom:10px;justify-content:space-around;font-size:13px'>"
        "<div style='text-align:center'><div style='color:#6c757d;font-size:11px;font-weight:bold'>CELL MAX</div>"
        "<div id='bms-cellmax' style='color:#dc3545;font-weight:bold;font-size:15px;margin-top:3px'>--</div></div>"
        "<div style='text-align:center'><div style='color:#6c757d;font-size:11px;font-weight:bold'>CELL MIN</div>"
        "<div id='bms-cellmin' style='color:#28a745;font-weight:bold;font-size:15px;margin-top:3px'>--</div></div>"
        "<div style='text-align:center'><div style='color:#6c757d;font-size:11px;font-weight:bold'>CHÊNH LỆCH</div>"
        "<div id='bms-celldiff' style='color:#ff9800;font-weight:bold;font-size:15px;margin-top:3px'>--</div></div>"
        "</div>"
        "<div id='bms-cells-wrap' style='display:none'>"
        "<div style='font-size:12px;font-weight:bold;color:#495057;margin-bottom:6px'>📊 Điện áp từng cell (V)</div>"
        "<div id='bms-cells-grid' style='display:grid;grid-template-columns:repeat(4,1fr);gap:5px'></div>"
        "</div>"
        "</div></div>"
    );

    server.sendContent(
        "<div class='card accordion' id='accTerm' style='border:2px solid #444'>"
        "<div class='acc-head dark' onclick='toggleAcc(\"accTerm\")'>"
        "<span>💻 Terminal</span><span class='arrow'>▼</span>"
        "</div>"
        "<div class='acc-body'>"
        "<div style='display:flex;gap:8px;margin:12px 0 8px 0'>"
        "<button style='margin:0;flex:1;padding:6px;font-size:13px;background:#444' onclick='clearTerm()'>Xóa</button>"
        "<button style='margin:0;flex:1;padding:6px;font-size:13px;background:#444' onclick='toggleTermPause()' id='termBtn'>Tạm dừng</button>"
        "</div>"
        "<div id='terminal' style='background:#1e1e1e;color:#0f0;padding:10px;border-radius:5px;font-family:monospace;font-size:12px;white-space:pre-wrap;max-height:250px;overflow-y:auto'>-- Chờ Serial --\n</div>"
        "</div></div>"
    );

    server.sendContent(
        "<div class='card accordion' id='accChip' style='border:2px solid #6c757d'>"
        "<div class='acc-head dark' onclick='toggleAcc(\"accChip\")'>"
        "<span>💻 Thông tin Chip</span><span class='arrow'>▼</span>"
        "</div>"
        "<div class='acc-body'>"
        "<div style='font-size:12px;color:#495057;margin:12px 0 10px 0;text-align:center'>"
        "Model: <b id='chipModel'>--</b> | Rev: <b id='chipRev'>--</b> | Core: <b id='chipCores'>--</b> | <b id='chipFreq'>--</b> MHz</div>"
        "<div style='background:#f8f9fa;border:1px solid #dee2e6;border-radius:8px;padding:10px;margin-bottom:10px'>"
        "<div style='display:flex;justify-content:space-between;font-size:12px;font-weight:bold;color:#495057;margin-bottom:6px'>"
        "<span>🧠 RAM</span><span><span id='heapPct'>--</span>% dùng</span></div>"
        "<div class='battery-bg' style='height:14px;margin:0 0 8px 0'><div id='heapBar' class='battery-bar' style='width:0%;background:#007bff;height:100%'></div></div>"
        "<div style='display:grid;grid-template-columns:1fr 1fr;gap:6px;font-size:11px;color:#6c757d'>"
        "<div>Tổng: <b style='color:#333'><span id='heapTotal'>--</span> KB</b></div>"
        "<div>Đã dùng: <b style='color:#dc3545'><span id='heapUsed'>--</span> KB</b></div>"
        "<div>Trống: <b style='color:#28a745'><span id='heapFree'>--</span> KB</b></div>"
        "<div>Còn (min): <b style='color:#ff9800'><span id='heapMin'>--</span> KB</b></div>"
        "</div></div>"
        "<div id='psramBox' style='display:none;background:#f8f9fa;border:1px solid #dee2e6;border-radius:8px;padding:10px;margin-bottom:10px'>"
        "<div style='display:flex;justify-content:space-between;font-size:12px;font-weight:bold;color:#495057;margin-bottom:6px'>"
        "<span>💾 PSRAM</span><span><span id='psramPct'>--</span>%</span></div>"
        "<div class='battery-bg' style='height:14px;margin:0 0 8px 0'><div id='psramBar' class='battery-bar' style='width:0%;background:#6f42c1;height:100%'></div></div>"
        "<div style='display:grid;grid-template-columns:1fr 1fr;gap:6px;font-size:11px;color:#6c757d'>"
        "<div>Tổng: <b style='color:#333'><span id='psramTotal'>--</span> KB</b></div>"
        "<div>Trống: <b style='color:#28a745'><span id='psramFree'>--</span> KB</b></div>"
        "</div></div>"
        "<div style='background:#f8f9fa;border:1px solid #dee2e6;border-radius:8px;padding:10px;margin-bottom:10px'>"
        "<div style='display:flex;justify-content:space-between;font-size:12px;font-weight:bold;color:#495057;margin-bottom:6px'>"
        "<span>💿 Flash</span><span><span id='flashPct'>--</span>%</span></div>"
        "<div class='battery-bg' style='height:14px;margin:0 0 8px 0'><div id='flashBar' class='battery-bar' style='width:0%;background:#ff9800;height:100%'></div></div>"
        "<div style='display:grid;grid-template-columns:1fr 1fr;gap:6px;font-size:11px;color:#6c757d'>"
        "<div>Tổng: <b style='color:#333'><span id='flashTotal'>--</span> KB</b></div>"
        "<div>Firmware: <b style='color:#dc3545'><span id='sketchSize'>--</span> KB</b></div>"
        "<div>Còn trống: <b style='color:#28a745'><span id='sketchFree'>--</span> KB</b></div>"
        "<div>Uptime: <b style='color:#17a2b8'><span id='uptime'>--</span></b></div>"
        "</div></div>"
        "<div style='background:#f8f9fa;border:1px solid #dee2e6;border-radius:8px;padding:10px'>"
        "<div style='display:flex;justify-content:space-between;font-size:12px;font-weight:bold;color:#495057;margin-bottom:6px'>"
        "<span>⚙️ CPU</span><span><span id='cpuLoad'>--</span>% tải</span></div>"
        "<div class='battery-bg' style='height:14px;margin:0 0 8px 0'><div id='cpuBar' class='battery-bar' style='width:0%;background:#dc3545;height:100%'></div></div>"
        "<div style='display:grid;grid-template-columns:1fr 1fr;gap:6px;font-size:11px;color:#6c757d'>"
        "<div>Tần số: <b style='color:#333'><span id='cpuFreq'>--</span> MHz</b></div>"
        "<div>Số core: <b style='color:#333'><span id='coreCount'>--</span></b></div>"
        "</div>"
        "<div id='chipExtraInfo' style='margin-top:8px;font-size:11px;color:#6c757d'></div>"
        "</div></div></div>"
    );

    server.sendContent(
        "<div class='card accordion' id='accOutput' style='border:2px solid #28a745'>"
        "<div class='acc-head green' onclick='toggleAcc(\"accOutput\")'>"
        "<span>⚡ Output Analog</span><span class='arrow'>▼</span>"
        "</div>"
        "<div class='acc-body'>"
        "<div style='text-align:center;font-size:20px;font-weight:bold;color:#28a745;margin-top:12px' id='vol-text'>0.000 V</div>"
        "<div style='text-align:center;font-size:12px;color:#6c757d;margin-top:4px'>Chip: <span id='chip-name'>--</span></div>"
        "<div style='margin-top:14px;padding:12px;background:#f8f9fa;border-radius:8px;border:1px solid #e9ecef'>"
        "<div style='display:flex;justify-content:space-between;align-items:center;margin-bottom:4px'>"
        "<label style='margin:0;font-size:13px'><strong>🎚️ Giả lập % pin:</strong></label>"
        "<span id='slider-val' style='font-size:16px;font-weight:bold;color:#ff9800'>50%</span>"
        "</div>"
        "<input type='range' id='simSlider' min='0' max='100' value='50' oninput='sendSim(this.value)'>"
        "<button id='realBtn' class='btn-success' style='margin-top:4px' onclick='setRealSoc()'>Đang chạy Pin Thực</button>"
        "</div>"
        "<div style='display:flex;justify-content:space-between;align-items:center;margin:16px 0 10px 0'>"
        "<label style='margin:0'><strong>📋 Bảng điện áp theo %</strong></label>"
        "<label style='margin:0;display:flex;align-items:center;gap:6px;cursor:pointer;font-size:13px'>"
        "<input type='checkbox' id='vmapEn' style='width:auto;margin:0' onchange='onVmapToggle()'>"
        "<span>Bật</span></label>"
        "</div>"
        "<div id='vmapBox' style='opacity:0.5;pointer-events:none'>"
        "<div class='sub-tabs'>"
        "<div class='sub-tab active' id='subTabPreset' onclick='switchSubTab(\"preset\")'>⚡ Có sẵn</div>"
        "<div class='sub-tab' id='subTabManual' onclick='switchSubTab(\"manual\")'>✏️ Thủ công</div>"
        "</div>"
        "<div id='subPreset'>"
        "<div style='display:flex;justify-content:space-between;align-items:center;margin-bottom:8px'>"
        "<div style='font-size:11px;color:#6c757d'>Bấm vào tên để <b>áp dụng ngay</b></div>"
        "<button id='btnFetchPresets' onclick='fetchPresets()' style='width:auto;margin:0;padding:6px 12px;font-size:11px;background:#6f42c1;display:none'>🔄 Cập nhật</button>"
        "</div>"
        "<div id='presetUpdatedBox' style='display:none;background:#e7f3ff;border-left:3px solid #007bff;padding:6px 10px;border-radius:5px;font-size:11px;margin-bottom:8px'>"
        "📅 Cập nhật: <b><span id='presetUpdatedText'>--</span></b></div>"
        "<div id='presetList'></div>"
        "<div id='presetListMsg' style='font-size:11px;color:#6c757d;text-align:center;margin-top:8px;display:none'></div>"
        "</div>"
        "<div id='subManual' style='display:none'>"
        "<div style='font-size:11px;color:#6c757d;margin-bottom:8px'>Mỗi dải gán 1 mức V cố định</div>"
        "<div class='vmap-head'><span class='h1'>Từ %</span><span class='h2'></span><span class='h3'>Đến %</span><span class='h4'>Điện áp (V)</span><span class='h5'></span></div>"
        "<div id='vmapList'></div>"
        "<button onclick='addVmapRow()' style='background:#17a2b8;margin-top:8px'>+ Thêm dải</button>"
        "<button class='btn-success' onclick='saveVmap()'>Lưu bảng</button>"
        "</div>"
        "</div>"
        "<hr>"
    );

    server.sendContent("<div class='mode-tabs' style='margin-top:5px'>");
#if HAS_DAC
    server.sendContent("<div class='mode-tab" + String(outMode == OUT_MODE_DAC ? " active" : "") + "' id='tabDac' onclick='selectMode(0)'>DAC</div>");
#else
    server.sendContent("<div class='mode-tab disabled' id='tabDac' onclick='selectMode(0)'>DAC (N/A)</div>");
#endif
    server.sendContent("<div class='mode-tab" + String(outMode == OUT_MODE_PWM ? " active" : "") + "' id='tabPwm' onclick='selectMode(1)'>PWM</div></div>");

    server.sendContent(
        "<div id='dacCfg' style='display:" + String((outMode == OUT_MODE_DAC && HAS_DAC) ? "block" : "none") + "'>"
        "<div class='input-row'><label><strong>Chân DAC:</strong></label><select id='dacPinSel'>"
        "<option value='25'" + String(dacPin == 25 ? " selected" : "") + ">GPIO 25</option>"
        "<option value='26'" + String(dacPin == 26 ? " selected" : "") + ">GPIO 26</option>"
        "</select></div></div>"
    );

    server.sendContent(
        "<div id='acquyAdcPinBox' style='display:none;background:#e7f3ff;border:1px solid #b8daff;border-radius:8px;padding:12px;margin-top:12px'>"
        "<div style='font-size:13px;font-weight:bold;color:#004085;margin-bottom:8px'>📌 Chân ADC đo Ắc quy</div>"
        "<div class='input-row' style='margin:0'><label><strong>GPIO:</strong></label>"
        "<input type='number' id='aqPin' min='0' max='39' value='32'></div>"
        "<div style='font-size:11px;color:#6c757d;margin-top:6px'>Cấu hình cầu phân áp & hệ bình xem ở mục <b>Nguồn Dữ Liệu</b></div>"
        "</div>"
    );

    server.sendContent(
        "<div id='pwmCfg' style='display:" + String(outMode == OUT_MODE_PWM ? "block" : "none") + "'>"
        "<div class='input-row'><label><strong>Chân PWM:</strong></label><input type='number' id='pwmPinIn' min='0' max='48' value='" + String(pwmPin) + "'></div>"
        "<div class='input-row'><label><strong>Tần số:</strong></label><input type='number' id='pwmFreqIn' value='" + String(pwmFreq) + "'></div>"
        "<div class='input-row'><label><strong>Độ phân giải:</strong></label><select id='pwmResIn'>"
        "<option value='8'" + String(pwmRes == 8 ? " selected" : "") + ">8-bit</option>"
        "<option value='10'" + String(pwmRes == 10 ? " selected" : "") + ">10-bit</option>"
        "<option value='12'" + String(pwmRes == 12 ? " selected" : "") + ">12-bit</option>"
        "</select></div></div>"
        "<button class='btn-warn' style='margin-top:12px' onclick='saveOutput()'>Lưu Output</button>"
        "<div style='font-size:12px;color:#6c757d;margin-top:8px;text-align:center'>Hiện tại: <span id='out-current'>--</span></div>"
        "</div></div>"
    );

    // ✅ WiFi: nút 📡 và 👁️ NẰM TRONG ô input
    server.sendContent(
        "<div class='card accordion' id='accWifi' style='border:2px solid #6f42c1'>"
        "<div class='acc-head' style='color:#6f42c1;border-bottom:1px solid #e9ecef' onclick='toggleAcc(\"accWifi\")'>"
        "<span>📶 WiFi Nhà (STA)</span><span class='arrow'>▼</span>"
        "</div>"
        "<div class='acc-body'>"
        "<div style='background:#e7f3ff;border-left:3px solid #007bff;padding:8px 10px;border-radius:5px;font-size:11px;margin:12px 0 10px 0;color:#004085'>"
        "ℹ️ AP luôn chạy song song. STA chỉ thử 15s — không được thì tắt.</div>"
        "<div style='display:flex;justify-content:space-between;align-items:center;margin-bottom:8px'>"
        "<span style='font-size:13px'>Trạng thái: <b><span id='sta-status'>--</span></b></span>"
        "<label style='margin:0;display:flex;align-items:center;gap:6px;cursor:pointer;font-size:13px'>"
        "<input type='checkbox' id='staEn' style='width:auto;margin:0'><span>Bật</span></label>"
        "</div>"
        "<div id='staInfoBox' style='display:none;background:#e7f3ff;border-left:3px solid #007bff;padding:8px 10px;border-radius:5px;font-size:12px;margin-bottom:10px'>"
        "IP: <b><span id='sta-ip'>--</span></b> | RSSI: <span id='sta-rssi'>--</span> dBm</div>"

        "<div class='input-row' style='align-items:stretch'><label><strong>SSID:</strong></label>"
        "<div class='input-with-icon'>"
        "<input type='text' id='staSSID' maxlength='32' placeholder='Tên WiFi'>"
        "<button class='icon-btn' onclick='scanWifi()' title='Quét WiFi xung quanh'>📡</button>"
        "</div></div>"

        "<div class='input-row' style='align-items:stretch'><label><strong>Mật khẩu:</strong></label>"
        "<div class='input-with-icon'>"
        "<input type='password' id='staPass' maxlength='64' placeholder='(để trống nếu mở)'>"
        "<button class='icon-btn' id='btnTogglePass' onclick='togglePassVisibility()' title='Hiện/Ẩn mật khẩu'>👁️</button>"
        "</div></div>"

        "<div style='display:flex;justify-content:space-between;align-items:center;margin-bottom:8px'>"
        "<div id='wifiStatus' style='color:#6c757d;font-size:12px'></div>"
        "<button onclick='closeWifiList()' id='btnCloseWifiList' "
        "style='width:auto;margin:0;padding:4px 10px;font-size:11px;background:#dc3545;display:none'>✕ Đóng</button>"
        "</div>"
        "<ul id='wifiList'></ul>"

        "<button class='btn-success' style='margin-top:8px' onclick='saveWifi()'>💾 Lưu WiFi</button>"
        "<button style='margin-top:6px;background:#17a2b8' onclick='forceAp()'>📡 Bật lại AP ngay</button>"
        "<div id='wifiMsg' style='font-size:12px;margin-top:6px'></div>"
        "</div></div>"
    );

    server.sendContent(
        "<div class='card accordion' id='accOta' style='border:2px solid #dc3545'>"
        "<div class='acc-head red' onclick='toggleAcc(\"accOta\")'>"
        "<span>⬆️ Cập nhật Firmware</span><span class='arrow'>▼</span>"
        "</div>"
        "<div class='acc-body'>"
        "<div style='background:#f8f9fa;border:1px solid #dee2e6;border-radius:8px;padding:12px;margin:12px 0'>"
        "<div style='display:flex;justify-content:space-between;align-items:center;margin-bottom:6px'>"
        "<div style='font-size:13px;font-weight:bold;color:#495057'>📌 Firmware hiện tại</div>"
        "<label style='margin:0;display:flex;align-items:center;gap:8px;cursor:pointer;font-size:12px'>"
        "<span id='autoUpdateStatus' style='font-size:10px;color:#6c757d'>Tắt</span>"
        "<span style='font-weight:bold;color:#28a745'>🔄 Auto</span>"
        "<span style='position:relative;display:inline-block;width:44px;height:24px'>"
        "<input type='checkbox' id='autoUpdateToggle' style='opacity:0;width:0;height:0' onchange='toggleAutoUpdate(this.checked)'>"
        "<span id='autoUpdateSlider' style='position:absolute;cursor:pointer;top:0;left:0;right:0;bottom:0;background:#ccc;transition:.3s;border-radius:24px'>"
        "<span id='autoUpdateKnob' style='position:absolute;height:18px;width:18px;left:3px;bottom:3px;background:white;transition:.3s;border-radius:50%'></span>"
        "</span></span></label>"
        "</div>"
        "<div style='font-size:15px;font-weight:bold;color:#007bff;margin-bottom:4px'>v" + String(FW_VERSION) + "</div>"
        "<div style='display:flex;justify-content:space-between;align-items:center;margin-bottom:8px'>"
        "<div style='font-size:11px;color:#6c757d'>📅 Ngày: <b><span id='fwDateView'>"
        + (fwDate.length() > 0 ? fwDate : "Chưa cập nhật") + "</span></b></div>"
        "<button onclick='toggleFwEdit()' style='width:auto;margin:0;padding:3px 8px;font-size:10px;background:#6c757d'>✏️ Sửa</button>"
        "</div>"
        "<div id='fwNotesView' style='font-size:12px;color:#333;background:#fff;padding:8px 10px;border-radius:5px;border:1px solid #e9ecef;line-height:1.5;white-space:pre-wrap'>"
        + (fwNotes.length() > 0 ? fwNotes : "Không có mô tả") + "</div>"
        "<div id='fwEditForm' style='display:none;margin-top:10px;padding-top:10px;border-top:1px dashed #dee2e6'>"
        "<div style='font-size:12px;color:#007bff;font-weight:bold;margin-bottom:6px'>✏️ Sửa thông tin</div>"
        "<div class='input-row' style='margin-bottom:8px'>"
        "<label style='flex:0 0 60px;font-size:12px'><strong>Ngày:</strong></label>"
        "<input type='text' id='fwDateInput' placeholder='05/10/2026' style='font-size:12px;padding:6px' value='"
        + (fwDate.length() > 0 ? fwDate : "") + "'></div>"
        "<div style='font-size:12px;font-weight:bold;margin-bottom:4px'>Mô tả:</div>"
        "<textarea id='fwNotesInput' rows='4' style='width:100%;padding:8px;border:1px solid #ccc;border-radius:5px;font-size:12px;font-family:inherit;box-sizing:border-box;resize:vertical' placeholder='Mô tả firmware...'>"
        + (fwNotes.length() > 0 ? fwNotes : "") + "</textarea>"
        "<div style='display:flex;gap:6px;margin-top:8px'>"
        "<button style='flex:1;margin:0;background:#6c757d' onclick='toggleFwEdit()'>Hủy</button>"
        "<button class='btn-success' style='flex:1;margin:0' onclick='saveFwInfo()'>💾 Lưu</button>"
        "</div>"
        "<div id='fwInfoMsg' style='font-size:11px;margin-top:6px'></div>"
        "</div></div>"
        "<div id='otaOnlineBox' style='display:none;background:#fff8e1;border:1px solid #ffcc80;border-radius:8px;padding:12px;margin:12px 0'>"
        "<div style='font-size:13px;font-weight:bold;color:#e65100;margin-bottom:8px'>🌐 Cập nhật Online</div>"
        "<button style='margin:0;background:#ff9800' onclick='checkUpdate()'>🔍 Kiểm tra phiên bản</button>"
        "<div id='updateInfoBox' style='display:none;margin-top:10px;background:#fff;border-radius:6px;padding:10px;border:1px solid #ffcc80'>"
        "<div style='font-size:13px;color:#e65100;font-weight:bold;margin-bottom:6px'>🆕 <span id='newVersionText'>--</span></div>"
        "<div style='font-size:11px;color:#6c757d;margin-bottom:6px'>📅 <span id='newDateText'>--</span></div>"
        "<div style='font-size:12px;color:#333;background:#f8f9fa;padding:8px;border-radius:5px;margin-bottom:10px;line-height:1.5' id='notesText'>--</div>"
        "<button id='btnOtaOnline' class='btn-danger' style='margin:0;background:#dc3545;font-weight:bold' onclick='otaOnline()'>🚀 Nạp Firmware Online</button>"
        "</div>"
        "<div id='onlineMsg' style='font-size:12px;margin-top:8px'></div>"
        "<div class='battery-bg' style='margin-top:10px'><div id='onlineProgress' class='battery-bar' style='width:0%;background:#ff9800'></div></div>"
        "</div>"
        "<div id='otaLocalBox' style='background:#fff5f5;border:1px solid #f5c2c7;border-radius:8px;padding:12px;margin:12px 0'>"
        "<div style='font-size:13px;font-weight:bold;color:#dc3545;margin-bottom:6px'>📦 Nạp từ file (.bin)</div>"
        "<input type='file' id='fwFile' accept='.bin' style='width:100%;padding:8px;border:1px solid #ccc;border-radius:5px;box-sizing:border-box'>"
        "<button class='btn-danger' style='margin-top:10px' onclick='uploadFirmware()'>🚀 Nạp Firmware</button>"
        "<div class='battery-bg' style='margin-top:12px'><div id='fwProgress' class='battery-bar' style='width:0%;background:#dc3545'></div></div>"
        "<div id='fwStatus' style='text-align:center;font-size:13px;margin-top:8px;color:#6c757d'>Chưa nạp</div>"
        "</div></div></div>"
    );

    // ============ JAVASCRIPT ============
    server.sendContent(
        "<script>"
        "function toggleAcc(id){"
        "let el=document.getElementById(id);"
        "if(!el)return;"
        "let wasOpen=el.classList.contains('open');"
        "document.querySelectorAll('.accordion.open').forEach(function(a){a.classList.remove('open');});"
        "if(!wasOpen){el.classList.add('open');onAccOpen(id);}"
        "}"
        "function onAccOpen(id){"
        "if(id==='accChip')updateChipInfo();"
        "else if(id==='accTerm')pollSerial();"
        "else if(id==='accBms')updateBmsInfo();"
        "else if(id==='accScan'){setTimeout(checkDivider,100);}"
        "else if(id==='accOutput'){updateData();"
        "if(!window.presetLoaded){window.presetLoaded=true;loadPresets();}}"
        "else if(id==='accOta'){"
        "fetchTO('/data?t='+Date.now(),5000).then(r=>r.json()).then(d=>{"
        "if(d.staConnected&&!window.autoChecked){window.autoChecked=true;checkUpdate();}}).catch(e=>{});"
        "}}"
        "function fetchTO(url,ms){return Promise.race([fetch(url),"
        "new Promise((_,rj)=>setTimeout(()=>rj(new Error('timeout')),ms))]);}"

        "let isManualSim=false,terminalData='',termPaused=false;"
        "let currentMode=" + String(outMode) + ";"
        "let hasDac=" + String(HAS_DAC ? "true" : "false") + ";"
        "let vmapInited=false;"
        "let activePresetId=-1;"
        "let activePresetName='';"
        "const MAX_TERM=8000;"

        "function appendTerminal(t){if(termPaused)return;terminalData+=t;"
        "if(terminalData.length>MAX_TERM)terminalData=terminalData.slice(-MAX_TERM);"
        "let e=document.getElementById('terminal');e.innerText=terminalData;e.scrollTop=e.scrollHeight;}"
        "function pollSerial(){if(termPaused)return;"
        "let acc=document.getElementById('accTerm');"
        "if(!acc||!acc.classList.contains('open'))return;"
        "fetchTO('/serial?t='+Date.now(),4000).then(r=>r.text()).then(t=>{if(t&&t.length>0)appendTerminal(t)}).catch(e=>{});}"
        "function clearTerm(){terminalData='';document.getElementById('terminal').innerText='';}"
        "function toggleTermPause(){termPaused=!termPaused;"
        "document.getElementById('termBtn').innerText=termPaused?'Tiếp tục':'Tạm dừng';}"
        "setInterval(pollSerial,1500);"

        "function selectMode(m){if(m===0&&!hasDac){alert('Không có DAC');return;}"
        "currentMode=m;"
        "document.getElementById('tabDac').className='mode-tab'+(m===0?' active':'')+(hasDac?'':' disabled');"
        "document.getElementById('tabPwm').className='mode-tab'+(m===1?' active':'');"
        "document.getElementById('dacCfg').style.display=(m===0&&hasDac)?'block':'none';"
        "document.getElementById('pwmCfg').style.display=(m===1)?'block':'none';}"

        "function saveOutput(){let p='mode='+currentMode;"
        "if(currentMode===0){p+='&dacpin='+document.getElementById('dacPinSel').value;}"
        "else{p+='&pwmpin='+document.getElementById('pwmPinIn').value;"
        "p+='&freq='+document.getElementById('pwmFreqIn').value;"
        "p+='&res='+document.getElementById('pwmResIn').value;}"
        "if(!confirm('Lưu?'))return;"
        "fetchTO('/saveout?'+p+'&t='+Date.now(),5000).then(r=>r.text()).then(t=>{"
        "if(t==='OK')alert('Đã lưu!');else alert('Lỗi: '+t);}).catch(e=>alert('Timeout'));}"

        "function updateData(){"
        "fetchTO('/data?t='+Date.now(),5000).then(r=>r.json()).then(d=>{"
        "let s=document.getElementById('conn-status');"
        "if(d.acquyMode){s.innerText='Đang đo Ắc quy (BLE tắt)';s.style.color='#ff9800';}"
        "else if(d.connected){s.innerText='Đã kết nối';s.style.color='green';}"
        "else{s.innerText='Đang tìm...';s.style.color='red';}"

        "let sm=document.getElementById('scan-mode');"
        "if(d.configMode){"
        "if(d.apClients>0){sm.innerText='AP: '+d.apClients+' client';sm.style.color='#28a745';}"
        "else if(d.apRemain===-2){sm.innerText='AP: luôn mở (chưa có STA)';sm.style.color='#17a2b8';}"
        "else if(d.apRemain>=0){sm.innerText='AP tự tắt sau: '+d.apRemain+'s';sm.style.color=d.apRemain<15?'#dc3545':'#ff9800';}"
        "else{sm.innerText='AP đang bật';sm.style.color='#17a2b8';}"
        "}else if(d.staConnected){sm.innerText='AP tắt (STA: '+d.staIP+')';sm.style.color='#6f42c1';}"
        "else{sm.innerText='AP đã tắt';sm.style.color='#6c757d';}"

        "let lm=document.getElementById('led-mode');"
        "if(d.ledMode===0){lm.innerText='Nháy';lm.style.color='#dc3545';}"
        "else if(d.ledMode===1){lm.innerText='Sáng';lm.style.color='#28a745';}"
        "else{lm.innerText='Chớp';lm.style.color='#17a2b8';}"

        "document.getElementById('soc-text').innerText=d.soc+'%';"
        "document.getElementById('soc-bar').style.width=d.soc+'%';"

        "let b=document.getElementById('soc-badge');let w=document.getElementById('soc-warning');"
        "if(d.manualSim){b.innerText='GIẢ LẬP';b.style.background='#ff9800';w.style.display='none';}"
        "else if(d.acquyMode){b.innerText='ẮC QUY';b.style.background='#17a2b8';w.style.display='none';}"
        "else if(!d.connected){b.innerText='CŨ('+d.savedSoc+'%)';b.style.background='#6c757d';"
        "w.style.display='block';w.innerText='Mất kết nối - SOC cũ: '+d.savedSoc+'%';}"
        "else if(d.socFresh){b.innerText='MỚI';b.style.background='#28a745';w.style.display='none';}"
        "else{let a=Math.round(d.socAge/1000);b.innerText='CŨ('+a+'s)';b.style.background='#ffc107';"
        "w.style.display='block';w.innerText='Dữ liệu cũ '+a+'s';}"

        "document.getElementById('chip-name').innerText=d.chipName;hasDac=d.hasDac;"
        "let ms=(d.outMode===0)?('DAC GPIO'+d.dacPin):('PWM GPIO'+d.pwmPin);"
        "document.getElementById('out-current').innerText=ms;"
        "document.getElementById('vol-text').innerText='Output: '+d.voltage+' V';"
        "if(d.outMode!==currentMode){currentMode=d.outMode;selectMode(currentMode);}"
        "activePresetName=d.activePresetName||'';"

        "isManualSim=d.manualSim;"
        "let rb=document.getElementById('realBtn');"
        "if(isManualSim){rb.innerText='Về Pin Thực';rb.className='btn-danger';}"
        "else{rb.innerText=(d.acquyMode||d.connected)?'Đang chạy Tự động':'Đang dùng SOC cũ';rb.className='btn-success';"
        "document.getElementById('simSlider').value=d.soc;"
        "document.getElementById('slider-val').innerText=d.soc+'%';}"

        "let accOut=document.getElementById('accOutput');"
        "let outOpen=accOut&&accOut.classList.contains('open');"
        "if(outOpen&&!vmapInited&&d.vmap!==undefined){vmapInited=true;"
        "document.getElementById('vmapEn').checked=d.vmapEnabled;"
        "onVmapToggle();renderVmap(d.vmap);}"

        "let accWifi=document.getElementById('accWifi');"
        "if(accWifi&&accWifi.classList.contains('open')){"
        "let st=document.getElementById('sta-status');"
        "if(d.staConnected){st.innerText='Đã kết nối';st.style.color='#28a745';"
        "document.getElementById('staInfoBox').style.display='block';"
        "document.getElementById('sta-ip').innerText=d.staIP||'--';"
        "document.getElementById('sta-rssi').innerText=d.staRSSI||'--';"
        "}else if(d.staEnabled){st.innerText='Đang kết nối... (tối đa 15s)';st.style.color='#ff9800';"
        "document.getElementById('staInfoBox').style.display='none';"
        "}else{st.innerText='Tắt';st.style.color='#6c757d';"
        "document.getElementById('staInfoBox').style.display='none';}"
        "let en=document.getElementById('staEn');"
        "if(!en.dataset.inited){en.dataset.inited='1';"
        "en.checked=d.staEnabled;"
        "document.getElementById('staSSID').value=d.staSSID||'';"
        "document.getElementById('staPass').value=d.staPass||'';}"
        "}"

        "let otaBox=document.getElementById('otaOnlineBox');"
        "window._staConnected=d.staConnected;"
        "let auToggle=document.getElementById('autoUpdateToggle');"
        "if(auToggle&&!auToggle.dataset.inited){"
        "auToggle.dataset.inited='1';"
        "auToggle.checked=d.autoUpdate||false;"
        "let knob=document.getElementById('autoUpdateKnob');"
        "let slider=document.getElementById('autoUpdateSlider');"
        "if(d.autoUpdate){slider.style.background='#28a745';knob.style.transform='translateX(20px)';}"
        "else{slider.style.background='#ccc';knob.style.transform='translateX(0)';}"
        "applyAutoUpdateUI(d.autoUpdate||false);"
        "}"
        "let auStatus=document.getElementById('autoUpdateStatus');"
        "if(auStatus){"
        "if(d.autoUpdate){"
        "auStatus.innerText=d.autoUpdateDone?'✅ Đã kiểm tra':'⏳ Chờ WiFi nhà...';"
        "auStatus.style.color=d.autoUpdateDone?'#28a745':'#ff9800';"
        "}else{auStatus.innerText='Tắt';auStatus.style.color='#6c757d';}"
        "}"
        "if(otaBox){otaBox.style.display=(d.staConnected&&!d.autoUpdate)?'block':'none';}"

        "let autoHint=document.getElementById('autoConnectHint');"
        "if(autoHint){autoHint.style.display=d.needAutoConnect?'block':'none';}"

        "let curMacEl=document.getElementById('curMacText');"
        "if(curMacEl){curMacEl.innerText=d.mac||'Chưa có';}"
        "let curNameEl=document.getElementById('curBmsName');"
        "if(curNameEl){curNameEl.innerText=d.bmsName?('📛 '+d.bmsName):'';}"

        "let aq=document.getElementById('acquyToggle');"
        "if(aq&&!aq.dataset.inited){"
        "aq.dataset.inited='1';"
        "aq.checked=d.acquyMode||false;"
        "applyAcquyUI(d.acquyMode||false);"
        "document.getElementById('battType').value=d.battType||0;"
        "onBattTypeChange();"
        "let bt=d.battType||0;"
        "if(bt===0){document.getElementById('battCells').value=d.battCells||4;}"
        "else{document.getElementById('battCellsInput').value=d.battCells||16;}"
        "updateBattInfo();"
        "document.getElementById('aqR1').value=d.aqR1;"
        "document.getElementById('aqR2').value=d.aqR2;"
        "document.getElementById('aqGain').value=d.aqGain||1;"
        "document.getElementById('aqOffset').value=d.aqOffset||0;"
        "document.getElementById('aqPin').value=d.aqPin;"
        "checkDivider();"
        "}"

        "if(d.acquyMode){"
        "let rv=document.getElementById('aqRawV');if(rv)rv.innerText=(d.aqRawV||0).toFixed(2)+' V';"
        "let rs=document.getElementById('aqSocRaw');if(rs)rs.innerText=(d.aqSocRaw||0)+'%';"
        "let sv=document.getElementById('aqStableV');if(sv)sv.innerText=(d.aqStableV||0).toFixed(2)+' V';"
        "let ss=document.getElementById('aqSocStable');if(ss)ss.innerText=(d.aqSocStable||0)+'%';"
        "let av=document.getElementById('aqAdcV');if(av)av.innerText=(d.aqAdcV||0).toFixed(2)+'V';"
        "let stt=document.getElementById('aqStableStatus');"
        "if(stt){"
        "if(d.aqIsStable){stt.innerText='✅ Ổn định';stt.style.color='#28a745';}"
        "else{stt.innerText='⚠️ Đang dao động';stt.style.color='#ff9800';}"
        "}"
        "let dg=document.getElementById('dispGain');if(dg)dg.innerText=(d.aqGain||1).toFixed(4);"
        "let dO=document.getElementById('dispOffset');if(dO)dO.innerText=(d.aqOffset||0).toFixed(3);"
        "let ovp=document.getElementById('ovpAlert');if(ovp)ovp.style.display=d.aqOvp?'block':'none';"
        "}"

        "let bmsNameEl=document.getElementById('bms-name-text');"
        "if(bmsNameEl){"
        "if(d.acquyMode){bmsNameEl.innerText='Chế độ Ắc quy ('+(d.battType===1?'LFP':'Chì')+' '+d.battCells+(d.battType===1?' cell':' bình')+')';bmsNameEl.style.color='#17a2b8';}"
        "else{"
        "let dn=d.bmsName||'';"
        "let dm=d.mac||'';"
        "let isDefaultName=(dn==='BMS '+dm);"
        "if(dn&&!isDefaultName){"
        "if(d.connected){bmsNameEl.innerText=dn+' ('+dm+')';bmsNameEl.style.color='#28a745';}"
        "else{bmsNameEl.innerText=dn+' — chưa kết nối';bmsNameEl.style.color='#ff9800';}"
        "}else if(dm){bmsNameEl.innerText='Chưa có tên — MAC: '+dm;bmsNameEl.style.color='#6c757d';}"
        "else{bmsNameEl.innerText='--';bmsNameEl.style.color='#6c757d';}"
        "}}"

        "let typeBadge=document.getElementById('bms-type-badge');"
        "if(typeBadge){"
        "if(d.acquyMode){typeBadge.style.display='none';}"
        "else if(d.connected&&d.mac&&d.mac.length>0&&d.bmsProto!==undefined){"
        "if(d.bmsProto===1){"
        "let vTag='';"
        "if(d.jkVer===2)vTag=' 32S';"
        "else if(d.jkVer===1)vTag=' 24S';"
        "typeBadge.innerText='⚡ JiKong'+vTag;"
        "typeBadge.style.background='#6f42c1';typeBadge.style.color='#fff';typeBadge.style.display='inline-block';"
        "}else if(d.bmsProto===0){"
        "typeBadge.innerText='🔋 BMS MOVE';typeBadge.style.background='#007bff';typeBadge.style.color='#fff';typeBadge.style.display='inline-block';"
        "}else if(d.bmsProto===254){"
        "typeBadge.innerText='❓ Không rõ';typeBadge.style.background='#6c757d';typeBadge.style.color='#fff';typeBadge.style.display='inline-block';"
        "}else{"
        "typeBadge.innerText='⏳ Đang nhận diện...';typeBadge.style.background='#ffc107';typeBadge.style.color='#333';typeBadge.style.display='inline-block';"
        "}"
        "}else{typeBadge.style.display='none';}"
        "}"

        "let btnFetch=document.getElementById('btnFetchPresets');"
        "if(btnFetch){btnFetch.style.display=(d.staConnected||d.configMode)?'inline-block':'none';}"

        "if(accOut&&accOut.classList.contains('open')){"
        "if(!window.urlsInited){window.urlsInited=true;"
        "if(d.presetInfoLoaded&&d.presetUpdated)showPresetUpdated(d.presetUpdated);}"
        "}"
        "}).catch(e=>{});}"

        "let pollTimer=null;"
        "function startPoll(){if(!pollTimer)pollTimer=setInterval(function(){"
        "if(document.hidden)return;"
        "updateData();"
        "},2000);}"
        "document.addEventListener('visibilitychange',function(){"
        "if(!document.hidden){updateData();startPoll();}"
        "else{clearInterval(pollTimer);pollTimer=null;}"
        "});"
        "startPoll();updateData();"

        "window.addEventListener('beforeunload',function(){"
        "let inp=document.getElementById('staPass');"
        "if(inp)inp.type='password';"
        "});"

        "function sendSim(v){document.getElementById('slider-val').innerText=v+'%';"
        "fetchTO('/sim?soc='+v+'&t='+Date.now(),3000).catch(e=>{});}"
        "function setRealSoc(){fetchTO('/sim?real=1&t='+Date.now(),3000).catch(e=>{});}"

        "function toggleAcquy(en){"
        "if(!confirm(en?'Bật chế độ ẮC QUY và TẮT Bluetooth BLE?':'Tắt chế độ ẮC QUY và BẬT lại Bluetooth?')){"
        "document.getElementById('acquyToggle').checked=!en;return;}"
        "fetchTO('/toggleacquy?en='+(en?'1':'0')+'&t='+Date.now(),5000)"
        ".then(r=>r.text()).then(t=>{"
        "if(t==='OK'){applyAcquyUI(en);updateData();}"
        "else{alert('Lỗi: '+t);document.getElementById('acquyToggle').checked=!en;}"
        "}).catch(e=>{alert('Lỗi: '+e.message);document.getElementById('acquyToggle').checked=!en;});}"

        "function applyAcquyUI(en){"
        "let scanArea=document.getElementById('bmsScanArea');"
        "let cfgBox=document.getElementById('acquyConfigBox');"
        "let adcPinBox=document.getElementById('acquyAdcPinBox');"
        "if(scanArea){scanArea.style.opacity=en?'0.3':'1';scanArea.style.pointerEvents=en?'none':'auto';}"
        "if(cfgBox)cfgBox.style.display=en?'block':'none';"
        "if(adcPinBox)adcPinBox.style.display=en?'block':'none';}"

        "function onBattTypeChange(){"
        "let t=parseInt(document.getElementById('battType').value);"
        "let leadBox=document.getElementById('leadOptions');"
        "let lfpBox=document.getElementById('lfpOptions');"
        "if(t===0){if(leadBox)leadBox.style.display='block';if(lfpBox)lfpBox.style.display='none';}"
        "else{if(leadBox)leadBox.style.display='none';if(lfpBox)lfpBox.style.display='block';}"
        "updateBattInfo();}"

        "function updateBattInfo(){"
        "let t=parseInt(document.getElementById('battType').value);"
        "let vE,vF,vW,nom;"
        "if(t===0){"
        "let c=parseInt(document.getElementById('battCells').value);"
        "vE=10.5*c; vF=13.8*c; vW=15.0*c; nom=12*c;"
        "}else{"
        "let c=parseInt(document.getElementById('battCellsInput').value)||0;"
        "vE=3.00*c; vF=3.45*c; vW=3.65*c; nom=(3.2*c).toFixed(1);"
        "}"
        "let info=document.getElementById('battInfoText');"
        "if(info){"
        "info.innerHTML='Không tải: <b>'+vE.toFixed(1)+'V</b>'"
        "+' | Đầy: <b>'+vF.toFixed(1)+'V</b>'"
        "+' | OVP: <b>'+vW.toFixed(1)+'V</b>'"
        "+' | Danh nghĩa: <b>'+nom+'V</b>';"
        "}"
        "checkDivider();}"

        "function checkDivider(){"
        "let r1=parseFloat(document.getElementById('aqR1').value);"
        "let r2=parseFloat(document.getElementById('aqR2').value);"
        "let warnBox=document.getElementById('dividerWarn');"
        "if(!warnBox)return;"
        "if(!r1||!r2||r1<=0||r2<=0){warnBox.style.display='none';return;}"
        "let t=parseInt(document.getElementById('battType').value);"
        "let vMax;"
        "if(t===0){"
        "let c=parseInt(document.getElementById('battCells').value)||1;"
        "vMax=15.0*c;"
        "}else{"
        "let c=parseInt(document.getElementById('battCellsInput').value)||1;"
        "vMax=3.65*c;"
        "}"
        "let vadc=vMax*r2/(r1+r2);"
        "let ratio=(r1+r2)/r2;"
        "let safe=3.10;"
        "let max=3.30;"
        "if(vadc>max){"
        "warnBox.style.display='block';"
        "warnBox.style.background='#f8d7da';warnBox.style.border='1px solid #f5c2c7';"
        "warnBox.style.color='#721c24';"
        "warnBox.innerHTML='❌ <b>NGUY HIỂM — CÓ THỂ CHÁY GPIO!</b><br>'"
        "+'V_ADC max = <b>'+vadc.toFixed(3)+'V</b> (vượt '+max+'V của ESP32)<br>'"
        "+'Hệ số chia: '+ratio.toFixed(2)+'× — <b>quá nhỏ</b><br>'"
        "+'👉 Cần tăng R1 hoặc giảm R2.';"
        "}"
        "else if(vadc>safe){"
        "warnBox.style.display='block';"
        "warnBox.style.background='#fff3cd';warnBox.style.border='1px solid #ffeeba';"
        "warnBox.style.color='#856404';"
        "warnBox.innerHTML='⚠️ <b>CẢNH BÁO — SÁT NGƯỠNG</b><br>'"
        "+'V_ADC max = <b>'+vadc.toFixed(3)+'V</b> (ngưỡng an toàn: '+safe+'V)<br>'"
        "+'Hệ số chia: '+ratio.toFixed(2)+'× — nên tăng thêm biên an toàn.<br>'"
        "+'👉 Khuyến nghị: R1='+(r1*1.2).toFixed(1)+'k, R2='+r2+'k';"
        "}"
        "else if(vadc<0.5){"
        "warnBox.style.display='block';"
        "warnBox.style.background='#e7f3ff';warnBox.style.border='1px solid #b8daff';"
        "warnBox.style.color='#004085';"
        "warnBox.innerHTML='ℹ️ <b>Độ phân giải thấp</b><br>'"
        "+'V_ADC max = '+vadc.toFixed(3)+'V (quá nhỏ, chỉ dùng ~'+(vadc/3.3*100).toFixed(1)+'% dải ADC)<br>'"
        "+'Kết quả đo sẽ kém chính xác.<br>'"
        "+'👉 Khuyến nghị: giảm R1 xuống ~'+(r1*0.5).toFixed(1)+'k, hoặc tăng R2.';"
        "}"
        "else{"
        "warnBox.style.display='block';"
        "warnBox.style.background='#d4edda';warnBox.style.border='1px solid #c3e6cb';"
        "warnBox.style.color='#155724';"
        "warnBox.innerHTML='✅ <b>PHÂN ÁP HỢP LỆ</b><br>'"
        "+'V_ADC max = <b>'+vadc.toFixed(3)+'V</b> (an toàn)<br>'"
        "+'Hệ số chia: <b>'+ratio.toFixed(2)+'×</b> — dùng ~'+(vadc/3.3*100).toFixed(1)+'% dải ADC';"
        "}"
        "}"

        "function quickCalibrate(){"
        "let vRef=parseFloat(document.getElementById('calRefV').value);"
        "let msg=document.getElementById('calMsg');"
        "if(!vRef||vRef<1){msg.innerText='❌ Nhập V thực > 1V';msg.style.color='#dc3545';return;}"
        "fetchTO('/data?t='+Date.now(),5000).then(r=>r.json()).then(d=>{"
        "let vMeas=d.aqRawV||0;"
        "if(vMeas<1){msg.innerText='❌ Chưa đọc được ADC (bật Ắc quy trước)';msg.style.color='#dc3545';return;}"
        "let g=parseFloat(document.getElementById('aqGain').value)||1;"
        "let off=parseFloat(document.getElementById('aqOffset').value)||0;"
        "let gNew=(vRef-off)/((vMeas-off)/g);"
        "if(gNew<0.5||gNew>2){msg.innerText='❌ Hệ số ngoài phạm vi ('+gNew.toFixed(4)+')';msg.style.color='#dc3545';return;}"
        "document.getElementById('aqGain').value=gNew.toFixed(4);"
        "msg.innerText='✅ Gain mới: '+gNew.toFixed(4)+' ('+vMeas.toFixed(2)+'V → '+vRef+'V). Bấm Lưu!';"
        "msg.style.color='#28a745';"
        "}).catch(e=>{msg.innerText='❌ Timeout';msg.style.color='#dc3545';});}"

        "function saveAcquyCfg(){"
        "let pin=document.getElementById('aqPin').value;"
        "let r1=document.getElementById('aqR1').value;"
        "let r2=document.getElementById('aqR2').value;"
        "let r1n=parseFloat(r1),r2n=parseFloat(r2);"
        "if(r1n>0&&r2n>0){"
        "let tt=parseInt(document.getElementById('battType').value);"
        "let vm;"
        "if(tt===0){let c=parseInt(document.getElementById('battCells').value)||1;vm=15.0*c;}"
        "else{let c=parseInt(document.getElementById('battCellsInput').value)||1;vm=3.65*c;}"
        "let vadc=vm*r2n/(r1n+r2n);"
        "if(vadc>3.30){"
        "if(!confirm('❌ CẢNH BÁO NGUY HIỂM!\\n\\nV_ADC = '+vadc.toFixed(3)+'V VƯỢT 3.3V — CÓ THỂ CHÁY GPIO ESP32!\\n\\nBạn có CHẮC CHẮN muốn lưu?'))return;"
        "}"
        "}"
        "let gain=document.getElementById('aqGain').value;"
        "let offset=document.getElementById('aqOffset').value;"
        "let t=parseInt(document.getElementById('battType').value);"
        "let c=0;"
        "if(t===0)c=document.getElementById('battCells').value;"
        "else c=document.getElementById('battCellsInput').value;"
        "let p='pin='+pin+'&r1='+r1+'&r2='+r2+'&gain='+gain+'&offset='+offset+'&btype='+t+'&cells='+c;"
        "fetchTO('/saveacquycfg?'+p+'&t='+Date.now(),5000).then(r=>r.text()).then(t=>{"
        "if(t==='OK')alert('Đã lưu cấu hình Ắc quy!');else alert('Lỗi: '+t);}).catch(e=>alert('Timeout'));}"

        "function scan(){"
        "let s=document.getElementById('status'),r=document.getElementById('results');"
        "s.innerText='Đang quét...';r.innerHTML='';"
        "fetchTO('/scan?t='+Date.now(),8000).then(x=>x.text()).then(h=>{"
        "s.innerText='';"
        "if(h.length<10)r.innerHTML='<li>Không có kết quả</li>';"
        "else r.innerHTML=h;"
        "}).catch(e=>{s.innerText='Lỗi: '+e.message;});}"

        "function save(mac,name){"
        "if(!mac||mac.length!==17){alert('MAC không hợp lệ');return;}"
        "mac=mac.trim().toUpperCase();name=name||'';"
        "let info=name?name+' ('+mac+')':mac;"
        "if(!confirm('Kết nối tới BMS:\\n'+info+' ?'))return;"
        "let s=document.getElementById('status');"
        "if(s){s.innerText='⏳ Đang lưu & kết nối...';s.style.color='#007bff';}"
        "fetchTO('/save?mac='+encodeURIComponent(mac)+'&name='+encodeURIComponent(name)+'&t='+Date.now(),5000)"
        ".then(r=>r.text()).then(t=>{"
        "if(t==='OK'){"
        "if(s){s.innerText='✅ Đã lưu '+mac;s.style.color='#28a745';}"
        "let el=document.getElementById('manualMac');if(el)el.value=mac;"
        "}else if(s){s.innerText='❌ '+t;s.style.color='#dc3545';}"
        "}).catch(e=>{if(s){s.innerText='❌ '+e.message;s.style.color='#dc3545';}});}"

        "function validateMac(s){s=s.trim().toUpperCase();"
        "if(s.length!==17)return null;"
        "let re=/^([0-9A-F]{2}:){5}[0-9A-F]{2}$/;"
        "if(!re.test(s))return null;return s;}"

        "function saveManualMac(){"
        "let el=document.getElementById('manualMac');"
        "let msg=document.getElementById('manualMacMsg');"
        "let mac=validateMac(el.value);"
        "if(!mac){msg.innerText='❌ MAC không hợp lệ!';msg.style.color='#dc3545';"
        "el.style.borderColor='#dc3545';return;}"
        "el.style.borderColor='#28a745';"
        "msg.innerText='⏳ Đang lưu...';msg.style.color='#007bff';"
        "fetchTO('/save?mac='+mac+'&t='+Date.now(),5000).then(r=>r.text()).then(t=>{"
        "if(t==='OK'){msg.innerText='✅ Đã lưu '+mac;msg.style.color='#28a745';el.value=mac;}"
        "else{msg.innerText='❌ '+t;msg.style.color='#dc3545';}}).catch(e=>{msg.innerText='❌ Timeout';msg.style.color='#dc3545';});}"
        "setTimeout(function(){let el=document.getElementById('manualMac');"
        "if(el&&!el.value){fetchTO('/data?t='+Date.now(),5000).then(r=>r.json()).then(d=>{"
        "if(d.mac&&d.mac.length>0)el.value=d.mac;}).catch(e=>{});}},1500);"

        "function clearMac(){"
        "if(!confirm('🗑️ Xóa MAC hiện tại?'))return;"
        "fetchTO('/clearmac?t='+Date.now(),5000).then(r=>r.text()).then(t=>{"
        "if(t==='OK'){alert('✅ Đã xóa MAC');"
        "let el=document.getElementById('curMacText');if(el)el.innerText='Chưa có';"
        "let ne=document.getElementById('curBmsName');if(ne)ne.innerText='';"
        "let m=document.getElementById('manualMac');if(m)m.value='';"
        "}else{alert('❌ Lỗi: '+t);}"
        "}).catch(e=>alert('❌ '+e.message));}"

        "function switchMacTab(which){"
        "let ts=document.getElementById('subTabScan');"
        "let tm=document.getElementById('subTabManualMac');"
        "let cs=document.getElementById('macSubScan');"
        "let cm=document.getElementById('macSubManual');"
        "if(which==='scan'){ts.classList.add('active');tm.classList.remove('active');"
        "cs.style.display='block';cm.style.display='none';}"
        "else{tm.classList.add('active');ts.classList.remove('active');"
        "cm.style.display='block';cs.style.display='none';}}"

        "function switchSubTab(which){"
        "let tp=document.getElementById('subTabPreset');"
        "let tm=document.getElementById('subTabManual');"
        "let cp=document.getElementById('subPreset');"
        "let cm=document.getElementById('subManual');"
        "if(which==='preset'){tp.classList.add('active');tm.classList.remove('active');"
        "cp.style.display='block';cm.style.display='none';}"
        "else{tm.classList.add('active');tp.classList.remove('active');"
        "cm.style.display='block';cp.style.display='none';}}"

        "function onVmapToggle(){"
        "let en=document.getElementById('vmapEn').checked;"
        "let b=document.getElementById('vmapBox');"
        "b.style.opacity=en?'1':'0.5';"
        "b.style.pointerEvents=en?'auto':'none';}"

        "function addVmapRow(s,e,v){"
        "let list=document.getElementById('vmapList');"
        "let row=document.createElement('div');row.className='vmap-row';"
        "row.innerHTML=\"<input class='pct' type='number' min='0' max='100' value='\"+(s!==undefined?s:0)+\"' oninput='clampPct(this)'>\""
        "+\"<span style='width:14px;text-align:center;color:#888'>→</span>\""
        "+\"<input class='pct' type='number' min='0' max='100' value='\"+(e!==undefined?e:100)+\"' oninput='clampPct(this)'>\""
        "+\"<input class='volt' type='number' min='0' max='3.3' step='0.01' value='\"+(v!==undefined?v.toFixed(2):'0.00')+\"'>\""
        "+\"<button onclick='this.parentNode.remove()'>✕</button>\";"
        "list.appendChild(row);}"
        "function clampPct(el){let x=parseInt(el.value);if(isNaN(x))return;"
        "if(x<0)x=0;if(x>100)x=100;el.value=x;}"
        "function renderVmap(arr){"
        "document.getElementById('vmapList').innerHTML='';"
        "if(!arr||arr.length===0){addVmapRow(0,20,0.7);addVmapRow(21,40,0.8);return;}"
        "for(let i=0;i<arr.length;i++)addVmapRow(arr[i].s,arr[i].e,arr[i].v);}"
        "function saveVmap(){"
        "let rows=document.querySelectorAll('#vmapList .vmap-row');"
        "let p='enabled='+(document.getElementById('vmapEn').checked?'1':'0')+'&count='+rows.length;"
        "for(let i=0;i<rows.length;i++){"
        "let inp=rows[i].querySelectorAll('input');"
        "let s=parseInt(inp[0].value)||0;let e=parseInt(inp[1].value)||0;let v=parseFloat(inp[2].value)||0;"
        "p+='&e'+i+'s='+s+'&e'+i+'e='+e+'&e'+i+'v='+v;}"
        "fetchTO('/savevmap?'+p+'&t='+Date.now(),5000).then(r=>r.text()).then(t=>{"
        "if(t==='OK'){alert('Đã lưu bảng điện áp!');"
        "document.querySelectorAll('.preset-btn').forEach(b=>b.classList.remove('active'));"
        "activePresetId=-1;activePresetName='';}"
        "else alert('Lỗi: '+t);}).catch(e=>alert('Timeout'));}"

        "function loadPresets(){"
        "fetchTO('/presets?t='+Date.now(),5000).then(r=>r.json()).then(list=>{"
        "let box=document.getElementById('presetList');if(!box)return;"
        "box.innerHTML='';"
        "if(!list||list.length===0){"
        "box.innerHTML='<div style=\"color:#888;text-align:center;padding:20px;font-size:13px\">Chưa có dải nào.<br><small>Bấm <b>🔄 Cập nhật</b> để tải từ server.</small></div>';"
        "return;}"
        "for(let i=0;i<list.length;i++){"
        "let p=list[i];"
        "let btn=document.createElement('button');"
        "btn.className='preset-btn';btn.dataset.pid=i;btn.dataset.pname=p.name;"
        "btn.innerHTML='<div class=\"p-title\">'+p.name+'</div>'"
        "+'<div class=\"p-sub\">'+p.subtitle+'</div>'"
        "+'<div class=\"p-ranges\">'+p.ranges+'</div>';"
        "btn.onclick=(function(id,name){return function(){applyPreset(id,name);};})(i,p.name);"
        "if(p.name===activePresetName){btn.classList.add('active');activePresetId=i;}"
        "box.appendChild(btn);}"
        "}).catch(e=>{});}"

        "function applyPreset(id,name){"
        "if(!confirm('⚡ Áp dụng bảng điện áp: '+name+' ?'))return;"
        "fetchTO('/applypreset?id='+id+'&t='+Date.now(),5000).then(r=>r.text()).then(t=>{"
        "if(t==='OK'){"
        "document.querySelectorAll('.preset-btn').forEach(b=>b.classList.remove('active'));"
        "let btn=document.querySelector('.preset-btn[data-pid=\"'+id+'\"]');"
        "if(btn)btn.classList.add('active');"
        "activePresetId=id;activePresetName=name;"
        "vmapInited=false;updateData();"
        "alert('✅ Đã áp dụng: '+name);"
        "}else alert('❌ Lỗi: '+t);}).catch(e=>alert('Timeout'));}"

        "function saveWifi(){"
        "let ssid=document.getElementById('staSSID').value.trim();"
        "let pass=document.getElementById('staPass').value;"
        "let en=document.getElementById('staEn').checked;"
        "let msg=document.getElementById('wifiMsg');"
        "if(en&&ssid.length===0){msg.innerText='❌ Nhập SSID!';msg.style.color='#dc3545';return;}"
        "msg.innerText='⏳ Đang lưu...';msg.style.color='#007bff';"
        "let p='ssid='+encodeURIComponent(ssid)+'&pass='+encodeURIComponent(pass)"
        "+'&en='+(en?'1':'0')+'&t='+Date.now();"
        "fetchTO('/savewifi?'+p,5000).then(r=>r.text()).then(t=>{"
        "if(t==='OK'){msg.innerText='✅ Đã lưu';msg.style.color='#28a745';}"
        "else{msg.innerText='❌ '+t;msg.style.color='#dc3545';}}).catch(e=>{msg.innerText='❌ Timeout';msg.style.color='#dc3545';});}"

        // ============ WiFi scan + Password toggle (NÚT TRONG Ô INPUT) ============
        "function scanWifi(){"
        "let st=document.getElementById('wifiStatus'),li=document.getElementById('wifiList');"
        "let closeBtn=document.getElementById('btnCloseWifiList');"
        "if(li.classList.contains('open')){closeWifiList();return;}"
        "st.innerText='⏳ Đang quét...';li.innerHTML='';"
        "li.classList.add('open');"
        "if(closeBtn)closeBtn.style.display='block';"
        "fetchTO('/wifiscan?t='+Date.now(),10000).then(r=>r.text()).then(h=>{"
        "st.innerText='';li.innerHTML=h;"
        "}).catch(e=>{st.innerText='❌ Lỗi: '+e.message;});}"
        "function closeWifiList(){"
        "let li=document.getElementById('wifiList');"
        "let closeBtn=document.getElementById('btnCloseWifiList');"
        "let st=document.getElementById('wifiStatus');"
        "if(li){li.classList.remove('open');li.innerHTML='';}"
        "if(closeBtn)closeBtn.style.display='none';"
        "if(st)st.innerText='';}"
        "function togglePassVisibility(){"
        "let inp=document.getElementById('staPass');"
        "let btn=document.getElementById('btnTogglePass');"
        "if(!inp||!btn)return;"
        "if(inp.type==='password'){"
        "inp.type='text';"
        "btn.innerText='🙈';"
        "btn.style.color='#28a745';"
        "}else{"
        "inp.type='password';"
        "btn.innerText='👁️';"
        "btn.style.color='#6c757d';"
        "}}"

        "function pickWifi(ssid){"
        "document.getElementById('staSSID').value=ssid;"
        "closeWifiList();"
        "document.getElementById('staPass').focus();}"
        "function forceAp(){"
        "fetchTO('/forceap?t='+Date.now(),5000).then(r=>r.text()).then(t=>{"
        "let msg=document.getElementById('wifiMsg');"
        "if(msg){msg.innerText=t;msg.style.color='#28a745';}"
        "alert(t);}).catch(e=>alert('Timeout'));}"

        "function showPresetUpdated(txt){"
        "let box=document.getElementById('presetUpdatedBox');"
        "let el=document.getElementById('presetUpdatedText');"
        "if(!box||!el)return;"
        "if(txt&&txt.length>0){box.style.display='block';el.innerText=txt;}"
        "else box.style.display='none';}"

        "function fetchPresets(){"
        "let lmsg=document.getElementById('presetListMsg');"
        "let box=document.getElementById('presetList');"
        "if(box){box.innerHTML='<div style=\"text-align:center;padding:20px;color:#6c757d;font-size:13px\">⏳ Đang tải...</div>';}"
        "if(lmsg){lmsg.style.display='block';lmsg.innerText='⏳ Đang tải...';lmsg.style.color='#6c757d';}"
        "fetchTO('/fetchpresets?t='+Date.now(),15000)"
        ".then(r=>r.text()).then(t=>{"
        "if(t.startsWith('OK|')){"
        "let parts=t.split('|');let n=parts[1]||'?';let d=parts[2]||'';"
        "showPresetUpdated(d);loadPresets();"
        "if(lmsg){lmsg.innerText='✅ Đã cập nhật '+n+' dải'+(d?' — '+d:'');lmsg.style.color='#28a745';}"
        "}else{"
        "if(box){box.innerHTML='<div style=\"text-align:center;padding:20px;color:#dc3545;font-size:13px\">❌ Tải thất bại</div>';}"
        "if(lmsg){lmsg.innerText='❌ '+t;lmsg.style.color='#dc3545';}"
        "}}).catch(e=>{"
        "if(box){box.innerHTML='<div style=\"text-align:center;padding:20px;color:#dc3545;font-size:13px\">❌ Lỗi kết nối</div>';}"
        "if(lmsg){lmsg.innerText='❌ '+e.message;lmsg.style.color='#dc3545';}});}"

        "function toggleFwEdit(){"
        "let form=document.getElementById('fwEditForm');"
        "let isOpen=form.style.display==='block';"
        "form.style.display=isOpen?'none':'block';"
        "if(!isOpen){setTimeout(function(){let d=document.getElementById('fwDateInput');if(d)d.focus();},100);}}"

        "function saveFwInfo(){"
        "let d=document.getElementById('fwDateInput').value.trim();"
        "let n=document.getElementById('fwNotesInput').value.trim();"
        "let msg=document.getElementById('fwInfoMsg');"
        "if(d.length===0){msg.innerText='❌ Nhập ngày!';msg.style.color='#dc3545';return;}"
        "msg.innerText='⏳ Đang lưu...';msg.style.color='#007bff';"
        "fetchTO('/savefwinfo?date='+encodeURIComponent(d)+'&notes='+encodeURIComponent(n)+'&t='+Date.now(),5000)"
        ".then(r=>r.text()).then(t=>{"
        "if(t==='OK'){"
        "document.getElementById('fwDateView').innerText=d;"
        "document.getElementById('fwNotesView').innerText=n||'Không có mô tả';"
        "msg.innerText='✅ Đã lưu';msg.style.color='#28a745';"
        "setTimeout(toggleFwEdit,800);"
        "}else{msg.innerText='❌ '+t;msg.style.color='#dc3545';}}).catch(e=>{msg.innerText='❌ Timeout';msg.style.color='#dc3545';});}"

        "function toggleAutoUpdate(en){"
        "let msg=document.getElementById('onlineMsg');"
        "let knob=document.getElementById('autoUpdateKnob');"
        "let slider=document.getElementById('autoUpdateSlider');"
        "if(en){slider.style.background='#28a745';knob.style.transform='translateX(20px)';}"
        "else{slider.style.background='#ccc';knob.style.transform='translateX(0)';}"
        "applyAutoUpdateUI(en);"
        "if(msg){msg.innerText='⏳ Đang lưu...';msg.style.color='#007bff';}"
        "fetchTO('/saveautoupdate?en='+(en?'1':'0')+'&t='+Date.now(),5000)"
        ".then(r=>r.text()).then(t=>{"
        "if(t==='OK'){"
        "if(msg){msg.innerText=en?'✅ Đã bật Auto Update':'✅ Đã tắt Auto Update';msg.style.color='#28a745';}"
        "if(en){setTimeout(function(){if(confirm('Kiểm tra bản cập nhật ngay?'))checkUpdate();},500);}"
        "}else{if(msg){msg.innerText='❌ '+t;msg.style.color='#dc3545';}}"
        "}).catch(e=>{if(msg){msg.innerText='❌ '+e.message;msg.style.color='#dc3545';}});}"

        "function applyAutoUpdateUI(en){"
        "let onlineBox=document.getElementById('otaOnlineBox');"
        "let localBox=document.getElementById('otaLocalBox');"
        "if(en){"
        "if(onlineBox)onlineBox.style.display='none';"
        "if(localBox)localBox.style.display='none';"
        "}else{"
        "if(onlineBox)onlineBox.style.display=(window._staConnected?'block':'none');"
        "if(localBox)localBox.style.display='block';"
        "}}"

        "function checkUpdate(){"
        "let msg=document.getElementById('onlineMsg');"
        "let box=document.getElementById('updateInfoBox');"
        "msg.innerText='🔍 Đang kiểm tra...';msg.style.color='#007bff';"
        "box.style.display='none';"
        "fetchTO('/checkupdate?t='+Date.now(),15000).then(r=>r.text()).then(t=>{"
        "if(t==='NO_WIFI'){msg.innerText='❌ Cần WiFi nhà';msg.style.color='#dc3545';return;}"
        "if(t==='FETCH_FAIL'){msg.innerText='❌ Không tải được update.json';msg.style.color='#dc3545';return;}"
        "if(t.startsWith('LATEST:')){"
        "let parts=t.split(':');let ver=parts[1]||'?';let synced=parts[2]==='SYNCED';"
        "msg.innerText='✅ Đang dùng bản mới nhất (v'+ver+')'+(synced?' — đã cập nhật mô tả':'');"
        "msg.style.color='#28a745';"
        "if(synced){setTimeout(function(){location.reload();},1500);}"
        "return;}"
        "if(t.startsWith('NEW:')){"
        "let rest=t.substring(4);let c1=rest.indexOf(':');"
        "let ver=rest.substring(0,c1);rest=rest.substring(c1+1);"
        "let c2=rest.indexOf(':');let date=rest.substring(0,c2);"
        "msg.innerText='🆕 Có bản mới v'+ver;msg.style.color='#e65100';"
        "document.getElementById('newVersionText').innerText='Phiên bản v'+ver;"
        "document.getElementById('newDateText').innerText=date||'--';"
        "box.style.display='block';"
        "fetchTO('/getnotes?t='+Date.now(),5000).then(r=>r.text()).then(n=>{"
        "document.getElementById('notesText').innerHTML=n||'(không có mô tả)';}).catch(e=>{});"
        "return;}"
        "}).catch(e=>{msg.innerText='❌ '+e.message;msg.style.color='#dc3545';});}"

        "function otaOnline(){"
        "if(!confirm('⚠️ Nạp firmware từ server? KHÔNG tắt nguồn!'))return;"
        "let msg=document.getElementById('onlineMsg');"
        "let bar=document.getElementById('onlineProgress');"
        "msg.innerText='⏳ Đang tải & nạp...';msg.style.color='#007bff';"
        "let pct=0;bar.style.width='5%';"
        "let fake=setInterval(function(){if(pct<90){pct+=2;bar.style.width=pct+'%';"
        "msg.innerText='Đang nạp... ~'+pct+'%';}},600);"
        "fetch('/otaonline?t='+Date.now())"
        ".then(r=>r.text()).then(t=>{"
        "clearInterval(fake);bar.style.width='100%';"
        "msg.innerText='✅ Nạp xong! ESP32 đang khởi động lại...';"
        "msg.style.color='#28a745';"
        "setTimeout(function(){location.reload();},10000);"
        "}).catch(e=>{clearInterval(fake);"
        "msg.innerText='❌ Lỗi (có thể ESP đang restart)';msg.style.color='#dc3545';"
        "setTimeout(function(){location.reload();},10000);});}"

        "function uploadFirmware(){"
        "let f=document.getElementById('fwFile').files[0];"
        "if(!f){alert('Chưa chọn file!');return;}"
        "if(!f.name.endsWith('.bin')){alert('Chỉ chấp nhận file .bin');return;}"
        "if(!confirm('Nạp firmware: '+f.name+' ('+(f.size/1024).toFixed(0)+' KB)?'))return;"
        "let fd=new FormData();fd.append('firmware',f,f.name);"
        "let xhr=new XMLHttpRequest();"
        "let bar=document.getElementById('fwProgress');"
        "let st=document.getElementById('fwStatus');"
        "xhr.upload.onprogress=function(e){"
        "if(e.lengthComputable){let pct=Math.round(e.loaded/e.total*100);bar.style.width=pct+'%';st.innerText='Đang nạp... '+pct+'%';st.style.color='#007bff';}};"
        "xhr.onload=function(){"
        "if(xhr.status===200){bar.style.width='100%';st.innerText='✅ Nạp thành công! Đang khởi động lại...';st.style.color='#28a745';setTimeout(function(){location.reload();},8000);}"
        "else{st.innerText='❌ Lỗi: '+xhr.responseText;st.style.color='#dc3545';bar.style.width='0%';}};"
        "xhr.onerror=function(){st.innerText='❌ Lỗi kết nối';st.style.color='#dc3545';};"
        "xhr.open('POST','/update',true);xhr.send(fd);}"

        "function fmtKB(b){return (b/1024).toFixed(1);}"
        "function fmtUptime(s){"
        "let d=Math.floor(s/86400),h=Math.floor((s%86400)/3600);let m=Math.floor((s%3600)/60),ss=s%60;"
        "if(d>0)return d+'d '+h+'h '+m+'m';if(h>0)return h+'h '+m+'m '+ss+'s';if(m>0)return m+'m '+ss+'s';return ss+'s';}"

        "function updateChipInfo(){"
        "fetchTO('/chipinfo?t='+Date.now(),5000).then(r=>r.json()).then(d=>{"
        "document.getElementById('chipModel').innerText=d.chipModel||'--';"
        "document.getElementById('chipRev').innerText='v'+d.chipRev;"
        "document.getElementById('chipCores').innerText=d.cores;"
        "document.getElementById('chipFreq').innerText=d.cpuFreq;"
        "document.getElementById('cpuFreq').innerText=d.cpuFreq;"
        "document.getElementById('coreCount').innerText=d.cores;"
        "let hp=((d.heapUsed/d.heapTotal)*100).toFixed(1);"
        "document.getElementById('heapPct').innerText=hp;document.getElementById('heapBar').style.width=hp+'%';"
        "let hc=hp>85?'#dc3545':(hp>70?'#ff9800':'#007bff');document.getElementById('heapBar').style.background=hc;"
        "document.getElementById('heapTotal').innerText=fmtKB(d.heapTotal);"
        "document.getElementById('heapUsed').innerText=fmtKB(d.heapUsed);"
        "document.getElementById('heapFree').innerText=fmtKB(d.heapFree);"
        "document.getElementById('heapMin').innerText=fmtKB(d.heapMin);"
        "if(d.psramTotal>0){"
        "document.getElementById('psramBox').style.display='block';"
        "let pp=((d.psramTotal-d.psramFree)/d.psramTotal*100).toFixed(1);"
        "document.getElementById('psramPct').innerText=pp;document.getElementById('psramBar').style.width=pp+'%';"
        "document.getElementById('psramTotal').innerText=fmtKB(d.psramTotal);"
        "document.getElementById('psramFree').innerText=fmtKB(d.psramFree);"
        "}else{document.getElementById('psramBox').style.display='none';}"
        "let appTotal=d.sketchSize+d.sketchFree;let fPct=(d.sketchSize/appTotal*100).toFixed(1);"
        "document.getElementById('flashPct').innerText=fPct;document.getElementById('flashBar').style.width=fPct+'%';"
        "document.getElementById('flashTotal').innerText=fmtKB(appTotal)+' (APP)';"
        "document.getElementById('sketchSize').innerText=fmtKB(d.sketchSize);"
        "document.getElementById('sketchFree').innerText=fmtKB(d.sketchFree);"
        "document.getElementById('uptime').innerText=fmtUptime(d.uptime);"
        "let cl=d.cpuLoad||0;document.getElementById('cpuLoad').innerText=cl.toFixed(1);"
        "document.getElementById('cpuBar').style.width=cl+'%';"
        "let cc=cl>80?'#dc3545':(cl>50?'#ff9800':'#28a745');document.getElementById('cpuBar').style.background=cc;"
        "document.getElementById('chipExtraInfo').innerText="
        "'Flash chip: '+fmtKB(d.flashSize)+' KB tổng | Heap min: '+fmtKB(d.heapMin)+' KB';"
        "}).catch(e=>{});}"
        "setInterval(function(){let acc=document.getElementById('accChip');if(acc&&acc.classList.contains('open'))updateChipInfo();},2000);"

        "function updateBmsInfo(){"
        "fetchTO('/data?t='+Date.now(),5000).then(r=>r.json()).then(d=>{"
        "let titleEl=document.getElementById('accBmsTitle');let headEl=document.getElementById('accBmsHead');"
        "if(titleEl&&headEl){"
        "if(d.acquyMode){titleEl.innerText='🔋 Chi Tiết Điện Áp / Ắc Quy';headEl.style.color='#ff9800';}"
        "else{titleEl.innerText='🔋 Chi Tiết Điện Áp / BMS';headEl.style.color='#17a2b8';}"
        "}"
        "let vc=0,maxV=-1,minV=999,mi=0,ni=0;"
        "for(let i=0;i<16;i++){let v=(d.cells&&d.cells[i])||0;"
        "if(v>0.5){vc++;if(v>maxV){maxV=v;mi=i}if(v<minV){minV=v;ni=i}}}"
        "let hasAny=d.hasTotalV||d.hasCurrent||d.hasPower||d.hasCells;"
        "document.getElementById('bms-empty').style.display=hasAny?'none':'block';"
        "let boxTV=document.getElementById('bms-box-totalv');let boxRaw=document.getElementById('aq-box-raw');let boxStable=document.getElementById('aq-box-stable');"
        "if(d.acquyMode){"
        "if(boxTV)boxTV.style.display='none';"
        "if(boxRaw){boxRaw.style.display='block';let rvEl=document.getElementById('aq-totalv-raw');if(rvEl){rvEl.innerText=(d.aqRawV||0).toFixed(2)+' V';let diff=Math.abs((d.aqRawV||0)-(d.aqStableV||0));rvEl.style.color=(diff>0.3)?'#ff9800':'#6c757d';}}"
        "if(boxStable){boxStable.style.display='block';let svEl=document.getElementById('aq-totalv-stable');if(svEl){svEl.innerText=(d.aqStableV||0).toFixed(2)+' V';svEl.style.color=d.aqIsStable?'#28a745':'#6c757d';}}"
        "}else{"
        "if(boxRaw)boxRaw.style.display='none';if(boxStable)boxStable.style.display='none';"
        "if(d.hasTotalV){if(boxTV){boxTV.style.display='block';document.getElementById('bms-totalv').innerText=d.totalV.toFixed(2)+' V';}}"
        "else{if(boxTV)boxTV.style.display='none';}"
        "}"
        "let statusLine=document.getElementById('aq-status-line');let statusText=document.getElementById('bms-aq-status-text');"
        "let adcVEl=document.getElementById('bms-aq-adc-v');let pinLabelEl=document.getElementById('bms-aq-pin-label');"
        "let ovpBadge=document.getElementById('bms-aq-ovp-badge');let gainEl=document.getElementById('bms-aq-gain');let offsetEl=document.getElementById('bms-aq-offset');"
        "if(statusLine&&statusText){"
        "if(d.acquyMode){"
        "statusLine.style.display='block';"
        "if(pinLabelEl)pinLabelEl.innerText='GPIO'+(d.aqPin||'--');"
        "if(adcVEl)adcVEl.innerText=(d.aqAdcV||0).toFixed(3)+' V';"
        "if(gainEl)gainEl.innerText=(d.aqGain||1).toFixed(4);"
        "if(offsetEl)offsetEl.innerText=(d.aqOffset||0).toFixed(3);"
        "if(d.aqIsStable){statusText.innerText='✅ Ổn định';statusText.style.color='#28a745';}"
        "else{let diff=Math.abs((d.aqRawV||0)-(d.aqStableV||0));if(diff>0.3){statusText.innerText='⚠️ Đang có tải ('+diff.toFixed(2)+'V sag)';statusText.style.color='#ff9800';}else{statusText.innerText='⏳ Đang chờ ổn định...';statusText.style.color='#6c757d';}}"
        "if(ovpBadge)ovpBadge.style.display=d.aqOvp?'inline-block':'none';"
        "}else{statusLine.style.display='none';}"
        "}"
        "let boxCC=document.getElementById('bms-box-cellcount');"
        "if(d.hasCells){boxCC.style.display='block';document.getElementById('bms-cellcount').innerText=vc;}else{boxCC.style.display='none';}"
        "let boxCur=document.getElementById('bms-box-current');"
        "if(d.hasCurrent){boxCur.style.display='block';let cur=d.current||0;let curEl=document.getElementById('bms-current');curEl.innerText=cur.toFixed(2)+' A';curEl.style.color=cur<0?'#28a745':(cur>0?'#dc3545':'#ff9800');}else{boxCur.style.display='none';}"
        "let boxPow=document.getElementById('bms-box-power');"
        "if(d.hasPower){boxPow.style.display='block';document.getElementById('bms-power').innerText=(d.power||0).toFixed(1)+' W';}else{boxPow.style.display='none';}"
        "let stats=document.getElementById('bms-cellstats');"
        "if(d.hasCells&&vc>0){"
        "stats.style.display='flex';"
        "document.getElementById('bms-cellmax').innerText='C'+(mi+1)+' - '+maxV.toFixed(3)+'V';"
        "document.getElementById('bms-cellmin').innerText='C'+(ni+1)+' - '+minV.toFixed(3)+'V';"
        "let diff=maxV-minV;let diffEl=document.getElementById('bms-celldiff');diffEl.innerText=(diff*1000).toFixed(0)+' mV';"
        "diffEl.style.color=diff>0.05?'#dc3545':(diff>0.03?'#ff9800':'#28a745');"
        "}else{stats.style.display='none';}"
        "let wrap=document.getElementById('bms-cells-wrap');"
        "if(d.hasCells&&vc>0){"
        "wrap.style.display='block';let grid=document.getElementById('bms-cells-grid');let html='';"
        "for(let i=0;i<16;i++){let v=(d.cells&&d.cells[i])||0;"
        "if(v>0.5){"
        "let color='#333';let bg='#f8f9fa';"
        "if(i===mi){color='#fff';bg='#dc3545';}else if(i===ni){color='#fff';bg='#28a745';}"
        "html+='<div style=\"background:'+bg+';color:'+color+';padding:6px 4px;border-radius:5px;text-align:center;font-size:11px;border:1px solid #dee2e6\">';"
        "html+='<div style=\"font-size:10px;font-weight:bold;opacity:.7\">C'+(i+1)+'</div>';"
        "html+='<div style=\"font-size:13px;font-weight:bold;margin-top:2px\">'+v.toFixed(3)+'</div></div>';"
        "}}"
        "grid.innerHTML=html;"
        "}else{wrap.style.display='none';}"
        "}).catch(e=>{});}"
        "setInterval(function(){let acc=document.getElementById('accBms');if(acc&&acc.classList.contains('open'))updateBmsInfo();},2000);"
        "</script></body></html>"
    );

    httpBusy = false;
}
// =====================================================
// SETUP
// =====================================================
void setup() {
    pinMode(LED_PIN, OUTPUT);
    for (int i = 0; i < 5; i++) {
        digitalWrite(LED_PIN, HIGH); delay(150);
        digitalWrite(LED_PIN, LOW);  delay(150);
    }
    digitalWrite(LED_PIN, HIGH);

    MySerial.begin(115200);
    delay(100);
    ledSetup();
    MySerial.println("\n>>> Khởi động Hệ thống <<<");

    WiFi.persistent(false);
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    delay(50);

    stateMutex = xSemaphoreCreateMutex();

    preferences.begin("bms_config", false);
    minVoltage = preferences.getFloat("min_v", minVoltage);
    maxVoltage = preferences.getFloat("max_v", maxVoltage);
    targetMac  = preferences.getString("bms_mac", "");
    savedSoc   = preferences.getUInt("last_soc", 0);
    outMode    = preferences.getInt("out_mode", OUT_MODE_DAC);
    dacPin     = preferences.getInt("dac_pin", 25);
    pwmPin     = preferences.getInt("pwm_pin", 14);
    pwmFreq    = preferences.getInt("pwm_freq", 5000);
    pwmRes     = preferences.getInt("pwm_res", 10);

    acquyMode   = preferences.getBool("aq_en", false);
    acquyAdcPin = preferences.getInt("aq_pin", 32);
    acquyR1     = preferences.getFloat("aq_r1", 100.0f);
    acquyR2     = preferences.getFloat("aq_r2", 4.7f);

    acquyCalibGain   = preferences.getFloat("aq_gain", 1.0f);
    acquyCalibOffset = preferences.getFloat("aq_off", 0.0f);

    float oldCalib = preferences.getFloat("aq_cal", 0.0f);
    if (oldCalib > 0.01f && acquyCalibGain == 1.0f) acquyCalibGain = oldCalib;
    if (acquyCalibGain < 0.5f || acquyCalibGain > 2.0f) acquyCalibGain = 1.0f;
    if (acquyCalibOffset < -5.0f || acquyCalibOffset > 5.0f) acquyCalibOffset = 0.0f;

    batteryType  = preferences.getInt("batt_type", BATT_TYPE_LEAD);
    batteryCells = preferences.getInt("batt_cells", 4);

    int oldType = preferences.getInt("aq_type", 0);
    if (oldType >= 12 && batteryCells == 4 && batteryType == BATT_TYPE_LEAD) {
        int c = oldType / 12;
        if (c >= 1 && c <= 6) batteryCells = c;
    }
    if (batteryType != BATT_TYPE_LEAD && batteryType != BATT_TYPE_LFP) batteryType = BATT_TYPE_LEAD;
    if (batteryCells < 1 || batteryCells > 32) batteryCells = 4;

    uint8_t savedProto = preferences.getUChar("bms_proto", 255);
    if (savedProto <= 1) {
        bmsProtocol = savedProto;
        LOG_INFO("📌 BMS đã lưu: %s\n", savedProto == 1 ? "JiKong" : "BMS MOVE");
    }
    jkVersion = preferences.getUChar("jk_ver", 0);
    if (jkVersion > 0 && jkVersion <= 2) {
        jkVersionProbed = true;
        LOG_INFO("📌 JK protocol đã lưu: %s\n", jkVersion == 2 ? "JK02_32S" : "JK02_24S");
    } else { jkVersion = 0; jkVersionProbed = false; }

    vmapEnabled = preferences.getBool("vmap_en", false);
    vmapCount   = preferences.getUChar("vmap_cnt", 0);
    if (vmapCount > MAX_VMAP_ENTRIES) vmapCount = MAX_VMAP_ENTRIES;
    for (int i = 0; i < vmapCount; i++) {
        String base = "vm" + String(i) + "_";
        vmap[i].startPct = preferences.getUChar((base + "s").c_str(), 0);
        vmap[i].endPct   = preferences.getUChar((base + "e").c_str(), 0);
        vmap[i].voltage  = preferences.getFloat((base + "v").c_str(), 0.0f);
    }

    staSSID    = preferences.getString("sta_ssid", "");
    staPass    = preferences.getString("sta_pass", "");
    staEnabled = preferences.getBool("sta_en", false);

    fwDate  = preferences.getString("fw_date", "");
    fwNotes = preferences.getString("fw_notes", "");
    activePresetName = preferences.getString("active_preset", "");

    autoUpdateEnabled = preferences.getBool("auto_update", false);
    autoUpdateDone    = false;

    bmsDeviceName = preferences.getString("bms_name", "");
    bool needAuto = preferences.getBool("need_auto", false);
    if (needAuto && targetMac.length() == 0) needAutoConnect = true;
    else needAutoConnect = false;

    preferences.end();

    pinMode(acquyAdcPin, INPUT);
    analogReadResolution(12);

    preferences.begin("bms_presets", false);
    int n = preferences.getInt("count", 0);
    if (n > MAX_REMOTE_PRESETS) n = MAX_REMOTE_PRESETS;
    remotePresetCount = 0;
    for (int i = 0; i < n; i++) {
        String base = "p" + String(i) + "_";
        remotePresets[i].name     = preferences.getString((base + "n").c_str(), "");
        remotePresets[i].subtitle = preferences.getString((base + "s").c_str(), "");
        remotePresets[i].ranges   = preferences.getString((base + "r").c_str(), "");
        uint8_t c = preferences.getUChar((base + "c").c_str(), 0);
        if (c > MAX_ENTRIES_PER_PRESET) c = MAX_ENTRIES_PER_PRESET;
        remotePresets[i].count = c;
        for (int j = 0; j < c; j++) {
            String ek = base + "e" + String(j) + "_";
            remotePresets[i].entries[j].startPct = preferences.getUChar((ek + "s").c_str(), 0);
            remotePresets[i].entries[j].endPct   = preferences.getUChar((ek + "e").c_str(), 0);
            remotePresets[i].entries[j].voltage  = preferences.getFloat((ek + "v").c_str(), 0.0f);
        }
        if (remotePresets[i].name.length() > 0) remotePresetCount++;
    }
    preferences.end();

    if (outMode != OUT_MODE_DAC && outMode != OUT_MODE_PWM) outMode = OUT_MODE_DAC;
    if (dacPin != 25 && dacPin != 26) dacPin = 25;
    if (pwmRes < 1 || pwmRes > 14) pwmRes = 10;
    if (minVoltage >= maxVoltage) { minVoltage = 2.07f; maxVoltage = 2.35f; }
    targetMac.toUpperCase();

#if !HAS_DAC
    if (outMode == OUT_MODE_DAC) outMode = OUT_MODE_PWM;
#endif

    MySerial.printf("🔌 Chip: %s | DAC=%d\n", CHIP_NAME, HAS_DAC);
    MySerial.printf("📌 FW: v%s\n", FW_VERSION);

    if (outMode == OUT_MODE_DAC) {
#if HAS_DAC
        pinMode(dacPin, OUTPUT); dacWrite(dacPin, 0);
#endif
    } else { pinMode(pwmPin, OUTPUT); setupPWM(); }

    if (savedSoc > 0 && savedSoc <= 100) applyOutput(savedSoc);

    if (!acquyMode) { bleInit(); bleAutoScan = true; }
    else { bleEnabled = false; bleInitialized = false; }

    xTaskCreatePinnedToCore(bleTask, "BLE_Task", 8192, nullptr, 0, &bleTaskHandle, 0);
    ledSetMode(LED_MODE_CONFIG_BLINK);

    server.on("/", handleRoot);
    server.on("/chipinfo", handleChipInfo);
    server.on("/data", handleData);
    server.on("/serial", handleSerial);
    server.on("/sim", handleSim);
    server.on("/saveout", handleSaveOutput);
    server.on("/savevmap", handleSaveVMap);
    server.on("/scan", handleScan);
    server.on("/save", handleSave);
    server.on("/clearmac", handleClearMac);
    server.on("/toggleacquy", handleToggleAcquy);
    server.on("/saveacquycfg", handleSaveAcquyCfg);
    server.on("/savewifi", handleSaveWifi);
    server.on("/wifiscan", handleWifiScan);
    server.on("/fetchpresets", handleFetchPresets);
    server.on("/presets", handlePresets);
    server.on("/applypreset", handleApplyPreset);
    server.on("/savefwinfo", handleSaveFwInfo);
    server.on("/getnotes", handleGetNotes);
    server.on("/saveautoupdate", handleSaveAutoUpdate);
    server.on("/checkupdate", handleCheckUpdate);
    server.on("/otaonline", HTTP_GET, handleOtaOnline);
    server.on("/forceap", handleForceAp);
    server.on("/update", HTTP_POST, []() { handleUpdateDone(); }, []() { handleUpdateUpload(); });

    server.on("/generate_204", handleCaptivePortal);
    server.on("/gen_204", handleCaptivePortal);
    server.on("/hotspot-detect.html", handleCaptivePortal);
    server.on("/library/test/success.html", handleCaptivePortal);
    server.on("/connecttest.txt", handleCaptivePortal);
    server.on("/redirect", handleCaptivePortal);
    server.on("/canonical.html", handleCaptivePortal);
    server.on("/success.txt", handleCaptivePortal);
    server.on("/ncsi.txt", handleCaptivePortal);

    server.onNotFound([]() {
        server.sendHeader("Location", "http://192.168.4.1/", true);
        server.send(302, "text/plain", "");
    });

    apStart();
    server.begin();
    MySerial.println("🌐 WebServer + Captive Portal started");

    if (staEnabled && staSSID != "") {
        WiFi.mode(WIFI_AP_STA);
        staStart();
    }
}

// =====================================================
// LOOP
// =====================================================
void loop() {
    loopCounter = loopCounter + 1;

    if (otaInProgress) {
        if (apActive && dnsServerStarted) dnsServer.processNextRequest();
        if (apActive || staConnected) server.handleClient();
        delay(1);
        return;
    }

    processAcquyReading();

    static unsigned long lastLedUpdate = 0;
    if (millis() - lastLedUpdate > 60) {
        lastLedUpdate = millis();
        ledUpdate();
    }

    measureCpuLoad();

    if (apActive) {
        if (dnsServerStarted) dnsServer.processNextRequest();
        apWatchdog();
    }

    if (staEnabled) staCheck();

    if (apActive || staConnected) {
        for (int i = 0; i < 4; i++) {
            server.handleClient();
            delay(0);
        }
    }

    autoUpdateTask();
    delay(2);
}
