#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <Update.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <DNSServer.h>
#include <NimBLEDevice.h>

#define HAS_DAC 1
#define CHIP_NAME "ESP32 Classic"

// ★ Log module bitmask
#define LOGMOD_BOOT 0x0001
#define LOGMOD_ADC 0x0002
#define LOGMOD_BLE 0x0004
#define LOGMOD_WIFI 0x0008
#define LOGMOD_OUT 0x0010
#define LOGMOD_OTA 0x0020
#define LOGMOD_ERR 0x0040
#define LOGMOD_SUM 0x0080
#define LOGMOD_ALL 0x00FF

uint16_t logModulesEnabled = LOGMOD_ALL;

#define LOGM(mod, fmt, ...) \
  do { \
    if (logModulesEnabled & mod) { \
      MySerial.printf("[%s] " fmt "\n", #mod + 7, ##__VA_ARGS__); \
    } \
  } while (0)

#define LOGM_ERR(fmt, ...) \
  do { \
    MySerial.printf("[ERR] ❌ " fmt "\n", ##__VA_ARGS__); \
  } while (0)

#define FW_VERSION "3.4"

#define BASE_URL "https://bjendaik.github.io/hienthipin/json/"
#define PRESET_URL BASE_URL "data.json"
#define UPDATE_JSON_URL BASE_URL "update.json"
#define ACQUY_JSON_URL BASE_URL "acquy.json"

// ★ Cấu hình LED
#define LED_PIN 2
#define LED_PWM_FREQ 5000
#define LED_PWM_RES 8
enum LedMode {
  LED_MODE_CONFIG_BLINK,
  LED_MODE_SOLID,
  LED_MODE_BREATHE
};
LedMode currentLedMode = LED_MODE_SOLID;

// ★ Cấu hình Output DAC / PWM
#define OUT_MODE_DAC 0
#define OUT_MODE_PWM 1
int outMode = OUT_MODE_DAC;
int dacPin = 25;
int pwmPin = 14;
int pwmFreq = 5000;
int pwmRes = 10;

// ★ Cấu hình WiFi AP (0 = luôn bật)
#define AP_CHECK_INTERVAL 1000UL
#define AP_TIMEOUT_MIN_S 10
#define AP_TIMEOUT_MAX_S 3600
unsigned long apTimeoutMs = 60000UL;

bool apActive = false;
bool apCfgPanelOpen = false;
String apSSID = "BMS_Config";
String apPass = "12345678";
unsigned long apLastClientTime = 0;
unsigned long apStartedTime = 0;
uint8_t apLastClientCount = 0;
unsigned long apLastCheck = 0;
unsigned long lastHttpRequestTime = 0;
#define TOUCH_HTTP() \
  do { lastHttpRequestTime = millis(); } while (0)
#define HTTP_KEEPALIVE_MS 30000UL
volatile bool otaInProgress = false;

DNSServer dnsServer;
const byte DNS_PORT = 53;
bool dnsServerStarted = false;

// ★ Cấu hình WiFi STA
#define STA_TIMEOUT_MS 15000UL
bool staConnected = false;
bool staEnabled = false;
String staSSID = "";
String staPass = "";
bool staAttempting = false;
unsigned long staAttemptStart = 0;

// ★ Thông tin Firmware & Auto Update
String fwDate = "";
String fwNotes = "";
bool autoUpdateEnabled = false;
bool autoUpdateDone = false;
bool autoUpdateInProgress = false;
unsigned long lastAutoCheck = 0;

bool needAutoConnect = false;
String bmsDeviceName = "";
String pendingScanName = "";

String remoteVersion = "";
String remoteFwUrl = "";
String remoteNotes = "";
String remoteDate = "";
bool remoteChecked = false;
bool remoteNewer = false;

// ★ Đo tải CPU
volatile unsigned long loopCounter = 0;
float cpuLoadAvg = 0;
float cpuLoad0 = 0;
float cpuLoad1 = 0;

// ★ Bộ đệm Serial cho Web Terminal
#define SERIAL_BUF_SIZE 4096
class MySerialClass : public Print {
public:
  char buf[SERIAL_BUF_SIZE];
  volatile size_t head = 0;
  volatile size_t tail = 0;
  SemaphoreHandle_t mtx = nullptr;
  void begin(unsigned long baud) {
    Serial.begin(baud);
    mtx = xSemaphoreCreateMutex();
  }
  void lock() {
    if (mtx) xSemaphoreTake(mtx, portMAX_DELAY);
  }
  void unlock() {
    if (mtx) xSemaphoreGive(mtx);
  }
  size_t write(uint8_t c) override {
    lock();
    Serial.write(c);
    size_t next = (head + 1) % SERIAL_BUF_SIZE;
    if (next != tail) {
      buf[head] = c;
      head = next;
    } else {
      tail = (tail + 1) % SERIAL_BUF_SIZE;
      buf[head] = c;
      head = next;
    }
    unlock();
    return 1;
  }
  size_t write(const uint8_t* data, size_t len) override {
    lock();
    Serial.write(data, len);
    for (size_t i = 0; i < len; i++) {
      size_t next = (head + 1) % SERIAL_BUF_SIZE;
      if (next != tail) {
        buf[head] = data[i];
        head = next;
      } else {
        tail = (tail + 1) % SERIAL_BUF_SIZE;
        buf[head] = data[i];
        head = next;
      }
    }
    unlock();
    return len;
  }
  String readAll() {
    String out = "";
    lock();
    while (tail != head) {
      out += buf[tail];
      tail = (tail + 1) % SERIAL_BUF_SIZE;
    }
    unlock();
    return out;
  }
};
MySerialClass MySerial;

// ★ Biến đồng bộ & BLE
TaskHandle_t bleTaskHandle = nullptr;
SemaphoreHandle_t stateMutex = nullptr;

bool bleAutoScan = true;
bool bleEnabled = true;
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
float bmsCells[16] = { 0 };
float bmsCurrent = 0.0f;
float bmsPower = 0.0f;
uint8_t bmsCellCount = 0;
bool bmsHasCurrent = false;
bool bmsHasPower = false;
bool bmsHasCells = false;
bool bmsHasTotalV = false;
String bmsRawString = "Đang chờ dữ liệu...";

WebServer server(80);
Preferences preferences;
String targetMac = "";

String cachedJsonData = "";
unsigned long cachedJsonTime = 0;
#define JSON_CACHE_MS 800
unsigned long lastHexUpdateMs = 0;
volatile bool httpBusy = false;

// ★ Cấu hình loại Bình / Pin & Ngưỡng điện áp
#define BATT_TYPE_LEAD 0
#define BATT_TYPE_LFP 1
int batteryType = BATT_TYPE_LEAD;
int batteryCells = 4;

#define LFP_V_EMPTY 2.80f
#define LFP_V_FULL 3.45f
#define LFP_V_MAX 3.65f
#define LFP_V_NOM 3.20f

#define LEAD_V_EMPTY 11.31f
#define LEAD_V_FULL 12.73f
#define LEAD_V_MAX 14.40f
#define LEAD_V_NOM 12.00f

// ★ Biến đo Ắc quy qua ADC
bool acquyMode = false;
int acquyAdcPin = 32;
float acquyR1 = 100.0f;
float acquyR2 = 2.05f;
float acquyCalibGain = 1.0f;
float acquyCalibOffset = 0.0f;
float acquyMeasuredV = 0.0f;
float acquyAdcRawV = 0.0f;
uint8_t acquySoc = 0;
bool acquyOverVoltage = false;
unsigned long lastAdcRead = 0;
float acquyRawV = 0.0f;
float acquyStableV = 0.0f;
uint8_t acquySocRaw = 0;
uint8_t acquySocStable = 0;

#define ACQY_MEDIAN_SIZE 5
#define ACQY_EMA_ALPHA 0.25f
#define ACQY_STABLE_DELTA 0.30f
#define ACQY_STABLE_TIME_MS 2500UL
#define ACQY_OUTPUT_DELAY_MS 800UL
#define ACQY_OUTPUT_HYST 1

float acquyMedianBuf[ACQY_MEDIAN_SIZE] = { 0 };
uint8_t acquyMedianIdx = 0;
bool acquyMedianFilled = false;
float acquyEmaV = 0.0f;
bool acquyEmaInit = false;
float acquyLastStableV = 0.0f;
unsigned long acquyStableStart = 0;
bool acquyStable = false;
unsigned long acquyLastOutputChange = 0;

const float COMP_LEARN_RATE = 0.005f;
const float SAG_THRESHOLD = 1.0f;
const float SAG_LIGHT = 0.4f;
float acquyOcvEstimated = 0.0f;
float acquySag = 0.0f;
// ★ Phát hiện SẠC
bool acquyCharging = false;
unsigned long acquyChargeStart = 0;
unsigned long acquyChargeStopMs = 0;
float chargeRiseThr = 0.30f;         // ngưỡng phát hiện bắt đầu sạc (V)
float chargeFullMargin = 0.05f;      // dư địa dưới vFullLim coi là đầy
float chargeOcvEma = 0.0f;           // OCV bám theo áp sạc (EMA)
bool chargeOcvEmaInit = false;
#define CHARGE_OCV_EMA_ALPHA 0.10f
#define CHARGE_FULL_HOLD_MS  30000UL
#define CHARGE_OCV_OFFSET    2.0f    // ★ OCV = áp sạc − 2V (ước lượng áp nghỉ)
unsigned long chargeFullHoldStart = 0;
float ocvBaseForChargeDetect = 0.0f; // nền OCV phát hiện bắt đầu sạc
float ocvBeforeCharge = 0.0f;        // ★ OCV gốc trước khi bắt đầu sạc

static unsigned long acquyLastStableUpdate = 0;
static uint8_t socFiltered = 0;
static bool socFilteredInit = false;

#define ACQY_ADC_ATTEN ADC_11db

// ★ Tham số phát hiện Tụt/Tăng áp & Ổn định
#define DROP_WINDOW_MS 500UL
#define DROP_BUF_SIZE 8

float dropThresholdLead = 1.00f;
float dropThresholdLfp = 0.50f;
float riseThresholdLead = 1.00f;
float riseThresholdLfp = 0.50f;
unsigned long settleLeadMs = 60000UL;
unsigned long settleLfpMs  = 60000UL;
float stableBandV = 0.50f;

enum AcqState {
  ACQ_IDLE,
  ACQ_TRANSIENT,
  ACQ_SETTLING
};
static AcqState acqState = ACQ_IDLE;
static unsigned long acqEventTime = 0;
static float acqSettleMinV = 999.0f;
static float acqSettleMaxV = 0.0f;
static bool acqIsRise = false;

static float dropBuf[DROP_BUF_SIZE];
static unsigned long dropBufTs[DROP_BUF_SIZE];
static uint8_t dropBufIdx = 0;

// ★ Bảng ánh xạ phần trăm ra điện áp Output (vMap & Presets)
#define MAX_VMAP_ENTRIES 20
struct VMapEntry {
  uint8_t startPct;
  uint8_t endPct;
  float voltage;
};
VMapEntry vmap[MAX_VMAP_ENTRIES];
uint8_t vmapCount = 0;
bool vmapEnabled = true;

#define MAX_REMOTE_PRESETS 30
#define MAX_ENTRIES_PER_PRESET 20
struct RemotePreset {
  String name;
  String subtitle;
  String ranges;
  uint8_t count;
  VMapEntry entries[MAX_ENTRIES_PER_PRESET];
};
RemotePreset remotePresets[MAX_REMOTE_PRESETS];
int remotePresetCount = 0;
String presetLastUpdated = "";
bool presetInfoLoaded = false;
unsigned long lastPresetSync = 0;
String activePresetName = "";

// ★ Bảng nội suy OCV -> SOC
#define MAX_OCV_POINTS 32
struct OcvPoint {
  float v;
  uint8_t soc;
};
OcvPoint leadOcvTable[MAX_OCV_POINTS];
uint8_t leadOcvN = 0;
OcvPoint lfpOcvTable[MAX_OCV_POINTS];
uint8_t lfpOcvN = 0;
String acquyJsonDate = "";

// ★ Biến BLE Client & Giao thức BMS
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
uint8_t jkDeviceInfoCmd[] = { 0xAA, 0x55, 0x90, 0xEB, 0x96, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10 };
uint8_t jkCellInfoNewCmd[] = { 0xAA, 0x55, 0x90, 0xEB, 0x96, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11 };
uint8_t jkCellInfoOldCmd[] = { 0xAA, 0x55, 0x90, 0xEB, 0x97, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x11 };

volatile uint8_t bmsProtocol = 255;
volatile bool probeSent = false;
unsigned long probeStartMs = 0;
volatile uint8_t jkVersion = 0;
volatile bool jkVersionProbed = false;
unsigned long jkDetectTime = 0;

#define JK_FRAME_SIZE 350
uint8_t jkFrameBuf[JK_FRAME_SIZE];
size_t jkFrameLen = 0;
unsigned long jkFrameStartMs = 0;

inline void lockState() {
  if (stateMutex) xSemaphoreTake(stateMutex, portMAX_DELAY);
}
inline void unlockState() {
  if (stateMutex) xSemaphoreGive(stateMutex);
}

// ★ Nguyên mẫu hàm dùng chung giữa các file
void getBatteryLimits(float& vEmpty, float& vFull, float& vMaxWarn);
float calcAdcMaxVoltage();
void ledSetup();
void ledSetMode(LedMode mode);
void ledUpdate();
void measureCpuLoad();
void staStart();
void staStop();
void staCheck();
void apStart();
void apStop();
void apWatchdog();
float getMappedVoltage(uint8_t soc);
void applyOutput(uint8_t soc);
void setupPWM();
void disablePWM();
void bleInit();
void bleDeinit();
void autoUpdateTask();
bool fetchPresetInfo(const String& url);
bool fetchPresetsFromUrl(const String& url);
bool fetchUpdateInfo();
bool fetchAcquyJson();
void loadOcvFromNVS();

#endif
