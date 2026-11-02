#define CONFIG_BT_NIMBLE_MAX_CONNECTIONS 4
#define CONFIG_BT_NIMBLE_MAX_BONDS 3
#define CONFIG_BT_NIMBLE_MAX_CCCDS 8

#include <Preferences.h>
#include <Update.h>
#include <NimBLEDevice.h>

#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32S2) || \
    defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6) || \
    defined(CONFIG_IDF_TARGET_ESP32H2)
  #define HAS_DAC 0
  #define CHIP_NAME "ESP32-S3/S2/C3/C6/H2"
#else
  #define HAS_DAC 1
  #define CHIP_NAME "ESP32 Classic"
#endif

#define LOGMOD_BOOT 0x0001
#define LOGMOD_ADC  0x0002
#define LOGMOD_BLE  0x0004
#define LOGMOD_OUT  0x0008
#define LOGMOD_OTA  0x0010
#define LOGMOD_SUM  0x0020
#define LOGMOD_ALL  0x003F

uint16_t logModulesEnabled = LOGMOD_ALL;

void bleSendLog(const String& s);
#define LOGM(mod, fmt, ...) do { \
    if (logModulesEnabled & mod) { \
        char _b[220]; snprintf(_b, sizeof(_b), "[%s] " fmt, #mod + 7, ##__VA_ARGS__); \
        Serial.println(_b); bleSendLog(_b); \
    } \
} while(0)
#define LOGM_ERR(fmt, ...) do { \
    char _b[220]; snprintf(_b, sizeof(_b), "[ERR] " fmt, ##__VA_ARGS__); \
    Serial.println(_b); bleSendLog(_b); \
} while(0)

#define FW_VERSION "3.4"

#define LED_PIN 2
enum LedMode { LED_MODE_CONFIG_BLINK, LED_MODE_SOLID, LED_MODE_BREATHE };
LedMode currentLedMode = LED_MODE_SOLID;

#define OUT_MODE_DAC 0
#define OUT_MODE_PWM 1
int outMode=OUT_MODE_DAC, dacPin=25, pwmPin=14, pwmFreq=5000, pwmRes=10;

#define LFP_V_EMPTY 2.80f
#define LFP_V_FULL  3.45f
#define LFP_V_MAX   3.65f
#define LEAD_V_EMPTY 11.31f
#define LEAD_V_FULL  12.73f
#define LEAD_V_MAX   14.40f
#define BATT_TYPE_LEAD 0
#define BATT_TYPE_LFP  1

bool acquyMode=false;
int acquyAdcPin=32, batteryType=BATT_TYPE_LEAD, batteryCells=4;
float acquyR1=100.0f, acquyR2=4.7f;
float acquyCalibGain=1.0f, acquyCalibOffset=0.0f;
float acquyMeasuredV=0, acquyAdcRawV=0, acquyRawV=0, acquyStableV=0;
uint8_t acquySoc=0, acquySocRaw=0, acquySocStable=0;
bool acquyOverVoltage=false;
unsigned long lastAdcRead=0;

#define ACQY_MEDIAN_SIZE 5
#define ACQY_EMA_ALPHA 0.25f
#define ACQY_STABLE_DELTA 0.30f
#define ACQY_STABLE_TIME_MS 2500UL
#define ACQY_OUTPUT_DELAY_MS 800UL
#define ACQY_OUTPUT_HYST 1
float acquyMedianBuf[ACQY_MEDIAN_SIZE]={0};
uint8_t acquyMedianIdx=0;
bool acquyMedianFilled=false;
float acquyEmaV=0; bool acquyEmaInit=false;
float acquyLastStableV=0; unsigned long acquyStableStart=0;
bool acquyStable=false; unsigned long acquyLastOutputChange=0;
float compFactorLearned=0.70f;
const float COMP_LEARN_RATE=0.005f;
const float SAG_THRESHOLD=1.0f;
const float SAG_LIGHT=0.4f;
float acquyOcvEstimated=0, acquySag=0;
static unsigned long acquyLastStableUpdate=0;
static uint8_t socFiltered=0; static bool socFilteredInit=false;
static float tempCompLogged=0;
#define ACQY_ADC_ATTEN ADC_11db

#define MAX_VMAP_ENTRIES 20
struct VMapEntry { uint8_t s, e; float v; };
VMapEntry vmap[MAX_VMAP_ENTRIES];
uint8_t vmapCount=0;

#define MAX_OCV_POINTS 32
struct OcvPoint { float v; uint8_t soc; };
OcvPoint leadOcvTable[MAX_OCV_POINTS]; uint8_t leadOcvN=0;
OcvPoint lfpOcvTable[MAX_OCV_POINTS];  uint8_t lfpOcvN=0;
String acquyJsonDate="";

float minVoltage=2.07f, maxVoltage=2.35f;
uint8_t currentSoc=0;
int currentOut=0;
bool manualSimMode=false;
uint8_t realBmsSoc=0, savedSoc=0;
unsigned long bmsSocTimestamp=0;
float bmsTotalVoltage=0, bmsCells[16]={0}, bmsCurrent=0, bmsPower=0;
uint8_t bmsCellCount=0;
bool bmsHasCurrent=false, bmsHasPower=false, bmsHasCells=false, bmsHasTotalV=false;

volatile unsigned long loopCounter = 0;

Preferences preferences;
String targetMac="";
String bmsDeviceName="";
bool needAutoConnect=false;
String pendingScanName="";

SemaphoreHandle_t nvsMutex = nullptr;

bool nvsBegin(const char* name, bool readOnly=false){
    if(!nvsMutex) return false;
    if(xSemaphoreTake(nvsMutex, pdMS_TO_TICKS(1000)) != pdTRUE){
        Serial.println("[NVS] ❌ Mutex timeout");
        return false;
    }
    bool ok = preferences.begin(name, readOnly);
    if(!ok){
        Serial.printf("[NVS] ❌ begin('%s') failed\n", name);
        xSemaphoreGive(nvsMutex);
    }
    return ok;
}
void nvsEnd(){
    preferences.end();
    if(nvsMutex) xSemaphoreGive(nvsMutex);
}

#define CMD_BUF_SIZE 128
struct CmdPacket {
    char cmd[CMD_BUF_SIZE];
};
QueueHandle_t cmdQueue = nullptr;
TaskHandle_t connectTaskHandle = nullptr;

SemaphoreHandle_t stateMutex=nullptr;
inline void lockState(){ if(stateMutex) xSemaphoreTake(stateMutex,portMAX_DELAY); }
inline void unlockState(){ if(stateMutex) xSemaphoreGive(stateMutex); }

NimBLEClient* pClient=nullptr;
NimBLERemoteCharacteristic* pChar=nullptr, *pWriteChar=nullptr;
volatile bool connected=false, doConnect=false, doDisconnect=false;
volatile bool bleClientInit=false, isScanning=false, doScan=false, scanReady=false;
volatile bool scanBusy=false;
String scanResultJson="[]";
NimBLEUUID serviceUUID("0000FFE0-0000-1000-8000-00805F9B34FB");
NimBLEUUID charUUID("0000FFE1-0000-1000-8000-00805F9B34FB");

uint8_t loginCmd[]={0xA5,0x0B,0x00,0x58,0x58,0x19,0x0A,0x1E,0x0E,0x28,0x0D};
uint8_t requestSocCmd[]={0xA5,0x40,0x90,0x08,0,0,0,0,0,0,0,0,0x7D};
uint8_t jkDeviceInfoCmd[]={0xAA,0x55,0x90,0xEB,0x96,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0x10};
uint8_t jkCellInfoNewCmd[]={0xAA,0x55,0x90,0xEB,0x96,0x01,0,0,0,0,0,0,0,0,0,0,0,0,0,0x11};
uint8_t jkCellInfoOldCmd[]={0xAA,0x55,0x90,0xEB,0x97,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0x11};

volatile uint8_t bmsProtocol=255;
volatile bool probeSent=false;
unsigned long probeStartMs=0;
volatile uint8_t jkVersion=0;
volatile bool jkVersionProbed=false;

volatile bool pendingOutputUpdate = false;
volatile uint8_t pendingOutputSoc = 0;

// ★ Biến toàn cục cho auto-scan
unsigned long lastAutoScanAttempt = 0;
unsigned long lastManualScanAttempt = 0;
volatile bool forceScanNow = false;

#define JK_FRAME_SIZE 350
uint8_t jkFrameBuf[JK_FRAME_SIZE];
size_t jkFrameLen=0;
unsigned long jkFrameStartMs=0;

#define UART_SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define UART_RX_CHAR_UUID "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define UART_TX_CHAR_UUID "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

#define OTA_SERVICE_UUID "0000ff00-0000-1000-8000-00805f9b34fb"
#define OTA_RX_CHAR_UUID "0000ff01-0000-1000-8000-00805f9b34fb"
#define OTA_TX_CHAR_UUID "0000ff02-0000-1000-8000-00805f9b34fb"

NimBLECharacteristic* pUartTx=nullptr;
NimBLECharacteristic* pOtaTx=nullptr;
volatile bool uartClientConnected=false;

bool otaActive=false;
size_t otaTotal=0, otaWritten=0;

void handleBleCommand(const String& line);
void sendStatus();
void processAcquyReading();
void bleClientStart();
void bleClientStop();
bool connectBms();
void connectTask(void* param);

// ============ BLE HELPERS ============
void bleSendLog(const String& s){
    if(!pUartTx || !uartClientConnected) return;
    String j="{\"t\":\"log\",\"msg\":\"";
    for(size_t i=0;i<s.length();i++){
        char c=s[i];
        if(c=='"'||c=='\\') j+='\\';
        if(c=='\n'){ j+="\\n"; continue; }
        j+=c;
    }
    j+="\"}";
    size_t total=j.length(), sent=0;
    const size_t CHUNK=20;
    while(sent<total){
        size_t n=(total-sent>CHUNK)?CHUNK:(total-sent);
        pUartTx->setValue((uint8_t*)j.c_str()+sent, n);
        if(!pUartTx->notify()) break;
        sent+=n;
        delay(5);
    }
}

void bleSendJson(const String& j){
    if(!pUartTx || !uartClientConnected) return;
    String out=j+"\n";
    size_t total=out.length(), sent=0;
    const size_t CHUNK=20;
    while(sent<total){
        size_t n=(total-sent>CHUNK)?CHUNK:(total-sent);
        pUartTx->setValue((uint8_t*)out.c_str()+sent, n);
        if(!pUartTx->notify()) return;
        sent+=n;
        delay(5);
    }
}

void sendAck(const String& msg="OK"){ bleSendJson("{\"t\":\"ack\",\"msg\":\""+msg+"\"}"); }
void sendErr(const String& msg){ bleSendJson("{\"t\":\"err\",\"msg\":\""+msg+"\"}"); }

// ============ OCV ============
static int interpolateOcv(const OcvPoint* t, uint8_t n, float v){
    if(n<2) return 0;
    if(v>=t[0].v) return t[0].soc;
    if(v<=t[n-1].v) return t[n-1].soc;
    for(uint8_t i=0;i<n-1;i++){
        if(v<=t[i].v && v>=t[i+1].v){
            float v1=t[i].v, s1=t[i].soc, v2=t[i+1].v, s2=t[i+1].soc;
            float k=(v-v2)/(v1-v2);
            return constrain((int)round(s2+k*(s1-s2)),0,100);
        }
    }
    return 0;
}
int calcSocFromV(float vBat){
    if(vBat<=0.1f || batteryCells<1) return 0;
    if(batteryType==BATT_TYPE_LEAD)
        return interpolateOcv(leadOcvTable, leadOcvN, vBat/(float)batteryCells);
    return interpolateOcv(lfpOcvTable, lfpOcvN, vBat/(float)batteryCells);
}
void getBatteryLimits(float &e, float &f, float &w){
    if(batteryType==BATT_TYPE_LFP){ e=LFP_V_EMPTY*batteryCells; f=LFP_V_FULL*batteryCells; w=LFP_V_MAX*batteryCells; }
    else { e=LEAD_V_EMPTY*batteryCells; f=LEAD_V_FULL*batteryCells; w=LEAD_V_MAX*batteryCells; }
}
static float applyTempComp(float vBat){
    float tc=temperatureRead();
    if(isnan(tc)||tc<-20||tc>80) return vBat;
    if(batteryCells<1) return vBat;
    float cpc = (batteryType==BATT_TYPE_LFP) ? (25.0f-tc)*0.0005f : (25.0f-tc)*0.003f;
    return vBat + cpc*batteryCells;
}

// ============ LED ============
void ledSetup(){ ledcDetach(LED_PIN); pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, HIGH); }
void ledSetMode(LedMode mode){
    currentLedMode=mode;
    ledcDetach(LED_PIN);
    pinMode(LED_PIN, OUTPUT);
    if(mode==LED_MODE_SOLID) digitalWrite(LED_PIN, HIGH);
    else if(mode==LED_MODE_CONFIG_BLINK) digitalWrite(LED_PIN, LOW);
    else { ledcAttach(LED_PIN, 5000, 8); ledcWrite(LED_PIN, 20); }
}
void ledUpdate(){
    static unsigned long lastUp=0, lastTog=0;
    static bool tg=false;
    static int bv=20, bd=1;
    if(millis()-lastUp<50) return;
    lastUp=millis();
    switch(currentLedMode){
        case LED_MODE_CONFIG_BLINK:
            if(millis()-lastTog>=300){ lastTog=millis(); tg=!tg; digitalWrite(LED_PIN, tg?HIGH:LOW); }
            break;
        case LED_MODE_SOLID: digitalWrite(LED_PIN, HIGH); break;
        case LED_MODE_BREATHE:
            bv+=bd*15;
            if(bv>=255){ bv=255; bd=-1; }
            if(bv<=10){ bv=10; bd=1; }
            ledcWrite(LED_PIN, bv);
            break;
    }
}

// ============ OUTPUT ============
float getMappedVoltage(uint8_t soc){
    if(vmapCount==0) return -1.0f;
    for(uint8_t i=0;i<vmapCount;i++)
        if(soc>=vmap[i].s && soc<=vmap[i].e) return vmap[i].v;
    if(soc<vmap[0].s) return vmap[0].v;
    return vmap[vmapCount-1].v;
}
void applyOutput(uint8_t soc){
    soc=constrain(soc,0,100);
    int ov=0;
    float mv=getMappedVoltage(soc);
    if(outMode==OUT_MODE_DAC){
#if HAS_DAC
        if(mv>=0) ov=constrain((int)round((mv/3.3f)*255.0f),0,255);
        else {
            int mn=constrain((int)round((minVoltage/3.3f)*255.0f),0,255);
            int mx=constrain((int)round((maxVoltage/3.3f)*255.0f),0,255);
            ov=map(soc,0,100,mn,mx);
        }
        dacWrite(dacPin, ov);
#endif
    } else {
        int md=(1<<pwmRes)-1;
        if(mv>=0) ov=constrain((int)round((mv/3.3f)*md),0,md);
        else {
            int mn=constrain((int)round((minVoltage/3.3f)*md),0,md);
            int mx=constrain((int)round((maxVoltage/3.3f)*md),0,md);
            ov=map(soc,0,100,mn,mx);
        }
        ledcWrite(pwmPin, ov);
    }
    lockState(); currentSoc=soc; currentOut=ov; unlockState();
}
void setupPWM(){ ledcAttach(pwmPin, pwmFreq, pwmRes); }
void disablePWM(){ ledcWrite(pwmPin,0); ledcDetach(pwmPin); }

// ============ ADC ============
float readAdcVolts(int pin){
    uint32_t s=0;
    for(int i=0;i<32;i++){ s+=analogReadMilliVolts(pin); delayMicroseconds(50); }
    return (float)s/32.0f/1000.0f;
}

void processAcquyReading(){
    if(!acquyMode) return;
    if(millis()-lastAdcRead<500) return;
    lastAdcRead=millis();

    float vAdc=readAdcVolts(acquyAdcPin);
    acquyAdcRawV=vAdc;
    if(acquyR2<=0.01f) acquyR2=4.7f;
    float ratio=(acquyR1+acquyR2)/acquyR2;
    float vBatRaw=vAdc*ratio*acquyCalibGain+acquyCalibOffset;
    acquyRawV=vBatRaw;

    acquyMedianBuf[acquyMedianIdx]=vBatRaw;
    acquyMedianIdx=(acquyMedianIdx+1)%ACQY_MEDIAN_SIZE;
    if(acquyMedianIdx==0) acquyMedianFilled=true;
    float srt[ACQY_MEDIAN_SIZE];
    uint8_t nm=acquyMedianFilled?ACQY_MEDIAN_SIZE:(acquyMedianIdx==0?ACQY_MEDIAN_SIZE:acquyMedianIdx);
    if(nm<1) nm=1;
    for(uint8_t i=0;i<nm;i++) srt[i]=acquyMedianBuf[i];
    for(uint8_t i=0;i<nm-1;i++)
        for(uint8_t j=i+1;j<nm;j++)
            if(srt[j]<srt[i]){ float t=srt[i]; srt[i]=srt[j]; srt[j]=t; }
    float vMed=srt[nm/2];

    if(!acquyEmaInit){ acquyEmaV=vMed; acquyEmaInit=true; }
    else acquyEmaV=acquyEmaV*(1-ACQY_EMA_ALPHA)+vMed*ACQY_EMA_ALPHA;
    acquyMeasuredV=acquyEmaV;

    float ocvRef;
    if(acquyLastStableV>0.1f && acquyLastStableUpdate>0 && (millis()-acquyLastStableUpdate)<30000UL)
        ocvRef=acquyLastStableV; else ocvRef=vMed;
    float sag=ocvRef-vMed; if(sag<0) sag=0;
    acquySag=sag;

    if(sag>SAG_THRESHOLD) compFactorLearned=constrain(compFactorLearned+COMP_LEARN_RATE,0.30f,0.95f);
    else if(sag>0.05f&&sag<SAG_LIGHT) compFactorLearned=constrain(compFactorLearned-COMP_LEARN_RATE,0.30f,0.95f);

    float bc=(batteryType==BATT_TYPE_LFP)?0.35f:0.65f;
    float fc=(compFactorLearned+bc)*0.5f;
    float vOcv=vMed+sag*fc;
    if(vOcv>ocvRef+1.0f) vOcv=ocvRef+1.0f;
    if(vOcv<vMed) vOcv=vMed;
    acquyOcvEstimated=vOcv;

    if(sag<SAG_LIGHT){
        if(fabsf(vMed-acquyLastStableV)<ACQY_STABLE_DELTA){
            if(acquyStableStart==0) acquyStableStart=millis();
            if(millis()-acquyStableStart>=ACQY_STABLE_TIME_MS){
                acquyStable=true;
                acquyLastStableV=vMed;
                acquyLastStableUpdate=millis();
            }
        } else { acquyStable=false; acquyStableStart=0; }
    } else { acquyStable=false; acquyStableStart=0; }

    acquyStableV=vOcv;
    float vOcvComp=applyTempComp(vOcv);
    tempCompLogged=vOcvComp;

    int socNew=calcSocFromV(vOcvComp);
    if(!socFilteredInit){ socFiltered=constrain(socNew,0,100); socFilteredInit=true; }
    int diff=socNew-(int)socFiltered;
    int db=(batteryType==BATT_TYPE_LFP)?((socFiltered>=20&&socFiltered<=90)?3:2):1;
    if(abs(diff)>=db) socFiltered=constrain(socNew,0,100);

    acquySocRaw=(uint8_t)constrain(socNew,0,100);
    acquySocStable=socFiltered;
    acquySoc=socFiltered;

    bool needApply;
    lockState();
    realBmsSoc=acquySocStable;
    bmsTotalVoltage=vOcvComp;
    bmsHasTotalV=true;
    bmsSocTimestamp=millis();
    needApply = !manualSimMode && (abs((int)socFiltered-(int)currentSoc)>=ACQY_OUTPUT_HYST);
    unlockState();

    if(needApply){
        if(millis()-acquyLastOutputChange>ACQY_OUTPUT_DELAY_MS){
            applyOutput(socFiltered);
            acquyLastOutputChange=millis();
        }
    }
}

// ============ JK PARSER ============
uint8_t detectJkVersion(const char* m, size_t len){
    if(len<3) return 1;
    char n[32]={0};
    size_t k=(len<31)?len:31;
    for(size_t i=0;i<k;i++){ char c=m[i]; if(c=='_') c='-'; n[i]=c; }
    if(strncmp(n,"JK-PB2",6)==0) return 2;
    if(strncmp(n,"JK-BD",5)==0) return 2;
    if(strncmp(n,"JK-HY",5)==0) return 2;
    if(strncmp(n,"JK-B2",5)==0) return 1;
    return 1;
}

bool parseJkFrame(uint8_t* data, size_t len){
    if(len>=4 && data[0]==0x55 && data[1]==0xAA && data[2]==0xEB && data[3]==0x90){
        jkFrameLen=0; jkFrameStartMs=millis();
    }
    if(jkFrameLen==0 && !(len>=4 && data[0]==0x55 && data[1]==0xAA)) return false;
    if(jkFrameLen>0 && millis()-jkFrameStartMs>1500){ jkFrameLen=0; return false; }
    size_t cp=len;
    if(jkFrameLen+cp>JK_FRAME_SIZE) cp=JK_FRAME_SIZE-jkFrameLen;
    memcpy(jkFrameBuf+jkFrameLen, data, cp);
    jkFrameLen+=cp;
    if(jkFrameLen<260) return false;
    uint8_t* f=jkFrameBuf;
    uint8_t ft=f[4];
    if(ft==0x03){
        if(!jkVersionProbed){
            char m[32]={0};
            memcpy(m, f+6, 16); m[16]=0;
            jkVersion=detectJkVersion(m,16);
            jkVersionProbed=true;
            if(nvsBegin("bms_config", false)){
                preferences.putUChar("jk_ver", jkVersion);
                nvsEnd();
            }
        }
        if(jkFrameLen<300){ jkFrameLen=0; return false; }
    }
    if(ft==0x02||ft==0x01){
        size_t tvOff=(jkVersion==2)?150:118;
        size_t cOff=(jkVersion==2)?158:126;
        size_t sOff=(jkVersion==2)?173:141;
        if(jkFrameLen<sOff+1){ jkFrameLen=0; return false; }
        uint8_t soc=f[sOff];
        float cells[16]={0}; uint8_t cc=0; float csum=0;
        for(int i=0;i<16;i++){
            uint16_t rv=(uint16_t)(f[6+i*2]|(f[7+i*2]<<8));
            float v=rv*0.001f;
            if(v>1.5f&&v<4.2f){ cells[i]=v; cc++; csum+=v; }
        }
        uint32_t rtv=(uint32_t)f[tvOff]|((uint32_t)f[tvOff+1]<<8)|((uint32_t)f[tvOff+2]<<16)|((uint32_t)f[tvOff+3]<<24);
        float tv=rtv*0.001f;
        int32_t rc=(int32_t)f[cOff]|((int32_t)f[cOff+1]<<8)|((int32_t)f[cOff+2]<<16)|((int32_t)f[cOff+3]<<24);
        float cur=rc*0.001f;
        if(soc>100 || tv<1.0f || tv>(4.5f*32.0f) || fabsf(cur)>500.0f){ jkFrameLen=0; return false; }
        if(cc>=4 && fabsf(csum-tv)>tv*0.15f){ jkFrameLen=0; return false; }

        lockState();
        realBmsSoc=soc;
        bmsTotalVoltage=tv; bmsHasTotalV=true;
        bmsHasCurrent=true; bmsCurrent=cur;
        bmsHasPower=true; bmsPower=tv*cur;
        bmsCellCount=cc; bmsHasCells=(cc>0);
        for(int i=0;i<16;i++) bmsCells[i]=cells[i];
        bool ms=manualSimMode;
        unlockState();

        bmsSocTimestamp=millis();
        if(soc!=savedSoc){
            savedSoc=soc;
            if(nvsBegin("bms_config", false)){
                preferences.putUInt("last_soc", savedSoc);
                nvsEnd();
            }
        }
        if(!ms){
            pendingOutputSoc = soc;
            pendingOutputUpdate = true;
        }
        jkFrameLen=0;
        return true;
    }
    if(jkFrameLen>=JK_FRAME_SIZE) jkFrameLen=0;
    return false;
}

// ============ BMS CLIENT ============
class MyClientCB : public NimBLEClientCallbacks {
    void onConnect(NimBLEClient* c) override {
        connected=true; bmsProtocol=255; probeSent=false; probeStartMs=0;
        jkFrameLen=0;
        Serial.println("[BMS] ✅ Connected");
        if(!acquyMode) ledSetMode(LED_MODE_BREATHE);
    }
    void onDisconnect(NimBLEClient* c, int reason) override {
        connected=false; pChar=nullptr; pWriteChar=nullptr;
        bmsSocTimestamp=0; probeSent=false; probeStartMs=0; jkFrameLen=0;
        Serial.printf("[BMS] ⚠️ Disconnect (reason=%d)\n", reason);
        if(!acquyMode) ledSetMode(LED_MODE_SOLID);
    }
};

class MyScanCB : public NimBLEScanCallbacks {
    void onResult(const NimBLEAdvertisedDevice* adv) override {
        if(!bleClientInit || acquyMode) return;
        String fm=String(adv->getAddress().toString().c_str());
        fm.toUpperCase();
        String tgt;
        lockState(); tgt=targetMac; unlockState();
        if(tgt!="" && fm==tgt){
            Serial.printf("[SCAN] Found target: %s\n", fm.c_str());
            if(adv->haveName()){ lockState(); pendingScanName=String(adv->getName().c_str()); unlockState(); }
            doConnect=true;
        }
    }
};
static MyScanCB scanCB;

void bleClientStart(){
    if(bleClientInit) return;
    NimBLEScan* ps=NimBLEDevice::getScan();
    ps->setScanCallbacks(&scanCB, false);
    ps->setActiveScan(true);
    ps->setInterval(100);
    ps->setWindow(20);
    ps->setMaxResults(20);
    bleClientInit=true;
    Serial.println("[BLE] Client ready");
}

void bleClientStop(){
    if(!bleClientInit) return;
    doScan=false; doConnect=false; doDisconnect=false;
    unsigned long t0=millis();
    while(isScanning && millis()-t0<3000) delay(50);
    if(pClient && pClient->isConnected()){ pClient->disconnect(); delay(200); }
    connected=false; pChar=nullptr; pWriteChar=nullptr; pClient=nullptr;
    bleClientInit=false;
}

void notifyCb(NimBLERemoteCharacteristic* c, uint8_t* data, size_t len, bool isNotify){
    if(acquyMode) return;
    if(bmsProtocol==255 && len>=4){
        if(data[0]==0x55 && data[1]==0xAA && data[2]==0xEB && data[3]==0x90){
            bmsProtocol=1; probeSent=true;
            Serial.println("[BMS] Protocol: JiKong");
            if(nvsBegin("bms_config", false)){
                preferences.putUChar("bms_proto", 1);
                nvsEnd();
            }
        } else if(data[0]==0xA5 && data[1]==0x7B){
            bmsProtocol=0; probeSent=true;
            Serial.println("[BMS] Protocol: MOVE");
            if(nvsBegin("bms_config", false)){
                preferences.putUChar("bms_proto", 0);
                nvsEnd();
            }
        }
    }
    NimBLERemoteCharacteristic* tw = pWriteChar?pWriteChar:pChar;
    if(bmsProtocol==255 && !probeSent && probeStartMs>0 && millis()-probeStartMs>2000){
        if(tw){ tw->writeValue(jkDeviceInfoCmd, sizeof(jkDeviceInfoCmd), false); probeSent=true; probeStartMs=millis(); }
    }
    if(bmsProtocol==255 && probeSent && probeStartMs>0 && millis()-probeStartMs>4000){
        bmsProtocol=254;
    }
    if(bmsProtocol==1){ parseJkFrame(data, len); return; }
    if(len>=123 && data[0]==0xA5 && data[1]==0x7B){
        uint8_t soc=data[11];
        float tv=((data[20]<<8)|data[19])/100.0f;
        bool hc=(len>=25);
        float cur=0;
        if(hc){ int16_t r=(int16_t)((data[22]<<8)|data[21]); cur=r/100.0f; }
        float pw=tv*cur;
        float cells[16]={0}; uint8_t cc=0;
        if(len>=82){ for(int i=0;i<16;i++){ cells[i]=((data[51+i*2]<<8)|data[50+i*2])/1000.0f; if(cells[i]>0.5f) cc++; } }
        if(soc>100 || tv<1.0f || tv>144.0f) return;
        lockState();
        realBmsSoc=soc; bmsTotalVoltage=tv; bmsHasTotalV=(tv>1.0f);
        bmsHasCurrent=hc; bmsCurrent=cur;
        bmsHasPower=hc; bmsPower=pw;
        bmsCellCount=cc; bmsHasCells=(cc>0);
        if(len>=82) for(int i=0;i<16;i++) bmsCells[i]=cells[i];
        bool ms=manualSimMode;
        unlockState();
        bmsSocTimestamp=millis();
        if(soc!=savedSoc){
            savedSoc=soc;
            if(nvsBegin("bms_config", false)){
                preferences.putUInt("last_soc", savedSoc);
                nvsEnd();
            }
        }
        if(!ms){
            pendingOutputSoc = soc;
            pendingOutputUpdate = true;
        }
    }
}

bool connectBms(){
    if(!bleClientInit || acquyMode){ 
        Serial.println("[BMS] Not ready"); 
        return false; 
    }
    String tgt;
    lockState(); tgt=targetMac; unlockState();
    if(tgt==""){ Serial.println("[BMS] No target MAC"); return false; }
    
    bmsProtocol=255; probeSent=false; probeStartMs=0; jkFrameLen=0;
    
    if(pClient==nullptr){
        pClient=NimBLEDevice::createClient();
        pClient->setClientCallbacks(new MyClientCB(), false);
        pClient->setConnectTimeout(5000);
        Serial.println("[BMS] Client created");
    }
    if(pClient->isConnected()){ pClient->disconnect(); delay(200); }
    
    Serial.printf("[BMS] Connecting to %s (heap=%d)...\n", tgt.c_str(), ESP.getFreeHeap());
    
    NimBLEAddress a1(std::string(tgt.c_str()), BLE_ADDR_RANDOM);
    bool ok=pClient->connect(a1, false);
    if(!ok){
        Serial.println("[BMS] Retry with PUBLIC addr...");
        NimBLEAddress a2(std::string(tgt.c_str()), BLE_ADDR_PUBLIC);
        ok=pClient->connect(a2, false);
    }
    if(!ok || !pClient->isConnected()){
        Serial.printf("[BMS] ❌ Connect failed (ok=%d, conn=%d)\n", ok, pClient->isConnected());
        return false;
    }
    
    Serial.println("[BMS] ✅ GATT connected, getting service...");
    NimBLERemoteService* svc=pClient->getService(serviceUUID);
    if(!svc) svc=pClient->getService("FFE0");
    if(!svc){ Serial.println("[BMS] ❌ No FFE0 service"); pClient->disconnect(); return false; }
    Serial.println("[BMS] ✅ Service found");
    
    pChar=svc->getCharacteristic(charUUID);
    if(!pChar) pChar=svc->getCharacteristic("FFE1");
    if(!pChar){ Serial.println("[BMS] ❌ No FFE1 char"); pClient->disconnect(); return false; }
    Serial.println("[BMS] ✅ Char found");
    
    pWriteChar=svc->getCharacteristic("FFE2");
    if(!pWriteChar || !pWriteChar->canWrite()) pWriteChar=pChar;
    
    if(pChar->canNotify()){
        pChar->subscribe(true, notifyCb);
        Serial.println("[BMS] ✅ Subscribed to notify");
    }
    delay(200);
    
    NimBLERemoteCharacteristic* tw=pWriteChar?pWriteChar:pChar;
    tw->writeValue(loginCmd, sizeof(loginCmd), false);
    delay(80);
    tw->writeValue(requestSocCmd, sizeof(requestSocCmd), false);
    delay(80);
    tw->writeValue(jkDeviceInfoCmd, sizeof(jkDeviceInfoCmd), false);
    probeStartMs=millis();
    
    lockState();
    if(pendingScanName.length()>0){ bmsDeviceName=pendingScanName; pendingScanName=""; }
    else if(bmsDeviceName.length()==0) bmsDeviceName="BMS "+tgt;
    String nameToSave=bmsDeviceName;
    unlockState();
    
    if(nvsBegin("bms_config", false)){
        preferences.putString("bms_name", nameToSave);
        nvsEnd();
    }
    
    Serial.println("[BMS] ✅ Connected and ready");
    return true;
}

void connectTask(void* param){
    Serial.println("[CONN] ⏳ Connect task started");
    delay(100);
    bool ok = connectBms();
    Serial.printf("[CONN] connectBms returned %d\n", ok);
    lockState();
    connectTaskHandle = nullptr;
    unlockState();
    vTaskDelete(nullptr);
}

// ============ BLE TASK ============
void bleTask(void* param){
    unsigned long lastQueryTime=0;
    CmdPacket pkt;
    
    Serial.println("[TASK] BLE task started");
    
    for(;;){
        if(pendingOutputUpdate){
            pendingOutputUpdate=false;
            applyOutput(pendingOutputSoc);
        }
        
        if(doDisconnect){
            doDisconnect=false;
            Serial.println("[TASK] >>> DISCONNECT");
            if(isScanning){ NimBLEScan* ps=NimBLEDevice::getScan(); ps->stop(); isScanning=false; delay(50); }
            if(pClient && pClient->isConnected()){ pClient->disconnect(); delay(200); }
            connected=false; pChar=nullptr; pWriteChar=nullptr; bmsSocTimestamp=0;
            Serial.println("[TASK] <<< DISCONNECT done");
        }
        
        if(doConnect){
            doConnect=false;
            Serial.println("[TASK] >>> CONNECT");
            if(isScanning){ NimBLEScan* ps=NimBLEDevice::getScan(); ps->stop(); isScanning=false; delay(50); }
            NimBLEScan* ps=NimBLEDevice::getScan();
            ps->clearResults();
            
            lockState();
            bool canCreate = (connectTaskHandle == nullptr);
            unlockState();
            
            if(canCreate){
                BaseType_t ret = xTaskCreatePinnedToCore(
                    connectTask, "BMS_Conn", 8192, nullptr, 1, &connectTaskHandle, 1);
                if(ret != pdPASS){
                    Serial.println("[TASK] ❌ Failed to create connect task");
                    lockState(); connectTaskHandle = nullptr; unlockState();
                } else {
                    Serial.println("[TASK] ✅ Connect task created");
                }
            } else {
                Serial.println("[TASK] Connect task already running");
            }
            Serial.println("[TASK] <<< CONNECT queued");
        }
        
        if(cmdQueue && xQueueReceive(cmdQueue, &pkt, 0) == pdTRUE){
            Serial.printf("[QUEUE] Processing: %s\n", pkt.cmd);
            handleBleCommand(String(pkt.cmd));
            Serial.println("[QUEUE] Done");
        }
        
        if(acquyMode){
            if(isScanning){ NimBLEScan* s=NimBLEDevice::getScan(); if(s) s->stop(); isScanning=false; }
            if(pClient && pClient->isConnected()) pClient->disconnect();
            connected=false; pChar=nullptr; pWriteChar=nullptr;
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }
        if(!bleClientInit){ vTaskDelay(pdMS_TO_TICKS(200)); continue; }

        // ===== Manual scan (từ web) =====
        if(doScan && !scanBusy){
            doScan=false;
            scanBusy=true;
            Serial.println("[TASK] Manual scan start");
            NimBLEScan* ps=NimBLEDevice::getScan();
            if(isScanning){ ps->stop(); isScanning=false; delay(100); }
            ps->clearResults();
            ps->setActiveScan(true);
            isScanning=true;
            NimBLEScanResults res=ps->getResults(4000, false);
            isScanning=false;
            String json="[";
            int cnt=0;
            for(int i=0;i<res.getCount();i++){
                const NimBLEAdvertisedDevice* d=res.getDevice(i);
                if(!d) continue;
                bool bms=false;
                if(d->isAdvertisingService(serviceUUID)) bms=true;
                if(d->haveName()){
                    String nm=String(d->getName().c_str());
                    nm.toUpperCase();
                    if(nm.startsWith("JK_")||nm.startsWith("JK-")||nm.startsWith("JK02")||nm.indexOf("BMS")>=0) bms=true;
                }
                if(!bms) continue;
                String mac=String(d->getAddress().toString().c_str());
                mac.toUpperCase();
                String name=d->haveName()?String(d->getName().c_str()):"BMS";
                name.replace("\"","\\\"");
                if(cnt>0) json+=",";
                json+="{\"mac\":\""+mac+"\",\"name\":\""+name+"\"}";
                cnt++;
            }
            json+="]";
            ps->clearResults();
            String out="{\"t\":\"scan\",\"list\":"+json+"}";
            lockState();
            scanResultJson=json;
            scanReady=true;
            unlockState();
            Serial.printf("[TASK] Manual scan done: %d BMS\n", cnt);
            bleSendJson(out);
            scanBusy=false;
            continue;
        }

        // ===== AUTO SCAN — quan trọng =====
        lockState();
        bool taskRunning = (connectTaskHandle != nullptr);
        String tgt = targetMac;
        unlockState();
        
        if(!scanBusy && !acquyMode && !connected && !doConnect && !doDisconnect && !doScan && !isScanning && !taskRunning){
            unsigned long now = millis();
            if(forceScanNow || (now - lastAutoScanAttempt > 3000)){
                lastAutoScanAttempt = now;
                forceScanNow = false;
                
                Serial.printf("[AUTO] Scanning... (target=%s)\n", tgt.length()>0 ? tgt.c_str() : "ANY");
                
                NimBLEScan* ps=NimBLEDevice::getScan();
                if(isScanning){ ps->stop(); isScanning=false; delay(50); }
                ps->clearResults();
                ps->setActiveScan(true);
                isScanning=true;
                NimBLEScanResults res=ps->getResults(4000, false);
                isScanning=false;
                
                // Thu thập BMS tìm thấy
                String foundMac="";
                String foundName="";
                int bmsCount=0;
                String firstMac="";
                String firstName="";
                
                for(int i=0;i<res.getCount();i++){
                    const NimBLEAdvertisedDevice* d=res.getDevice(i);
                    if(!d) continue;
                    bool bms=false;
                    if(d->isAdvertisingService(serviceUUID)) bms=true;
                    if(d->haveName()){
                        String nm=String(d->getName().c_str());
                        nm.toUpperCase();
                        if(nm.startsWith("JK_")||nm.startsWith("JK-")||nm.startsWith("JK02")||nm.indexOf("BMS")>=0) bms=true;
                    }
                    if(!bms) continue;
                    
                    String mac=String(d->getAddress().toString().c_str());
                    mac.toUpperCase();
                    String name=d->haveName()?String(d->getName().c_str()):"BMS";
                    
                    Serial.printf("[AUTO] Found BMS: %s (%s)\n", mac.c_str(), name.c_str());
                    
                    if(bmsCount == 0){
                        firstMac = mac;
                        firstName = name;
                    }
                    bmsCount++;
                    
                    if(tgt.length() > 0 && mac == tgt){
                        foundMac = mac;
                        foundName = name;
                        break;
                    }
                }
                
                ps->clearResults();
                
                if(tgt.length() > 0){
                    // Có MAC target
                    if(foundMac.length() > 0){
                        Serial.printf("[AUTO] ✅ Found target %s, connecting...\n", foundMac.c_str());
                        lockState();
                        pendingScanName = foundName;
                        unlockState();
                        doConnect = true;
                    } else {
                        Serial.printf("[AUTO] Target %s not found (%d BMS seen)\n", tgt.c_str(), bmsCount);
                    }
                } else {
                    // Chưa có MAC — nếu chỉ có 1 BMS thì kết nối + lưu MAC
                    if(bmsCount == 1){
                        Serial.printf("[AUTO] ✅ Only 1 BMS: %s (%s), connecting + saving...\n", firstMac.c_str(), firstName.c_str());
                        
                        lockState();
                        targetMac = firstMac;
                        bmsDeviceName = firstName;
                        String nameToSave = firstName;
                        unlockState();
                        
                        if(nvsBegin("bms_config", false)){
                            preferences.putString("bms_mac", firstMac);
                            preferences.putString("bms_name", nameToSave);
                            preferences.putBool("need_auto", false);
                            preferences.putUChar("bms_proto", 255);
                            preferences.putUChar("jk_ver", 0);
                            nvsEnd();
                            Serial.printf("[AUTO] ✅ Saved MAC: %s\n", firstMac.c_str());
                        }
                        
                        lockState();
                        pendingScanName = firstName;
                        unlockState();
                        doConnect = true;
                    } else if(bmsCount > 1){
                        Serial.printf("[AUTO] %d BMS found, need user to select. Waiting...\n", bmsCount);
                    } else {
                        Serial.println("[AUTO] No BMS found, will retry");
                    }
                }
            }
        }

        // Query BMS định kỳ
        if(connected && pChar!=nullptr){
            if(millis()-lastQueryTime>2500){
                lastQueryTime=millis();
                NimBLERemoteCharacteristic* tw=pWriteChar?pWriteChar:pChar;
                if(bmsProtocol==1){
                    static uint8_t jkToggle=0;
                    jkToggle++;
                    if(!jkVersionProbed) tw->writeValue(jkDeviceInfoCmd, sizeof(jkDeviceInfoCmd), false);
                    else {
                        if(jkToggle%2==0) tw->writeValue(jkCellInfoNewCmd, sizeof(jkCellInfoNewCmd), false);
                        else tw->writeValue(jkCellInfoOldCmd, sizeof(jkCellInfoOldCmd), false);
                    }
                } else if(bmsProtocol==0) tw->writeValue(requestSocCmd, sizeof(requestSocCmd), false);
                else if(bmsProtocol==255) tw->writeValue(requestSocCmd, sizeof(requestSocCmd), false);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ============ STATUS ============
// ============ STATUS ============
void sendStatus(){
    if(!uartClientConnected) return;
    lockState();
    
    // ★ Lấy MAC ESP32 (dùng ESP API gốc, không phụ thuộc NimBLE)
    uint64_t mac = ESP.getEfuseMac();
    char espMacStr[18];
    snprintf(espMacStr, sizeof(espMacStr), "%02X:%02X:%02X:%02X:%02X:%02X",
        (uint8_t)(mac >> 40), (uint8_t)(mac >> 32), (uint8_t)(mac >> 24),
        (uint8_t)(mac >> 16), (uint8_t)(mac >> 8), (uint8_t)mac);
    
    String j="{";
    j+="\"t\":\"status\",";
    j+="\"espMac\":\""+String(espMacStr)+"\",";   // ★ MAC ESP32
    j+="\"soc\":"+String(currentSoc)+",";
    j+="\"v\":"+String(bmsHasTotalV?bmsTotalVoltage:0.0f,2)+",";
    j+="\"i\":"+String(bmsHasCurrent?bmsCurrent:0.0f,2)+",";
    j+="\"p\":"+String(bmsHasPower?bmsPower:0.0f,1)+",";
    j+="\"n\":"+String(bmsCellCount)+",";
    j+="\"mode\":\""+String(acquyMode?"ACQ":"BMS")+"\",";
    j+="\"sim\":"+String(manualSimMode?"true":"false")+",";
    j+="\"connected\":"+String(connected?"true":"false")+",";
    j+="\"mac\":\""+targetMac+"\",";              // MAC BMS
    j+="\"bmsName\":\""+bmsDeviceName+"\",";
    j+="\"bmsProto\":"+String((int)bmsProtocol)+",";
    j+="\"jkVer\":"+String((int)jkVersion)+",";
    j+="\"acquyMode\":"+String(acquyMode?"true":"false")+",";
    j+="\"outMode\":"+String(outMode)+",";
    j+="\"dacPin\":"+String(dacPin)+",";
    j+="\"pwmPin\":"+String(pwmPin)+",";
    j+="\"pwmFreq\":"+String(pwmFreq)+",";
    j+="\"pwmRes\":"+String(pwmRes)+",";
    j+="\"hasDac\":"+String(HAS_DAC?1:0)+",";
    j+="\"chipName\":\""+String(CHIP_NAME)+"\",";
    j+="\"battType\":"+String(batteryType)+",";
    j+="\"battCells\":"+String(batteryCells)+",";
    j+="\"aqPin\":"+String(acquyAdcPin)+",";
    j+="\"aqR1\":"+String(acquyR1,2)+",";
    j+="\"aqR2\":"+String(acquyR2,2)+",";
    j+="\"aqGain\":"+String(acquyCalibGain,4)+",";
    j+="\"aqOffset\":"+String(acquyCalibOffset,3)+",";
    j+="\"cells\":[";
    for(int i=0;i<16;i++){ j+=String(bmsCells[i],3); if(i<15) j+=","; }
    j+="],";
    // ★ Gửi vmap hiện tại
    j+="\"vmap\":[";
    for(uint8_t i=0;i<vmapCount;i++){
        j+="{\"s\":"+String(vmap[i].s)+",\"e\":"+String(vmap[i].e)+",\"v\":"+String(vmap[i].v,3)+"}";
        if(i<vmapCount-1) j+=",";
    }
    j+="],";
    // ★ Gửi voltage thực tế đang xuất
    {
        float outV = 0;
        if(outMode==OUT_MODE_DAC){
            outV = currentOut * 3.3f / 255.0f;
        } else {
            int maxVal = (1<<pwmRes)-1;
            outV = (float)currentOut * 3.3f / (float)maxVal;
        }
        j+="\"voltage\":"+String(outV,2)+",";
    }
    j+="\"fw\":{\"ver\":\""+String(FW_VERSION)+"\"},";
    j+="\"aq\":{";
    j+="\"raw\":"+String(acquyRawV,2)+",";
    j+="\"ocv\":"+String(acquyOcvEstimated,2)+",";
    j+="\"sag\":"+String(acquySag,2)+",";
    j+="\"stable\":"+String(acquyStable?"true":"false")+",";
    j+="\"adc\":"+String(acquyAdcRawV,3)+",";
    j+="\"gain\":"+String(acquyCalibGain,4)+",";
    j+="\"offset\":"+String(acquyCalibOffset,3);
    j+="}";
    unlockState();
    j+="}";
    bleSendJson(j);
}

// ============ COMMAND HANDLER ============
void handleBleCommand(const String& line){
    String cmd=line;
    cmd.trim();
    if(cmd.length()==0) return;

    Serial.printf("[CMD] %s\n", cmd.c_str());

    if(cmd=="HELLO"){ sendAck("OK"); sendStatus(); return; }
    if(cmd=="GET_STATUS"){ sendStatus(); return; }

    if(cmd.startsWith("SOC:")){
        uint8_t v=constrain(cmd.substring(4).toInt(),0,100);
        lockState(); manualSimMode=true; unlockState();
        applyOutput(v);
        sendAck("OK");
        return;
    }
    if(cmd=="REAL"){
        uint8_t soc; bool conn;
        lockState(); manualSimMode=false; soc=realBmsSoc; conn=connected||acquyMode; unlockState();
        if(conn && soc>0) applyOutput(soc);
        sendAck("OK");
        return;
    }

    if(cmd=="MODE:BMS"){
        if(acquyMode){
            acquyMode=false;
            if(nvsBegin("bms_config", false)){
                preferences.putBool("aq_en", false);
                nvsEnd();
            }
            bleClientStart();
            if(connected) ledSetMode(LED_MODE_BREATHE);
            else ledSetMode(LED_MODE_SOLID);
        }
        sendAck("OK");
        return;
    }
    if(cmd=="MODE:ACQ"){
        if(!acquyMode){
            acquyMode=true;
            if(nvsBegin("bms_config", false)){
                preferences.putBool("aq_en", true);
                nvsEnd();
            }
            bleClientStop();
            acquyMedianFilled=false; acquyMedianIdx=0;
            acquyEmaInit=false; acquyStable=false; acquyStableStart=0;
            compFactorLearned=(batteryType==BATT_TYPE_LFP)?0.40f:0.70f;
            socFilteredInit=false;
            acquyLastStableUpdate=0;
            acquyLastStableV=0.0f;
            ledSetMode(LED_MODE_SOLID);
        }
        sendAck("OK");
        return;
    }

    if(cmd=="SCAN"){ doScan=true; sendAck("OK"); return; }

    if(cmd.startsWith("SAVE_MAC:")){
        String rest=cmd.substring(9);
        int c=rest.indexOf('|');
        String mac=(c>0)?rest.substring(0,c):rest;
        String name=(c>0)?rest.substring(c+1):"";
        mac.toUpperCase(); mac.trim();
        if(mac.length()!=17){ sendErr("MAC invalid"); return; }

        Serial.printf("[CMD] SAVE_MAC %s\n", mac.c_str());
        sendAck("OK");
        
        lockState();
        targetMac=mac;
        if(name.length()>0) bmsDeviceName=name;
        else if(bmsDeviceName.length()==0) bmsDeviceName="BMS "+mac;
        String nameToSave=bmsDeviceName;
        unlockState();

        Serial.println("[NVS] Writing MAC...");
        if(nvsBegin("bms_config", false)){
            preferences.putString("bms_mac", targetMac);
            preferences.putString("bms_name", nameToSave);
            preferences.putBool("need_auto", false);
            preferences.putUChar("bms_proto", 255);
            preferences.putUChar("jk_ver", 0);
            nvsEnd();
            Serial.println("[NVS] ✅ MAC saved");
        } else {
            Serial.println("[NVS] ❌ Failed to write MAC");
        }
        
        delay(50);
        if(nvsBegin("bms_config", true)){
            String verify = preferences.getString("bms_mac", "");
            nvsEnd();
            if(verify == targetMac){
                Serial.printf("[NVS] ✅ Verify OK: %s\n", verify.c_str());
            } else {
                Serial.printf("[NVS] ❌ Verify FAIL: saved='%s' expected='%s'\n", 
                    verify.c_str(), targetMac.c_str());
            }
        }
        
        jkVersion=0; jkVersionProbed=false; needAutoConnect=false;
        forceScanNow = true;  // ★ Ép scan ngay lập tức
        doDisconnect=true; doConnect=true;
        Serial.println("[CMD] SAVE_MAC done, force scan");
        return;
    }

    if(cmd=="CLEAR_MAC"){
        sendAck("OK");
        lockState(); targetMac=""; bmsDeviceName=""; unlockState();
        if(nvsBegin("bms_config", false)){
            preferences.putString("bms_mac", "");
            preferences.putString("bms_name", "");
            preferences.putBool("need_auto", true);
            preferences.putUChar("bms_proto", 255);
            preferences.putUChar("jk_ver", 0);
            nvsEnd();
        }
        jkVersion=0; jkVersionProbed=false;
        forceScanNow = true;  // ★ Ép scan tìm BMS mới
        return;
    }

    if(cmd.startsWith("SAVE_OUT:")){
        String b=cmd.substring(9);
        int p1=b.indexOf(','); int p2=b.indexOf(',',p1+1); int p3=b.indexOf(',',p2+1);
        if(p1<0||p2<0||p3<0){ sendErr("bad out"); return; }
        int m=b.substring(0,p1).toInt();
        int pin=b.substring(p1+1,p2).toInt();
        int frq=b.substring(p2+1,p3).toInt();
        int res=b.substring(p3+1).toInt();
#if !HAS_DAC
        if(m==OUT_MODE_DAC){ sendErr("no DAC"); return; }
#endif
        Serial.printf("[SAVE_OUT] mode=%d, pin=%d, freq=%d, res=%d\n", m, pin, frq, res);
        sendAck("OK");
        
        if(outMode==OUT_MODE_DAC){
#if HAS_DAC
            dacWrite(dacPin,0); pinMode(dacPin, INPUT);
#endif
        } else disablePWM();
        outMode=m;
        if(outMode==OUT_MODE_DAC){ dacPin=pin; }
        else { pwmPin=pin; pwmFreq=frq; pwmRes=res; }
        if(outMode==OUT_MODE_DAC){
#if HAS_DAC
            pinMode(dacPin, OUTPUT); dacWrite(dacPin,0);
#endif
        } else { pinMode(pwmPin, OUTPUT); setupPWM(); }
        
        Serial.println("[NVS] Writing OUT...");
        if(nvsBegin("bms_config", false)){
            preferences.putInt("out_mode", outMode);
            preferences.putInt("dac_pin", dacPin);
            preferences.putInt("pwm_pin", pwmPin);
            preferences.putInt("pwm_freq", pwmFreq);
            preferences.putInt("pwm_res", pwmRes);
            nvsEnd();
            Serial.println("[NVS] ✅ OUT saved");
        } else {
            Serial.println("[NVS] ❌ Failed to write OUT");
        }
        
        delay(50);
        if(nvsBegin("bms_config", true)){
            int sm=preferences.getInt("out_mode", -1);
            int sdp=preferences.getInt("dac_pin", -1);
            int spp=preferences.getInt("pwm_pin", -1);
            nvsEnd();
            Serial.printf("[NVS] Verify OUT: mode=%d/%d, dacPin=%d/%d, pwmPin=%d/%d\n",
                sm, outMode, sdp, dacPin, spp, pwmPin);
        }
        
        applyOutput(currentSoc);
        return;
    }

    if(cmd.startsWith("SET_VMAP:")){
        String body=cmd.substring(9);
        int sc=body.indexOf(';');
        if(sc<0){ sendErr("bad vmap"); return; }
        int count=body.substring(0,sc).toInt();
        if(count<1||count>MAX_VMAP_ENTRIES){ sendErr("count"); return; }
        VMapEntry tmp[MAX_VMAP_ENTRIES];
        int idx=0, pos=sc+1;
        while(idx<count && pos<(int)body.length()){
            int end=body.indexOf(';',pos);
            String item=(end<0)?body.substring(pos):body.substring(pos,end);
            int c1=item.indexOf(',');
            int c2=item.indexOf(',',c1+1);
            if(c1<0||c2<0) break;
            tmp[idx].s=constrain(item.substring(0,c1).toInt(),0,100);
            tmp[idx].e=constrain(item.substring(c1+1,c2).toInt(),0,100);
            tmp[idx].v=constrain(item.substring(c2+1).toFloat(),0.0f,3.3f);
            if(tmp[idx].s>tmp[idx].e){ uint8_t t=tmp[idx].s; tmp[idx].s=tmp[idx].e; tmp[idx].e=t; }
            idx++;
            if(end<0) break;
            pos=end+1;
        }
        if(idx<1){ sendErr("empty"); return; }
        for(int i=0;i<idx-1;i++)
            for(int j=i+1;j<idx;j++)
                if(tmp[j].s<tmp[i].s){ VMapEntry t=tmp[i]; tmp[i]=tmp[j]; tmp[j]=t; }
        lockState();
        vmapCount=idx;
        for(int i=0;i<idx;i++) vmap[i]=tmp[i];
        uint8_t soc=currentSoc;
        unlockState();
        if(nvsBegin("bms_config", false)){
            preferences.putUChar("vmap_cnt", vmapCount);
            for(int i=0;i<MAX_VMAP_ENTRIES;i++){
                String base="vm"+String(i)+"_";
                if(i<vmapCount){
                    preferences.putUChar((base+"s").c_str(), vmap[i].s);
                    preferences.putUChar((base+"e").c_str(), vmap[i].e);
                    preferences.putFloat((base+"v").c_str(), vmap[i].v);
                } else {
                    preferences.remove((base+"s").c_str());
                    preferences.remove((base+"e").c_str());
                    preferences.remove((base+"v").c_str());
                }
            }
            nvsEnd();
        }
        applyOutput(soc);
        sendAck("OK");
        return;
    }

    if(cmd.startsWith("SET_OCV:")){
        String body=cmd.substring(8);
        int c1=body.indexOf(':');
        if(c1<0){ sendErr("bad ocv"); return; }
        String type=body.substring(0,c1);
        String rest=body.substring(c1+1);
        int sc=rest.indexOf(';');
        if(sc<0){ sendErr("no entries"); return; }
        int count=rest.substring(0,sc).toInt();
        if(count<2||count>MAX_OCV_POINTS){ sendErr("count"); return; }
        OcvPoint tmp[MAX_OCV_POINTS];
        int idx=0, pos=sc+1;
        while(idx<count && pos<(int)rest.length()){
            int end=rest.indexOf(';',pos);
            String item=(end<0)?rest.substring(pos):rest.substring(pos,end);
            int comma=item.indexOf(',');
            if(comma<0) break;
            tmp[idx].v=item.substring(0,comma).toFloat();
            int s=item.substring(comma+1).toInt();
            if(tmp[idx].v<=0||s<0||s>100) break;
            tmp[idx].soc=(uint8_t)s;
            idx++;
            if(end<0) break;
            pos=end+1;
        }
        if(idx<2){ sendErr("need 2+"); return; }
        for(int i=0;i<idx-1;i++)
            for(int j=i+1;j<idx;j++)
                if(tmp[j].v>tmp[i].v){ OcvPoint t=tmp[i]; tmp[i]=tmp[j]; tmp[j]=t; }
        lockState();
        if(type=="lead"){ leadOcvN=idx; for(int i=0;i<idx;i++) leadOcvTable[i]=tmp[i]; }
        else { lfpOcvN=idx; for(int i=0;i<idx;i++) lfpOcvTable[i]=tmp[i]; }
        unlockState();
        if(nvsBegin("bms_ocv", false)){
            if(type=="lead"){
                preferences.putUChar("lead_n", idx);
                for(int i=0;i<idx;i++){ String b="l"+String(i)+"_";
                    preferences.putFloat((b+"v").c_str(), tmp[i].v);
                    preferences.putUChar((b+"s").c_str(), tmp[i].soc); }
            } else {
                preferences.putUChar("lfp_n", idx);
                for(int i=0;i<idx;i++){ String b="f"+String(i)+"_";
                    preferences.putFloat((b+"v").c_str(), tmp[i].v);
                    preferences.putUChar((b+"s").c_str(), tmp[i].soc); }
            }
            acquyJsonDate="BLE";
            preferences.putString("date", acquyJsonDate);
            nvsEnd();
        }
        sendAck("OK");
        return;
    }

    if(cmd=="GET_OCV"){
        String j="{\"t\":\"ocv\",\"date\":\""+acquyJsonDate+"\",";
        j+="\"lead\":[";
        for(uint8_t i=0;i<leadOcvN;i++){
            j+="{\"v\":"+String(leadOcvTable[i].v,3)+",\"soc\":"+String(leadOcvTable[i].soc)+"}";
            if(i<leadOcvN-1) j+=",";
        }
        j+="],\"lfp\":[";
        for(uint8_t i=0;i<lfpOcvN;i++){
            j+="{\"v\":"+String(lfpOcvTable[i].v,3)+",\"soc\":"+String(lfpOcvTable[i].soc)+"}";
            if(i<lfpOcvN-1) j+=",";
        }
        j+="]}";
        bleSendJson(j);
        return;
    }

    if(cmd.startsWith("SAVE_ACQ:")){
        String b=cmd.substring(9);
        float vals[7]={0}; int cnt=0, pos=0;
        while(cnt<7 && pos<(int)b.length()){
            int c=b.indexOf(',',pos);
            String v=(c<0)?b.substring(pos):b.substring(pos,c);
            vals[cnt++]=v.toFloat();
            if(c<0) break;
            pos=c+1;
        }
        if(cnt<7){ sendErr("bad acq"); return; }
        acquyAdcPin=(int)vals[0];
        acquyR1=vals[1]; acquyR2=vals[2];
        acquyCalibGain=vals[3]; acquyCalibOffset=vals[4];
        batteryType=(int)vals[5]; batteryCells=(int)vals[6];
        if(batteryType!=BATT_TYPE_LEAD && batteryType!=BATT_TYPE_LFP) batteryType=BATT_TYPE_LEAD;
        if(batteryCells<1) batteryCells=1;
        if(batteryCells>32) batteryCells=32;
        if(acquyAdcPin<0||acquyAdcPin>39) acquyAdcPin=32;
        if(acquyR1<0.1f) acquyR1=100.0f;
        if(acquyR2<0.1f) acquyR2=4.7f;
        
        Serial.printf("[SAVE_ACQ] pin=%d, r1=%.1f, r2=%.1f, gain=%.4f, offset=%.3f, btype=%d, cells=%d\n",
            acquyAdcPin, acquyR1, acquyR2, acquyCalibGain, acquyCalibOffset, batteryType, batteryCells);
        sendAck("OK");
        
        Serial.println("[NVS] Writing ACQ...");
        if(nvsBegin("bms_config", false)){
            preferences.putInt("aq_pin", acquyAdcPin);
            preferences.putFloat("aq_r1", acquyR1);
            preferences.putFloat("aq_r2", acquyR2);
            preferences.putInt("batt_type", batteryType);
            preferences.putInt("batt_cells", batteryCells);
            preferences.putFloat("aq_gain", acquyCalibGain);
            preferences.putFloat("aq_off", acquyCalibOffset);
            nvsEnd();
            Serial.println("[NVS] ✅ ACQ saved");
        } else {
            Serial.println("[NVS] ❌ Failed to write ACQ");
        }
        
        delay(50);
        if(nvsBegin("bms_config", true)){
            int sp=preferences.getInt("aq_pin", -1);
            int sbt=preferences.getInt("batt_type", -1);
            int sc=preferences.getInt("batt_cells", -1);
            float sg=preferences.getFloat("aq_gain", -1);
            nvsEnd();
            Serial.printf("[NVS] Verify ACQ: pin=%d/%d, btype=%d/%d, cells=%d/%d, gain=%.4f/%.4f\n",
                sp, acquyAdcPin, sbt, batteryType, sc, batteryCells, sg, acquyCalibGain);
        }
        
        pinMode(acquyAdcPin, INPUT);
        analogSetPinAttenuation(acquyAdcPin, ACQY_ADC_ATTEN);
        acquyMedianFilled=false; acquyMedianIdx=0;
        acquyEmaInit=false; acquyStable=false; acquyStableStart=0;
        compFactorLearned=(batteryType==BATT_TYPE_LFP)?0.40f:0.70f;
        socFilteredInit=false;
        acquyLastStableUpdate=0;
        acquyLastStableV=0.0f;
        return;
    }

    if(cmd.startsWith("SET_LOG:")){
        logModulesEnabled=(uint16_t)cmd.substring(8).toInt();
        if(nvsBegin("bms_config", false)){
            preferences.putUShort("log_mask", logModulesEnabled);
            nvsEnd();
        }
        sendAck("OK");
        return;
    }

    sendErr("Unknown: "+cmd);
}

// ============ BLE CALLBACKS ============
class UartRxCB : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& connInfo) override {
        std::string s=c->getValue();
        if(s.empty()) return;
        
        static char rxBuffer[512];
        static size_t rxLen = 0;
        static unsigned long lastDataTime = 0;
        
        if(rxLen > 0 && millis() - lastDataTime > 3000){
            Serial.println("[BLE] RX timeout, reset buffer");
            rxLen = 0;
        }
        lastDataTime = millis();
        
        size_t copyLen = s.length();
        if(rxLen + copyLen >= sizeof(rxBuffer)){
            Serial.println("[BLE] RX buffer overflow, reset");
            rxLen = 0;
        }
        memcpy(rxBuffer + rxLen, s.data(), copyLen);
        rxLen += copyLen;
        
        size_t start = 0;
        for(size_t i = 0; i < rxLen; i++){
            if(rxBuffer[i] == '\n'){
                size_t cmdLen = i - start;
                if(cmdLen > 0 && cmdQueue){
                    CmdPacket pkt;
                    size_t cLen = (cmdLen < CMD_BUF_SIZE-1) ? cmdLen : (CMD_BUF_SIZE-1);
                    memcpy(pkt.cmd, rxBuffer + start, cLen);
                    pkt.cmd[cLen] = '\0';
                    if(xQueueSend(cmdQueue, &pkt, 0) != pdTRUE){
                        Serial.println("[QUEUE] Full, dropped");
                    }
                }
                start = i + 1;
            }
        }
        
        if(start > 0){
            size_t remain = rxLen - start;
            if(remain > 0){
                memmove(rxBuffer, rxBuffer + start, remain);
            }
            rxLen = remain;
        }
    }
};

class OtaRxCB : public NimBLECharacteristicCallbacks {
    void onWrite(NimBLECharacteristic* c, NimBLEConnInfo& connInfo) override {
        std::string s=c->getValue();
        const uint8_t* d=(const uint8_t*)s.data();
        size_t n=s.length();
        if(n==0) return;
        if(n>4 && memcmp(d,"OTA_",4)==0){
            String cmd((const char*)d, n);
            if(cmd.endsWith("\n")) cmd.remove(cmd.length()-1);
            if(cmd.startsWith("OTA_START:")){
                otaTotal=cmd.substring(10).toInt();
                if(otaTotal>0 && Update.begin(otaTotal)){
                    otaActive=true; otaWritten=0;
                    pOtaTx->setValue("ACK"); pOtaTx->notify();
                } else {
                    pOtaTx->setValue("ERR"); pOtaTx->notify();
                }
                return;
            }
            if(cmd=="OTA_END"){
                if(otaActive && Update.end(true)){
                    pOtaTx->setValue("ACK"); pOtaTx->notify();
                    delay(500);
                    ESP.restart();
                } else {
                    pOtaTx->setValue("ERR"); pOtaTx->notify();
                }
                otaActive=false;
                return;
            }
        }
        if(otaActive){
            if(Update.write((uint8_t*)d, n)==n){
                otaWritten+=n;
                pOtaTx->setValue("ACK"); pOtaTx->notify();
            } else {
                pOtaTx->setValue("ERR"); pOtaTx->notify();
                otaActive=false;
            }
        }
    }
};

class ServerCB : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* s, NimBLEConnInfo& info) override {
        uartClientConnected=true;
        uint16_t mtu = s->getPeerMTU(info.getConnHandle());
        Serial.printf("[BLE] Web connected, MTU=%u (payload=%u)\n", mtu, mtu-3);
        if(!NimBLEDevice::getAdvertising()->isAdvertising()){
            NimBLEDevice::startAdvertising();
        }
    }
    void onDisconnect(NimBLEServer* s, NimBLEConnInfo& info, int reason) override {
        uartClientConnected=false;
        Serial.printf("[BLE] Web disconnected (reason=%d)\n", reason);
        NimBLEDevice::startAdvertising();
    }
};

void initBleServer(){
    NimBLEDevice::init("BMS_Controller");
    NimBLEDevice::setMTU(517);
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);

    auto* srv=NimBLEDevice::createServer();
    srv->setCallbacks(new ServerCB());

    auto* svc=srv->createService(UART_SERVICE_UUID);
    auto* rx=svc->createCharacteristic(UART_RX_CHAR_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    rx->setCallbacks(new UartRxCB());
    pUartTx=svc->createCharacteristic(UART_TX_CHAR_UUID, NIMBLE_PROPERTY::NOTIFY);
    svc->start();

    auto* osvc=srv->createService(OTA_SERVICE_UUID);
    auto* orx=osvc->createCharacteristic(OTA_RX_CHAR_UUID,
        NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_NR);
    orx->setCallbacks(new OtaRxCB());
    pOtaTx=osvc->createCharacteristic(OTA_TX_CHAR_UUID, NIMBLE_PROPERTY::NOTIFY);
    osvc->start();

    auto* adv=NimBLEDevice::getAdvertising();
    adv->addServiceUUID(UART_SERVICE_UUID);
    adv->addServiceUUID(OTA_SERVICE_UUID);
    adv->setName("BMS_Controller");
    adv->enableScanResponse(true);
    adv->start();
}

void loadOcvFromNVS(){
    if(!nvsBegin("bms_ocv", true)) return;
    uint8_t nl=preferences.getUChar("lead_n", 0);
    uint8_t nf=preferences.getUChar("lfp_n", 0);
    if(nl>=2 && nl<=MAX_OCV_POINTS){
        leadOcvN=nl;
        for(uint8_t i=0;i<nl;i++){ String b="l"+String(i)+"_";
            leadOcvTable[i].v=preferences.getFloat((b+"v").c_str(), 0);
            leadOcvTable[i].soc=preferences.getUChar((b+"s").c_str(), 0); }
    }
    if(nf>=2 && nf<=MAX_OCV_POINTS){
        lfpOcvN=nf;
        for(uint8_t i=0;i<nf;i++){ String b="f"+String(i)+"_";
            lfpOcvTable[i].v=preferences.getFloat((b+"v").c_str(), 0);
            lfpOcvTable[i].soc=preferences.getUChar((b+"s").c_str(), 0); }
    }
    acquyJsonDate=preferences.getString("date", "");
    nvsEnd();
}

void setup(){
    pinMode(LED_PIN, OUTPUT);
    Serial.begin(115200);
    delay(200);
    ledSetup();
    Serial.println();
    Serial.println(">>> BMS Controller v" FW_VERSION " <<<");

    nvsMutex = xSemaphoreCreateMutex();
    if(!nvsMutex) Serial.println("[BOOT] ❌ NVS mutex failed");
    else Serial.println("[BOOT] ✅ NVS mutex created");

    if(nvsBegin("test", false)){
        preferences.putString("key", "hello");
        String v = preferences.getString("key", "");
        nvsEnd();
        Serial.printf("[NVS] Test: '%s' (%s)\n", v.c_str(), v=="hello"?"✅":"❌");
    } else {
        Serial.println("[NVS] ❌ Test begin failed");
    }

    cmdQueue = xQueueCreate(20, sizeof(CmdPacket));
    if(!cmdQueue){
        Serial.println("[BOOT] ❌ Queue failed");
    } else {
        Serial.println("[BOOT] ✅ Command queue created");
    }

    stateMutex=xSemaphoreCreateMutex();

    if(nvsBegin("bms_config", true)){
        minVoltage=preferences.getFloat("min_v", minVoltage);
        maxVoltage=preferences.getFloat("max_v", maxVoltage);
        targetMac=preferences.getString("bms_mac", "");
        savedSoc=preferences.getUInt("last_soc", 0);
        outMode=preferences.getInt("out_mode", OUT_MODE_DAC);
        dacPin=preferences.getInt("dac_pin", 25);
        pwmPin=preferences.getInt("pwm_pin", 14);
        pwmFreq=preferences.getInt("pwm_freq", 5000);
        pwmRes=preferences.getInt("pwm_res", 10);
        acquyMode=preferences.getBool("aq_en", false);
        acquyAdcPin=preferences.getInt("aq_pin", 32);
        acquyR1=preferences.getFloat("aq_r1", 100.0f);
        acquyR2=preferences.getFloat("aq_r2", 4.7f);
        acquyCalibGain=preferences.getFloat("aq_gain", 1.0f);
        acquyCalibOffset=preferences.getFloat("aq_off", 0.0f);
        batteryType=preferences.getInt("batt_type", BATT_TYPE_LEAD);
        batteryCells=preferences.getInt("batt_cells", 4);
        uint8_t savedProto=preferences.getUChar("bms_proto", 255);
        if(savedProto<=1) bmsProtocol=savedProto;
        jkVersion=preferences.getUChar("jk_ver", 0);
        if(jkVersion>0 && jkVersion<=2) jkVersionProbed=true;
        else { jkVersion=0; jkVersionProbed=false; }
        vmapCount=preferences.getUChar("vmap_cnt", 0);
        if(vmapCount>MAX_VMAP_ENTRIES) vmapCount=MAX_VMAP_ENTRIES;
        for(int i=0;i<vmapCount;i++){ String base="vm"+String(i)+"_";
            vmap[i].s=preferences.getUChar((base+"s").c_str(), 0);
            vmap[i].e=preferences.getUChar((base+"e").c_str(), 0);
            vmap[i].v=preferences.getFloat((base+"v").c_str(), 0.0f); }
        logModulesEnabled=preferences.getUShort("log_mask", LOGMOD_ALL);
        bmsDeviceName=preferences.getString("bms_name", "");
        bool needAuto=preferences.getBool("need_auto", false);
        if(needAuto && targetMac.length()==0) needAutoConnect=true;
        nvsEnd();
    }

    if(acquyCalibGain<0.5f||acquyCalibGain>2.0f) acquyCalibGain=1.0f;
    if(acquyCalibOffset<-5.0f||acquyCalibOffset>5.0f) acquyCalibOffset=0.0f;
    if(batteryType!=BATT_TYPE_LEAD && batteryType!=BATT_TYPE_LFP) batteryType=BATT_TYPE_LEAD;
    if(batteryCells<1||batteryCells>32) batteryCells=4;

    loadOcvFromNVS();

    pinMode(acquyAdcPin, INPUT);
    analogReadResolution(12);
    analogSetAttenuation(ACQY_ADC_ATTEN);
    analogSetPinAttenuation(acquyAdcPin, ACQY_ADC_ATTEN);

    if(outMode!=OUT_MODE_DAC && outMode!=OUT_MODE_PWM) outMode=OUT_MODE_DAC;
#if !HAS_DAC
    if(outMode==OUT_MODE_DAC) outMode=OUT_MODE_PWM;
#endif
    if(dacPin!=25 && dacPin!=26) dacPin=25;
    if(pwmRes<1||pwmRes>14) pwmRes=10;

    if(outMode==OUT_MODE_DAC){
#if HAS_DAC
        pinMode(dacPin, OUTPUT); dacWrite(dacPin, 0);
#endif
    } else { pinMode(pwmPin, OUTPUT); setupPWM(); }

    if(savedSoc>0 && savedSoc<=100) applyOutput(savedSoc);

    initBleServer();
    if(!acquyMode) bleClientStart();

    xTaskCreatePinnedToCore(bleTask, "BLE_Task", 16384, nullptr, 0, nullptr, 0);
    ledSetMode(LED_MODE_CONFIG_BLINK);
    Serial.printf("[BOOT] MAC='%s', outMode=%d, dacPin=%d, pwmPin=%d\n",
        targetMac.c_str(), outMode, dacPin, pwmPin);
    Serial.printf("[BOOT] ACQ: pin=%d, r1=%.1f, r2=%.1f, gain=%.4f, offset=%.3f, btype=%d, cells=%d\n",
        acquyAdcPin, acquyR1, acquyR2, acquyCalibGain, acquyCalibOffset, batteryType, batteryCells);
}

void loop(){
    loopCounter++;
    processAcquyReading();
    ledUpdate();

    static unsigned long lastStatus=0;
    if(millis()-lastStatus>5000){
        lastStatus=millis();
        sendStatus();
    }

    static unsigned long lastMem=0;
    if(millis()-lastMem>15000){
        lastMem=millis();
        Serial.printf("[MEM] Heap=%d\n", ESP.getFreeHeap());
    }

    delay(5);
}
