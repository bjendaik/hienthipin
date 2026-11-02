#include "Config.h"
#include "SystemUtils.h"
#include "AcquyManager.h"
#include "BleBms.h"
#include "CloudOta.h"
#include "IndexHtml.h"
#include "WebUI.h"

// =====================================================
// SETUP
// =====================================================
void setup() {
  pinMode(LED_PIN, OUTPUT);
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(150);
    digitalWrite(LED_PIN, LOW);
    delay(150);
  }
  digitalWrite(LED_PIN, HIGH);

  MySerial.begin(115200);
  delay(100);
  ledSetup();
  MySerial.println("\n>>> Khởi động Hệ thống v" FW_VERSION " <<<");

  WiFi.persistent(false);
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  delay(50);

  stateMutex = xSemaphoreCreateMutex();

  // ★ Đọc cấu hình từ NVS
  preferences.begin("bms_config", false);
  minVoltage = preferences.getFloat("min_v", minVoltage);
  maxVoltage = preferences.getFloat("max_v", maxVoltage);
  targetMac = preferences.getString("bms_mac", "");
  savedSoc = preferences.getUInt("last_soc", 0);
  outMode = preferences.getInt("out_mode", OUT_MODE_DAC);
  dacPin = preferences.getInt("dac_pin", 25);
  pwmPin = preferences.getInt("pwm_pin", 14);
  pwmFreq = preferences.getInt("pwm_freq", 5000);
  pwmRes = preferences.getInt("pwm_res", 10);

  acquyMode = preferences.getBool("aq_en", false);
  acquyAdcPin = preferences.getInt("aq_pin", 32);
  acquyR1 = preferences.getFloat("aq_r1", 100.0f);
  acquyR2 = preferences.getFloat("aq_r2", 4.7f);
  acquyCalibGain = preferences.getFloat("aq_gain", 1.0f);
  acquyCalibOffset = preferences.getFloat("aq_off", 0.0f);

  float oldCalib = preferences.getFloat("aq_cal", 0.0f);
  if (oldCalib > 0.01f && acquyCalibGain == 1.0f) acquyCalibGain = oldCalib;
  if (acquyCalibGain < 0.5f || acquyCalibGain > 2.0f) acquyCalibGain = 1.0f;
  if (acquyCalibOffset < -5.0f || acquyCalibOffset > 5.0f) acquyCalibOffset = 0.0f;

  batteryType = preferences.getInt("batt_type", BATT_TYPE_LEAD);
  batteryCells = preferences.getInt("batt_cells", 4);
  dropThresholdLead = preferences.getFloat("drop_lead", 1.00f);
  dropThresholdLfp = preferences.getFloat("drop_lfp", 0.50f);
  riseThresholdLead = preferences.getFloat("rise_lead", 1.00f);
  riseThresholdLfp = preferences.getFloat("rise_lfp", 0.50f);
  settleLeadMs = preferences.getUInt("settle_lead", 5000UL);
  settleLfpMs = preferences.getUInt("settle_lfp", 3000UL);
  stableBandV = preferences.getFloat("stable_band", 0.50f);

  if (batteryType != BATT_TYPE_LEAD && batteryType != BATT_TYPE_LFP) batteryType = BATT_TYPE_LEAD;
  if (batteryCells < 1 || batteryCells > 32) batteryCells = 4;

  uint8_t savedProto = preferences.getUChar("bms_proto", 255);
  if (savedProto <= 1) bmsProtocol = savedProto;
  jkVersion = preferences.getUChar("jk_ver", 0);
  if (jkVersion > 0 && jkVersion <= 2) jkVersionProbed = true;
  else {
    jkVersion = 0;
    jkVersionProbed = false;
  }

  vmapEnabled = true;
  vmapCount = preferences.getUChar("vmap_cnt", 0);
  if (vmapCount > MAX_VMAP_ENTRIES) vmapCount = MAX_VMAP_ENTRIES;
  for (int i = 0; i < vmapCount; i++) {
    String base = "vm" + String(i) + "_";
    vmap[i].startPct = preferences.getUChar((base + "s").c_str(), 0);
    vmap[i].endPct = preferences.getUChar((base + "e").c_str(), 0);
    vmap[i].voltage = preferences.getFloat((base + "v").c_str(), 0.0f);
  }

  staSSID = preferences.getString("sta_ssid", "");
  staPass = preferences.getString("sta_pass", "");
  staEnabled = preferences.getBool("sta_en", false);

  apSSID = preferences.getString("ap_ssid", "BMS_Config");
  apPass = preferences.getString("ap_pass", "12345678");
  if (apSSID.length() == 0 || apSSID.length() > 32) apSSID = "BMS_Config";
  if (apPass.length() < 8 || apPass.length() > 63) apPass = "12345678";

  uint32_t savedTimeout = preferences.getUInt("ap_timeout", 60);
  if (savedTimeout != 0 && savedTimeout < AP_TIMEOUT_MIN_S) savedTimeout = AP_TIMEOUT_MIN_S;
  if (savedTimeout > AP_TIMEOUT_MAX_S) savedTimeout = AP_TIMEOUT_MAX_S;
  apTimeoutMs = (unsigned long)savedTimeout * 1000UL;

  apCfgPanelOpen = preferences.getBool("ap_cfg_open", false);

  logModulesEnabled = preferences.getUShort("log_mask", LOGMOD_ALL);
  fwDate = preferences.getString("fw_date", "");
  fwNotes = preferences.getString("fw_notes", "");
  activePresetName = preferences.getString("active_preset", "");
  autoUpdateEnabled = preferences.getBool("auto_update", false);
  autoUpdateDone = false;

  bmsDeviceName = preferences.getString("bms_name", "");
  bool needAuto = preferences.getBool("need_auto", false);
  if (needAuto && targetMac.length() == 0) needAutoConnect = true;
  else needAutoConnect = false;

  preferences.end();

  // ★ Nạp bảng OCV từ NVS
  loadOcvFromNVS();

  pinMode(acquyAdcPin, INPUT);
  analogReadResolution(12);
  analogSetAttenuation(ACQY_ADC_ATTEN);
#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32S2) || defined(CONFIG_IDF_TARGET_ESP32C3) || defined(CONFIG_IDF_TARGET_ESP32C6) || defined(CONFIG_IDF_TARGET_ESP32H2)
  analogSetPinAttenuation(acquyAdcPin, ACQY_ADC_ATTEN);
#endif

  LOGM(LOGMOD_BOOT, "Firmware v%s", FW_VERSION);
  LOGM(LOGMOD_BOOT, "Chip: %s | DAC=%d", CHIP_NAME, HAS_DAC);
  LOGM(LOGMOD_BOOT, "ADC: pin=GPIO%d atten=%d", acquyAdcPin, (int)ACQY_ADC_ATTEN);
  LOGM(LOGMOD_BOOT, "Log mask: 0x%04X", logModulesEnabled);
  LOGM(LOGMOD_BOOT, "AP: SSID=%s | Timeout=%lus", apSSID.c_str(), apTimeoutMs / 1000);

  // ★ Nạp danh sách Preset từ NVS
  preferences.begin("bms_presets", false);
  int n = preferences.getInt("count", 0);
  if (n > MAX_REMOTE_PRESETS) n = MAX_REMOTE_PRESETS;
  remotePresetCount = 0;
  for (int i = 0; i < n; i++) {
    String base = "p" + String(i) + "_";
    remotePresets[i].name = preferences.getString((base + "n").c_str(), "");
    remotePresets[i].subtitle = preferences.getString((base + "s").c_str(), "");
    remotePresets[i].ranges = preferences.getString((base + "r").c_str(), "");
    uint8_t c = preferences.getUChar((base + "c").c_str(), 0);
    if (c > MAX_ENTRIES_PER_PRESET) c = MAX_ENTRIES_PER_PRESET;
    remotePresets[i].count = c;
    for (int j = 0; j < c; j++) {
      String ek = base + "e" + String(j) + "_";
      remotePresets[i].entries[j].startPct = preferences.getUChar((ek + "s").c_str(), 0);
      remotePresets[i].entries[j].endPct = preferences.getUChar((ek + "e").c_str(), 0);
      remotePresets[i].entries[j].voltage = preferences.getFloat((ek + "v").c_str(), 0.0f);
    }
    if (remotePresets[i].name.length() > 0) remotePresetCount++;
  }
  preferences.end();

  if (outMode != OUT_MODE_DAC && outMode != OUT_MODE_PWM) outMode = OUT_MODE_DAC;
  if (dacPin != 25 && dacPin != 26) dacPin = 25;
  if (pwmRes < 1 || pwmRes > 14) pwmRes = 10;
  if (minVoltage >= maxVoltage) {
    minVoltage = 2.07f;
    maxVoltage = 2.35f;
  }
  targetMac.toUpperCase();

  if (outMode == OUT_MODE_DAC && !HAS_DAC) outMode = OUT_MODE_PWM;
  if (outMode == OUT_MODE_DAC) {
    pinMode(dacPin, OUTPUT);
    dacWrite(dacPin, 0);
  } else {
    pinMode(pwmPin, OUTPUT);
    setupPWM();
  }

  if (savedSoc > 0 && savedSoc <= 100) applyOutput(savedSoc);

  if (!acquyMode) {
    bleInit();
    bleAutoScan = true;
  } else {
    bleEnabled = false;
    bleInitialized = false;
  }

  xTaskCreatePinnedToCore(bleTask, "BLE_Task", 8192, nullptr, 0, &bleTaskHandle, 0);
  ledSetMode(LED_MODE_CONFIG_BLINK);

  // ★ Đăng ký toàn bộ HTTP Endpoints từ WebUI.h
  registerWebRoutes();

  apStart();
  server.begin();

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

  static unsigned long lastSumLog = 0;
  if (millis() - lastSumLog > 60000) {
    lastSumLog = millis();
    LOGM(LOGMOD_SUM, "SOC=%d%% Heap=%luKB CPU=%.0f%% | BMS=%s WiFi=%s",
         currentSoc, ESP.getFreeHeap() / 1024, cpuLoadAvg,
         acquyMode ? "ACQ" : (connected ? "OK" : "---"),
         apActive ? "AP" : (staConnected ? "STA" : "OFF"));
  }

  autoUpdateTask();
  delay(2);
}
