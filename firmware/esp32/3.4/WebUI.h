#ifndef WEB_UI_H
#define WEB_UI_H

#include "Config.h"
#include "IndexHtml.h"

void handleChipInfo() {
  TOUCH_HTTP();
  uint32_t heapTotal = ESP.getHeapSize();
  uint32_t heapFree = ESP.getFreeHeap();
  uint32_t heapUsed = heapTotal - heapFree;
  uint32_t heapMin = ESP.getMinFreeHeap();
  uint32_t flashSize = ESP.getFlashChipSize();
  uint32_t sketchSize = ESP.getSketchSize();
  uint32_t sketchFree = ESP.getFreeSketchSpace();
  uint32_t cpuFreq = ESP.getCpuFreqMHz();
  uint32_t chipRev = ESP.getChipRevision();
  uint8_t coreCount = ESP.getChipCores();
  unsigned long uptime = millis() / 1000;
  String json = "{";
  json += "\"heapTotal\":" + String(heapTotal) + ",";
  json += "\"heapFree\":" + String(heapFree) + ",";
  json += "\"heapUsed\":" + String(heapUsed) + ",";
  json += "\"heapMin\":" + String(heapMin) + ",";
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
    voltage = currentOut * 3.3f / 255.0f;
  } else {
    int maxDuty = (1 << pwmRes) - 1;
    voltage = currentOut * 3.3f / (float)maxDuty;
  }
  bool isFresh = false;
  unsigned long age = 0;
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
  safeBmsName.replace("\\", "\\\\");
  safeBmsName.replace("\"", "\\\"");
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
  json += "\"aqOcvEst\":" + String(acquyOcvEstimated, 2) + ",";
  json += "\"aqSag\":" + String(acquySag, 2) + ",";
  json += "\"aqSocRaw\":" + String(acquySocRaw) + ",";
  json += "\"aqSocStable\":" + String(acquySocStable) + ",";
  json += "\"aqIsStable\":" + String(acquyStable ? "true" : "false") + ",";
  json += "\"aqTempC\":" + String(temperatureRead(), 1) + ",";
  json += "\"aqCharging\":" + String(acquyCharging ? "true" : "false") + ",";
  json += "\"aqAdcMax\":" + String(calcAdcMaxVoltage(), 3) + ",";
  json += "\"dropThrLead\":" + String(dropThresholdLead, 2) + ",";
  json += "\"dropThrLfp\":" + String(dropThresholdLfp, 2) + ",";
  json += "\"riseThrLead\":" + String(riseThresholdLead, 2) + ",";
  json += "\"riseThrLfp\":" + String(riseThresholdLfp, 2) + ",";
  json += "\"settleLead\":" + String(settleLeadMs / 1000) + ",";
  json += "\"settleLfp\":" + String(settleLfpMs / 1000) + ",";
  json += "\"stableBand\":" + String(stableBandV, 2) + ",";
  json += "\"acqState\":" + String((int)acqState) + ",";
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
  for (int i = 0; i < 16; i++) {
    json += String(bmsCells[i], 3);
    if (i < 15) json += ",";
  }
  json += "]";
  json += ",\"vmapEnabled\":true";
  json += ",\"vmap\":[";
  for (uint8_t i = 0; i < vmapCount; i++) {
    json += "{\"s\":" + String(vmap[i].startPct) + ",\"e\":" + String(vmap[i].endPct) + ",\"v\":" + String(vmap[i].voltage, 2) + "}";
    if (i < vmapCount - 1) json += ",";
  }
  json += "]";
  json += ",\"staEnabled\":" + String(staEnabled ? "true" : "false");
  json += ",\"staConnected\":" + String(staConnected ? "true" : "false");
  json += ",\"apTimeout\":" + String(apTimeoutMs / 1000);
  json += ",\"apSSID\":\"" + apSSID + "\"";
  json += ",\"apPass\":\"" + apPass + "\"";
  json += ",\"apCfgOpen\":" + String(apCfgPanelOpen ? "true" : "false");
  json += ",\"logMask\":" + String(logModulesEnabled);
  json += ",\"staSSID\":\"" + staSSID + "\"";
  json += ",\"staPass\":\"" + staPass + "\"";
  if (staConnected) {
    json += ",\"staIP\":\"" + WiFi.localIP().toString() + "\"";
    json += ",\"staRSSI\":" + String(WiFi.RSSI());
  }
  json += ",\"presetUpdated\":\"" + presetLastUpdated + "\"";
  json += ",\"presetInfoLoaded\":" + String(presetInfoLoaded ? "true" : "false");
  String safeActiveName = activePresetName;
  safeActiveName.replace("\\", "\\\\");
  safeActiveName.replace("\"", "\\\"");
  json += ",\"activePresetName\":\"" + safeActiveName + "\"";
  String safeFwNotes = fwNotes;
  safeFwNotes.replace("\"", "\\\"");
  safeFwNotes.replace("\n", "\\n");
  json += ",\"fwVersion\":\"" + String(FW_VERSION) + "\"";
  json += ",\"fwDate\":\"" + fwDate + "\"";
  json += ",\"fwNotes\":\"" + safeFwNotes + "\"";
  json += ",\"autoUpdate\":" + String(autoUpdateEnabled ? "true" : "false");
  json += ",\"autoUpdateDone\":" + String(autoUpdateDone ? "true" : "false");
  json += ",\"apActive\":" + String(apActive ? "true" : "false");
  if (apActive) {
    uint8_t cc = WiFi.softAPgetStationNum();
    unsigned long ref = (apLastClientTime > 0) ? apLastClientTime : apStartedTime;
    long remain = (long)apTimeoutMs - (long)(millis() - ref);
    if (cc > 0) remain = -1;
    if (!staConnected) remain = -2;
    if (apTimeoutMs == 0) remain = -3;
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
    lockState();
    manualSimMode = true;
    unlockState();
    applyOutput(simSoc);
    server.send(200, "text/plain", "OK");
  } else if (server.hasArg("real")) {
    uint8_t realSoc, cached;
    bool conn;
    lockState();
    manualSimMode = false;
    realSoc = realBmsSoc;
    conn = connected || acquyMode;
    cached = savedSoc;
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

  // ★ FIX: chỉ báo lỗi nếu chọn DAC mà board không có
  if (newMode == OUT_MODE_DAC && !HAS_DAC) {
    server.send(400, "text/plain", "No DAC");
    return;
  }
  if (newMode == OUT_MODE_DAC && newDacPin != 25 && newDacPin != 26) {
    server.send(400, "text/plain", "DAC pin invalid");
    return;
  }
  if (newMode == OUT_MODE_PWM) {
    if (newPwmPin < 0 || newPwmPin > 48) {
      server.send(400, "text/plain", "PWM pin invalid");
      return;
    }
    if (newFreq < 1 || newFreq > 40000000) {
      server.send(400, "text/plain", "Freq invalid");
      return;
    }
    if (newRes < 1 || newRes > 14) {
      server.send(400, "text/plain", "Res invalid");
      return;
    }
  }
  if (outMode == OUT_MODE_DAC) {
    dacWrite(dacPin, 0);
    pinMode(dacPin, INPUT);
  } else disablePWM();
  outMode = newMode;
  dacPin = newDacPin;
  pwmPin = newPwmPin;
  pwmFreq = newFreq;
  pwmRes = newRes;
  if (outMode == OUT_MODE_DAC) {
    pinMode(dacPin, OUTPUT);
    dacWrite(dacPin, 0);
  } else {
    pinMode(pwmPin, OUTPUT);
    setupPWM();
  }
  preferences.begin("bms_config", false);
  preferences.putInt("out_mode", outMode);
  preferences.putInt("dac_pin", dacPin);
  preferences.putInt("pwm_pin", pwmPin);
  preferences.putInt("pwm_freq", pwmFreq);
  preferences.putInt("pwm_res", pwmRes);
  preferences.end();
  applyOutput(currentSoc);
  LOGM(LOGMOD_OUT, "Mode=%s GPIO%d | Freq=%dHz Res=%dbit",
       outMode == OUT_MODE_DAC ? "DAC" : "PWM",
       outMode == OUT_MODE_DAC ? dacPin : pwmPin, pwmFreq, pwmRes);
  server.send(200, "text/plain", "OK");
}

void handleSaveVMap() {
  TOUCH_HTTP();
  int count = server.hasArg("count") ? server.arg("count").toInt() : 0;
  if (count < 0) count = 0;
  if (count > MAX_VMAP_ENTRIES) count = MAX_VMAP_ENTRIES;
  VMapEntry newMap[MAX_VMAP_ENTRIES];
  int newCount = 0;
  for (int i = 0; i < count; i++) {
    String ks = "e" + String(i) + "s";
    String ke = "e" + String(i) + "e";
    String kv = "e" + String(i) + "v";
    if (!server.hasArg(ks) || !server.hasArg(ke) || !server.hasArg(kv)) continue;
    int s = constrain(server.arg(ks).toInt(), 0, 100);
    int e = constrain(server.arg(ke).toInt(), 0, 100);
    float v = constrain(server.arg(kv).toFloat(), 0.0f, 3.3f);
    if (s > e) {
      int t = s;
      s = e;
      e = t;
    }
    newMap[newCount].startPct = (uint8_t)s;
    newMap[newCount].endPct = (uint8_t)e;
    newMap[newCount].voltage = v;
    newCount++;
  }
  for (int i = 0; i < newCount - 1; i++)
    for (int j = i + 1; j < newCount; j++)
      if (newMap[j].startPct < newMap[i].startPct) {
        VMapEntry tmp = newMap[i];
        newMap[i] = newMap[j];
        newMap[j] = tmp;
      }
  for (int i = 1; i < newCount; i++) {
    if (newMap[i].startPct > newMap[i - 1].endPct + 1) newMap[i].startPct = newMap[i - 1].endPct + 1;
    if (newMap[i].startPct <= newMap[i - 1].endPct) newMap[i].startPct = newMap[i - 1].endPct + 1;
  }
  lockState();
  vmapEnabled = true;
  vmapCount = (uint8_t)newCount;
  for (int i = 0; i < newCount; i++) vmap[i] = newMap[i];
  uint8_t soc = currentSoc;
  unlockState();
  activePresetName = "";
  preferences.begin("bms_config", false);
  preferences.putBool("vmap_en", true);
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
  LOGM(LOGMOD_OUT, "vMap: BẬT | %d dải", newCount);
  server.send(200, "text/plain", "OK");
}

void handlePresets() {
  TOUCH_HTTP();
  String json = "[";
  for (int i = 0; i < remotePresetCount; i++) {
    String name = remotePresets[i].name;
    String sub = remotePresets[i].subtitle;
    String rng = remotePresets[i].ranges;
    name.replace("\"", "\\\"");
    sub.replace("\"", "\\\"");
    rng.replace("\"", "\\\"");
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
  if (!server.hasArg("id")) {
    server.send(400, "text/plain", "Missing id");
    return;
  }
  int id = server.arg("id").toInt();
  if (id < 0 || id >= remotePresetCount) {
    server.send(400, "text/plain", "Preset khong ton tai");
    return;
  }
  RemotePreset& p = remotePresets[id];
  if (p.count == 0) {
    server.send(400, "text/plain", "Preset rong");
    return;
  }
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
  LOGM(LOGMOD_OUT, "Preset: %s", p.name.c_str());
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
    LOGM(LOGMOD_OUT, "Đã tải %d preset", remotePresetCount);
  } else server.send(500, "text/plain", "Tai that bai");
}

void handleFetchAcquyJson() {
  TOUCH_HTTP();
  bool ok = fetchAcquyJson();
  if (ok) {
    String resp = "OK|" + String(leadOcvN) + "|" + String(lfpOcvN) + "|" + acquyJsonDate;
    server.send(200, "text/plain", resp);
  } else {
    server.send(500, "text/plain", "FAIL");
  }
}

void handleAcquyJsonInfo() {
  TOUCH_HTTP();
  String json = "{";
  json += "\"date\":\"" + acquyJsonDate + "\",";
  json += "\"leadN\":" + String(leadOcvN) + ",";
  json += "\"lfpN\":" + String(lfpOcvN) + ",";
  json += "\"lead\":[";
  for (uint8_t i = 0; i < leadOcvN; i++) {
    json += "{\"v\":" + String(leadOcvTable[i].v, 3) + ",\"soc\":" + String(leadOcvTable[i].soc) + "}";
    if (i < leadOcvN - 1) json += ",";
  }
  json += "],\"lfp\":[";
  for (uint8_t i = 0; i < lfpOcvN; i++) {
    json += "{\"v\":" + String(lfpOcvTable[i].v, 3) + ",\"soc\":" + String(lfpOcvTable[i].soc) + "}";
    if (i < lfpOcvN - 1) json += ",";
  }
  json += "]}";
  server.sendHeader("Cache-Control", "no-cache");
  server.send(200, "application/json", json);
}

void handleSaveOcvManual() {
  TOUCH_HTTP();
  if (!server.hasArg("type")) {
    server.send(400, "text/plain", "Missing type");
    return;
  }
  String type = server.arg("type");
  int count = server.hasArg("count") ? server.arg("count").toInt() : 0;
  if (count < 2) {
    server.send(400, "text/plain", "Cần >=2 điểm");
    return;
  }
  if (count > MAX_OCV_POINTS) count = MAX_OCV_POINTS;

  OcvPoint newTable[MAX_OCV_POINTS];
  int n = 0;
  for (int i = 0; i < count; i++) {
    String kv = "e" + String(i) + "v";
    String ks = "e" + String(i) + "s";
    if (!server.hasArg(kv) || !server.hasArg(ks)) continue;
    float v = server.arg(kv).toFloat();
    int s = server.arg(ks).toInt();
    if (v <= 0.0f || s < 0 || s > 100) continue;
    newTable[n].v = v;
    newTable[n].soc = (uint8_t)s;
    n++;
  }
  if (n < 2) {
    server.send(400, "text/plain", "Dữ liệu không hợp lệ");
    return;
  }

  for (int i = 0; i < n - 1; i++)
    for (int j = i + 1; j < n; j++)
      if (newTable[j].v > newTable[i].v) {
        OcvPoint t = newTable[i];
        newTable[i] = newTable[j];
        newTable[j] = t;
      }

  for (int i = 1; i < n; i++) {
    if (newTable[i].soc > newTable[i - 1].soc) {
      server.send(400, "text/plain", "SOC phải giảm dần khi V giảm");
      return;
    }
  }

  lockState();
  if (type == "lead") {
    leadOcvN = n;
    for (int i = 0; i < n; i++) leadOcvTable[i] = newTable[i];
  } else {
    lfpOcvN = n;
    for (int i = 0; i < n; i++) lfpOcvTable[i] = newTable[i];
  }
  unlockState();

  preferences.begin("bms_ocv", false);
  if (type == "lead") {
    preferences.putUChar("lead_n", n);
    for (int i = 0; i < n; i++) {
      String b = "l" + String(i) + "_";
      preferences.putFloat((b + "v").c_str(), newTable[i].v);
      preferences.putUChar((b + "s").c_str(), newTable[i].soc);
    }
  } else {
    preferences.putUChar("lfp_n", n);
    for (int i = 0; i < n; i++) {
      String b = "f" + String(i) + "_";
      preferences.putFloat((b + "v").c_str(), newTable[i].v);
      preferences.putUChar((b + "s").c_str(), newTable[i].soc);
    }
  }
  acquyJsonDate = "Thủ công";
  preferences.putString("date", acquyJsonDate);
  preferences.end();

  LOGM(LOGMOD_ADC, "💾 Lưu OCV %s: %d điểm", type.c_str(), n);
  server.send(200, "text/plain", "OK|" + String(n));
}

void handleSaveWifi() {
  TOUCH_HTTP();
  if (!server.hasArg("ssid")) {
    server.send(400, "text/plain", "Missing ssid");
    return;
  }
  String newSSID = server.arg("ssid");
  String newPass = server.hasArg("pass") ? server.arg("pass") : "";
  bool enable = server.hasArg("en") && server.arg("en") == "1";
  newSSID.trim();
  if (newSSID.length() == 0) {
    server.send(400, "text/plain", "SSID rong");
    return;
  }
  if (newSSID.length() > 32) newSSID = newSSID.substring(0, 32);
  if (newPass.length() > 64) newPass = newPass.substring(0, 64);
  staSSID = newSSID;
  staPass = newPass;
  staEnabled = enable;
  preferences.begin("bms_config", false);
  preferences.putString("sta_ssid", staSSID);
  preferences.putString("sta_pass", staPass);
  preferences.putBool("sta_en", staEnabled);
  preferences.end();
  WiFi.disconnect(false, false);
  staConnected = false;
  staAttempting = false;
  if (staEnabled) {
    if (apActive) WiFi.mode(WIFI_AP_STA);
    else WiFi.mode(WIFI_STA);
    staStart();
  } else staStop();
  LOGM(LOGMOD_WIFI, "Lưu SSID: %s | en=%d", staSSID.c_str(), staEnabled);
  server.send(200, "text/plain", "OK");
}

void handleSaveSta() {
  TOUCH_HTTP();
  if (!server.hasArg("en")) {
    server.send(400, "text/plain", "Missing arg");
    return;
  }
  bool en = (server.arg("en") == "1");
  staEnabled = en;
  preferences.begin("bms_config", false);
  preferences.putBool("sta_en", staEnabled);
  preferences.end();
  if (en && staSSID != "") {
    if (apActive) WiFi.mode(WIFI_AP_STA);
    else WiFi.mode(WIFI_STA);
    staStart();
  } else if (!en) {
    staStop();
  }
  LOGM(LOGMOD_WIFI, "STA: %s", en ? "BẬT" : "TẮT");
  server.send(200, "text/plain", "OK");
}

void handleSetLogModules() {
  TOUCH_HTTP();
  if (!server.hasArg("mask")) {
    server.send(400, "text/plain", "Missing mask");
    return;
  }
  logModulesEnabled = (uint16_t)strtoul(server.arg("mask").c_str(), nullptr, 10);
  preferences.begin("bms_config", false);
  preferences.putUShort("log_mask", logModulesEnabled);
  preferences.end();
  LOGM(LOGMOD_BOOT, "Log modules = 0x%04X", logModulesEnabled);
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
                               "<button style='width:auto;' onclick=\"pickWifi('"
              + safeSsid + "')\">Chọn</button></li>";
    }
  }
  WiFi.scanDelete();
  server.send(200, "text/html", html);
}

void handleSave() {
  TOUCH_HTTP();
  if (!server.hasArg("mac")) {
    server.send(400, "text/plain", "Missing mac");
    return;
  }

  // ★ BƯỚC 1: Khai báo + validate newMac TRƯỚC
  String newMac = server.arg("mac");
  newMac.toUpperCase();
  newMac.trim();
  if (newMac.length() != 17) {
    server.send(400, "text/plain", "MAC sai do dai");
    return;
  }
  for (int i = 0; i < 17; i++) {
    char c = newMac[i];
    if (i % 3 == 2) {
      if (c != ':') {
        server.send(400, "text/plain", "Thieu :");
        return;
      }
    } else {
      if (!isxdigit(c)) {
        server.send(400, "text/plain", "Ky tu khong hop le");
        return;
      }
    }
  }

  String newName = server.hasArg("name") ? server.arg("name") : "";
  newName.trim();
  if (newName.length() > 40) newName = newName.substring(0, 40);

  // ★ BƯỚC 2: Reset dữ liệu BMS cũ (SAU khi đã validate newMac OK)
  lockState();
  bmsTotalVoltage = 0.0f;
  bmsHasTotalV = false;
  bmsCurrent = 0.0f;
  bmsHasCurrent = false;
  bmsPower = 0.0f;
  bmsHasPower = false;
  bmsCellCount = 0;
  bmsHasCells = false;
  for (int i = 0; i < 16; i++) bmsCells[i] = 0.0f;
  bmsSocTimestamp = 0;
  realBmsSoc = 0;

  // ★ BƯỚC 3: Gán targetMac + name
  targetMac = newMac;
  if (newName.length() > 0) bmsDeviceName = newName;
  else if (bmsDeviceName.length() == 0) bmsDeviceName = "BMS " + newMac;
  String nameToSave = bmsDeviceName;
  unlockState();
  cachedJsonData = "";
  cachedJsonTime = 0;

  // ★ BƯỚC 4: Lưu NVS
  preferences.begin("bms_config", false);
  preferences.putString("bms_mac", targetMac);
  preferences.putString("bms_name", nameToSave);
  preferences.putBool("need_auto", false);
  preferences.putUChar("bms_proto", 255);
  preferences.putUChar("jk_ver", 0);
  preferences.end();
  jkVersion = 0;
  jkVersionProbed = false;
  needAutoConnect = false;
  if (!acquyMode) {
    doDisconnect = true;
    doConnect = true;
  }
  LOGM(LOGMOD_BLE, "Lưu MAC: %s", newMac.c_str());
  server.send(200, "text/plain", "OK");
}

void handleClearMac() {
  TOUCH_HTTP();
  lockState();
  targetMac = "";
  bmsDeviceName = "";
  // ★ Reset dữ liệu BMS cũ để UI không hiển thị số liệu "ma"
  bmsTotalVoltage = 0.0f;
  bmsHasTotalV = false;
  bmsCurrent = 0.0f;
  bmsHasCurrent = false;
  bmsPower = 0.0f;
  bmsHasPower = false;
  bmsCellCount = 0;
  bmsHasCells = false;
  for (int i = 0; i < 16; i++) bmsCells[i] = 0.0f;
  bmsSocTimestamp = 0;
  realBmsSoc = 0;
  unlockState();
  cachedJsonData = "";
  cachedJsonTime = 0;

  preferences.begin("bms_config", false);
  preferences.putString("bms_mac", "");
  preferences.putString("bms_name", "");
  preferences.putBool("need_auto", true);
  preferences.putUChar("bms_proto", 255);
  preferences.putUChar("jk_ver", 0);
  preferences.end();
  jkVersion = 0;
  jkVersionProbed = false;
  // ★ Ngắt BLE hiện tại để tránh nhận thêm dữ liệu từ BMS cũ
  if (pClient && pClient->isConnected()) {
    pClient->disconnect();
  }
  connected = false;
  pChar = nullptr;
  pWriteChar = nullptr;
  doConnect = false;
  needAutoConnect = true;

  LOGM(LOGMOD_BLE, "Xóa MAC");
  server.send(200, "text/plain", "OK");
}

void handleScan() {
  TOUCH_HTTP();
  if (acquyMode) {
    server.send(200, "text/html", "<li style='color:#888;'>Đang bật chế độ ẮC QUY. Hãy tắt để quét.</li>");
    return;
  }
  lockState();
  scanReady = false;
  scanResultHtml = "";
  unlockState();
  doScan = true;
  unsigned long t0 = millis();
  while (millis() - t0 < 5000) {
    lockState();
    bool ok = scanReady;
    unlockState();
    if (ok) break;
    delay(50);
  }
  String html;
  lockState();
  if (scanReady) {
    html = scanResultHtml;
    scanReady = false;
  }
  unlockState();
  if (html.length() > 0) server.send(200, "text/html", html);
  else server.send(200, "text/html", "<li style='color:#888;'>Không có kết quả</li>");
}

void handleToggleAcquy() {
  TOUCH_HTTP();
  if (!server.hasArg("en")) {
    server.send(400, "text/plain", "Missing arg");
    return;
  }
  acquyMode = (server.arg("en") == "1");
  preferences.begin("bms_config", false);
  preferences.putBool("aq_en", acquyMode);
  preferences.end();
  if (acquyMode) {
    bleDeinit();
    acquyMedianFilled = false;
    acquyMedianIdx = 0;
    acquyEmaInit = false;
    acquyStable = false;
    acquyStableStart = 0;
    socFilteredInit = false;
    acquyLastStableUpdate = 0;
    acquyLastStableV = 0.0f;
    acqState = ACQ_IDLE;
    acqIsRise = false;
    dropBufIdx = 0;
    for (uint8_t i = 0; i < DROP_BUF_SIZE; i++) {
      dropBuf[i] = 0.0f;
      dropBufTs[i] = 0;
    }
    if (!apActive) ledSetMode(LED_MODE_SOLID);
    LOGM(LOGMOD_ADC, "Chế độ ẮC QUY bật | GPIO%d", acquyAdcPin);
  } else {
    // ★ RESET dữ liệu Acquy cũ để UI không hiển thị số liệu "ma"
    lockState();
    bmsTotalVoltage = 0.0f;
    bmsHasTotalV = false;
    bmsCurrent = 0.0f;
    bmsHasCurrent = false;
    bmsPower = 0.0f;
    bmsHasPower = false;
    bmsCellCount = 0;
    bmsHasCells = false;
    for (int i = 0; i < 16; i++) bmsCells[i] = 0.0f;
    bmsSocTimestamp = 0;
    acquyRawV = 0.0f;
    acquyStableV = 0.0f;
    acquyOcvEstimated = 0.0f;
    acquySag = 0.0f;
    acquyMeasuredV = 0.0f;
    acquyAdcRawV = 0.0f;
    realBmsSoc = 0;
    unlockState();
    // ★ Xoá cache JSON để UI cập nhật ngay (không đợi 800ms)
    cachedJsonData = "";
    cachedJsonTime = 0;

    bleInit();
    bleAutoScan = true;
    if (!apActive) ledSetMode(connected ? LED_MODE_BREATHE : LED_MODE_SOLID);
    LOGM(LOGMOD_ADC, "Chế độ ẮC QUY tắt | BLE bật lại");
  }
  server.send(200, "text/plain", "OK");
}

void handleSaveAcquyCfg() {
  TOUCH_HTTP();
  if (server.hasArg("pin")) acquyAdcPin = server.arg("pin").toInt();
  if (server.hasArg("r1")) acquyR1 = server.arg("r1").toFloat();
  if (server.hasArg("r2")) acquyR2 = server.arg("r2").toFloat();
  if (server.hasArg("btype")) batteryType = server.arg("btype").toInt();
  if (server.hasArg("cells")) batteryCells = server.arg("cells").toInt();
  if (server.hasArg("gain")) acquyCalibGain = server.arg("gain").toFloat();
  if (server.hasArg("offset")) acquyCalibOffset = server.arg("offset").toFloat();
  if (batteryType != BATT_TYPE_LEAD && batteryType != BATT_TYPE_LFP) batteryType = BATT_TYPE_LEAD;
  if (batteryCells < 1) batteryCells = 1;
  if (batteryCells > 32) batteryCells = 32;
  if (acquyAdcPin < 0 || acquyAdcPin > 39) acquyAdcPin = 32;
  if (acquyR1 < 0.1f) acquyR1 = 100.0f;
  if (acquyR2 < 0.1f) acquyR2 = 4.7f;
  if (acquyCalibGain < 0.5f || acquyCalibGain > 2.0f) acquyCalibGain = 1.0f;
  if (acquyCalibOffset < -5.0f || acquyCalibOffset > 5.0f) acquyCalibOffset = 0.0f;
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
  analogReadResolution(12);
  analogSetAttenuation(ACQY_ADC_ATTEN);
  acquyMedianFilled = false;
  acquyMedianIdx = 0;
  acquyEmaInit = false;
  acquyStable = false;
  acquyStableStart = 0;
  socFilteredInit = false;
  acquyLastStableUpdate = 0;
  acquyLastStableV = 0.0f;
  acqState = ACQ_IDLE;
  acqIsRise = false;
  dropBufIdx = 0;
  for (uint8_t i = 0; i < DROP_BUF_SIZE; i++) {
    dropBuf[i] = 0.0f;
    dropBufTs[i] = 0;
  }
  LOGM(LOGMOD_ADC, "Lưu cấu hình: %s %d cell | R1=%.1fk R2=%.1fk | Gain=%.4f Offset=%.3f",
       batteryType == BATT_TYPE_LFP ? "LFP" : "Chì", batteryCells, acquyR1, acquyR2, acquyCalibGain, acquyCalibOffset);
  server.send(200, "text/plain", "OK");
}

void handleSaveAcqParams() {
  TOUCH_HTTP();
  if (server.hasArg("dropLead")) dropThresholdLead = server.arg("dropLead").toFloat();
  if (server.hasArg("dropLfp")) dropThresholdLfp = server.arg("dropLfp").toFloat();
  if (server.hasArg("riseLead")) riseThresholdLead = server.arg("riseLead").toFloat();
  if (server.hasArg("riseLfp")) riseThresholdLfp = server.arg("riseLfp").toFloat();
  if (server.hasArg("settleLead")) settleLeadMs = server.arg("settleLead").toInt() * 1000UL;
  if (server.hasArg("settleLfp")) settleLfpMs = server.arg("settleLfp").toInt() * 1000UL;
  if (server.hasArg("band")) stableBandV = server.arg("band").toFloat();

  dropThresholdLead = constrain(dropThresholdLead, 0.1f, 5.0f);
  dropThresholdLfp = constrain(dropThresholdLfp, 0.1f, 2.0f);
  riseThresholdLead = constrain(riseThresholdLead, 0.1f, 5.0f);
  riseThresholdLfp = constrain(riseThresholdLfp, 0.1f, 2.0f);
  settleLeadMs = constrain(settleLeadMs, 1000UL, 120000UL);
  settleLfpMs = constrain(settleLfpMs, 1000UL, 120000UL);
  stableBandV = constrain(stableBandV, 0.05f, 2.0f);

  preferences.begin("bms_config", false);
  preferences.putFloat("drop_lead", dropThresholdLead);
  preferences.putFloat("drop_lfp", dropThresholdLfp);
  preferences.putFloat("rise_lead", riseThresholdLead);
  preferences.putFloat("rise_lfp", riseThresholdLfp);
  preferences.putUInt("settle_lead", settleLeadMs);
  preferences.putUInt("settle_lfp", settleLfpMs);
  preferences.putFloat("stable_band", stableBandV);
  preferences.end();

  LOGM(LOGMOD_ADC, "Params: drop LFP=%.2f Chì=%.2f | rise LFP=%.2f Chì=%.2f | settle LFP=%lus Chì=%lus | band=%.2f",
       dropThresholdLfp, dropThresholdLead, riseThresholdLfp, riseThresholdLead,
       settleLfpMs / 1000, settleLeadMs / 1000, stableBandV);
  server.send(200, "text/plain", "OK");
}

void handleSaveApCfg() {
  TOUCH_HTTP();
  String newSSID = server.hasArg("ssid") ? server.arg("ssid") : "";
  String newPass = server.hasArg("pass") ? server.arg("pass") : "";
  newSSID.trim();

  if (newSSID.length() == 0 || newSSID.length() > 32) {
    server.send(400, "text/plain", "SSID phải 1-32 ký tự");
    return;
  }
  if (newPass.length() < 8 || newPass.length() > 63) {
    server.send(400, "text/plain", "Mật khẩu phải 8-63 ký tự");
    return;
  }

  apSSID = newSSID;
  apPass = newPass;

  preferences.begin("bms_config", false);
  preferences.putString("ap_ssid", apSSID);
  preferences.putString("ap_pass", apPass);
  preferences.end();

  LOGM(LOGMOD_WIFI, "Lưu AP: %s/%s", apSSID.c_str(), apPass.c_str());

  server.send(200, "text/plain", "OK");
  delay(1000);
  if (apActive) {
    if (dnsServerStarted) {
      dnsServer.stop();
      dnsServerStarted = false;
    }
    WiFi.softAPdisconnect(true);
    delay(200);
    apActive = false;
    apStart();
  }
}

void handleSaveApTimeout() {
  TOUCH_HTTP();
  if (!server.hasArg("sec")) {
    server.send(400, "text/plain", "Missing sec");
    return;
  }
  int sec = server.arg("sec").toInt();
  if (sec != 0 && (sec < AP_TIMEOUT_MIN_S || sec > AP_TIMEOUT_MAX_S)) {
    String err = "Phải là 0 (luôn bật) hoặc " + String(AP_TIMEOUT_MIN_S) + "-" + String(AP_TIMEOUT_MAX_S) + "s";
    server.send(400, "text/plain", err);
    return;
  }
  apTimeoutMs = (unsigned long)sec * 1000UL;

  preferences.begin("bms_config", false);
  preferences.putUInt("ap_timeout", sec);
  preferences.end();

  LOGM(LOGMOD_WIFI, "AP timeout: %ds", sec);
  server.send(200, "text/plain", "OK");
}

void handleSaveApCfgPanel() {
  TOUCH_HTTP();
  if (!server.hasArg("en")) {
    server.send(400, "text/plain", "Missing arg");
    return;
  }
  apCfgPanelOpen = (server.arg("en") == "1");
  preferences.begin("bms_config", false);
  preferences.putBool("ap_cfg_open", apCfgPanelOpen);
  preferences.end();
  server.send(200, "text/plain", "OK");
}

void handleSaveFwInfo() {
  TOUCH_HTTP();
  if (!server.hasArg("date") || !server.hasArg("notes")) {
    server.send(400, "text/plain", "Missing arg");
    return;
  }
  fwDate = server.arg("date");
  fwNotes = server.arg("notes");
  if (fwDate.length() > 32) fwDate = fwDate.substring(0, 32);
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
  if (!server.hasArg("en")) {
    server.send(400, "text/plain", "Missing arg");
    return;
  }
  bool en = server.arg("en") == "1";
  bool wasEnabled = autoUpdateEnabled;
  autoUpdateEnabled = en;
  if (en && !wasEnabled) {
    autoUpdateDone = false;
    lastAutoCheck = 0;
  }
  preferences.begin("bms_config", false);
  preferences.putBool("auto_update", autoUpdateEnabled);
  preferences.end();
  LOGM(LOGMOD_OTA, "Auto update: %s", en ? "BẬT" : "TẮT");
  server.send(200, "text/plain", "OK");
}

void handleUpdateDone() {
  server.sendHeader("Connection", "close");
  if (Update.hasError()) {
    String err = "Update failed. Error #";
    err += Update.errorString();
    LOGM_ERR("Update failed: %s", err.c_str());
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
    doScan = false;
    doConnect = false;
    doDisconnect = false;
    if (pClient && pClient->isConnected()) pClient->disconnect();
    connected = false;
    pChar = nullptr;
    pWriteChar = nullptr;
    LOGM(LOGMOD_OTA, "Bắt đầu nạp: %s", upload.filename.c_str());
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial);
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) Update.printError(Serial);
  } else if (upload.status == UPLOAD_FILE_END) {
    if (!Update.end(true)) {
      LOGM_ERR("Update.end lỗi");
      Update.printError(Serial);
    } else {
      LOGM(LOGMOD_OTA, "Nạp thành công: %u bytes", upload.totalSize);
    }
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
  if (!staConnected) {
    server.send(200, "text/plain", "NO_WIFI");
    return;
  }
  bool ok = fetchUpdateInfo();
  if (!ok) {
    server.send(500, "text/plain", "FETCH_FAIL");
    return;
  }
  if (remoteNewer) {
    bool updated = false;
    if (remoteNotes.length() > 0 && remoteNotes != fwNotes) {
      fwNotes = remoteNotes;
      updated = true;
    }
    if (remoteDate.length() > 0 && remoteDate != fwDate) {
      fwDate = remoteDate;
      updated = true;
    }
    if (updated) {
      preferences.begin("bms_config", false);
      preferences.putString("fw_date", fwDate);
      preferences.putString("fw_notes", fwNotes);
      preferences.end();
    }
    String resp = "NEW:";
    resp += remoteVersion + ":" + remoteDate + ":0:1:" + remoteFwUrl;
    LOGM(LOGMOD_OTA, "Có bản mới: v%s (hiện v%s)", remoteVersion.c_str(), FW_VERSION);
    server.send(200, "text/plain", resp);
    return;
  }
  bool synced = false;
  if (fwNotes.length() == 0 || fwDate.length() == 0) {
    if (remoteNotes.length() > 0) fwNotes = remoteNotes;
    if (remoteDate.length() > 0) fwDate = remoteDate;
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
  if (!staConnected) {
    server.send(400, "text/plain", "Can WiFi nha");
    return;
  }
  if (remoteFwUrl.length() == 0) {
    server.send(400, "text/plain", "Chua co URL");
    return;
  }
  otaInProgress = true;
  WiFi.setSleep(false);
  doScan = false;
  doConnect = false;
  doDisconnect = false;
  if (pClient && pClient->isConnected()) {
    pClient->disconnect();
    delay(500);
  }
  connected = false;
  pChar = nullptr;
  pWriteChar = nullptr;
  delay(1000);
  server.sendHeader("Connection", "close");
  server.send(200, "text/plain; charset=utf-8", "OK");
  delay(200);
  LOGM(LOGMOD_OTA, "Bắt đầu nạp online từ: %s", remoteFwUrl.c_str());
  HTTPClient http;
  WiFiClient client;
  WiFiClientSecure clientSecure;
  if (remoteFwUrl.startsWith("https://")) {
    clientSecure.setInsecure();
    http.begin(clientSecure, remoteFwUrl);
  } else http.begin(client, remoteFwUrl);
  http.setTimeout(60000);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  int code = http.GET();
  if (code != 200) {
    LOGM_ERR("OTA HTTP %d", code);
    otaInProgress = false;
    http.end();
    return;
  }
  int contentLen = http.getSize();
  if (contentLen <= 0) {
    LOGM_ERR("OTA size lỗi");
    otaInProgress = false;
    http.end();
    return;
  }
  if (!Update.begin(contentLen)) {
    LOGM_ERR("Update.begin lỗi");
    otaInProgress = false;
    http.end();
    return;
  }
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
        if (written % 102400 < 1024) {
          LOGM(LOGMOD_OTA, "Nạp: %d/%d (%d%%)", written, contentLen, (written * 100) / contentLen);
        }
      }
    } else delay(10);
  }
  http.end();
  if (written != contentLen) {
    LOGM_ERR("Nạp thiếu: %d/%d", written, contentLen);
    Update.abort();
    otaInProgress = false;
    return;
  }
  if (Update.end(true)) {
    LOGM(LOGMOD_OTA, "Nạp thành công — restart");
    delay(2000);
    ESP.restart();
  }
  otaInProgress = false;
}

void handleCaptivePortal() {
  TOUCH_HTTP();
  server.sendHeader("Location", "http://192.168.4.1/", true);
  server.send(302, "text/plain", "");
}

// =====================================================
// TRANG CHỦ (chỉ gọi hàm từ IndexHtml.h)
// =====================================================
void handleRoot() {
  TOUCH_HTTP();
  sendIndexHtml();
}

void registerWebRoutes() {
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
  server.on("/saveacqparams", handleSaveAcqParams);
  server.on("/savewifi", handleSaveWifi);
  server.on("/savesta", handleSaveSta);
  server.on("/saveapcfg", handleSaveApCfg);
  server.on("/saveaptimeout", handleSaveApTimeout);
  server.on("/saveapcfgpanel", handleSaveApCfgPanel);
  server.on("/setlogmodules", handleSetLogModules);
  server.on("/wifiscan", handleWifiScan);
  server.on("/fetchpresets", handleFetchPresets);
  server.on("/presets", handlePresets);
  server.on("/applypreset", handleApplyPreset);
  server.on("/savefwinfo", handleSaveFwInfo);
  server.on("/getnotes", handleGetNotes);
  server.on("/saveautoupdate", handleSaveAutoUpdate);
  server.on("/checkupdate", handleCheckUpdate);
  server.on("/otaonline", HTTP_GET, handleOtaOnline);
  server.on(
    "/update", HTTP_POST, []() {
      handleUpdateDone();
    },
    []() {
      handleUpdateUpload();
    });
  server.on("/fetchacquyjson", handleFetchAcquyJson);
  server.on("/acquyjsoninfo", handleAcquyJsonInfo);
  server.on("/saveocvmanual", handleSaveOcvManual);

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
}

#endif