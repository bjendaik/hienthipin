#ifndef SYSTEM_UTILS_H
#define SYSTEM_UTILS_H

#include "Config.h"

void getBatteryLimits(float& vEmpty, float& vFull, float& vMaxWarn) {
  if (batteryType == BATT_TYPE_LFP) {
    vEmpty = LFP_V_EMPTY * batteryCells;
    vFull = LFP_V_FULL * batteryCells;
    vMaxWarn = LFP_V_MAX * batteryCells;
  } else {
    vEmpty = LEAD_V_EMPTY * batteryCells;
    vFull = LEAD_V_FULL * batteryCells;
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

void ledSetup() {
  ledcDetach(LED_PIN);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
}

void ledSetMode(LedMode mode) {
  currentLedMode = mode;
  ledcDetach(LED_PIN);
  pinMode(LED_PIN, OUTPUT);
  if (mode == LED_MODE_SOLID) digitalWrite(LED_PIN, HIGH);
  else if (mode == LED_MODE_CONFIG_BLINK) digitalWrite(LED_PIN, LOW);
  else {
    ledcAttach(LED_PIN, LED_PWM_FREQ, LED_PWM_RES);
    ledcWrite(LED_PIN, 20);
  }
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
      if (millis() - lastToggle >= 300) {
        lastToggle = millis();
        toggleState = !toggleState;
        digitalWrite(LED_PIN, toggleState ? HIGH : LOW);
      }
      break;
    case LED_MODE_SOLID: digitalWrite(LED_PIN, HIGH); break;
    case LED_MODE_BREATHE:
      breatheVal += breatheDir * 15;
      if (breatheVal >= 255) {
        breatheVal = 255;
        breatheDir = -1;
      }
      if (breatheVal <= 10) {
        breatheVal = 10;
        breatheDir = 1;
      }
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
  lastCheck = now;
  lastLoop = loopCounter;
  float maxLoops = elapsed / 5.0f;
  if (maxLoops < 1) maxLoops = 1;
  float load = 100.0f - (loops / maxLoops * 100.0f);
  if (load < 0) load = 0;
  if (load > 100) load = 100;
  loadSmooth = loadSmooth * 0.5f + load * 0.5f;
  cpuLoadAvg = loadSmooth;
  cpuLoad0 = cpuLoadAvg;
  cpuLoad1 = cpuLoadAvg;
}

void staStart() {
  if (staSSID == "") return;
  staAttempting = true;
  staAttemptStart = millis();
  LOGM(LOGMOD_WIFI, "STA kết nối '%s'...", staSSID.c_str());
  if (apActive) WiFi.mode(WIFI_AP_STA);
  else WiFi.mode(WIFI_STA);
  WiFi.begin(staSSID.c_str(), staPass.c_str());
}

void staStop() {
  WiFi.disconnect(false, false);
  staConnected = false;
  staAttempting = false;
  if (apActive) WiFi.mode(WIFI_AP_STA);
  else WiFi.mode(WIFI_OFF);
  LOGM(LOGMOD_WIFI, "STA đã ngắt");
}

void staCheck() {
  if (!staEnabled || staSSID == "") return;
  wl_status_t st = WiFi.status();
  if (st == WL_CONNECTED) {
    if (!staConnected) {
      staConnected = true;
      staAttempting = false;
      LOGM(LOGMOD_WIFI, "✅ STA OK | IP=%s RSSI=%d dBm",
           WiFi.localIP().toString().c_str(), WiFi.RSSI());
    }
    return;
  }
  if (staConnected) {
    staConnected = false;
    staAttempting = false;
    LOGM(LOGMOD_WIFI, "⚠️ STA mất kết nối");
    if (!apActive) {
      LOGM(LOGMOD_WIFI, "STA mất → bật lại AP");
      apStart();
      ledSetMode(LED_MODE_CONFIG_BLINK);
    }
    return;
  }
  if (staAttempting) {
    if (millis() - staAttemptStart > STA_TIMEOUT_MS) {
      staAttempting = false;
      staEnabled = false;
      staConnected = false;
      LOGM(LOGMOD_WIFI, "⛔ STA timeout 15s → TẮT STA");
      preferences.begin("bms_config", false);
      preferences.putBool("sta_en", false);
      preferences.end();
      if (apActive) WiFi.mode(WIFI_AP);
      else {
        WiFi.disconnect(false, false);
        WiFi.mode(WIFI_OFF);
      }
    }
    return;
  }
}

void apStart() {
  if (apActive) return;
  if (staEnabled && staSSID != "") WiFi.mode(WIFI_AP_STA);
  else WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  WiFi.softAP(apSSID.c_str(), apPass.c_str(), 1);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  apActive = true;
  apStartedTime = millis();
  apLastClientTime = 0;
  apLastClientCount = 0;
  apLastCheck = millis();
  LOGM(LOGMOD_WIFI, "📡 Phát WiFi: %s/%s | IP=%s",
       apSSID.c_str(), apPass.c_str(), WiFi.softAPIP().toString().c_str());
  if (!dnsServerStarted) {
    dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
    dnsServerStarted = true;
  }
}

void apStop() {
  if (!apActive) return;
  if (!staConnected) {
    apLastClientTime = millis();
    return;
  }
  LOGM(LOGMOD_WIFI, "📴 Tắt AP (STA có IP)");
  if (dnsServerStarted) {
    dnsServer.stop();
    dnsServerStarted = false;
  }
  WiFi.softAPdisconnect(true);
  delay(50);
  WiFi.mode(WIFI_STA);
  delay(50);
  apActive = false;
  if (!acquyMode && connected) ledSetMode(LED_MODE_BREATHE);
  else ledSetMode(LED_MODE_SOLID);
}

void apWatchdog() {
  if (!apActive) return;
  if (otaInProgress) return;
  if (apTimeoutMs == 0) {
    apLastClientTime = millis();
    return;
  }  // ★ 0 = luôn bật
  if (millis() - apLastCheck < AP_CHECK_INTERVAL) return;
  apLastCheck = millis();
  uint8_t cc = WiFi.softAPgetStationNum();
  if (cc > 0) {
    apLastClientTime = millis();
    apLastClientCount = cc;
    return;
  }
  if (lastHttpRequestTime > 0) {
    if (millis() - lastHttpRequestTime < HTTP_KEEPALIVE_MS) {
      apLastClientTime = millis();
      apLastClientCount = 1;
      return;
    }
  }
  if (apLastClientCount > 0) {
    apLastClientCount = 0;
    apLastClientTime = millis();
    return;
  }
  if (!staConnected) {
    apLastClientTime = millis();
    return;
  }
  unsigned long ref = (apLastClientTime > 0) ? apLastClientTime : apStartedTime;
  if (millis() - ref >= apTimeoutMs) apStop();
}

float getMappedVoltage(uint8_t soc) {
  if (vmapCount == 0) return -1.0f;
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
    if (mappedV >= 0.0f) {
      outValue = constrain((int)round((mappedV / 3.3f) * 255.0f), 0, 255);
    } else {
      int minDac = constrain((int)round((minVoltage / 3.3f) * 255.0f), 0, 255);
      int maxDac = constrain((int)round((maxVoltage / 3.3f) * 255.0f), 0, 255);
      outValue = map(soc, 0, 100, minDac, maxDac);
    }
    dacWrite(dacPin, outValue);
  } else {
    int maxDuty = (1 << pwmRes) - 1;
    if (mappedV >= 0.0f) {
      outValue = constrain((int)round((mappedV / 3.3f) * maxDuty), 0, maxDuty);
    } else {
      int minDuty = constrain((int)round((minVoltage / 3.3f) * maxDuty), 0, maxDuty);
      int maxDutyVal = constrain((int)round((maxVoltage / 3.3f) * maxDuty), 0, maxDuty);
      outValue = map(soc, 0, 100, minDuty, maxDutyVal);
    }
    ledcWrite(pwmPin, outValue);
  }

  lockState();
  currentSoc = soc;
  currentOut = outValue;
  unlockState();
}

void setupPWM() {
  ledcAttach(pwmPin, pwmFreq, pwmRes);
}

void disablePWM() {
  ledcWrite(pwmPin, 0);
  ledcDetach(pwmPin);
}

#endif