#ifndef BLE_BMS_H
#define BLE_BMS_H

#include "Config.h"

uint8_t detectJkVersion(const char* model, size_t len) {
  if (len < 3) return 1;
  char norm[32] = { 0 };
  size_t n = (len < 31) ? len : 31;
  for (size_t i = 0; i < n; i++) {
    char c = model[i];
    if (c == '_') c = '-';
    norm[i] = c;
  }
  if (strncmp(norm, "JK-PB2", 6) == 0) return 2;
  if (strncmp(norm, "JK-BD", 5) == 0) return 2;
  if (strncmp(norm, "JK-HY", 5) == 0) return 2;
  if (strncmp(norm, "JK-B2", 5) == 0) return 1;
  return 1;
}

bool parseJkFrame(uint8_t* data, size_t len) {
  if (len >= 4 && data[0] == 0x55 && data[1] == 0xAA && data[2] == 0xEB && data[3] == 0x90) {
    jkFrameLen = 0;
    jkFrameStartMs = millis();
  }
  if (jkFrameLen == 0 && !(len >= 4 && data[0] == 0x55 && data[1] == 0xAA)) return false;
  if (jkFrameLen > 0 && millis() - jkFrameStartMs > 1500) {
    jkFrameLen = 0;
    return false;
  }
  size_t toCopy = len;
  if (jkFrameLen + toCopy > JK_FRAME_SIZE) toCopy = JK_FRAME_SIZE - jkFrameLen;
  memcpy(jkFrameBuf + jkFrameLen, data, toCopy);
  jkFrameLen += toCopy;
  if (jkFrameLen < 260) return false;
  uint8_t* frame = jkFrameBuf;
  uint8_t frameType = frame[4];
  if (frameType == 0x03) {
    if (!jkVersionProbed) {
      char model[32] = { 0 };
      memcpy(model, frame + 6, 16);
      model[16] = 0;
      jkVersion = detectJkVersion(model, 16);
      jkVersionProbed = true;
      LOGM(LOGMOD_BLE, "JK protocol: %s", jkVersion == 2 ? "JK02_32S" : "JK02_24S");
      preferences.begin("bms_config", false);
      preferences.putUChar("jk_ver", jkVersion);
      preferences.end();
    }
    if (jkFrameLen < 300) {
      jkFrameLen = 0;
      return false;
    }
  }
  if (frameType == 0x02 || frameType == 0x01) {
    size_t totalVOff = (jkVersion == 2) ? 150 : 118;
    size_t currentOff = (jkVersion == 2) ? 158 : 126;
    size_t socOff = (jkVersion == 2) ? 173 : 141;
    size_t minLen = socOff + 1;
    if (jkFrameLen < minLen) {
      jkFrameLen = 0;
      return false;
    }
    uint8_t soc = frame[socOff];
    float cells[16] = { 0 };
    uint8_t cellCount = 0;
    float cellSum = 0.0f;
    for (int i = 0; i < 16; i++) {
      uint16_t rawV = (uint16_t)(frame[6 + i * 2] | (frame[7 + i * 2] << 8));
      float v = rawV * 0.001f;
      if (v > 1.5f && v < 4.2f) {
        cells[i] = v;
        cellCount++;
        cellSum += v;
      }
    }
    uint32_t rawTotal = (uint32_t)frame[totalVOff] | ((uint32_t)frame[totalVOff + 1] << 8) | ((uint32_t)frame[totalVOff + 2] << 16) | ((uint32_t)frame[totalVOff + 3] << 24);
    float totalV = rawTotal * 0.001f;
    int32_t rawCur = (int32_t)frame[currentOff] | ((int32_t)frame[currentOff + 1] << 8) | ((int32_t)frame[currentOff + 2] << 16) | ((int32_t)frame[currentOff + 3] << 24);
    float current = rawCur * 0.001f;
    bool socValid = (soc <= 100);
    bool totalVValid = (totalV > 1.0f && totalV < (4.5f * 32.0f));
    bool cellsConsistent = true;
    if (cellCount >= 4 && totalVValid) {
      if (fabsf(cellSum - totalV) > totalV * 0.15f) cellsConsistent = false;
    }
    bool currentValid = (fabsf(current) < 500.0f);
    if (!socValid || !totalVValid || !cellsConsistent || !currentValid) {
      jkFrameLen = 0;
      return false;
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
      if (soc != savedSoc) {
        savedSoc = soc;
        preferences.begin("bms_config", false);
        preferences.putUInt("last_soc", savedSoc);
        preferences.end();
      }
    }
    if (soc <= 100 && !manual) applyOutput(soc);

    static unsigned long lastBmsLog = 0;
    if (millis() - lastBmsLog > 30000) {
      lastBmsLog = millis();
      LOGM(LOGMOD_BLE, "SOC=%d%% V=%.2fV I=%.2fA P=%.0fW Cells=%d",
           soc, totalV, current, totalV * current, cellCount);
    }
    jkFrameLen = 0;
    return true;
  }
  if (jkFrameLen >= JK_FRAME_SIZE) jkFrameLen = 0;
  return false;
}

class MyClientCallback : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient* pclient) override {
    connected = true;
    bmsProtocol = 255;
    probeSent = false;
    probeStartMs = 0;
    jkFrameLen = 0;
    jkDetectTime = 0;
    LOGM(LOGMOD_BLE, "✅ BMS đã kết nối");
    if (!apActive && !acquyMode) ledSetMode(LED_MODE_BREATHE);
  }
  void onDisconnect(NimBLEClient* pclient, int reason) override {
    connected = false;
    pChar = nullptr;
    pWriteChar = nullptr;
    bmsSocTimestamp = 0;
    probeSent = false;
    probeStartMs = 0;
    jkFrameLen = 0;
    jkDetectTime = 0;
    // ★ Reset dữ liệu cũ để UI không hiển thị số liệu "ma"
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
    unlockState();
    cachedJsonData = "";
    cachedJsonTime = 0;
    LOGM(LOGMOD_BLE, "⚠️ BMS mất kết nối! reason=%d", reason);
    if (!apActive && !acquyMode) ledSetMode(LED_MODE_SOLID);
  }
};

class MyScanCallbacks : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* adv) override {
    if (!bleInitialized || acquyMode) return;
    String foundMac = String(adv->getAddress().toString().c_str());
    foundMac.toUpperCase();
    String tgt;
    lockState();
    tgt = targetMac;
    unlockState();
    if (tgt != "" && foundMac == tgt) {
      if (adv->haveName()) {
        lockState();
        pendingScanName = String(adv->getName().c_str());
        unlockState();
      }
      doConnect = true;
    }
  }
};
static MyScanCallbacks scanCallbacksInstance;

void bleInit() {
  if (bleInitialized) return;
  if (!NimBLEDevice::isInitialized()) NimBLEDevice::init("ESP32_BMS_Controller");
  NimBLEScan* pBLEScan = NimBLEDevice::getScan();
  pBLEScan->setScanCallbacks(&scanCallbacksInstance, false);
  pBLEScan->setActiveScan(true);
  pBLEScan->setInterval(160);
  pBLEScan->setWindow(60);
  bleInitialized = true;
  bleEnabled = true;
  LOGM(LOGMOD_BLE, "BLE khởi tạo");
}

void bleDeinit() {
  if (!bleInitialized) return;
  LOGM(LOGMOD_BLE, "BLE tắt");
  bleShuttingDown = true;
  doScan = false;
  doConnect = false;
  doDisconnect = false;
  unsigned long t0 = millis();
  while (isScanning && millis() - t0 < 3000) delay(50);
  if (pClient && pClient->isConnected()) {
    pClient->disconnect();
    delay(200);
  }
  connected = false;
  pChar = nullptr;
  pWriteChar = nullptr;
  pClient = nullptr;
  bleEnabled = false;
  bleInitialized = false;
  delay(50);
  NimBLEDevice::deinit(true);
  bleShuttingDown = false;
}

void notifyCallback(NimBLERemoteCharacteristic* c, uint8_t* data, size_t len, bool isNotify) {
  if (acquyMode) return;
  if (bmsProtocol == 255 && len >= 4) {
    if (data[0] == 0x55 && data[1] == 0xAA && data[2] == 0xEB && data[3] == 0x90) {
      bmsProtocol = 1;
      probeSent = true;
      jkDetectTime = millis();
      LOGM(LOGMOD_BLE, "Nhận diện: JiKong");
      preferences.begin("bms_config", false);
      preferences.putUChar("bms_proto", 1);
      preferences.end();
    } else if (data[0] == 0xA5 && data[1] == 0x7B) {
      bmsProtocol = 0;
      probeSent = true;
      LOGM(LOGMOD_BLE, "Nhận diện: BMS MOVE");
      preferences.begin("bms_config", false);
      preferences.putUChar("bms_proto", 0);
      preferences.end();
    }
  }
  NimBLERemoteCharacteristic* targetWrite = (pWriteChar != nullptr) ? pWriteChar : pChar;
  if (bmsProtocol == 255 && !probeSent && probeStartMs > 0 && millis() - probeStartMs > 2000) {
    if (targetWrite != nullptr) {
      targetWrite->writeValue(jkDeviceInfoCmd, sizeof(jkDeviceInfoCmd), false);
      probeSent = true;
      probeStartMs = millis();
    }
  }
  if (bmsProtocol == 255 && probeSent && probeStartMs > 0 && millis() - probeStartMs > 4000) {
    bmsProtocol = 254;
    LOGM(LOGMOD_BLE, "Không nhận diện được BMS");
  }
  if (bmsProtocol == 1) {
    parseJkFrame(data, len);
    return;
  }
  if (len >= 123 && data[0] == 0xA5 && data[1] == 0x7B) {
    uint8_t soc = data[11];
    float totalV = ((data[20] << 8) | data[19]) / 100.0f;
    bool hasCur = (len >= 25);
    float current = 0.0f;
    if (hasCur) {
      int16_t rawCurrent = (int16_t)((data[22] << 8) | data[21]);
      current = rawCurrent / 100.0f;
    }
    float power = totalV * current;
    float cells[16] = { 0 };
    uint8_t cellCount = 0;
    if (len >= 82) {
      for (int i = 0; i < 16; i++) {
        cells[i] = ((data[51 + i * 2] << 8) | data[50 + i * 2]) / 1000.0f;
        if (cells[i] > 0.5f) cellCount++;
      }
    }
    if (soc > 100 || totalV < 1.0f || totalV > 144.0f) return;
    lockState();
    realBmsSoc = soc;
    bmsTotalVoltage = totalV;
    bmsHasTotalV = (totalV > 1.0f);
    bmsHasCurrent = hasCur;
    bmsCurrent = current;
    bmsHasPower = hasCur;
    bmsPower = power;
    bmsCellCount = cellCount;
    bmsHasCells = (cellCount > 0);
    if (len >= 82)
      for (int i = 0; i < 16; i++) bmsCells[i] = cells[i];
    bool manual = manualSimMode;
    unlockState();
    if (soc <= 100) {
      bmsSocTimestamp = millis();
      if (soc != savedSoc) {
        savedSoc = soc;
        preferences.begin("bms_config", false);
        preferences.putUInt("last_soc", savedSoc);
        preferences.end();
      }
    }
    if (soc <= 100 && !manual) applyOutput(soc);
  }
}

bool connectToServer() {
  if (!bleEnabled || acquyMode) return false;
  String tgt;
  lockState();
  tgt = targetMac;
  unlockState();
  if (tgt == "") return false;
  bmsProtocol = 255;
  probeSent = false;
  probeStartMs = 0;
  jkFrameLen = 0;
  jkDetectTime = 0;
  if (pClient == nullptr) {
    pClient = NimBLEDevice::createClient();
    pClient->setClientCallbacks(new MyClientCallback(), false);
    pClient->setConnectTimeout(5000);
  }
  if (pClient->isConnected()) {
    pClient->disconnect();
    delay(200);
  }
  LOGM(LOGMOD_BLE, "Kết nối tới %s...", tgt.c_str());
  NimBLEAddress targetAddr(std::string(tgt.c_str()), BLE_ADDR_RANDOM);
  bool connSuccess = pClient->connect(targetAddr, false);
  if (!connSuccess) {
    NimBLEAddress targetAddrPub(std::string(tgt.c_str()), BLE_ADDR_PUBLIC);
    connSuccess = pClient->connect(targetAddrPub, false);
  }
  if (!connSuccess || !pClient->isConnected()) {
    LOGM(LOGMOD_BLE, "❌ Kết nối thất bại");
    return false;
  }
  NimBLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (!pRemoteService) pRemoteService = pClient->getService("FFE0");
  if (!pRemoteService) {
    LOGM(LOGMOD_BLE, "❌ Không tìm thấy Service");
    pClient->disconnect();
    return false;
  }
  pChar = pRemoteService->getCharacteristic(charUUID);
  if (!pChar) pChar = pRemoteService->getCharacteristic("FFE1");
  if (!pChar) {
    LOGM(LOGMOD_BLE, "❌ Không tìm thấy Char");
    pClient->disconnect();
    return false;
  }
  pWriteChar = pRemoteService->getCharacteristic("FFE2");
  if (!pWriteChar || !pWriteChar->canWrite()) pWriteChar = pChar;
  if (pChar->canNotify()) pChar->subscribe(true, notifyCallback);
  delay(200);
  NimBLERemoteCharacteristic* targetWrite = (pWriteChar != nullptr) ? pWriteChar : pChar;
  targetWrite->writeValue(loginCmd, sizeof(loginCmd), false);
  delay(80);
  targetWrite->writeValue(requestSocCmd, sizeof(requestSocCmd), false);
  delay(80);
  targetWrite->writeValue(jkDeviceInfoCmd, sizeof(jkDeviceInfoCmd), false);
  probeStartMs = millis();
  lockState();
  if (pendingScanName.length() > 0) {
    bmsDeviceName = pendingScanName;
    pendingScanName = "";
  } else if (bmsDeviceName.length() == 0) bmsDeviceName = "BMS " + tgt;
  String nameToSave = bmsDeviceName;
  unlockState();
  preferences.begin("bms_config", false);
  preferences.putString("bms_name", nameToSave);
  preferences.end();
  return true;
}

void bleTask(void* param) {
  unsigned long lastScanAttempt = 0;
  unsigned long lastQueryTime = 0;
  for (;;) {
    if (acquyMode || otaInProgress) {
      if (isScanning) {
        NimBLEScan* pScan = NimBLEDevice::getScan();
        if (pScan) pScan->stop();
        isScanning = false;
      }
      if (pClient && pClient->isConnected()) pClient->disconnect();
      connected = false;
      pChar = nullptr;
      pWriteChar = nullptr;
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }
    if (!bleEnabled || bleShuttingDown) {
      vTaskDelay(pdMS_TO_TICKS(200));
      continue;
    }
    if (doScan) {
      doScan = false;
      NimBLEScan* pScan = NimBLEDevice::getScan();
      if (isScanning) {
        pScan->stop();
        isScanning = false;
        delay(100);
      }
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
        safeName.replace("'", "\\'");
        safeName.replace("\"", "&quot;");
        bmsCount++;
        html += "<li><div><strong>🔋 " + name + "</strong><br><small style='font-family:monospace'>" + mac + "</small></div>";
        html += "<button style='width:auto;' onclick=\"save('" + mac + "','" + safeName + "')\">Kết Nối</button></li>";
      }
      if (bmsCount == 0) html = "<li style='color:#888;'>Không tìm thấy BMS.</li>";
      pScan->clearResults();
      lockState();
      scanResultHtml = html;
      scanReady = true;
      unlockState();
    }
    if (doDisconnect) {
      doDisconnect = false;
      if (pClient && pClient->isConnected()) {
        pClient->disconnect();
        delay(200);
      }
      connected = false;
      pChar = nullptr;
      pWriteChar = nullptr;
      bmsSocTimestamp = 0;
    }
    if (doConnect) {
      doConnect = false;
      NimBLEScan* pScan = NimBLEDevice::getScan();
      if (isScanning) {
        pScan->stop();
        isScanning = false;
        delay(100);
      }
      pScan->clearResults();
      connectToServer();
    }
    if (bleAutoScan && !otaInProgress && needAutoConnect) {
      String tgt;
      lockState();
      tgt = targetMac;
      unlockState();
      if (tgt == "" && !connected && !doConnect && !doScan) {
        if (millis() - lastScanAttempt > 5000) {
          lastScanAttempt = millis();
          NimBLEScan* pScan = NimBLEDevice::getScan();
          if (isScanning) {
            pScan->stop();
            isScanning = false;
            delay(50);
          }
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
            LOGM(LOGMOD_BLE, "AutoConnect: %s (%s)", foundName.c_str(), foundMac.c_str());
            lockState();
            targetMac = foundMac;
            bmsDeviceName = foundName;
            unlockState();
            preferences.begin("bms_config", false);
            preferences.putString("bms_mac", foundMac);
            preferences.putString("bms_name", foundName);
            preferences.putBool("need_auto", false);
            preferences.putUChar("bms_proto", 255);
            preferences.putUChar("jk_ver", 0);
            preferences.end();
            jkVersion = 0;
            jkVersionProbed = false;
            needAutoConnect = false;
            doConnect = true;
          }
        }
      } else if (tgt != "") {
        needAutoConnect = false;
        preferences.begin("bms_config", false);
        preferences.putBool("need_auto", false);
        preferences.end();
      }
    }
    if (bleAutoScan && !otaInProgress && !needAutoConnect) {
      String tgt;
      lockState();
      tgt = targetMac;
      unlockState();
      if (tgt != "" && !connected && !doConnect && !doScan) {
        if (millis() - lastScanAttempt > 5000) {
          lastScanAttempt = millis();
          NimBLEScan* pScan = NimBLEDevice::getScan();
          if (isScanning) {
            pScan->stop();
            isScanning = false;
            delay(50);
          }
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
          else {
            if (jkToggle % 2 == 0) targetWrite->writeValue(jkCellInfoNewCmd, sizeof(jkCellInfoNewCmd), false);
            else targetWrite->writeValue(jkCellInfoOldCmd, sizeof(jkCellInfoOldCmd), false);
          }
        } else if (bmsProtocol == 0) targetWrite->writeValue(requestSocCmd, sizeof(requestSocCmd), false);
        else if (bmsProtocol == 255) targetWrite->writeValue(requestSocCmd, sizeof(requestSocCmd), false);
      }
    }
    if (httpBusy) vTaskDelay(pdMS_TO_TICKS(80));
    else vTaskDelay(pdMS_TO_TICKS(50));
  }
}

#endif