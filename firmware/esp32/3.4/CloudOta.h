#ifndef CLOUD_OTA_H
#define CLOUD_OTA_H

#include "Config.h"

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
  String rest = s.substring(comma + 1);
  rest.trim();
  int sp1 = rest.indexOf(' ');
  if (sp1 < 0) return s;
  String day = rest.substring(0, sp1);
  int sp2 = rest.indexOf(' ', sp1 + 1);
  if (sp2 < 0) return s;
  String mon = rest.substring(sp1 + 1, sp2);
  int sp3 = rest.indexOf(' ', sp2 + 1);
  String year;
  // ★ FIX: nếu không có space thứ 3 (không timezone), lấy đến hết
  if (sp3 < 0) year = rest.substring(sp2 + 1);
  else year = rest.substring(sp2 + 1, sp3);
  year.trim();
  String monthNum = "??";
  if (mon == "Jan") monthNum = "01";
  else if (mon == "Feb") monthNum = "02";
  else if (mon == "Mar") monthNum = "03";
  else if (mon == "Apr") monthNum = "04";
  else if (mon == "May") monthNum = "05";
  else if (mon == "Jun") monthNum = "06";
  else if (mon == "Jul") monthNum = "07";
  else if (mon == "Aug") monthNum = "08";
  else if (mon == "Sep") monthNum = "09";
  else if (mon == "Oct") monthNum = "10";
  else if (mon == "Nov") monthNum = "11";
  else if (mon == "Dec") monthNum = "12";
  if (day.length() == 1) day = "0" + day;
  return day + "/" + monthNum + "/" + year;
}

bool fetchPresetInfo(const String& url) {
  if (url.length() == 0 || !url.startsWith("http")) return false;
  if (!staConnected && !apActive) return false;
  HTTPClient http;
  WiFiClient client;
  WiFiClientSecure clientSecure;
  if (url.startsWith("https://")) {
    clientSecure.setInsecure();
    http.begin(clientSecure, url);
  } else http.begin(client, url);
  http.setTimeout(8000);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  const char* headerKeys[] = { "Last-Modified", "X-Updated", "ETag" };
  http.collectHeaders(headerKeys, 3);
  int code = http.sendRequest("HEAD");
  if (code != 200) {
    http.end();
    if (url.startsWith("https://")) {
      clientSecure.setInsecure();
      http.begin(clientSecure, url);
    } else http.begin(client, url);
    http.setTimeout(8000);
    http.addHeader("Range", "bytes=0-0");
    code = http.GET();
  }
  if (code != 200 && code != 206) {
    http.end();
    presetInfoLoaded = true;
    return false;
  }
  String updated = http.header("X-Updated");
  if (updated.length() == 0) {
    String lm = http.header("Last-Modified");
    if (lm.length() > 0) updated = parseHttpDate(lm);
  }
  http.end();
  if (updated.length() > 0) {
    presetLastUpdated = updated;
    presetInfoLoaded = true;
    return true;
  }
  presetInfoLoaded = true;
  return false;
}

bool fetchPresetsFromUrl(const String& url) {
  if (url.length() == 0) return false;
  if (!staConnected && !apActive) return false;
  HTTPClient http;
  WiFiClient client;
  WiFiClientSecure clientSecure;
  if (url.startsWith("https://")) {
    clientSecure.setInsecure();
    http.begin(clientSecure, url);
  } else http.begin(client, url);
  http.setTimeout(10000);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  int code = http.GET();
  if (code != 200) {
    http.end();
    return false;
  }
  String body = http.getString();
  http.end();
  if (body.length() < 20) return false;
  String jsonDate = parseJsonString(body, 0, "date");
  if (jsonDate.length() > 0) {
    presetLastUpdated = jsonDate;
    presetInfoLoaded = true;
  }
  int arrStart = body.indexOf("\"presets\"");
  if (arrStart < 0) arrStart = 0;
  int p = body.indexOf('[', arrStart);
  if (p < 0) return false;
  int newCount = 0;
  int pos = p + 1;
  while (newCount < MAX_REMOTE_PRESETS) {
    int objStart = body.indexOf('{', pos);
    if (objStart < 0) break;
    int objEnd = body.indexOf('}', objStart);
    if (objEnd < 0) break;
    int entriesPos = body.indexOf("\"entries\"", objStart);
    if (entriesPos < 0 || entriesPos > objEnd + 200) {
      pos = objEnd + 1;
      continue;
    }
    RemotePreset& rp = remotePresets[newCount];
    rp.name = parseJsonString(body, objStart, "name");
    rp.subtitle = parseJsonString(body, objStart, "subtitle");
    rp.ranges = parseJsonString(body, objStart, "ranges");
    if (rp.name.length() == 0) {
      pos = objEnd + 1;
      continue;
    }
    int entArr = body.indexOf('[', entriesPos);
    int entEnd = body.indexOf(']', entArr);
    if (entArr < 0 || entEnd < 0) {
      pos = objEnd + 1;
      continue;
    }
    int ec = 0, ep = entArr + 1;
    while (ec < MAX_ENTRIES_PER_PRESET) {
      int es = body.indexOf('{', ep);
      if (es < 0 || es > entEnd) break;
      int ee = body.indexOf('}', es);
      if (ee < 0 || ee > entEnd) break;
      String entryStr = body.substring(es, ee + 1);
      rp.entries[ec].startPct = (uint8_t)parseJsonNumber(entryStr, 0, "s");
      rp.entries[ec].endPct = (uint8_t)parseJsonNumber(entryStr, 0, "e");
      rp.entries[ec].voltage = parseJsonNumber(entryStr, 0, "v");
      ec++;
      ep = ee + 1;
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
  HTTPClient http;
  WiFiClient client;
  WiFiClientSecure clientSecure;
  String url = UPDATE_JSON_URL;
  if (url.startsWith("https://")) {
    clientSecure.setInsecure();
    http.begin(clientSecure, url);
  } else http.begin(client, url);
  http.setTimeout(10000);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  int code = http.GET();
  if (code != 200) {
    http.end();
    return false;
  }
  String body = http.getString();
  http.end();
  if (body.length() < 20) return false;
  remoteVersion = parseJsonString(body, 0, "version");
  remoteFwUrl = parseJsonString(body, 0, "url");
  remoteNotes = parseJsonString(body, 0, "notes");
  remoteDate = parseJsonString(body, 0, "date");
  remoteNotes.replace("\\n", "\n");
  remoteChecked = true;
  remoteNewer = (remoteVersion != String(FW_VERSION));
  return true;
}

static uint8_t parseOcvArray(const String& body, const String& key, OcvPoint* out, uint8_t maxN) {
  int keyPos = body.indexOf("\"" + key + "\"");
  if (keyPos < 0) return 0;
  int arrStart = body.indexOf('[', keyPos);
  if (arrStart < 0) return 0;
  int arrEnd = body.indexOf(']', arrStart);
  if (arrEnd < 0) return 0;
  uint8_t n = 0;
  int pos = arrStart + 1;
  while (n < maxN) {
    int os = body.indexOf('{', pos);
    if (os < 0 || os > arrEnd) break;
    int oe = body.indexOf('}', os);
    if (oe < 0 || oe > arrEnd) break;
    String obj = body.substring(os, oe + 1);
    float v = parseJsonNumber(obj, 0, "v");
    int s = (int)parseJsonNumber(obj, 0, "soc");
    if (v > 0.0f && s >= 0 && s <= 100) {
      out[n].v = v;
      out[n].soc = (uint8_t)s;
      n++;
    }
    pos = oe + 1;
  }
  return n;
}

bool fetchAcquyJson() {
  if (!staConnected && !apActive) return false;
  HTTPClient http;
  WiFiClient client;
  WiFiClientSecure clientSecure;
  String url = ACQUY_JSON_URL;
  if (url.startsWith("https://")) {
    clientSecure.setInsecure();
    http.begin(clientSecure, url);
  } else http.begin(client, url);
  http.setTimeout(10000);
  http.setFollowRedirects(HTTPC_STRICT_FOLLOW_REDIRECTS);
  int code = http.GET();
  if (code != 200) {
    http.end();
    return false;
  }
  String body = http.getString();
  http.end();
  if (body.length() < 50) return false;

  String jsonDate = parseJsonString(body, 0, "date");
  OcvPoint tmpLead[MAX_OCV_POINTS], tmpLfp[MAX_OCV_POINTS];
  uint8_t nLead = parseOcvArray(body, "lead", tmpLead, MAX_OCV_POINTS);
  uint8_t nLfp = parseOcvArray(body, "lfp", tmpLfp, MAX_OCV_POINTS);
  if (nLead < 2 && nLfp < 2) return false;

  preferences.begin("bms_ocv", false);
  if (nLead >= 2) {
    lockState();
    leadOcvN = nLead;
    for (uint8_t i = 0; i < nLead; i++) leadOcvTable[i] = tmpLead[i];
    unlockState();
    preferences.putUChar("lead_n", nLead);
    for (uint8_t i = 0; i < nLead; i++) {
      String b = "l" + String(i) + "_";
      preferences.putFloat((b + "v").c_str(), tmpLead[i].v);
      preferences.putUChar((b + "s").c_str(), tmpLead[i].soc);
    }
  }
  if (nLfp >= 2) {
    lockState();
    lfpOcvN = nLfp;
    for (uint8_t i = 0; i < nLfp; i++) lfpOcvTable[i] = tmpLfp[i];
    unlockState();
    preferences.putUChar("lfp_n", nLfp);
    for (uint8_t i = 0; i < nLfp; i++) {
      String b = "f" + String(i) + "_";
      preferences.putFloat((b + "v").c_str(), tmpLfp[i].v);
      preferences.putUChar((b + "s").c_str(), tmpLfp[i].soc);
    }
  }
  acquyJsonDate = jsonDate;
  preferences.putString("date", acquyJsonDate);
  preferences.end();

  LOGM(LOGMOD_ADC, "📥 acquy.json: Lead=%d, LFP=%d (%s)", nLead, nLfp, acquyJsonDate.c_str());
  return true;
}

void loadOcvFromNVS() {
  preferences.begin("bms_ocv", false);
  uint8_t nLead = preferences.getUChar("lead_n", 0);
  uint8_t nLfp = preferences.getUChar("lfp_n", 0);
  if (nLead >= 2 && nLead <= MAX_OCV_POINTS) {
    leadOcvN = nLead;
    for (uint8_t i = 0; i < nLead; i++) {
      String b = "l" + String(i) + "_";
      leadOcvTable[i].v = preferences.getFloat((b + "v").c_str(), 0.0f);
      leadOcvTable[i].soc = preferences.getUChar((b + "s").c_str(), 0);
    }
  }
  if (nLfp >= 2 && nLfp <= MAX_OCV_POINTS) {
    lfpOcvN = nLfp;
    for (uint8_t i = 0; i < nLfp; i++) {
      String b = "f" + String(i) + "_";
      lfpOcvTable[i].v = preferences.getFloat((b + "v").c_str(), 0.0f);
      lfpOcvTable[i].soc = preferences.getUChar((b + "s").c_str(), 0);
    }
  }
  acquyJsonDate = preferences.getString("date", "");
  preferences.end();
  LOGM(LOGMOD_BOOT, "OCV NVS: Lead=%d, LFP=%d (%s)", leadOcvN, lfpOcvN, acquyJsonDate.c_str());
}

void autoUpdateTask() {
  if (!autoUpdateEnabled || autoUpdateDone) return;
  if (autoUpdateInProgress || otaInProgress) return;
  if (!staConnected) return;
  unsigned long now = millis();
  if (lastAutoCheck == 0) {
    lastAutoCheck = now;
    return;
  }
  if (now - lastAutoCheck < 10000) return;
  autoUpdateDone = true;
  if (!fetchUpdateInfo() || !remoteNewer) return;
  LOGM(LOGMOD_OTA, "Auto update: có bản mới v%s", remoteVersion.c_str());
  if (remoteNotes.length() > 0) fwNotes = remoteNotes;
  if (remoteDate.length() > 0) fwDate = remoteDate;
  preferences.begin("bms_config", false);
  preferences.putString("fw_date", fwDate);
  preferences.putString("fw_notes", fwNotes);
  preferences.end();
  if (remoteFwUrl.length() == 0) return;
  autoUpdateInProgress = true;
  otaInProgress = true;
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
  WiFi.setSleep(false);
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
    http.end();
    autoUpdateInProgress = false;
    otaInProgress = false;
    return;
  }
  int contentLen = http.getSize();
  if (contentLen <= 0) {
    http.end();
    autoUpdateInProgress = false;
    otaInProgress = false;
    return;
  }
  if (!Update.begin(contentLen)) {
    http.end();
    autoUpdateInProgress = false;
    otaInProgress = false;
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
      }
    } else delay(10);
  }
  http.end();
  if (written != contentLen) {
    Update.abort();
    autoUpdateInProgress = false;
    otaInProgress = false;
    return;
  }
  if (Update.end(true)) {
    delay(2000);
    ESP.restart();
  }
  autoUpdateInProgress = false;
  otaInProgress = false;
}

#endif