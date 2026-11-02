#ifndef ACQUY_MANAGER_H
#define ACQUY_MANAGER_H

#include "Config.h"


// Hàm nội suy OCV -> SOC từ bảng động
static int interpolateOcv(const OcvPoint* table, uint8_t n, float v) {
  if (n < 2) return 0;
  if (v >= table[0].v) return table[0].soc;
  if (v <= table[n - 1].v) return table[n - 1].soc;
  for (uint8_t i = 0; i < n - 1; i++) {
    if (v <= table[i].v && v >= table[i + 1].v) {
      float v1 = table[i].v, s1 = (float)table[i].soc;
      float v2 = table[i + 1].v, s2 = (float)table[i + 1].soc;
      float t = (v - v2) / (v1 - v2);
      int soc = (int)round(s2 + t * (s1 - s2));
      if (soc < 0) soc = 0;
      if (soc > 100) soc = 100;
      return soc;
    }
  }
  return 0;
}

int calcSocFromV(float vBat) {
  if (vBat <= 0.1f) return 0;
  if (batteryCells < 1) return 0;

  float vCell = vBat / (float)batteryCells;

  if (batteryType == BATT_TYPE_LEAD) {
    if (leadOcvN >= 2) return interpolateOcv(leadOcvTable, leadOcvN, vCell);
    // Fallback: nội suy tuyến tính giữa V_EMPTY và V_FULL
    float vE = LEAD_V_EMPTY, vF = LEAD_V_FULL;
    if (vCell >= vF) return 100;
    if (vCell <= vE) return 0;
    return (int)round((vCell - vE) / (vF - vE) * 100.0f);
  } else {
    if (lfpOcvN >= 2) return interpolateOcv(lfpOcvTable, lfpOcvN, vCell);
    float vE = LFP_V_EMPTY, vF = LFP_V_FULL;
    if (vCell >= vF) return 100;
    if (vCell <= vE) return 0;
    return (int)round((vCell - vE) / (vF - vE) * 100.0f);
  }
}

float readAdcVolts(int pin) {
  uint32_t sum_mV = 0;
  for (int i = 0; i < 32; i++) {
    sum_mV += analogReadMilliVolts(pin);
    delayMicroseconds(50);
  }
  return (float)sum_mV / 32.0f / 1000.0f;
}

void processAcquyReading() {
  if (!acquyMode) return;
  if (millis() - lastAdcRead < 100) return;  // Chu kỳ lấy mẫu 100ms
  lastAdcRead = millis();
  unsigned long now = millis();

  static unsigned long settleWindowStart = 0;
  static unsigned long lastSocDropMs = 0;
  static unsigned long chargeDetectStart = 0;
  static float preSagOcv = 0.0f;

  if (acquyR2 <= 0.01f) acquyR2 = 4.7f;
  float ratio = (acquyR1 + acquyR2) / acquyR2;

  // --- KHỞI TẠO LẦN ĐẦU (BẬT KHÓA ĐIỆN / ĐỔI CẤU HÌNH) ---
  if (!acquyEmaInit || !socFilteredInit) {
    float sumInit = 0.0f;
    for (int i = 0; i < ACQY_MEDIAN_SIZE; i++) {
      float vA = readAdcVolts(acquyAdcPin);
      float vB = vA * ratio * acquyCalibGain + acquyCalibOffset;
      acquyMedianBuf[i] = vB;
      sumInit += vB;
      delay(5);
    }
    acquyMedianFilled = true;
    acquyMedianIdx = 0;
    float vInit = sumInit / (float)ACQY_MEDIAN_SIZE;

    acquyAdcRawV = readAdcVolts(acquyAdcPin);
    acquyRawV = vInit;
    acquyEmaV = vInit;
    acquyMeasuredV = vInit;
    acquyEmaInit = true;

    acquyLastStableV = vInit;
    acquyOcvEstimated = vInit;
    acquyStableV = vInit;
    preSagOcv = vInit;
    acquyStable = true;
    acquyStableStart = now;
    acquyLastStableUpdate = now;

    for (uint8_t i = 0; i < DROP_BUF_SIZE; i++) {
      dropBuf[i] = vInit;
      dropBufTs[i] = now;
    }
    dropBufIdx = 0;
    acqState = ACQ_IDLE;

    int initSoc = calcSocFromV(vInit);
    if (initSoc < 0) initSoc = 0;
    if (initSoc > 100) initSoc = 100;
    socFiltered = (uint8_t)initSoc;
    socFilteredInit = true;
    lastSocDropMs = now;
    chargeDetectStart = 0;

    if (!manualSimMode) {
      applyOutput(socFiltered);
      acquyLastOutputChange = now;
    }
    LOGM(LOGMOD_ADC, "🟢 Khởi tạo OCV gốc: %.2fV -> SOC=%d%%", vInit, socFiltered);
  }

  // --- ĐỌC ADC & LỌC MEDIAN + EMA ---
  float vAdc = readAdcVolts(acquyAdcPin);
  acquyAdcRawV = vAdc;
  float vBatRaw = vAdc * ratio * acquyCalibGain + acquyCalibOffset;
  acquyRawV = vBatRaw;

  acquyMedianBuf[acquyMedianIdx] = vBatRaw;
  acquyMedianIdx = (acquyMedianIdx + 1) % ACQY_MEDIAN_SIZE;
  if (acquyMedianIdx == 0) acquyMedianFilled = true;

  float sorted[ACQY_MEDIAN_SIZE];
  uint8_t nMedian = acquyMedianFilled ? ACQY_MEDIAN_SIZE : (acquyMedianIdx == 0 ? ACQY_MEDIAN_SIZE : acquyMedianIdx);
  if (nMedian < 1) nMedian = 1;
  for (uint8_t i = 0; i < nMedian; i++) sorted[i] = acquyMedianBuf[i];
  for (uint8_t i = 0; i < nMedian - 1; i++) {
    for (uint8_t j = i + 1; j < nMedian; j++) {
      if (sorted[j] < sorted[i]) {
        float t = sorted[i];
        sorted[i] = sorted[j];
        sorted[j] = t;
      }
    }
  }
  float vBatMedian = sorted[nMedian / 2];

  acquyEmaV = acquyEmaV * (1.0f - ACQY_EMA_ALPHA) + vBatMedian * ACQY_EMA_ALPHA;
  acquyMeasuredV = acquyEmaV;

  float vEmptyLim, vFullLim, vMaxWarnLim;
  getBatteryLimits(vEmptyLim, vFullLim, vMaxWarnLim);
  acquyOverVoltage = (vBatMedian >= vMaxWarnLim);

  dropBuf[dropBufIdx] = vBatMedian;
  dropBufTs[dropBufIdx] = now;
  dropBufIdx = (dropBufIdx + 1) % DROP_BUF_SIZE;

  float vWinMax = vBatMedian;
  float vWinMin = vBatMedian;
  for (uint8_t i = 0; i < DROP_BUF_SIZE; i++) {
    if (now - dropBufTs[i] <= DROP_WINDOW_MS) {
      if (dropBuf[i] > vWinMax) vWinMax = dropBuf[i];
      if (dropBuf[i] < vWinMin) vWinMin = dropBuf[i];
    }
  }

  bool isLfp = (batteryType == BATT_TYPE_LFP);
  float dropThr = isLfp ? dropThresholdLfp : dropThresholdLead;
  float riseThr = isLfp ? riseThresholdLfp : riseThresholdLead;
  unsigned long waitMs = isLfp ? settleLfpMs : settleLeadMs;

  float deltaDown = vWinMax - vBatMedian;
  float deltaUp = vBatMedian - vWinMin;
  float totalSagFromOcv = acquyLastStableV - vBatMedian;

  // ★ FIX SẠC: khai báo sag sớm
  float sag = acquyLastStableV - vBatMedian;
  if (sag < 0.0f) sag = 0.0f;

  // ★ FIX SẠC: phát hiện sạc / ngắt sạc
  bool isChargingNow = (vBatMedian > acquyLastStableV + chargeRiseThr);

  if (isChargingNow && !acquyCharging) {
    acquyCharging = true;
    acquyChargeStart = now;
    LOGM(LOGMOD_ADC, "🔌 Bắt đầu SẠC: V=%.2f > OCV=%.2f",
         vBatMedian, acquyLastStableV);
  } else if (acquyCharging && !isChargingNow && (vBatMedian < acquyLastStableV + chargeRiseThr * 0.5f)) {
    acquyCharging = false;
    acquyChargeStopMs = now;
    acqState = ACQ_TRANSIENT;
    acqEventTime = now;
    acqIsRise = false;
    LOGM(LOGMOD_ADC, "🔋 Ngắt sạc — chờ hồi áp (V=%.2f, OCV=%.2f)",
         vBatMedian, acquyLastStableV);
  }

  // ============ STATE MACHINE PHÁT HIỆN TẢI & HỒI ÁP ============
  switch (acqState) {
    case ACQ_IDLE:
      {
        if (deltaDown >= dropThr || totalSagFromOcv >= dropThr) {
          preSagOcv = acquyLastStableV;
          acqState = ACQ_TRANSIENT;
          acqEventTime = now;
          acqIsRise = false;
          acquyStable = false;
          acquyStableStart = 0;
          LOGM(LOGMOD_ADC, "⬇️ TỤT ÁP %.2fV (OCV %.2f→%.2fV) chờ %lus",
               max(deltaDown, totalSagFromOcv), acquyLastStableV, vBatMedian, waitMs / 1000);
        } else if (deltaUp >= riseThr) {
          acqState = ACQ_TRANSIENT;
          acqEventTime = now;
          acqIsRise = true;
          acquyStable = false;
          acquyStableStart = 0;
          LOGM(LOGMOD_ADC, "⬆️ HỒI ÁP +%.2fV (%.2f→%.2fV) chờ %lus",
               deltaUp, vWinMin, vBatMedian, waitMs / 1000);
        } else {
          if ((vWinMax - vWinMin) < (stableBandV * 0.5f)) {
            if (acquyStableStart == 0) acquyStableStart = now;
            if (now - acquyStableStart >= ACQY_STABLE_TIME_MS) {
              acquyStable = true;
              if (vBatMedian >= acquyLastStableV) {
                acquyLastStableV = acquyLastStableV * 0.70f + vBatMedian * 0.30f;
              } else {
                // ★ FIX: khai báo maxDecay đúng scope
                float maxDecay = 0.0002f;
                if (sag < SAG_LIGHT) {
                  if (acquyLastStableV - vBatMedian > maxDecay) acquyLastStableV -= maxDecay;
                  else acquyLastStableV = vBatMedian;
                } else {
                  // Không decay khi đang tải — giữ OCV cuối
                }
              }
              acquyLastStableUpdate = now;
            }
          } else {
            acquyStable = false;
            acquyStableStart = 0;
          }
        }
        break;
      }

    case ACQ_TRANSIENT:
      {
        if (!acqIsRise && deltaUp >= riseThr) {
          acqIsRise = true;
          acqEventTime = now;
          LOGM(LOGMOD_ADC, "⬆️ Phát hiện nhả ga hồi áp +%.2fV", deltaUp);
        }
        if ((vWinMax - vWinMin) > stableBandV) {
          acqEventTime = now;
        }

        if (now - acqEventTime >= waitMs) {
          acqState = ACQ_SETTLING;
          settleWindowStart = now;
          acqSettleMinV = vBatMedian;
          acqSettleMaxV = vBatMedian;
          LOGM(LOGMOD_ADC, "⏳ Hết %lus — bắt đầu đo độ phẳng 2s (%s)",
               waitMs / 1000, acqIsRise ? "hồi áp" : "có tải");
        }
        break;
      }

    case ACQ_SETTLING:
      {
        if (!acqIsRise && deltaUp >= riseThr) {
          acqIsRise = true;
          acqState = ACQ_TRANSIENT;
          acqEventTime = now;
          break;
        }

        if (vBatMedian < acqSettleMinV) acqSettleMinV = vBatMedian;
        if (vBatMedian > acqSettleMaxV) acqSettleMaxV = vBatMedian;
        float band = acqSettleMaxV - acqSettleMinV;

        if (band >= stableBandV) {
          settleWindowStart = now;
          acqSettleMinV = vBatMedian;
          acqSettleMaxV = vBatMedian;
        } else if (now - settleWindowStart >= 2000UL) {
          float vFlat = (acqSettleMinV + acqSettleMaxV) * 0.5f;

          // ★ FIX SẠC: đang sạc → KHÔNG chốt OCV theo áp sạc
          if (acquyCharging) {
            if (vFlat >= vFullLim - chargeFullMargin) {
              acquyLastStableV = vFullLim;
              LOGM(LOGMOD_ADC, "⚡ Sạc đầy — OCV = vFullLim = %.2fV", vFullLim);
            }
            // Ngược lại: giữ nguyên OCV cũ
            acquyStable = true;
            acquyLastStableUpdate = now;
            acqState = ACQ_IDLE;
          }
          // ★ FIX SẠC: vừa ngắt sạc < 5 phút → chốt OCV nhanh
          else if (acquyChargeStopMs > 0 && (now - acquyChargeStopMs) < 300000UL) {
            float dropAfterCharge = acquyLastStableV - vFlat;
            if (dropAfterCharge < 0.30f) {
              // Không tải → chốt OCV = áp nghỉ
              acquyLastStableV = vFlat;
              acquyStable = true;
              acquyLastStableUpdate = now;
              acqState = ACQ_IDLE;
              LOGM(LOGMOD_ADC, "✅ Ngắt sạc → chốt OCV = %.2fV", vFlat);
            } else {
              // Còn tải → chờ tiếp
              acqState = ACQ_TRANSIENT;
              acqEventTime = now;
              acquyStable = false;
            }
          }
          // Logic cũ — hồi áp sau xả
          else if (acqIsRise || vFlat >= acquyLastStableV - (dropThr * 0.6f)) {
            acquyStable = true;
            if (preSagOcv > 0.1f && vFlat < preSagOcv && (preSagOcv - vFlat) < 1.0f) {
              acquyLastStableV = acquyLastStableV * 0.60f + vFlat * 0.40f;
            } else if (vFlat > acquyLastStableV) {
              acquyLastStableV = vFlat;
            } else {
              acquyLastStableV = acquyLastStableV * 0.75f + vFlat * 0.25f;
            }
            acquyLastStableUpdate = now;
            acqState = ACQ_IDLE;
            LOGM(LOGMOD_ADC, "✅ Chốt OCV sau hồi áp: %.2fV (áp nghỉ: %.2fV)", acquyLastStableV, vFlat);
          } else {
            acquyStable = false;
            float maxLoadDecay = 0.0005f;
            if (acquyLastStableV - vFlat > maxLoadDecay) {
              acquyLastStableV -= maxLoadDecay;
            }
            settleWindowStart = now;
            acqSettleMinV = vBatMedian;
            acqSettleMaxV = vBatMedian;
          }
        }
        break;
      }
  }

  // ============ TÍNH BÙ NỘI TRỞ & OCV ============
  // ★ FIX: bỏ đoạn gán ocvEst không tồn tại
  acquySag = sag;
  acquyOcvEstimated = acquyLastStableV;
  acquyStableV = acquyLastStableV;

  float vOcvComp = acquyLastStableV;

  int socRawNow = calcSocFromV(vBatMedian);
  if (socRawNow < 0) socRawNow = 0;
  if (socRawNow > 100) socRawNow = 100;

  int socTarget = calcSocFromV(vOcvComp);
  if (socTarget < 0) socTarget = 0;
  if (socTarget > 100) socTarget = 100;

  acquySocRaw = (uint8_t)socRawNow;

  // ============ BỘ KHÓA 1 CHIỀU & RATE LIMITER ============
  if (socTarget < (int)socFiltered) {
    // Giảm SOC — KHÔNG làm khi đang sạc
    if (acquyCharging) {
      chargeDetectStart = 0;
    } else {
      unsigned long dropIntervalMs = (acquyStable && sag < SAG_LIGHT) ? 8000UL : 35000UL;
      if (now - lastSocDropMs >= dropIntervalMs) {
        socFiltered--;
        lastSocDropMs = now;
      }
      chargeDetectStart = 0;
    }
  } else if (socTarget > (int)socFiltered) {
    // ★ FIX SẠC: đang sạc giữa chừng → KHÔNG tăng
    if (acquyCharging && vBatMedian < vFullLim - chargeFullMargin) {
      chargeDetectStart = 0;
    }
    // ★ FIX SẠC: đang sạc & chạm đầy → cho tăng nhanh
    else if (acquyCharging && vBatMedian >= vFullLim - chargeFullMargin) {
      if (acquyStable && now - lastSocDropMs >= 3000UL) {
        socFiltered++;
        lastSocDropMs = now;
      }
    }
    // ★ FIX SẠC: vừa ngắt sạc < 5 phút → tăng nhanh
    else if (acquyChargeStopMs > 0 && (now - acquyChargeStopMs) < 300000UL && acquyStable) {
      if (now - lastSocDropMs >= 2000UL) {
        socFiltered++;
        lastSocDropMs = now;
      }
    }
    // Logic cũ
    else if (acquyStable && sag < 0.2f && (socTarget >= (int)socFiltered + 5 || vBatMedian >= vFullLim)) {
      if (chargeDetectStart == 0) chargeDetectStart = now;
      if (now - chargeDetectStart >= 15000UL) {
        socFiltered++;
        chargeDetectStart = now - 12000UL;
        lastSocDropMs = now;
      }
    } else {
      chargeDetectStart = 0;
    }
  } else {
    chargeDetectStart = 0;
  }

  acquySocStable = socFiltered;
  acquySoc = socFiltered;

  lockState();
  realBmsSoc = acquySocStable;
  bmsTotalVoltage = vOcvComp;
  bmsHasTotalV = true;
  bmsSocTimestamp = now;
  unlockState();

  if (!manualSimMode) {
    if (abs((int)socFiltered - (int)currentSoc) >= ACQY_OUTPUT_HYST) {
      if (now - acquyLastOutputChange > ACQY_OUTPUT_DELAY_MS) {
        applyOutput(socFiltered);
        acquyLastOutputChange = now;
      }
    }
  }

    // ★ FIX SẠC: xóa cờ ngắt sạc sau 5 phút
  if (acquyChargeStopMs > 0 && (now - acquyChargeStopMs) > 300000UL) {
    acquyChargeStopMs = 0;
  }

  static unsigned long lastAdcLog = 0;
  if (now - lastAdcLog > 10000) {
    lastAdcLog = now;
    LOGM(LOGMOD_ADC, "V_do=%.2fV | OCV=%.2fV | Sag=%.2fV | SOC_Raw=%d%% | SOC_Chuan=%d%% | State=%d",
         acquyRawV, acquyLastStableV, acquySag, acquySocRaw, acquySocStable, (int)acqState);
  }
}

#endif