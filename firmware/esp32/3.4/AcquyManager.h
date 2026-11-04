#ifndef ACQUY_MANAGER_H
#define ACQUY_MANAGER_H

#include "Config.h"


// =====================================================
// Nội suy OCV -> SOC từ bảng động
// =====================================================
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


// =====================================================
// Xử lý đọc Ắc quy
// =====================================================
void processAcquyReading() {
  if (!acquyMode) return;
  if (millis() - lastAdcRead < 100) return;  // Chu kỳ lấy mẫu 100ms
  lastAdcRead = millis();
  unsigned long now = millis();

  static unsigned long settleWindowStart = 0;
  static unsigned long lastSocDropMs = 0;
  static unsigned long chargeDetectStart = 0;
  static unsigned long lastBlindDropMs = 0;
  static unsigned long heavyLoadStartMs = 0;
  static float preSagOcv = 0.0f;

  if (acquyR2 <= 0.01f) acquyR2 = 4.7f;
  float ratio = (acquyR1 + acquyR2) / acquyR2;

  // =====================================================
  // KHỞI TẠO LẦN ĐẦU (BẬT KHÓA ĐIỆN / ĐỔI CẤU HÌNH)
  // =====================================================
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
    lastBlindDropMs = 0;
    heavyLoadStartMs = 0;
    chargeDetectStart = 0;

    // Reset cờ sạc
    acquyCharging = false;
    acquyChargeStart = 0;
    acquyChargeStopMs = 0;
    chargeOcvEmaInit = false;
    chargeFullHoldStart = 0;
    ocvBaseForChargeDetect = 0.0f;
    ocvBeforeCharge = 0.0f;

    if (!manualSimMode) {
      applyOutput(socFiltered);
      acquyLastOutputChange = now;
    }
    LOGM(LOGMOD_ADC, "🟢 Khởi tạo OCV gốc: %.2fV -> SOC=%d%%", vInit, socFiltered);
  }

  // =====================================================
  // ĐỌC ADC & LỌC MEDIAN + EMA
  // =====================================================
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

  // =====================================================
  // ★ PHÁT HIỆN SẠC / NGẮT SẠC
  // =====================================================
  if (!acquyCharging && ocvBaseForChargeDetect == 0.0f) {
    ocvBaseForChargeDetect = acquyLastStableV;
  }

  float sagFromCharge = acquyLastStableV - vBatMedian;
  if (sagFromCharge < 0.0f) sagFromCharge = 0.0f;

  // ----- BẮT ĐẦU SẠC -----
  if (!acquyCharging && vBatMedian > ocvBaseForChargeDetect + chargeRiseThr) {
    acquyCharging = true;
    acquyChargeStart = now;
    chargeOcvEma = vBatMedian;
    chargeOcvEmaInit = true;
    chargeFullHoldStart = 0;
    acqState = ACQ_IDLE;
    acquyStable = false;
    acquyStableStart = 0;
    // ★ LƯU OCV GỐC trước sạc
    ocvBeforeCharge = acquyLastStableV;
    LOGM(LOGMOD_ADC, "🔌 BẮT ĐẦU SẠC: V=%.2f > OCV_nền=%.2f (+%.2fV) | OCV_gốc=%.2fV",
         vBatMedian, ocvBaseForChargeDetect,
         vBatMedian - ocvBaseForChargeDetect, ocvBeforeCharge);
  }

  // ----- ĐANG SẠC: OCV = ÁP SẠC − OFFSET -----
  if (acquyCharging) {
    if (!chargeOcvEmaInit) {
      chargeOcvEma = vBatMedian;
      chargeOcvEmaInit = true;
    } else {
      chargeOcvEma = chargeOcvEma * (1.0f - CHARGE_OCV_EMA_ALPHA)
                   + vBatMedian * CHARGE_OCV_EMA_ALPHA;
    }
    // ★ OCV = áp sạc − 2V (ước lượng áp nghỉ)
    float ocvCharging = chargeOcvEma - CHARGE_OCV_OFFSET;

    // Clamp: không thấp hơn OCV gốc, không cao hơn vFullLim
    if (ocvCharging < ocvBeforeCharge) ocvCharging = ocvBeforeCharge;
    if (ocvCharging > vFullLim) ocvCharging = vFullLim;

    // OCV chỉ tăng khi sạc (không nhảy lùi)
    if (ocvCharging > acquyLastStableV) acquyLastStableV = ocvCharging;

    // Kiểm tra "đầy thật": áp gần vFullLim giữ ổn định
    if (vBatMedian >= vFullLim - chargeFullMargin) {
      if (chargeFullHoldStart == 0) {
        chargeFullHoldStart = now;
      } else if (now - chargeFullHoldStart >= CHARGE_FULL_HOLD_MS) {
        acquyCharging = false;
        acquyChargeStopMs = now;
        ocvBaseForChargeDetect = 0.0f;
        chargeFullHoldStart = 0;
        ocvBeforeCharge = 0.0f;
        acquyLastStableV = vFullLim;
        acqState = ACQ_IDLE;
        acquyStable = true;
        acquyStableStart = now;
        LOGM(LOGMOD_ADC, "✅ SẠC ĐẦY: OCV=%.2fV, SOC=100%%", vFullLim);
        goto endChargeCheck;
      }
    } else {
      chargeFullHoldStart = 0;
    }

    // ★ CHỈ THOÁT SẠC KHI CÓ SỰ KIỆN TẢI NẶNG
    if (sagFromCharge >= dropThr) {
      acquyCharging = false;
      acquyChargeStopMs = now;
      ocvBaseForChargeDetect = 0.0f;
      chargeFullHoldStart = 0;
      // ★ KHÔI PHỤC OCV GỐC trước sạc
      if (ocvBeforeCharge > 0.1f) {
        acquyLastStableV = ocvBeforeCharge;
      }
      ocvBeforeCharge = 0.0f;
      acqState = ACQ_TRANSIENT;
      acqEventTime = now;
      acqIsRise = false;
      acquyStable = false;
      acquyStableStart = 0;
      LOGM(LOGMOD_ADC, "🔥 TẢI NẶNG (sag=%.2fV ≥ %.2fV) → NGẮT SẠC, khôi phục OCV=%.2fV",
           sagFromCharge, dropThr, acquyLastStableV);
    }

  endChargeCheck:;
  }

  // =====================================================
  // KHI ĐANG SẠC: KHÔNG CHẠY STATE MACHINE XẢ
  // =====================================================
  if (acquyCharging) {
    acquySag = 0.0f;
    acquyOcvEstimated = acquyLastStableV;
    acquyStableV = acquyLastStableV;

    int socNow = calcSocFromV(acquyLastStableV);
    if (socNow < 0) socNow = 0;
    if (socNow > 100) socNow = 100;
    acquySocStable = (uint8_t)socNow;
    acquySoc = acquySocStable;
    int socRawNow2 = calcSocFromV(vBatMedian);
    if (socRawNow2 < 0) socRawNow2 = 0;
    if (socRawNow2 > 100) socRawNow2 = 100;
    acquySocRaw = (uint8_t)socRawNow2;

    socFiltered = acquySocStable;
    socFilteredInit = true;

    lockState();
    realBmsSoc = acquySocStable;
    bmsTotalVoltage = vBatMedian;   // hiển thị áp sạc thực
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

    static unsigned long lastChargeLog = 0;
    if (now - lastChargeLog > 10000) {
      lastChargeLog = now;
      LOGM(LOGMOD_ADC, "🔌 ĐANG SẠC: V=%.2f | OCV=%.2f (=V−%.1fV) | SOC=%d%% | Full=%.2fV",
           vBatMedian, acquyLastStableV, CHARGE_OCV_OFFSET, acquySocStable, vFullLim);
    }
    return;  // thoát sớm, không chạy xả
  }

  // =====================================================
  // STATE MACHINE XẢ BÌNH THƯỜNG
  // =====================================================
  float totalSagFromOcv = acquyLastStableV - vBatMedian;
  float sag = acquyLastStableV - vBatMedian;
  if (sag < 0.0f) sag = 0.0f;

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

              // ★ OCV bám áp đo theo mức tải
              if (sag < SAG_LIGHT) {
                // Tải nhẹ → OCV = áp đo luôn (không trễ)
                acquyLastStableV = vBatMedian;
              } else if (sag < dropThr) {
                // Tải vừa → bám chậm (EMA 8%/chu kỳ)
                acquyLastStableV = acquyLastStableV * 0.92f + vBatMedian * 0.08f;
              }
              // Tải nặng (sag >= dropThr): không bám, giữ OCV cũ
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

          // ★ Vừa ngắt sạc < 5 phút → chốt OCV nhanh
          if (acquyChargeStopMs > 0 && (now - acquyChargeStopMs) < 300000UL) {
            float dropAfterCharge = acquyLastStableV - vFlat;
            if (dropAfterCharge < 0.30f) {
              acquyLastStableV = vFlat;
              acquyStable = true;
              acquyLastStableUpdate = now;
              acqState = ACQ_IDLE;
              LOGM(LOGMOD_ADC, "✅ Ngắt sạc → chốt OCV = %.2fV", vFlat);
            } else {
              acqState = ACQ_TRANSIENT;
              acqEventTime = now;
              acquyStable = false;
            }
          }
          // Logic cũ — hồi áp sau xả
          else if (acqIsRise || vFlat >= acquyLastStableV - (dropThr * 0.6f)) {
            // ★ CHẶN VỌT: nhả tải đột ngột → áp vọt cao → không chốt
            float overshoot = vFlat - acquyLastStableV;
            float overshootLimit = isLfp ? 0.30f : 0.80f;
            if (overshoot > overshootLimit) {
              acquyStable = false;
              acqState = ACQ_TRANSIENT;
              acqEventTime = now;
              acqSettleMinV = vBatMedian;
              acqSettleMaxV = vBatMedian;
              LOGM(LOGMOD_ADC, "⚠️ Vọt áp +%.2fV (vượt %.2fV) → bỏ qua, chờ ổn định",
                   overshoot, overshootLimit);
            } else {
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
            }
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

  // =====================================================
  // TÍNH BÙ NỘI TRỞ & OCV
  // =====================================================
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

  // =====================================================
  // BỘ KHÓA 1 CHIỀU & RATE LIMITER — TẢI NẶNG DUY TRÌ GIẢM NHANH
  // =====================================================
  bool heavyLoadNow = (sag >= dropThr);
  bool veryHeavyNow = (sag >= dropThr * 2.0f);
  if (heavyLoadNow) {
    if (heavyLoadStartMs == 0) heavyLoadStartMs = now;
  } else {
    heavyLoadStartMs = 0;
  }
  unsigned long heavyLoadDurMs = (heavyLoadStartMs > 0) ? (now - heavyLoadStartMs) : 0;

  #define HEAVY_LOAD_PERSIST_MS 10000UL

  if (socTarget < (int)socFiltered) {
    // ★ Tải càng nặng + duy trì càng lâu → giảm càng nhanh
    unsigned long dropIntervalMs;
    if (veryHeavyNow && heavyLoadDurMs >= HEAVY_LOAD_PERSIST_MS) {
      dropIntervalMs = 3000UL;   // rất nặng + ≥10s → 1%/3s
    } else if (heavyLoadNow && heavyLoadDurMs >= HEAVY_LOAD_PERSIST_MS) {
      dropIntervalMs = 8000UL;   // nặng + ≥10s → 1%/8s
    } else if (heavyLoadNow) {
      dropIntervalMs = 25000UL;  // nặng < 10s → coi như vừa
    } else if (sag >= SAG_LIGHT) {
      dropIntervalMs = 25000UL;  // vừa → 1%/25s
    } else {
      dropIntervalMs = 60000UL;  // nhẹ → 1%/60s
    }
    if (!acquyStable && dropIntervalMs < 35000UL) dropIntervalMs = 35000UL;
    if (now - lastSocDropMs >= dropIntervalMs) {
      socFiltered--;
      lastSocDropMs = now;
      if (heavyLoadDurMs >= HEAVY_LOAD_PERSIST_MS) {
        LOGM(LOGMOD_ADC, "🔥 Tải nặng duy trì %lus (sag=%.2fV) → SOC giảm nhanh: %d%%",
             heavyLoadDurMs / 1000, sag, socFiltered);
      }
    }
    chargeDetectStart = 0;
  } else if (socTarget > (int)socFiltered) {
    // ★ Chênh áp đo so với OCV (áp đo > OCV → dấu hiệu sạc/hồi áp/tải nhẹ)
    float gapUp = vBatMedian - vOcvComp;
    if (gapUp < 0.0f) gapUp = 0.0f;

    // Vừa ngắt sạc < 5 phút → tăng nhanh (ưu tiên)
    if (acquyChargeStopMs > 0 && (now - acquyChargeStopMs) < 300000UL && acquyStable) {
      if (now - lastSocDropMs >= 2000UL) {
        socFiltered++;
        lastSocDropMs = now;
      }
    }
    // ★ Chênh lớn → tăng từ từ chắc; chênh nhỏ → tăng nhanh
    else if (acquyStable && sag < 0.2f) {
      unsigned long riseIntervalMs;
      if (gapUp >= chargeRiseThr * 2.0f) {
        riseIntervalMs = 15000UL;  // chênh rất lớn (chắc sạc) → 1%/15s
      } else if (gapUp >= chargeRiseThr) {
        riseIntervalMs = 10000UL;  // chênh lớn → 1%/10s
      } else if (gapUp >= 0.1f) {
        riseIntervalMs = 6000UL;   // chênh vừa → 1%/6s
      } else {
        riseIntervalMs = 4000UL;   // chênh nhỏ (tải nhẹ) → 1%/4s
      }
      if (now - lastSocDropMs >= riseIntervalMs) {
        socFiltered++;
        lastSocDropMs = now;
      }
    } else {
      chargeDetectStart = 0;
    }
  } else {
    chargeDetectStart = 0;
  }

  // =====================================================
  // ★ GIẢM MÙ KHI TẢI NHẸ + ÁP PHẲNG
  // =====================================================
  if (socTarget == (int)socFiltered && acquyStable && sag > 0.05f && sag < SAG_LIGHT) {
    if (lastBlindDropMs == 0) lastBlindDropMs = now;
    if (now - lastBlindDropMs >= 60000UL) {
      if (socFiltered > 0) {
        socFiltered--;
        LOGM(LOGMOD_ADC, "🕐 Tải nhẹ phẳng (sag=%.2fV) → SOC giảm mù: %d%%", sag, socFiltered);
      }
      lastBlindDropMs = now;
    }
  } else if (sag >= SAG_LIGHT || socTarget != (int)socFiltered) {
    lastBlindDropMs = 0;
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

  // ★ Xóa cờ ngắt sạc sau 5 phút
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
