# 📱 BMS & ACQUY Controller v3.4

Firmware ESP32 đọc SOC pin qua BMS Bluetooth (JiKong / BMS MOVE) **hoặc** đo trực tiếp ắc quy qua ADC, xuất điện áp tương ứng ra DAC/PWM, cấu hình & theo dõi qua Web UI, hỗ trợ OTA online.

![Platform](https://img.shields.io/badge/platform-ESP32-blue)
![Framework](https://img.shields.io/badge/framework-Arduino-teal)
![Version](https://img.shields.io/badge/version-3.4-green)
![License](https://img.shields.io/badge/license-MIT-lightgrey)

---

## ✨ Tính năng chính

### 🔋 Hai chế độ đo nguồn

| Chế độ | Mô tả |
|--------|-------|
| **BMS (Bluetooth)** | Kết nối BMS JiKong / BMS MOVE qua BLE, đọc SOC, điện áp tổng, dòng, công suất, 16 cell |
| **Ắc quy (ADC)** | Đọc điện áp qua cầu phân áp R1/R2, lọc Median + EMA, nội suy OCV → SOC |

### ⚡ Xử lý ắc quy thông minh

- **Nội suy OCV → SOC** từ bảng động (tải từ server hoặc nhập thủ công)
- **Phát hiện sạc / ngắt sạc** tự động (OCV = áp sạc − 2V)
- **State machine**: `IDLE` → `TRANSIENT` → `SETTLING` — chống nhảy SOC khi tải thay đổi
- **Bù sụt áp (sag)** theo mức tải: nhẹ / vừa / nặng → tốc độ cập nhật SOC khác nhau
- **Rate limiter** 1 chiều: SOC giảm nhanh khi tải nặng duy trì, tăng chậm khi hồi áp
- **Giảm mù** khi tải nhẹ + áp phẳng (tránh SOC đứng yên)
- Hỗ trợ **Ắc quy chì** (12V/bình) và **LFP** (LiFePO4, 2.8–3.45V/cell)
- Cảnh báo **quá áp (OVP)**

### 📡 Bluetooth (BLE)

- Tự động nhận diện giao thức BMS: **JiKong** (JK02_24S / JK02_32S) hoặc **BMS MOVE**
- Auto-connect khi chỉ tìm thấy 1 BMS
- Quét, lưu MAC, nhập MAC thủ công
- Cache dữ liệu — hiển thị SOC cũ khi mất kết nối

### 🌐 Web UI

- Giao diện responsive, accordion gọn gàng
- **Monitor**: SOC, trạng thái kết nối, AP/STA
- **Terminal**: xem log Serial realtime, lọc theo module (ADC/BLE/WiFi/OUT/OTA/SUM/BOOT)
- **Chip Info**: RAM, Flash, CPU load, uptime
- **Output**: DAC (GPIO25/26) hoặc PWM (tần số/độ phân giải tùy chỉnh)
- **Bảng điện áp theo %**: dùng preset có sẵn (tải từ server) hoặc nhập thủ công
- **Bảng OCV → SOC**: chỉnh sửa & preview trực tiếp trên web
- **Cài đặt WiFi**: AP (SSID/mật khẩu/timeout), STA (kết nối WiFi nhà)
- **OTA**: nạp từ file `.bin` hoặc online từ server

### 🔄 OTA & Cloud

- Auto-update khi có bản mới (tùy chọn)
- Tải preset điện áp & bảng OCV từ GitHub Pages
- Nguồn dữ liệu: `https://bjendaik.github.io/hienthipin/json/`

### 💡 LED & hệ thống

- LED báo trạng thái: nháy (config) / sáng (kết nối) / breathe (BMS OK)
- Mutex bảo vệ state đa task
- AP tự tắt sau timeout (cấu hình được, `0` = luôn bật)
- Captive portal — tự mở trang cấu hình khi kết nối AP

---

## 🔌 Phần cứng

### Sơ đồ chân mặc định

| Chức năng | GPIO | Ghi chú |
|-----------|------|---------|
| **ADC đo ắc quy** | GPIO32 | Ưu tiên ADC1: 32/33/34/35 |
| **DAC output** | GPIO25 (hoặc 26) | ESP32 Classic |
| **PWM output** | GPIO14 | Tần số/độ phân giải tùy chỉnh |
| **LED** | GPIO2 | PWM 5kHz, 8-bit |

### Cầu phân áp đo ắc quy
V_battery ──[ R1 ]──┬── GPIO32 (ADC)
                    │
                  [ R2 ]
                    │
                   GND

- **Điện áp ADC** = `V_battery × R2 / (R1 + R2)`
- **Điện áp đo được** = `V_ADC × (R1 + R2) / R2 × Gain + Offset`
- Mặc định: `R1 = 100kΩ`, `R2 = 2.05kΩ`
- ⚠️ **V_ADC phải ≤ 3.3V** — Web UI tự cảnh báo nếu vượt ngưỡng

### Ví dụ cấu hình theo loại bình

| Loại bình | R1 | R2 | V_ADC max |
|-----------|-----|-----|-----------|
| 12V (1 bình chì) | 100k | 20k | ~2.4V |
| 48V (4 bình chì) | 100k | 4.7k | ~2.8V |
| 16S LFP (58.4V) | 100k | 4.7k | ~2.7V |

---

## 🚀 Cài đặt & Build

### Yêu cầu

- **Arduino IDE** 2.x hoặc **PlatformIO**
- **ESP32 board package** (Espressif) ≥ 2.0.0
- Thư viện:
  - `NimBLE-Arduino` (≥ 1.4.0) — BLE nhẹ, tiết kiệm RAM

### Các bước

```bash
# 1. Clone repo
git clone https://github.com/bjendaik/hienthipin.git
cd hienthipin

# 2. Mở file .ino trong Arduino IDE
#    hoặc dùng PlatformIO: platformio run

# 3. Chọn board: ESP32 Dev Module
# 4. Chọn Partition Scheme: "Default 4MB with spiffs"
# 5. Nạp firmware
bash```
hienthipin3.4.ino    ← setup() / loop() chính
├── Config.h         ← Định nghĩa toàn bộ biến, struct, hằng số
├── SystemUtils.h    ← LED, CPU load, WiFi AP/STA, output DAC/PWM
├── AcquyManager.h   ← Đọc ADC, lọc, nội suy OCV→SOC, state machine
├── BleBms.h         ← BLE client, parse frame JiKong / BMS MOVE
├── CloudOta.h       ← Tải preset/OCV/update từ server, OTA online
├── IndexHtml.h      ← Toàn bộ HTML/CSS/JS của Web UI
└── WebUI.h          ← HTTP handlers, đăng ký routes

📖 Hướng dẫn sử dụng
Lần đầu khởi động
ESP32 phát WiFi BMS_Config / mật khẩu 12345678

Kết nối → captive portal tự mở (hoặc vào http://192.168.4.1)

Chọn nguồn dữ liệu:

🔋 BMS: bấm Quét Bluetooth → chọn BMS

⚡ Ắc quy: nhập thông số bình, R1/R2, hiệu chuẩn

Hiệu chuẩn ắc quy
Đo điện áp thực tế bằng VOM

Nhập vào ô "V thực tế" → bấm 🎯 Tính Gain

Kiểm tra điện áp hiển thị khớp với VOM (±0.1V)

Cấu hình bảng điện áp Output
Có sẵn: chọn preset → Áp dụng

Thủ công: thêm dải % đầu → % cuối → V output

Output = V_map (theo SOC) hoặc nội suy tuyến tính minV → maxV

OTA
Online: bật STA → Kiểm tra phiên bản → Nạp Firmware Online

Local: chọn file .bin → Nạp Firmware

Auto: bật toggle 🔄 Auto (chỉ khi có WiFi)

🔧 API Endpoints
<details> <summary>Xem danh sách đầy đủ</summary>
Endpoint	Method	Mô tả
/
GET	Web UI
/data
GET	JSON trạng thái đầy đủ
/chipinfo
GET	RAM/Flash/CPU
/serial	GET	Log Serial mới
/sim?soc=N / /sim?real=1	GET	Giả lập / về SOC thực
/saveout	GET	Lưu cấu hình DAC/PWM
/savevmap	GET	Lưu bảng điện áp
/presets	GET	Danh sách preset
/applypreset?id=N	GET	Áp dụng preset
/fetchpresets	GET	Tải preset từ server
/scan	GET	Quét BMS
/save?mac=...	GET	Lưu MAC BMS
/clearmac	GET	Xóa MAC
/toggleacquy?en=0/1	GET	Đổi chế độ BMS ↔ Ắc quy
/saveacquycfg	GET	Lưu cấu hình ắc quy
/saveacqparams	GET	Lưu ngưỡng tụt/tăng áp
/acquyjsoninfo	GET	Bảng OCV hiện tại
/fetchacquyjson	GET	Tải OCV từ server
/saveocvmanual	GET	Lưu OCV thủ công
/savewifi / /savesta	GET	Cấu hình WiFi
/saveapcfg / /saveaptimeout	GET	Cấu hình AP
/setlogmodules?mask=N	GET	Lọc log
/checkupdate	GET	Kiểm tra bản mới
/otaonline	GET	OTA online
/update	POST	OTA từ file
</details>
📊 Luồng xử lý ắc quy
text
┌─────────────┐
│  Đọc ADC    │ (32 mẫu, 50µs/mẫu)
└──────┬──────┘
       ▼
┌─────────────┐
│ Median (5)  │ → khử nhiễu xung
└──────┬──────┘
       ▼
┌─────────────┐
│ EMA α=0.25  │ → làm mượt
└──────┬──────┘
       ▼
┌─────────────────────────────┐
│ Phát hiện sạc?              │
│ V > OCV_nền + 0.3V          │
└──────┬──────────────────────┘
       │ Có                    │ Không
       ▼                       ▼
┌──────────────┐      ┌──────────────────┐
│ OCV = V − 2V │      │ State machine xả │
│ SOC tăng dần │      │ IDLE/TRANSIENT/  │
└──────────────┘      │ SETTLING         │
                      └────────┬─────────┘
                               ▼
                      ┌──────────────────┐
                      │ Nội suy OCV→SOC  │
                      │ + Rate limiter   │
                      └────────┬─────────┘
                               ▼
                      ┌──────────────────┐
                      │ applyOutput(SOC) │
                      └──────────────────┘
⚙️ Tham số cấu hình chính
Tham số	Mặc định	Mô tả
CHARGE_OCV_OFFSET	2.0V	OCV = áp sạc − 2V
CHARGE_FULL_HOLD_MS	30s	Giữ áp đầy để xác nhận 100%
chargeRiseThr	0.30V	Ngưỡng phát hiện bắt đầu sạc
dropThresholdLead
1.00V	Tụt áp coi là tải nặng (chì)
dropThresholdLfp	0.50V	Tụt áp coi là tải nặng (LFP)
settleLeadMs	60s	Thời gian chờ ổn định sau biến động
stableBandV	0.50V	Biên độ coi là phẳng
ACQY_MEDIAN_SIZE	5	Cửa sổ lọc median
ACQY_EMA_ALPHA	0.25	Hệ số EMA
SAG_LIGHT	0.4V	Ngưỡng tải nhẹ
🐛 Troubleshooting
<details> <summary><b>ADC đọc sai / lệch nhiều</b></summary>
Kiểm tra R1/R2 đúng giá trị thực tế (sai số 1% trở lên)

Dùng hiệu chuẩn nhanh: đo VOM → nhập V thực → Tính Gain

Tránh chân ADC2 (GPIO 0/2/4/12-15/25-27) khi WiFi bật

Thêm tụ 100nF từ chân ADC xuống GND để giảm nhiễu

</details><details> <summary><b>BMS không kết nối được</b></summary>
Đảm bảo BMS đang bật và trong tầm (≤ 10m)

Xóa MAC cũ → quét lại

Một số BMS MOVE yêu cầu login trước — firmware đã gửi sẵn loginCmd

Kiểm tra Terminal log module BLE

</details><details> <summary><b>SOC nhảy loạn</b></summary>
Tăng sett
leLeadMs
 / 
settleLfpMs
 (chờ lâu hơn)

Giảm stableBandV (yêu cầu phẳng hơn)

Kiểm tra bảng OCV → SOC có đúng thứ tự giảm dần không

Với LFP: vùng 3.2–3.3V phẳng → SOC khó chính xác, nên dùng BMS

</details><details> <summary><b>Web UI không load</b></summary>
Kết nối đúng SSID AP (mặc định BMS_Config)

Thử http://192.168.4.1 trực tiếp

Xóa cache trình duyệt

Kiểm tra log BOOT/WiFi qua Serial

</details><details> <summary><b>OTA thất bại</b></summary>
Đảm bảo file .bin đúng định dạng (không phải .bin.gz)

Partition scheme phải có 2 app slots (OTA)

WiFi phải ổn định — không OTA qua AP

Nếu online: kiểm tra update.json trên server có URL đúng

</details>
📝 Changelog
v3.4
✨ Thêm phát hiện sạc/ngắt sạc tự động (OCV = áp sạc − 2V)

✨ Khôi phục OCV gốc khi ngắt sạc do tải nặng

⚡ Rate limiter thích ứng theo mức tải (rất nặng/nặng/vừa/nhẹ)

⚡ Giảm mù khi tải nhẹ + áp phẳng

🐛 Fix parse HTTP date không có timezone

🐛 Fix ADC pin attenuation cho ESP32-S3/C3

v3.0
Thêm chế độ Ắc quy (ADC) song song với BMS

Bảng OCV → SOC động (tải từ server / nhập tay)

Web UI đầy đủ: Terminal, Chip Info, Preset, OCV Editor

📄 License
MIT License — xem file LICENSE.

🤝 Đóng góp
Pull request & issue luôn được chào đón!

👤 Tác giả
bjendaik — GitHub

<p align="center"> <i>Made with ❤️ for Vietnamese DIY battery community</i> </p> ```
