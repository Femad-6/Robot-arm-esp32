




// wifi + 串口
/*  ESP32-S3-WROOM-1
 *  10 Hz 串口/UDP 输出：2×柔性传感器 + 2×MPU6050 (Ax/Ay/Az, Gx/Gy/Gz)
 *  稳定化：MPU DLPF、IMU多次采样平均、毛刺限幅、EMA、上电零偏标定
 *
 *  CSV:
 *  t_ms,flex1_raw,flex1_ema,flex2_raw,flex2_ema,
 *  Ax1,Ay1,Az1,Gx1,Gy1,Gz1,Ax2,Ay2,Az2,Gx2,Gy2,Gz2
 */

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <WiFi.h>
#include <WiFiUdp.h>

/* --------- I2C & 引脚 --------- */
#define I2C_SDA_PIN   8
#define I2C_SCL_PIN   9
#define I2C_FREQ_HZ   400000

/* --------- 柔性传感器 ADC --------- */
#define FLEX1_PIN     1   // 柔性1 -> GPIO1 (ADC1)
#define FLEX2_PIN     2   // 柔性2 -> GPIO2 (ADC1)
#define ADC_BITS      12
#define ADC_MAX       ((1<<ADC_BITS)-1)
#ifndef ADC_11db
#define ADC_11db ADC_ATTEN_DB_11
#endif

/* --------- MPU6050 寄存器 --------- */
#define MPU_ADDR_68       0x68
#define MPU_ADDR_69       0x69
#define REG_SMPLRT_DIV    0x19
#define REG_CONFIG        0x1A
#define REG_GYRO_CONFIG   0x1B
#define REG_ACCEL_CONFIG  0x1C
#define REG_PWR_MGMT_1    0x6B
#define REG_PWR_MGMT_2    0x6C
#define REG_WHO_AM_I      0x75
#define REG_ACCEL_XOUT_H  0x3B   // 连读 14B 到 GYRO_ZOUT_L

/* --------- 常量 --------- */
// 出厂默认量程：加速度 ±2g、陀螺 ±250°/s
const float ACCEL_LSB_2G  = 16384.0f;  // LSB/g
const float GYRO_LSB_250  = 131.0f;    // LSB/(deg/s)

/* 输出频率 10 Hz */
const uint32_t PERIOD_MS = 100;
uint32_t last_ms = 0;

/* 柔性：多采样 + EMA */
const int   FLEX_OVERSAMPLE = 8;
const float FLEX_EMA_ALPHA  = 0.2f;
int   flex1_raw = 0, flex2_raw = 0;
float flex1_ema = 0.0f, flex2_ema = 0.0f;

/* IMU：多采样 + 突变限幅 + EMA */
const int   IMU_OVERSAMPLE       = 8;      // 每周期读多次求平均
const float ACC_SPIKE_THRESH_G   = 0.20f;  // 加速度毛刺阈值(g)
const float GYR_SPIKE_THRESH_DPS = 30.0f;  // 陀螺毛刺阈值(°/s)
const float ACC_EMA_ALPHA        = 0.3f;   // 加速度EMA系数
const float GYR_EMA_ALPHA        = 0.3f;   // 陀螺EMA系数

/* ======== Wi-Fi / UDP ======== */
static const char* WIFI_SSID  = "HONOR";
static const char* WIFI_PASS  = "74123000"; // 开放热点请设为 ""
static const char* DEST_IP    = "192.168.43.55";
static const uint16_t DEST_PORT = 5005;

WiFiUDP udp;

void wifiEnsureConnected() {
  if (WiFi.status() == WL_CONNECTED) return;

  static uint32_t last_try = 0;
  uint32_t now = millis();
  if (now - last_try < 3000) return;  // 3 秒重试节流
  last_try = now;

  WiFi.disconnect(true, true);
  delay(100);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  for (int i=0; i<25; ++i) {  // 最长 ~2.5s
    if (WiFi.status() == WL_CONNECTED) break;
    delay(100);
  }

  if (WiFi.status() == WL_CONNECTED) {
    udp.begin(0); // 自动端口
    Serial.print(F("[WiFi] Connected. IP="));
    Serial.println(WiFi.localIP());
  } else {
    Serial.println(F("[WiFi] Connecting..."));
  }
}

/* ======== I2C 工具 ======== */
static inline void i2cWrite8(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission(true);
}

static inline bool i2cReadBytes(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;        // restart
  uint8_t r = Wire.requestFrom((int)addr, (int)len, (int)true);
  if (r != len) return false;
  for (uint8_t i=0;i<len;i++) buf[i] = Wire.read();
  return true;
}

/* ======== IMU 滤波与标定状态 ======== */
struct ImuFilterState {
  // EMA
  float ax_e=0, ay_e=0, az_e=0;
  float gx_e=0, gy_e=0, gz_e=0;
  // 上一平均值（突变限幅）
  float ax_p=0, ay_p=0, az_p=0;
  float gx_p=0, gy_p=0, gz_p=0;
  bool  inited=false;
};

struct ImuCalib {
  float ax_b=0, ay_b=0, az_b=0;  // 可选：把重力偏 1g 到 0
  float gx_b=0, gy_b=0, gz_b=0;
  bool  ready=false;
};

ImuFilterState imu1, imu2;
ImuCalib imu1_bias, imu2_bias;

/* ======== MPU6050 初始化到稳态 ======== */
bool mpuInit(uint8_t addr) {
  uint8_t who=0;
  if (!i2cReadBytes(addr, REG_WHO_AM_I, &who, 1)) return false;
  if (who != 0x68) return false;

  // 唤醒 & 选择PLL
  i2cWrite8(addr, REG_PWR_MGMT_1, 0x01); // 时钟=PLL XGYRO
  delay(10);
  i2cWrite8(addr, REG_PWR_MGMT_2, 0x00); // 使能所有轴
  delay(10);

  // 采样率与DLPF：1kHz/(9+1)=100Hz，DLPF_CFG=4(~20Hz带宽)
  i2cWrite8(addr, REG_SMPLRT_DIV, 9);
  i2cWrite8(addr, REG_CONFIG,     0x04);

  // 量程：±2g / ±250 dps
  i2cWrite8(addr, REG_ACCEL_CONFIG, 0x00);
  i2cWrite8(addr, REG_GYRO_CONFIG,  0x00);
  delay(30);
  return true;
}

/* 一次性连读 14B，保证同一拍数据 */
static inline bool mpuReadOnce14(uint8_t addr,
                                 float& ax, float& ay, float& az,
                                 float& gx, float& gy, float& gz) {
  uint8_t raw14[14];
  if (!i2cReadBytes(addr, REG_ACCEL_XOUT_H, raw14, 14)) return false;

  int16_t axr = (int16_t)((raw14[0]<<8)  | raw14[1]);
  int16_t ayr = (int16_t)((raw14[2]<<8)  | raw14[3]);
  int16_t azr = (int16_t)((raw14[4]<<8)  | raw14[5]);
  int16_t gxr = (int16_t)((raw14[8]<<8)  | raw14[9]);
  int16_t gyr = (int16_t)((raw14[10]<<8) | raw14[11]);
  int16_t gzr = (int16_t)((raw14[12]<<8) | raw14[13]);

  ax = axr / ACCEL_LSB_2G;
  ay = ayr / ACCEL_LSB_2G;
  az = azr / ACCEL_LSB_2G;
  gx = gxr / GYRO_LSB_250;
  gy = gyr / GYRO_LSB_250;
  gz = gzr / GYRO_LSB_250;
  return true;
}

/* 上电静止零偏标定（可选但强烈建议） */
bool mpuCalibrate(uint8_t addr, ImuCalib& cb, uint16_t n=200) {
  float sax=0, say=0, saz=0, sgx=0, sgy=0, sgz=0;
  for (uint16_t i=0;i<n;i++) {
    float ax,ay,az,gx,gy,gz;
    if (!mpuReadOnce14(addr, ax,ay,az,gx,gy,gz)) return false;
    sax+=ax; say+=ay; saz+=az; sgx+=gx; sgy+=gy; sgz+=gz;
    delay(5);
  }
  cb.ax_b = sax/n;
  cb.ay_b = say/n;
  cb.az_b = saz/n - 1.0f;  // 若静止且Z≈+1g，把重力平到0
  cb.gx_b = sgx/n;
  cb.gy_b = sgy/n;
  cb.gz_b = sgz/n;
  cb.ready = true;
  return true;
}

/* 多次采样平均 + 毛刺限幅 + EMA + 去偏 */
bool mpuReadFiltered(uint8_t addr, ImuFilterState& st,
                     float& ax, float& ay, float& az,
                     float& gx, float& gy, float& gz) {
  // 1) 多次采样求平均
  float sax=0, say=0, saz=0, sgx=0, sgy=0, sgz=0;
  for (int i=0; i<IMU_OVERSAMPLE; ++i) {
    float ax1, ay1, az1, gx1, gy1, gz1;
    if (!mpuReadOnce14(addr, ax1, ay1, az1, gx1, gy1, gz1)) return false;
    sax += ax1; say += ay1; saz += az1;
    sgx += gx1; sgy += gy1; sgz += gz1;
    delayMicroseconds(300);
  }
  float ax_avg = sax / IMU_OVERSAMPLE;
  float ay_avg = say / IMU_OVERSAMPLE;
  float az_avg = saz / IMU_OVERSAMPLE;
  float gx_avg = sgx / IMU_OVERSAMPLE;
  float gy_avg = sgy / IMU_OVERSAMPLE;
  float gz_avg = sgz / IMU_OVERSAMPLE;

  // 1.5) 去偏（若已标定）
  ImuCalib* pcb = nullptr;
  if (addr == MPU_ADDR_68) pcb = &imu1_bias;
  else if (addr == MPU_ADDR_69) pcb = &imu2_bias;
  if (pcb && pcb->ready) {
    ax_avg -= pcb->ax_b; ay_avg -= pcb->ay_b; az_avg -= pcb->az_b;
    gx_avg -= pcb->gx_b; gy_avg -= pcb->gy_b; gz_avg -= pcb->gz_b;
  }

  if (!st.inited) {
    st.ax_p = st.ax_e = ax_avg;
    st.ay_p = st.ay_e = ay_avg;
    st.az_p = st.az_e = az_avg;
    st.gx_p = st.gx_e = gx_avg;
    st.gy_p = st.gy_e = gy_avg;
    st.gz_p = st.gz_e = gz_avg;
    st.inited = true;
  }

  // 2) 毛刺限幅（相对上一次平均值）
  auto limit = [](float prev, float cur, float thr){
    float d = cur - prev;
    if (fabsf(d) > thr) return prev + copysignf(thr, d);
    return cur;
  };
  ax_avg = limit(st.ax_p, ax_avg, ACC_SPIKE_THRESH_G);
  ay_avg = limit(st.ay_p, ay_avg, ACC_SPIKE_THRESH_G);
  az_avg = limit(st.az_p, az_avg, ACC_SPIKE_THRESH_G);
  gx_avg = limit(st.gx_p, gx_avg, GYR_SPIKE_THRESH_DPS);
  gy_avg = limit(st.gy_p, gy_avg, GYR_SPIKE_THRESH_DPS);
  gz_avg = limit(st.gz_p, gz_avg, GYR_SPIKE_THRESH_DPS);

  st.ax_p = ax_avg; st.ay_p = ay_avg; st.az_p = az_avg;
  st.gx_p = gx_avg; st.gy_p = gy_avg; st.gz_p = gz_avg;

  // 3) EMA 平滑
  st.ax_e = ACC_EMA_ALPHA * ax_avg + (1.0f - ACC_EMA_ALPHA) * st.ax_e;
  st.ay_e = ACC_EMA_ALPHA * ay_avg + (1.0f - ACC_EMA_ALPHA) * st.ay_e;
  st.az_e = ACC_EMA_ALPHA * az_avg + (1.0f - ACC_EMA_ALPHA) * st.az_e;
  st.gx_e = GYR_EMA_ALPHA * gx_avg + (1.0f - GYR_EMA_ALPHA) * st.gx_e;
  st.gy_e = GYR_EMA_ALPHA * gy_avg + (1.0f - GYR_EMA_ALPHA) * st.gy_e;
  st.gz_e = GYR_EMA_ALPHA * gz_avg + (1.0f - GYR_EMA_ALPHA) * st.gz_e;

  ax = st.ax_e; ay = st.ay_e; az = st.az_e;
  gx = st.gx_e; gy = st.gy_e; gz = st.gz_e;
  return true;
}

/* 柔性读取：多采样平均 */
static inline int readFlexOversampleAvg(uint8_t pin) {
  long sum = 0;
  for (int i=0;i<FLEX_OVERSAMPLE;i++) {
    sum += analogRead(pin);
    delayMicroseconds(120);
  }
  return (int)(sum / FLEX_OVERSAMPLE);
}

/* ======== 生命周期 ======== */
void setup() {
  Serial.begin(115200);
  delay(200);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ);
  #ifdef ARDUINO_ARCH_ESP32
  Wire.setTimeOut(50); // 防 I2C 卡死
  #endif
  delay(50);

  analogReadResolution(ADC_BITS);
  #ifdef analogSetPinAttenuation
  analogSetPinAttenuation(FLEX1_PIN, ADC_11db);
  analogSetPinAttenuation(FLEX2_PIN, ADC_11db);
  #endif

  // 初始化两块 MPU（0x68 / 0x69）
  bool ok1 = mpuInit(MPU_ADDR_68);
  bool ok2 = mpuInit(MPU_ADDR_69);
  if (!ok1 && !ok2) {
    Serial.println(F("ERR: No MPU6050 found at 0x68 nor 0x69"));
    while(1){ delay(1000); }
  } else {
    if (!ok1) Serial.println(F("WARN: MPU @0x68 not found"));
    if (!ok2) Serial.println(F("WARN: MPU @0x69 not found"));
  }

  // 上电静止零偏标定（可选）
  delay(200);
  if (ok1) mpuCalibrate(MPU_ADDR_68, imu1_bias, 200);
  if (ok2) mpuCalibrate(MPU_ADDR_69, imu2_bias, 200);

  // 柔性 EMA 初始化
  flex1_raw = readFlexOversampleAvg(FLEX1_PIN);
  flex2_raw = readFlexOversampleAvg(FLEX2_PIN);
  flex1_ema = (float)flex1_raw;
  flex2_ema = (float)flex2_raw;

  // 建立各IMU滤波状态（做一次有效读取）
  float ax,ay,az,gx,gy,gz;
  if (ok1 && mpuReadFiltered(MPU_ADDR_68, imu1, ax,ay,az,gx,gy,gz)) { /* ready */ }
  if (ok2 && mpuReadFiltered(MPU_ADDR_69, imu2, ax,ay,az,gx,gy,gz)) { /* ready */ }

  Serial.println(F("# t_ms,flex1_raw,flex1_ema,flex2_raw,flex2_ema,Ax1,Ay1,Az1,Gx1,Gy1,Gz1,Ax2,Ay2,Az2,Gx2,Gy2,Gz2"));

  // Wi-Fi 连接（首次）
  wifiEnsureConnected();

  last_ms = millis();
}

void loop() {
  uint32_t now = millis();
  if ((now - last_ms) < PERIOD_MS) return;
  // 防止阻塞导致节拍偏移
  while ((now - last_ms) >= PERIOD_MS) last_ms += PERIOD_MS;

  // 柔性 ×2
  int f1 = readFlexOversampleAvg(FLEX1_PIN);
  int f2 = readFlexOversampleAvg(FLEX2_PIN);
  flex1_ema = FLEX_EMA_ALPHA * f1 + (1.0f - FLEX_EMA_ALPHA) * flex1_ema;
  flex2_ema = FLEX_EMA_ALPHA * f2 + (1.0f - FLEX_EMA_ALPHA) * flex2_ema;
  flex1_raw = f1; flex2_raw = f2;

  // IMU ×2（若某个不存在，则输出 NaN）
  float ax1=NAN, ay1=NAN, az1=NAN, gx1=NAN, gy1=NAN, gz1=NAN;
  float ax2=NAN, ay2=NAN, az2=NAN, gx2=NAN, gy2=NAN, gz2=NAN;
  bool ok1 = mpuReadFiltered(MPU_ADDR_68, imu1, ax1,ay1,az1,gx1,gy1,gz1);
  bool ok2 = mpuReadFiltered(MPU_ADDR_69, imu2, ax2,ay2,az2,gx2,gy2,gz2);

  // ======== 串口 + UDP 同步输出（CSV 一致） ========
  char line[256];
  int n = 0;

  n += snprintf(line + n, sizeof(line) - n,
                "%lu,%d,%.1f,%d,%.1f,",
                (unsigned long)now, flex1_raw, flex1_ema, flex2_raw, flex2_ema);

  if (ok1) {
    n += snprintf(line + n, sizeof(line) - n,
                  "%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,",
                  ax1, ay1, az1, gx1, gy1, gz1);
  } else {
    n += snprintf(line + n, sizeof(line) - n, "NaN,NaN,NaN,NaN,NaN,NaN,");
  }

  if (ok2) {
    n += snprintf(line + n, sizeof(line) - n,
                  "%.3f,%.3f,%.3f,%.2f,%.2f,%.2f",
                  ax2, ay2, az2, gx2, gy2, gz2);
  } else {
    n += snprintf(line + n, sizeof(line) - n, "NaN,NaN,NaN,NaN,NaN,NaN");
  }

  // 末尾换行
  n += snprintf(line + n, sizeof(line) - n, "\n");

  // 1) 串口原样输出
  Serial.print(line);

  // 2) UDP 同步发送（若 Wi-Fi 已连接）
  wifiEnsureConnected();
  if (WiFi.status() == WL_CONNECTED) {
    udp.beginPacket(DEST_IP, DEST_PORT);
    udp.write((const uint8_t*)line, strlen(line));
    udp.endPacket();
  }
}
