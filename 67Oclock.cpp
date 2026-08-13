#include <Wire.h>
#include <RTClib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>

// ----- ตั้งเวลาปลุก (ตั้งชั่วโมงและนาทีที่ต้องการ) -----
const int ALARM_HOUR   = 7;
const int ALARM_MINUTE = 0;

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
RTC_DS3231 rtc;

// เกณฑ์ตรวจจับท่าทาง Six Seven
const float GRAVITY           = 9.8;
const float START_THRESHOLD   = 4.0;
const float QUIET_THRESHOLD   = 2.0;
const unsigned long QUIET_DURATION = 600;
const float PEAK_THRESHOLD    = 15.0;
const unsigned long MIN_DURATION   = 300;
const unsigned long MAX_DURATION   = 15000;

enum GestureState { IDLE, MOVING };
GestureState gestureState = IDLE;
unsigned long moveStartTime = 0;
unsigned long lastBigMotionTime = 0;
float peakDeviation = 0;

enum SystemState { NORMAL, ALARM_ACTIVE, SUCCESS_DISPLAY };
SystemState systemState = NORMAL;

unsigned long successDisplayStart = 0;
const unsigned long SUCCESS_DISPLAY_DURATION = 2000;
bool alarmFiredThisMinute = false;

unsigned long lastBlinkTime = 0;
bool ledOn = false;
const unsigned long BLINK_INTERVAL = 300;

unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_INTERVAL = 1000;

unsigned long lastMpuSample = 0;
const unsigned long MPU_SAMPLE_INTERVAL = 100;

// ==========================================
// ฟังก์ชันจัดการ MPU6050 ผ่าน Wire โดยตรง (No Library)
// ==========================================

bool initMPUDirect() {
  // 1. ปลุก MPU6050 จาก Sleep Mode
  Wire.beginTransmission(0x69);
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0x00); // ตั้งค่าเป็น 0
  if (Wire.endTransmission() != 0) return false;

  // 2. ตั้ง Range ความเร่งเป็น +/-8g (AFS_SEL = 2)
  Wire.beginTransmission(0x69);
  Wire.write(0x1C); // ACCEL_CONFIG register
  Wire.write(0x10); // 8g sensitivity (4096 LSB/g)
  return (Wire.endTransmission() == 0);
}

bool readAccelDirect(float &ax, float &ay, float &az) {
  Wire.beginTransmission(0x69);
  Wire.write(0x3B); // เริ่มอ่านตั้งแต่ ACCEL_XOUT_H (0x3B)
  if (Wire.endTransmission(false) != 0) return false;

  if (Wire.requestFrom(0x69, 6) != 6) return false; // ขออ่านข้อมูล 6 Bytes (X, Y, Z)

  int16_t rawX = (Wire.read() << 8) | Wire.read();
  int16_t rawY = (Wire.read() << 8) | Wire.read();
  int16_t rawZ = (Wire.read() << 8) | Wire.read();

  // แปลงค่า Raw เป็นหน่วย m/s^2 (สเกล 8g = 4096 LSB/g)
  ax = (rawX / 4096.0) * 9.80665;
  ay = (rawY / 4096.0) * 9.80665;
  az = (rawZ / 4096.0) * 9.80665;
  return true;
}

void setup() {
  Serial.begin(9600);
  delay(500);
  Serial.println(F("=== STARTING SYSTEM ==="));

  Wire.begin();
  Wire.setClock(100000); // 100kHz Standard I2C

  #if defined(Wire_h) && defined(TWCR)
  Wire.setWireTimeout(3000, true);
  #endif

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // 1. OLED
  Serial.print(F("1. Initializing OLED... "));
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("FAIL!"));
    while (1);
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  Serial.println(F("OK"));

  // 2. RTC
  Serial.print(F("2. Initializing RTC... "));
  if (!rtc.begin()) {
    Serial.println(F("FAIL!"));
    while (1);
  }
  if (rtc.lostPower()) {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  Serial.println(F("OK"));

  // 3. MPU6050 Direct Init
  Serial.print(F("3. Initializing MPU6050 Direct (0x69)... "));
  if (!initMPUDirect()) {
    Serial.println(F("FAIL! Check AD0 -> 3.3V and Power"));
    while (1);
  }
  Serial.println(F("OK"));

  Serial.println(F("=== SYSTEM READY ==="));
}

void loop() {
  unsigned long now = millis();
  DateTime nowTime = rtc.now();

  switch (systemState) {

    case NORMAL: {
      if (nowTime.hour() == ALARM_HOUR && nowTime.minute() == ALARM_MINUTE) {
        if (!alarmFiredThisMinute) {
          alarmFiredThisMinute = true;
          enterAlarmState();
        }
      } else {
        alarmFiredThisMinute = false;
      }

      if (now - lastDisplayUpdate >= DISPLAY_INTERVAL) {
        lastDisplayUpdate = now;
        drawClock(nowTime);
      }
      break;
    }

    case ALARM_ACTIVE: {
      // ไฟกระพริบเตือน
      if (now - lastBlinkTime >= BLINK_INTERVAL) {
        lastBlinkTime = now;
        ledOn = !ledOn;
        digitalWrite(LED_BUILTIN, ledOn ? HIGH : LOW);
        display.invertDisplay(ledOn);
      }

      // อ่านเซนเซอร์ MPU6050 ตรวจจับท่าทาง
      if (now - lastMpuSample >= MPU_SAMPLE_INTERVAL) {
        lastMpuSample = now;
        if (checkGesture(now)) {
          digitalWrite(LED_BUILTIN, LOW);
          display.invertDisplay(false);
          systemState = SUCCESS_DISPLAY;
          successDisplayStart = now;
          drawSuccess();
        }
      }
      break;
    }

    case SUCCESS_DISPLAY: {
      if (now - successDisplayStart >= SUCCESS_DISPLAY_DURATION) {
        systemState = NORMAL;
        gestureState = IDLE;
        lastDisplayUpdate = 0;
      }
      break;
    }

  }
}

                                                                                                                                                                                                                                                                                                                                                                                                                                                                    void enterAlarmState() {
  systemState = ALARM_ACTIVE;
  gestureState = IDLE;
  peakDeviation = 0;
  lastBlinkTime = millis();
  ledOn = false;

  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(10, 5);
  display.println(F("ALARM !!!"));
  display.setTextSize(1);
  display.setCursor(0, 35);
  display.println(F("Do 'Six Seven' Pose"));
  display.println(F("to Stop Alarm!"));
  display.display();

  Serial.println(F(">> ALARM TRIGGERED!"));
}
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  bool checkGesture(unsigned long now) {
  float ax, ay, az;
  if (!readAccelDirect(ax, ay, az)) return false;

  float magnitude = sqrt(ax * ax + ay * ay + az * az);
  float deviation = fabs(magnitude - GRAVITY);

  if (gestureState == IDLE) {
    if (deviation > START_THRESHOLD) {
      gestureState = MOVING;
      moveStartTime = now;
      lastBigMotionTime = now;
      peakDeviation = deviation;
      Serial.println(F(">> Gesture Started..."));
    }
  } else {
    if (deviation > peakDeviation) peakDeviation = deviation;
    if (deviation > QUIET_THRESHOLD) lastBigMotionTime = now;

    if (now - lastBigMotionTime > QUIET_DURATION) {
      unsigned long duration = now - moveStartTime;

      bool ok = (peakDeviation >= PEAK_THRESHOLD) &&
                (duration >= MIN_DURATION) &&
                (duration <= MAX_DURATION);

      Serial.print(F("Gesture End | Peak: "));
      Serial.print(peakDeviation);
      Serial.print(F(" Duration(ms): "));
      Serial.print(duration);
      Serial.println(ok ? F(" => PASSED!") : F(" => FAILED!"));

      gestureState = IDLE;
      return ok;
    }
  }
  return false;
}
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                              void drawClock(DateTime nowTime) {
  display.clearDisplay();

  display.setTextSize(3);
  display.setCursor(10, 20);
  char timeStr[9];
  sprintf(timeStr, "%02d:%02d:%02d", nowTime.hour(), nowTime.minute(), nowTime.second());
  display.println(timeStr);

  display.setTextSize(1);
  display.setCursor(0, 0);
  char dateStr[12];
  sprintf(dateStr, "%04d-%02d-%02d", nowTime.year(), nowTime.month(), nowTime.day());
  display.println(dateStr);

  display.display();
}

void drawSuccess() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(15, 25);
  display.println(F("SUCCESS!"));
  display.display();
}
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                