// ============================================================
//  Gesture Alarm Clock - Arduino Uno   (v5 - ALL FIXES COMBINED)
//  OLED SSD1306 + DS3231 + MPU6050 + DFPlayer Mini + 3 buttons
//
//  ถ้าไฟล์ .ino ปัจจุบันของคุณ "ไม่มี" บรรทัด
//      const uint8_t BTN_PINS[3] = { BTN_SET, BTN_UP, BTN_DOWN };
//  แปลว่ายังเป็นไฟล์เก่า ให้ลบทิ้งแล้ววางไฟล์นี้แทนทั้งหมด
// ============================================================

#include <Wire.h>
#include <RTClib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>
#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>
#include <avr/wdt.h>
#include <EEPROM.h>

#define DEBUG_SERIAL    1   // 1 = print seconds to Serial Monitor (9600)
#define USE_I2C_TIMEOUT 1   // set 0 if time still glitches

bool initMPUDirect();
void recoverI2CBus();
bool readAccelDirect(float &ax, float &ay, float &az);
void handleButtons(DateTime t);
void enterAlarmState();
void stopAlarmSound();
bool updateGestureProgress();
void drawClock(DateTime nowTime);
void drawAlarmScreen();
void drawSensorErrorScreen();
void drawSuccess();
void drawSettingScreen(DateTime nowTime);
void saveTimeToEEPROM(uint32_t unixTime);
void saveAlarmToEEPROM();
void loadAlarmFromEEPROM();
bool buttonPressed(uint8_t idx, bool allowRepeat);

// clear MCUSR before main() -> prevents watchdog reset loop
uint8_t mcusr_mirror __attribute__((section(".noinit")));
void disableWDTEarly(void) __attribute__((naked, used, section(".init3")));
void disableWDTEarly(void) {
  mcusr_mirror = MCUSR;
  MCUSR = 0;
  wdt_disable();
}

// ค่านี้ใช้เฉพาะตอน EEPROM ยังไม่เคยมีการบันทึกเวลาปลุกเลย (เครื่องใหม่เอี่ยม)
int alarmHour   = 14;
int alarmMinute = 39;

#define BTN_SET  2
#define BTN_UP   3
#define BTN_DOWN 4

const unsigned long DEBOUNCE_DELAY = 30;
const unsigned long REPEAT_DELAY   = 220;

// ----- ปุ่ม 3 ตัว เก็บเป็น array ของตัวแปรพื้นฐาน -----
// index 0 = SET, 1 = UP, 2 = DOWN
// (เลิกใช้ struct เพราะ Arduino auto-prototype generator มีบั๊กกับ
//  ฟังก์ชันที่รับ struct แบบ by-reference ทำให้คอมไพล์ไม่ผ่าน)
const uint8_t BTN_PINS[3] = { BTN_SET, BTN_UP, BTN_DOWN };
bool          btnLastReading[3]  = { HIGH, HIGH, HIGH };
bool          btnStableState[3]  = { HIGH, HIGH, HIGH };
unsigned long btnLastChangeTime[3] = { 0, 0, 0 };
unsigned long btnLastRepeatTime[3] = { 0, 0, 0 };

#define IDX_SET  0
#define IDX_UP   1
#define IDX_DOWN 2

enum SettingMode { SET_NONE, SET_HOUR, SET_MINUTE };
SettingMode settingMode = SET_NONE;

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1

// 100kHz ปลอดภัยกว่าเมื่อมี 3 อุปกรณ์บนบัสเดียวกัน / สายยาว
const uint32_t I2C_CLOCK = 100000UL;

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
RTC_DS3231 rtc;

SoftwareSerial mySoftwareSerial(11, 10); // D11 = RX, D10 = TX
DFRobotDFPlayerMini myDFPlayer;
bool dfPlayerOk = false;

// 0x69 = AD0 tied HIGH (ยืนยันแล้วว่าคุณต่อแบบนี้) | 0x68 = ชนกับ DS3231
#define MPU_ADDR 0x69
bool mpuOk = false;

const float GRAVITY         = 9.80665;
// ----- ปรับให้ทำท่า Six Seven ผ่านง่ายมาก (Very Easy Mode) -----
// ขยับเบาๆ ก็ผ่านได้ ไม่ต้องออกแรง
const float FORCE_THRESHOLD = 6.0;
const float FORCE_MULT      = 0.25;
const float DECAY_RATE      = 0.4;

float gestureProgress = 0.0;

enum SystemState { NORMAL, ALARM_ACTIVE, SUCCESS_DISPLAY };
SystemState systemState = NORMAL;

unsigned long successDisplayStart = 0;
const unsigned long SUCCESS_DISPLAY_DURATION = 2000;   // ปรับความยาวหน้าจอ SUCCESS ที่นี่ (ms)
bool alarmFiredThisMinute = false;

unsigned long lastBlinkTime = 0;
bool ledOn = false;
const unsigned long BLINK_INTERVAL = 300;

// วาดนาฬิกาตอน "วินาทีเปลี่ยนจริง" จาก RTC ไม่ใช่นับ millis() เอง
// (นับ millis() เองจะเลื่อนสะสมจน overhead ของ loop ทำให้ข้ามวินาที)
uint8_t lastDrawnSecond = 255;

unsigned long lastSetupDraw = 0;
const unsigned long SETUP_DRAW_INTERVAL = 200;

unsigned long lastMpuSample = 0;
const unsigned long MPU_SAMPLE_INTERVAL = 100;

unsigned long lastAlarmDraw = 0;
const unsigned long ALARM_DRAW_INTERVAL = 200;

int  mpuFailStreak = 0;
const int MPU_FAIL_ESCAPE = 100;   // 100 x 100ms = 10 วินาที
bool sensorEscapeMode = false;

// ============================================================
//  Debounce แบบยืนยันสถานะหลังนิ่ง (robust debounce)
//  ทุกครั้งที่ค่าดิบเปลี่ยน จะรีเซ็ตนาฬิกาจับเวลาเสมอ ไม่ทิ้งการเปลี่ยนแปลงไปเงียบๆ
//  แบบโค้ดต้นฉบับที่ทำให้ปุ่มค้าง
// ============================================================
bool buttonPressed(uint8_t idx, bool allowRepeat) {
  unsigned long now = millis();
  bool reading = digitalRead(BTN_PINS[idx]);

  if (reading != btnLastReading[idx]) {
    btnLastChangeTime[idx] = now;
    btnLastReading[idx] = reading;
  }

  bool pressedEdge = false;

  if ((now - btnLastChangeTime[idx]) > DEBOUNCE_DELAY) {
    if (reading != btnStableState[idx]) {
      btnStableState[idx] = reading;
      if (btnStableState[idx] == LOW) {
        pressedEdge = true;
        btnLastRepeatTime[idx] = now;
      }
    } else if (allowRepeat && btnStableState[idx] == LOW &&
               (now - btnLastRepeatTime[idx]) >= REPEAT_DELAY) {
      btnLastRepeatTime[idx] = now;
      pressedEdge = true;
    }
  }

  return pressedEdge;
}

// ============================================================
int i2cFailCount = 0;
const int I2C_FAIL_LIMIT = 5;

bool initMPUDirect() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);              // ปลุกออกจาก sleep
  if (Wire.endTransmission() != 0) return false;

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x10);              // +/-8g -> 4096 LSB/g
  return (Wire.endTransmission() == 0);
}

void recoverI2CBus() {
  Wire.end();
  delay(50);
  Wire.begin();
  Wire.setClock(I2C_CLOCK);
#if defined(ARDUINO_ARCH_AVR) && USE_I2C_TIMEOUT
  Wire.setWireTimeout(3000, true);
#endif
  mpuOk = initMPUDirect();
  i2cFailCount = 0;
}

bool readAccelDirect(float &ax, float &ay, float &az) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) {
    if (++i2cFailCount >= I2C_FAIL_LIMIT) recoverI2CBus();
    return false;
  }

  if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)6) != 6) {
    if (++i2cFailCount >= I2C_FAIL_LIMIT) recoverI2CBus();
    return false;
  }

  // ต้องอ่านทีละไบต์ใส่ตัวแปรก่อน
  // (Wire.read() << 8) | Wire.read() ลำดับการเรียกไม่แน่นอน high/low byte อาจสลับกัน
  uint8_t b[6];
  for (uint8_t i = 0; i < 6; i++) b[i] = Wire.read();

  int16_t rawX = ((int16_t)b[0] << 8) | b[1];
  int16_t rawY = ((int16_t)b[2] << 8) | b[3];
  int16_t rawZ = ((int16_t)b[4] << 8) | b[5];

  ax = (rawX / 4096.0) * GRAVITY;
  ay = (rawY / 4096.0) * GRAVITY;
  az = (rawZ / 4096.0) * GRAVITY;

  i2cFailCount = 0;
  return true;
}

// ============================================================
//  EEPROM layout:
//  0-4   : เวลาปัจจุบันสำรอง (slot 0) -> uint32 + marker
//  5-9   : เวลาปัจจุบันสำรอง (slot 1) -> uint32 + marker
//  10-12 : เวลาปลุกที่ตั้งไว้ -> hour(1) + minute(1) + marker(1)
// ============================================================
#define EEPROM_MARKER      0xA5
#define EEPROM_SLOT0_ADDR  0
#define EEPROM_SLOT1_ADDR  5
#define EEPROM_ALARM_ADDR  10

unsigned long lastEepromSave = 0;
// ลดจาก 300000UL (5 นาที) เหลือ 30 วินาที เพื่อลดผลกระทบถ้าเวลาต้องย้อนกลับ
// ตอนบอร์ดรีเซ็ตกะทันหัน (ปัญหานี้มักมาจากแบตสำรอง CR2032 ของ RTC อ่อน/หมด)
const unsigned long EEPROM_SAVE_INTERVAL = 30000UL;

bool readEEPROMSlot(int addr, uint32_t &outTime) {
  if (EEPROM.read(addr + 4) != EEPROM_MARKER) return false;
  EEPROM.get(addr, outTime);
  return true;
}

uint32_t getLastKnownTime() {
  uint32_t t0 = 0, t1 = 0;
  bool v0 = readEEPROMSlot(EEPROM_SLOT0_ADDR, t0);
  bool v1 = readEEPROMSlot(EEPROM_SLOT1_ADDR, t1);
  if (v0 && v1) return (t0 > t1) ? t0 : t1;
  if (v0) return t0;
  if (v1) return t1;
  return 0;
}

void saveTimeToEEPROM(uint32_t unixTime) {
  uint32_t t0 = 0, t1 = 0;
  bool v0 = readEEPROMSlot(EEPROM_SLOT0_ADDR, t0);
  bool v1 = readEEPROMSlot(EEPROM_SLOT1_ADDR, t1);

  int targetAddr;
  if (!v0)      targetAddr = EEPROM_SLOT0_ADDR;
  else if (!v1) targetAddr = EEPROM_SLOT1_ADDR;
  else          targetAddr = (t0 <= t1) ? EEPROM_SLOT0_ADDR : EEPROM_SLOT1_ADDR;

  EEPROM.put(targetAddr, unixTime);
  EEPROM.write(targetAddr + 4, EEPROM_MARKER);
}

// บันทึกเวลาปลุกที่ตั้งไว้ล่าสุด (เรียกทุกครั้งที่กด SET ออกจากโหมดตั้งเวลา)
void saveAlarmToEEPROM() {
  EEPROM.update(EEPROM_ALARM_ADDR,     (uint8_t)alarmHour);
  EEPROM.update(EEPROM_ALARM_ADDR + 1, (uint8_t)alarmMinute);
  EEPROM.update(EEPROM_ALARM_ADDR + 2, EEPROM_MARKER);
}

// โหลดเวลาปลุกกลับมาตอนบูต ถ้าไม่เคยบันทึกไว้เลยจะใช้ค่าเริ่มต้นในโค้ดแทน
void loadAlarmFromEEPROM() {
  if (EEPROM.read(EEPROM_ALARM_ADDR + 2) != EEPROM_MARKER) return; // ไม่เคยบันทึก

  uint8_t h = EEPROM.read(EEPROM_ALARM_ADDR);
  uint8_t m = EEPROM.read(EEPROM_ALARM_ADDR + 1);

  if (h < 24 && m < 60) {   // กันค่าขยะกรณี EEPROM เสีย
    alarmHour   = h;
    alarmMinute = m;
  }
}

// ============================================================
void handleButtons(DateTime t) {
  if (buttonPressed(IDX_SET, false)) {
    if (settingMode == SET_NONE) {
      settingMode = SET_HOUR;
      lastSetupDraw = 0;
    } else if (settingMode == SET_HOUR) {
      settingMode = SET_MINUTE;
      lastSetupDraw = 0;
    } else {
      settingMode = SET_NONE;
      alarmFiredThisMinute = false;
      lastDrawnSecond = 255;
      saveAlarmToEEPROM();
      if (t.year() >= 2020) saveTimeToEEPROM(t.unixtime());
    }
  }

  if (settingMode == SET_NONE) return;

  if (buttonPressed(IDX_UP, true)) {
    if (settingMode == SET_HOUR) alarmHour   = (alarmHour + 1) % 24;
    else                         alarmMinute = (alarmMinute + 1) % 60;
    lastSetupDraw = 0;
  }

  if (buttonPressed(IDX_DOWN, true)) {
    if (settingMode == SET_HOUR) alarmHour   = (alarmHour - 1 + 24) % 24;
    else                         alarmMinute = (alarmMinute - 1 + 60) % 60;
    lastSetupDraw = 0;
  }
}

// ============================================================
void setup() {
  wdt_disable();
  wdt_enable(WDTO_8S);

#if DEBUG_SERIAL
  Serial.begin(9600);
  delay(300);
#endif
  wdt_reset();

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(BTN_SET,  INPUT_PULLUP);
  pinMode(BTN_UP,   INPUT_PULLUP);
  pinMode(BTN_DOWN, INPUT_PULLUP);

  loadAlarmFromEEPROM();

  Wire.begin();

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    while (1) {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      delay(100);
    }
  }

  // set clock AFTER display.begin() - the library overrides it
  Wire.setClock(I2C_CLOCK);
#if defined(ARDUINO_ARCH_AVR) && USE_I2C_TIMEOUT
  Wire.setWireTimeout(3000, true);
#endif

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  wdt_reset();

  if (!rtc.begin()) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 24);
    display.println(F("RTC DS3231 NOT FOUND"));
    display.println(F("check SDA/SCL wiring"));
    display.display();
    while (1);
  }
  wdt_reset();

  // ----- ตรวจสอบสถานะแบตสำรอง (CR2032) ของ RTC -----
  // ถ้าค่านี้เป็น true ทุกครั้งที่บูตแม้เพิ่งเสียบไฟไปไม่นาน แปลว่าแบตสำรองน่าจะ
  // อ่อนหรือหมด ทำให้ RTC จำเวลาข้ามการรีเซ็ต/ไฟดับไม่ได้ เวลาจะถูกดึงกลับไปเป็น
  // ค่าที่เซฟไว้ล่าสุดใน EEPROM ทุกครั้งที่บอร์ดรีสตาร์ท (อาการ "เวลาย้อนกลับ")
  bool rtcLostPower = rtc.lostPower();
#if DEBUG_SERIAL
  Serial.print(F("RTC lostPower = "));
  Serial.println(rtcLostPower ? F("TRUE  <-- check CR2032 battery!") : F("false (ok)"));
#endif

  if (rtcLostPower) {
    DateTime compileTime = DateTime(F(__DATE__), F(__TIME__));
    uint32_t lastKnown = getLastKnownTime();
    if (lastKnown > compileTime.unixtime()) rtc.adjust(DateTime(lastKnown));
    else                                    rtc.adjust(compileTime);

    // แสดงคำเตือนบนจอสักครู่ ให้เห็นชัดว่าเวลาที่ตั้งไว้ถูกดึงกลับไปใช้ค่าสำรอง
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 16);
    display.println(F("RTC lost power!"));
    display.println(F("Check CR2032"));
    display.println(F("battery on RTC"));
    display.println(F("board."));
    display.display();
    delay(2000);
    wdt_reset();
  }

  mpuOk = initMPUDirect();
  wdt_reset();

  mySoftwareSerial.begin(9600);
  wdt_reset();
  dfPlayerOk = myDFPlayer.begin(mySoftwareSerial, true, true);
  wdt_reset();
  if (dfPlayerOk) myDFPlayer.volume(30);   // ดังสุด (0-30 คือช่วงของ DFPlayer Mini)

  if (!mpuOk || !dfPlayerOk) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setCursor(0, 20);
    if (!mpuOk)      display.println(F("MPU6050 not found"));
    if (!dfPlayerOk) display.println(F("DFPlayer no response"));
    display.println(F("running anyway..."));
    display.display();
    delay(1500);
    wdt_reset();
  }

  wdt_enable(WDTO_4S);
}

// ============================================================
void loop() {
  wdt_reset();

  unsigned long now = millis();
  DateTime nowTime = rtc.now();
  bool timeValid = (nowTime.year() >= 2020 && nowTime.year() <= 2099);

#if DEBUG_SERIAL
  {
    static uint8_t prevSec = 255;
    if (nowTime.second() != prevSec) {
      prevSec = nowTime.second();
      Serial.println(prevSec);
    }
  }
#endif

  if (timeValid && (now - lastEepromSave >= EEPROM_SAVE_INTERVAL)) {
    lastEepromSave = now;
    saveTimeToEEPROM(nowTime.unixtime());
  }

  if (systemState == NORMAL) handleButtons(nowTime);

  switch (systemState) {

    case NORMAL: {
      if (settingMode == SET_NONE && timeValid) {
        if (nowTime.hour() == alarmHour && nowTime.minute() == alarmMinute) {
          if (!alarmFiredThisMinute) {
            alarmFiredThisMinute = true;
            enterAlarmState();
          }
        } else {
          alarmFiredThisMinute = false;
        }
      }

      if (settingMode != SET_NONE) {
        if (now - lastSetupDraw >= SETUP_DRAW_INTERVAL) {
          lastSetupDraw = now;
          drawSettingScreen(nowTime);
        }
      } else if (timeValid && nowTime.second() != lastDrawnSecond) {
        lastDrawnSecond = nowTime.second();
        drawClock(nowTime);
      }
      break;
    }

    case ALARM_ACTIVE: {
      if (now - lastBlinkTime >= BLINK_INTERVAL) {
        lastBlinkTime = now;
        ledOn = !ledOn;
        digitalWrite(LED_BUILTIN, ledOn ? HIGH : LOW);
      }

      if (sensorEscapeMode && buttonPressed(IDX_SET, false)) {
        digitalWrite(LED_BUILTIN, LOW);
        stopAlarmSound();
        sensorEscapeMode = false;
        mpuFailStreak = 0;
        systemState = SUCCESS_DISPLAY;
        successDisplayStart = now;
        drawSuccess();
        break;
      }

      bool completed = false;
      if (now - lastMpuSample >= MPU_SAMPLE_INTERVAL) {
        lastMpuSample = now;
        completed = updateGestureProgress();
      }

      if (completed) {
        digitalWrite(LED_BUILTIN, LOW);
        stopAlarmSound();
        systemState = SUCCESS_DISPLAY;
        successDisplayStart = now;
        lastDrawnSecond = 255;
        drawSuccess();
        break;
      }

      if (now - lastAlarmDraw >= ALARM_DRAW_INTERVAL) {
        lastAlarmDraw = now;
        if (sensorEscapeMode) drawSensorErrorScreen();
        else                  drawAlarmScreen();
      }
      break;
    }

    case SUCCESS_DISPLAY: {
      if (now - successDisplayStart >= SUCCESS_DISPLAY_DURATION) {
        systemState = NORMAL;
        lastDrawnSecond = 255;
      }
      break;
    }
  }
}

// ============================================================
void enterAlarmState() {
  systemState = ALARM_ACTIVE;
  gestureProgress = 0.0;
  lastBlinkTime = millis();
  lastAlarmDraw = 0;
  ledOn = false;
  mpuFailStreak = 0;
  sensorEscapeMode = false;

  // play(1) = เล่นจบแล้วเงียบ ปลุกไม่ตื่น -> loop(1) เล่นวนจนกว่าจะหยุด
  if (dfPlayerOk) myDFPlayer.loop(1);
}

void stopAlarmSound() {
  if (!dfPlayerOk) return;
  myDFPlayer.disableLoop();
  myDFPlayer.stop();
}

bool updateGestureProgress() {
  float ax, ay, az;

  if (!readAccelDirect(ax, ay, az)) {
    if (!mpuOk) mpuOk = initMPUDirect();
    if (++mpuFailStreak >= MPU_FAIL_ESCAPE) sensorEscapeMode = true;
    return false;
  }
  mpuFailStreak = 0;
  mpuOk = true;

  float magnitude = sqrt(ax * ax + ay * ay + az * az);

  // ----- ตัวกรองค่าขยะ (sanity clamp) -----
  // ระหว่างที่ DFPlayer เล่นเสียงผ่าน SoftwareSerial การส่งคำสั่งจะปิด interrupt
  // ชั่วคราว ทำให้บางครั้งอ่านค่า I2C จากเซ็นเซอร์ผิดเพี้ยนได้ ถ้าค่าที่ได้เกิน
  // ขอบเขตที่เป็นไปได้จริงจากการขยับมือ (เกิน ~6g) ให้ทิ้งค่ารอบนี้ไปเลย
  // ไม่นับเป็นทั้งแรงกระแทกและไม่นับเป็นการอ่านล้มเหลว (ไม่กระทบ mpuFailStreak)
  const float MAX_PLAUSIBLE_ACCEL = GRAVITY * 6.0;
  if (magnitude > MAX_PLAUSIBLE_ACCEL || isnan(magnitude)) {
    return (gestureProgress >= 100.0);   // ข้ามรอบนี้ ใช้ค่า progress เดิม
  }

  float deviation = fabs(magnitude - GRAVITY);

  if (deviation > FORCE_THRESHOLD) {
    gestureProgress += (deviation - FORCE_THRESHOLD) * FORCE_MULT;
    if (gestureProgress > 100.0) gestureProgress = 100.0;
  } else {
    gestureProgress -= DECAY_RATE;
    if (gestureProgress < 0.0) gestureProgress = 0.0;
  }

  return (gestureProgress >= 100.0);
}

// ============================================================
void drawAlarmScreen() {
  display.clearDisplay();

  display.setTextSize(2);
  display.setCursor(12, 0);
  display.println(F("ALARM !!!"));

  display.setTextSize(1);
  display.setCursor(8, 20);
  display.println(F("Do 'Six Seven' Pose"));

  display.drawRect(10, 34, 108, 14, SSD1306_WHITE);
  int fillWidth = map((int)gestureProgress, 0, 100, 0, 104);
  if (fillWidth > 0) display.fillRect(12, 36, fillWidth, 10, SSD1306_WHITE);

  display.setCursor(50, 52);
  display.print((int)gestureProgress);
  display.println(F("%"));

  display.display();
}

void drawSensorErrorScreen() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(12, 0);
  display.println(F("ALARM !!!"));

  display.setTextSize(1);
  display.setCursor(0, 24);
  display.println(F("Sensor not responding"));
  display.setCursor(0, 40);
  display.println(F("Press SET to stop"));
  display.display();
}

void drawClock(DateTime nowTime) {
  display.clearDisplay();

  display.setTextSize(1);
  display.setCursor(0, 0);
  char dateStr[12];
  sprintf(dateStr, "%04d-%02d-%02d", nowTime.year(), nowTime.month(), nowTime.day());
  display.print(dateStr);

  display.setCursor(86, 0);
  char alarmStr[8];
  sprintf(alarmStr, "A%02d:%02d", alarmHour, alarmMinute);
  display.print(alarmStr);

  // setTextSize(3) กับ "HH:MM:SS" = 144px ล้นจอ 128px -> HH:MM ใหญ่ + SS เล็ก
  display.setTextSize(3);
  display.setCursor(19, 24);
  char hm[6];
  sprintf(hm, "%02d:%02d", nowTime.hour(), nowTime.minute());
  display.print(hm);

  display.setTextSize(1);
  display.setCursor(112, 40);
  char ss[3];
  sprintf(ss, "%02d", nowTime.second());
  display.print(ss);

  display.display();
}

void drawSuccess() {
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(15, 25);
  display.println(F("SUCCESS!"));
  display.display();
}

void drawSettingScreen(DateTime nowTime) {
  display.clearDisplay();

  display.setTextSize(1);
  display.setCursor(0, 0);
  char nowStr[9];
  sprintf(nowStr, "%02d:%02d:%02d", nowTime.hour(), nowTime.minute(), nowTime.second());
  display.print(F("Now "));
  display.print(nowStr);

  display.setCursor(19, 10);
  display.print(F("-- SET ALARM --"));

  display.setTextSize(3);
  char buf[3];

  sprintf(buf, "%02d", alarmHour);
  display.setCursor(19, 24);
  display.print(buf);

  display.setCursor(55, 24);
  display.print(F(":"));

  sprintf(buf, "%02d", alarmMinute);
  display.setCursor(76, 24);
  display.print(buf);

  if ((millis() / 400) % 2 == 0) {
    if (settingMode == SET_HOUR)        display.drawLine(19, 50, 55, 50, SSD1306_WHITE);
    else if (settingMode == SET_MINUTE) display.drawLine(76, 50, 112, 50, SSD1306_WHITE);
  }

  display.setTextSize(1);
  display.setCursor(0, 56);
  display.print(F("SET=next UP/DN=change"));

  display.display();
}
