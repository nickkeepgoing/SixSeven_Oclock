# Six Seven O'Clock ⏰

**An alarm clock you can only silence by physically doing the "six seven" (67) gesture — raising your arm up and bringing it back down.**

No snooze. No tapping a button and rolling over. The alarm keeps playing until a wrist-mounted motion sensor confirms you actually moved. The whole point is to force your body awake, not just your thumb.

> Computer Science project · Platform: Arduino + external modules

---

## Why it exists

Every alarm app has the same flaw: silencing it takes zero effort, so you silence it half-asleep and go right back under. Six Seven O'Clock removes the easy exit. The only way to stop the sound is to perform a real, deliberate arm movement that a sensor on your wrist has to recognize — by the time you've done it, you're awake.

## What it does

- 🕐 **Shows the current time** on an OLED display
- ⏰ **Lets you set an alarm** — over USB from a computer, or with on-device buttons
- 🔊 **Plays a real MP3** out loud through a speaker when the alarm fires
- 💪 **Detects the "six seven" gesture** from a wrist-mounted motion sensor to stop the alarm
- 🔋 **Keeps time with no power** — a battery-backed real-time clock means unplugging it won't reset the time

---

## How it works

```
  Normal          Alarm time         Detecting           Silenced
 ┌───────┐  →   ┌──────────┐   →   ┌───────────┐   →   ┌──────────┐
 │ 07:00 │      │ ALARM !!! │       │ reading   │       │ SUCCESS! │
 │ shows │      │ music on  │       │ wrist     │       │ music off│
 │ time  │      │ 🔊        │       │ motion    │       │ → normal │
 └───────┘      └──────────┘       └───────────┘       └──────────┘
```

1. **Normal mode** — the display shows the current time; the system compares it against the set alarm continuously.
2. **Alarm fires** — at the set time, music plays through the speaker and the screen shows an alarm message.
3. **Detection mode** — the wrist sensor is read continuously, watching for the gesture.
4. **Gesture check** — an arm-up peak followed by an arm-down peak, within a time window, counts as a valid "six seven".
5. **Silenced** — on a correct gesture the music stops, a success screen shows, and the clock returns to normal.

The gesture logic reads acceleration from the sensor, measures how far it deviates from gravity, and requires a strong motion peak that settles within a set duration window. Thresholds are tuned to avoid false triggers from small movements.

---

## Hardware

| Component | Role |
|---|---|
| Arduino Uno / Nano | Main controller |
| DS3231 RTC | Real-time clock, keeps time on battery (CR2032) |
| OLED SSD1306 0.96" | Time / status display |
| DFPlayer Mini | Plays MP3 files |
| MicroSD card (1–2GB, FAT32) | Stores the alarm music |
| Speaker 3W 4–8Ω | Audio output |
| MPU6050 (GY-521) | Wrist-mounted motion sensor |
| Push buttons × 3 | Set the time/alarm on the device |

**Also needed:** breadboard + jumper wires, ~60 cm of thin wire to run the sensor to the wrist, a 1kΩ resistor, a USB cable, a wrist strap/small case for the sensor, and an enclosure for the main unit.

Approximate total cost: **600–1,200 THB.**

### Why Arduino + MPU6050

Other platforms were considered and rejected: **micro:bit** (can't play real audio, small screen), **HuskyLens AI camera** (expensive, unreliable in the dim light of early morning), and **ultrasonic sensors** (can only measure distance, can't tell gestures apart). Arduino + MPU6050 won because it gives the finest control over gesture detection, is cheap, and a sensor strapped to the wrist measures real arm movement accurately with no lighting problems.

---

## Wiring

All I2C modules share the SDA/SCL bus (A4/A5):

| Module | Module pins | → Arduino |
|---|---|---|
| DS3231 RTC | VCC / GND / SDA / SCL | 5V / GND / A4 / A5 |
| OLED SSD1306 | VCC / GND / SDA / SCL | 5V / GND / A4 / A5 |
| MPU6050 (wrist) | VCC / GND / SDA / SCL | 5V / GND / A4 / A5 |
| MPU6050 | **AD0** | **3.3V** (changes I2C address) |
| DFPlayer Mini | VCC / GND / TX / RX | 5V / GND / D2 / D3 (RX via 1kΩ) |
| DFPlayer Mini | SPK1 / SPK2 | → Speaker |
| Buttons SET/UP/DOWN | one leg each / other leg | D4, D5, D6 / GND |

### ⚠️ Easy mistakes to avoid

1. **DFPlayer RX must go through a 1kΩ resistor.** The module is 3.3V logic but the Arduino sends 5V — wiring it directly can damage the module.
2. **The MPU6050 defaults to address `0x68`, which collides with the DS3231.** Tie the sensor's `AD0` pin to 3.3V to move it to `0x69` so both can share the I2C bus.
3. **Keep the wrist I2C wires under ~60–70 cm** and solder them well — the wrist moves a lot and loose joints cause dropouts.
4. **The Uno has only one hardware serial port** (used when setting the time over USB), so the DFPlayer uses a `SoftwareSerial` port on D2/D3 to avoid a conflict.

---

## Software

- **Language:** C / C++ via the Arduino IDE.
- **Optional:** a Python (tkinter/pyserial) GUI on the computer for setting the time — not required.

### Libraries (install in the Arduino IDE)

| Library | Used for |
|---|---|
| `RTClib` | Read/set the DS3231 |
| `Adafruit_SSD1306` + `Adafruit_GFX` | Drive the OLED |
| `DFRobotDFPlayerMini` | Control music playback |
| `SoftwareSerial` | Extra serial port for the DFPlayer |
| `Wire` | I2C communication |

### Code layout

The program is organized around four responsibilities:

1. **Clock** — reads the DS3231, draws the time on the OLED, compares against the alarm.
2. **Audio** — tells the DFPlayer to play when the alarm fires and stop when the gesture succeeds.
3. **Gesture detection** — the hardest part: reads the accelerometer continuously and matches the arm-up / arm-down pattern.
4. **Settings** — accepts time/alarm changes over serial and from the buttons, and saves them (the DS3231's backup battery keeps time through power loss).

### Files in this repo

- `alarm_clock_veryeasy67_20260827f.ino` — the full build (RTC + OLED + MPU6050 + DFPlayer + 3 buttons, with alarm/time saved to EEPROM).
- `67Oclock.cpp` — a minimal variant that uses the built-in LED and screen flash instead of the DFPlayer, useful for testing the clock and gesture logic without the audio hardware.

---

## Build it in phases

Wire and test one piece at a time — it's far easier to find a fault than wiring everything at once.

1. RTC + OLED → show the time on screen
2. Add DFPlayer + speaker → play a song
3. Add MPU6050 → read acceleration in the Serial Monitor
4. Write the "six seven" gesture logic
5. Add the setting buttons → combine everything
6. Assemble the case, build the wrist strap, test for real

---

## Status & known risks

- **Gesture detection is the hardest part** and needs the most tuning — budget plenty of time to adjust the acceleration thresholds and time window against real movement.
- Don't wire the DFPlayer RX without the 1kΩ resistor, or you may kill the module.
- Remember the MPU6050 address fix (`AD0 → 3.3V`), or it collides with the RTC.
- Wrist wires can work loose with repeated motion — solder them firmly.
- For louder sound, add an audio amplifier (e.g. PAM8403).

**Difficulty:** more challenging than a typical project because several modules run together — best suited to someone with a few weeks to spare and patience for trial and error.
