/*
  =====================================================================
   GaugeTach X27 v0.2  -  X27-series gauge stepper, direct drive
  ---------------------------------------------------------------------
   Board  : Seeed XIAO ESP32-C5  (Arduino IDE, esp32 core 3.3.x,
            Tools > Board > esp32 > XIAO_ESP32C5, USB CDC On Boot: Enabled)
   Motor  : X27.168-style gauge stepper (stop modified for ~345 deg),
            driven straight from 4 GPIO pins. No driver chip.
   Home   : drives the needle into the mechanical stop (like a factory cluster)
   Input  : ignition pulses (coil -) through an optocoupler on D10

   Smooth motion: instead of the usual 6-state "partial step" drive, each
   motor pin gets a 25 kHz PWM signal following a sine pattern, updated
   2000 times a second. The needle position is continuous, so there are
   no visible steps at all (true sine microstepping, done in firmware).

   No extra libraries needed. Serial monitor 115200, "Newline". Type "help".
  =====================================================================
*/

#include <Arduino.h>
#include <Preferences.h>
#include <math.h>
#include "esp_timer.h"
#include "driver/gpio.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <Update.h>
#if defined(CONFIG_BLUEDROID_ENABLED)
#include <BLE2902.h>          // NimBLE (ESP32-C5) adds this descriptor automatically
#endif

#define FW_VERSION "0.6.0"
// Releases: bump FW_VERSION, Sketch > Export Compiled Binary, publish the .bin
// and version.json in the GaugeTachFirmware repo (see its README).
// Tools > Partition Scheme must stay "8M with spiffs (3MB APP/1.5MB SPIFFS)":
// it has the two app slots OTA needs.

// ------------------------------------------------------------------
//  Pins  (GPIO numbers; XIAO labels in comments)
//  X27 pins 1..4 in order, same convention as the SwitecX25 library.
// ------------------------------------------------------------------
// Set to false to drive the X27 straight from the GPIO pins (3.3 V) again.
constexpr bool USE_TB6612 = true;

//                                   direct drive     |  TB6612FNG
constexpr uint8_t PIN_M1   = 1;   // D0  -> X27 pin 1 |  AIN1
constexpr uint8_t PIN_M2   = 0;   // D1  -> X27 pin 2 |  AIN2
constexpr uint8_t PIN_M3   = 11;  // D6  -> X27 pin 3 |  BIN1
constexpr uint8_t PIN_M4   = 12;  // D7  -> X27 pin 4 |  BIN2
// TB6612: PWMA, PWMB, STBY and VCC to 3.3 V; VM to 5 V; GND common.
//         AO1 -> X27 pin 1, AO2 -> pin 2, BO1 -> pin 3, BO2 -> pin 4.
constexpr uint8_t PIN_TACH = 10;  // D10 <- optocoupler output (falling edge = spark)
constexpr uint8_t PIN_IGN  = 9;   // D9  <- ignition sense: 12 V via 2.2k / 560 divider (HIGH = key on)
// Free: D4/D5 (I2C), D8, D9.  D2/D3 are strapping pins - leave unused.

// ------------------------------------------------------------------
//  Fixed constants
// ------------------------------------------------------------------
constexpr uint32_t PWM_FREQ        = 25000;   // above hearing
constexpr uint8_t  PWM_BITS        = 10;
constexpr uint32_t PWM_MAX         = (1u << PWM_BITS) - 1;
constexpr uint32_t MOTION_PERIOD_US = 500;    // 2 kHz motion / coil update
constexpr float    DEG_PER_ECYCLE  = 2.0f;    // X27: 6 partial steps of 1/3 deg = one electrical cycle
constexpr uint32_t CFG_MAGIC       = 0x47545832;  // "GTX2"
constexpr uint16_t CFG_VERSION     = 2;      // bumped: new safer speed defaults
constexpr uint8_t  CAL_MAX         = 8;

// ------------------------------------------------------------------
//  Configuration  (what the BLE app will edit later)
// ------------------------------------------------------------------
struct Config {
  uint32_t magic;
  uint16_t version;

  // Gauge face
  float    sweepDeg;          // needle travel from 0 to full scale
  uint8_t  calN;
  float    calVal[CAL_MAX];   // input value (rpm), ascending
  float    calDeg[CAL_MAX];   // needle angle at that value

  // Input
  float    pulsesPerRev;      // 4-stroke: cylinders / 2  (V8 = 4)
  float    maxInputRpm;       // faster pulses are ignored (bounce/ringing)
  float    filterMs;          // reading smoothing

  // Needle motion
  float    maxSpeedDps;       // deg/s   (X27 tops out around 400-600)
  float    accelDps2;         // deg/s^2
  float    deadbandDeg;
  uint8_t  drivePct;          // coil drive strength, 100 = full 3.3 V
  uint8_t  invertDir;
  uint8_t  ignSense;          // 1 = park needle at 0 when ignition power drops (D9)
                              // (sits in padding, so saved configs still load; old ones read 0 = off)

  // Homing against the mechanical stop
  float    zeroOffsetDeg;     // the 0 mark is this far ABOVE the stop
  float    homeSpeedDps;
  float    overtravelDeg;     // drive this far past full sweep to be sure we hit the stop
  uint8_t  wakeSweep;
};

Config cfg;
Config savedCfg;                    // what's in flash (for the app's "unsaved" flag)
Preferences prefs;
volatile uint32_t cfgRev   = 1;     // bumps on every settings change; app re-reads on change
volatile bool     cfgUnsaved = false;

void setDefaults() {
  memset(&cfg, 0, sizeof(cfg));
  cfg.magic         = CFG_MAGIC;
  cfg.version       = CFG_VERSION;
  cfg.sweepDeg      = 345.0f;
  cfg.calN          = 2;
  cfg.calVal[0] = 0;     cfg.calDeg[0] = 0;
  cfg.calVal[1] = 7000;  cfg.calDeg[1] = 345.0f;
  cfg.pulsesPerRev  = 4.0f;      // V8, 4-stroke
  cfg.maxInputRpm   = 9000.0f;
  cfg.filterMs      = 60.0f;
  cfg.maxSpeedDps   = 250.0f;     // safe at 3.3 V; raise once proven
  cfg.accelDps2     = 800.0f;
  cfg.deadbandDeg   = 0.15f;
  cfg.drivePct      = 100;
  cfg.invertDir     = 0;
  cfg.zeroOffsetDeg = 1.0f;
  cfg.homeSpeedDps  = 120.0f;
  cfg.overtravelDeg = 15.0f;
  cfg.wakeSweep     = 1;
}

void loadConfig() {
  setDefaults();
  prefs.begin("gaugex27", true);
  Config tmp;
  size_t n = prefs.getBytes("cfg", &tmp, sizeof(tmp));
  prefs.end();
  if (n == sizeof(tmp) && tmp.magic == CFG_MAGIC && tmp.version == CFG_VERSION) {
    memcpy(&cfg, &tmp, sizeof(cfg));
    Serial.println(F("Config loaded from flash."));
  } else {
    Serial.println(F("No saved config, using defaults."));
  }
  memcpy(&savedCfg, &cfg, sizeof(cfg));
}

// ------------------------------------------------------------------
//  Coil drive: sine microstepping
//
//  The X27 partial-step table (SwitecX25: 0x9,0x1,0x7,0x6,0xE,0x8) always
//  drives pins 2 and 3 identically, so electrically it's a 3-terminal,
//  3-phase motor: pin1 at 60 deg, pins 2+3 at 180 deg, pin4 at 300 deg.
//  We drive those three as smooth sine waves (with min/max centering so
//  the coils see the full 3.3 V), which reproduces the table exactly at
//  each partial step and interpolates smoothly in between.
// ------------------------------------------------------------------
constexpr int COS_N = 512;
float cosTab[COS_N];

void buildCosTable() {
  for (int i = 0; i < COS_N; i++) cosTab[i] = cosf(2.0f * (float)M_PI * i / COS_N);
}

// cos of an angle given in cycles (1.0 = 360 deg)
static inline float cosCycles(float x) {
  x -= floorf(x);
  float f = x * COS_N;
  int i = (int)f;
  float t = f - i;
  float a = cosTab[i & (COS_N - 1)];
  float b = cosTab[(i + 1) & (COS_N - 1)];
  return a + (b - a) * t;
}

bool coilsEnabled = false;

void coilsOff() {
  ledcWrite(PIN_M1, 0); ledcWrite(PIN_M2, 0);
  ledcWrite(PIN_M3, 0); ledcWrite(PIN_M4, 0);
}

// TB6612 H-bridge, one coil. v in -1..+1.
// PWMx is tied high, so: IN1=H IN2=L drive forward, IN1=L IN2=H reverse,
// IN1=H IN2=H short brake. We hold one input high and PWM the other with
// the inverted duty, so the off-time is "brake" (slow decay). That keeps
// coil current proportional to duty, which is what smooth microstepping needs.
static inline void writeBridge(uint8_t in1, uint8_t in2, float v) {
  uint32_t mag = (uint32_t)lroundf(fminf(fabsf(v), 1.0f) * PWM_MAX);
  uint32_t low = PWM_MAX - mag;                 // duty of the "brake" input
  if (v >= 0) { ledcWrite(in1, PWM_MAX); ledcWrite(in2, low); }
  else        { ledcWrite(in1, low);     ledcWrite(in2, PWM_MAX); }
}

// elecDeg: needle angle in the motor's own coordinate (degrees)
void writeCoils(float elecDeg) {
  if (!coilsEnabled) { coilsOff(); return; }
  float cyc = elecDeg / DEG_PER_ECYCLE;
  if (cfg.invertDir) cyc = -cyc;

  if (USE_TB6612) {
    // Coil A = pins 1-2, coil B = pins 3-4. The X27 step table puts the two
    // coil voltages 120 deg apart; these sines reproduce it exactly at each
    // partial step and move smoothly in between, at the full 5 V.
    float k = cfg.drivePct / 100.0f;
    writeBridge(PIN_M1, PIN_M2, k * cosCycles(cyc - 1.0f / 12.0f));
    writeBridge(PIN_M3, PIN_M4, k * cosCycles(cyc - 5.0f / 12.0f));
    return;
  }

  float a1 = cosCycles(cyc - 1.0f / 6.0f);   // pin 1        (60 deg)
  float a2 = cosCycles(cyc - 0.5f);          // pins 2 and 3 (180 deg)
  float a4 = cosCycles(cyc - 5.0f / 6.0f);   // pin 4        (300 deg)
  float mx = fmaxf(a1, fmaxf(a2, a4));
  float mn = fminf(a1, fminf(a2, a4));
  float off = -(mx + mn) * 0.5f;
  float k = 0.57735027f * (cfg.drivePct / 100.0f);   // 1/sqrt(3): full rail between pins
  uint32_t d1 = (uint32_t)lroundf((0.5f + (a1 + off) * k) * PWM_MAX);
  uint32_t d2 = (uint32_t)lroundf((0.5f + (a2 + off) * k) * PWM_MAX);
  uint32_t d4 = (uint32_t)lroundf((0.5f + (a4 + off) * k) * PWM_MAX);
  ledcWrite(PIN_M1, d1);
  ledcWrite(PIN_M2, d2);
  ledcWrite(PIN_M3, d2);
  ledcWrite(PIN_M4, d4);
}

// ------------------------------------------------------------------
//  Motion planner (runs every 0.5 ms in a high-priority timer task)
//  Position is continuous (degrees, float). Accel/vel limited, lands on
//  the stopping curve v = sqrt(2 a d) so there's no overshoot.
// ------------------------------------------------------------------
enum class MoveMode : uint8_t { Position, Velocity };

portMUX_TYPE motionMux = portMUX_INITIALIZER_UNLOCKED;

// Commands (written by loop, read by the tick)
volatile MoveMode mMode   = MoveMode::Position;
volatile float    mTarget = 0;      // deg
volatile float    mCmdVel = 0;      // deg/s, Velocity mode
volatile float    mVmax   = 400;
volatile float    mAccel  = 1500;

// State (written by the tick, read by loop)
volatile float    mPos    = 0;      // deg, gauge coordinate (0 = zero mark)
volatile float    mVel    = 0;      // deg/s
volatile float    mElecOffset = 0;  // motor coordinate = mPos + mElecOffset

esp_timer_handle_t motionTimer = nullptr;

void motionTick(void*) {
  const float dt = MOTION_PERIOD_US * 1e-6f;

  portENTER_CRITICAL(&motionMux);
  MoveMode mode = mMode;
  float target = mTarget, cmdVel = mCmdVel, vmax = mVmax, acc = mAccel;
  float pos = mPos, vel = mVel, eoff = mElecOffset;
  portEXIT_CRITICAL(&motionMux);

  float vDes;
  if (mode == MoveMode::Position) {
    float err = target - pos;
    float snapV = fmaxf(3.0f, sqrtf(2.0f * acc * 0.02f));
    if (fabsf(err) < 0.02f && fabsf(vel) < snapV) {
      pos = target; vel = 0; vDes = 0;
    } else {
      float vStop = sqrtf(2.0f * acc * fabsf(err));
      vDes = copysignf(fminf(vmax, vStop), err);
    }
  } else {
    vDes = fmaxf(-vmax, fminf(vmax, cmdVel));
  }

  float dv = vDes - vel;
  float dvMax = acc * dt;
  bool braking = (vel > 0 && dv < 0 && vDes >= 0) || (vel < 0 && dv > 0 && vDes <= 0);
  if (braking) dvMax *= 1.5f;
  if (dv >  dvMax) dv =  dvMax;
  if (dv < -dvMax) dv = -dvMax;
  vel += dv;
  if (mode == MoveMode::Velocity && vDes == 0 && fabsf(vel) < 0.5f) vel = 0;
  pos += vel * dt;

  portENTER_CRITICAL(&motionMux);
  // If loop shifted the coordinates during this tick, apply the shift too.
  float shift = mElecOffset - eoff;
  mPos = pos - shift;
  mVel = vel;
  eoff = mElecOffset;
  portEXIT_CRITICAL(&motionMux);

  writeCoils(pos - shift + eoff);
}

void setLimits(float dps, float dps2) {
  portENTER_CRITICAL(&motionMux);
  mVmax = dps; mAccel = dps2;
  portEXIT_CRITICAL(&motionMux);
}
void useGaugeLimits() { setLimits(cfg.maxSpeedDps, cfg.accelDps2); }

void setTarget(float deg) {
  portENTER_CRITICAL(&motionMux);
  mMode = MoveMode::Position; mTarget = deg;
  portEXIT_CRITICAL(&motionMux);
}

void setVelocity(float dps) {
  portENTER_CRITICAL(&motionMux);
  mMode = MoveMode::Velocity; mCmdVel = dps;
  portEXIT_CRITICAL(&motionMux);
}

// Relabel the current position as newPos without moving the needle.
void relabelPosition(float newPos) {
  portENTER_CRITICAL(&motionMux);
  float delta = newPos - mPos;
  mPos += delta;
  mTarget += delta;
  mElecOffset -= delta;      // keeps the coil phase exactly where it was
  portEXIT_CRITICAL(&motionMux);
}

bool settled() {
  return mMode == MoveMode::Position && mVel == 0 && mPos == mTarget;
}

bool moveToBlocking(float deg, uint32_t timeoutMs = 8000) {
  setTarget(deg);
  uint32_t t0 = millis();
  while (!settled()) {
    if (millis() - t0 > timeoutMs) return false;
    delay(1);
  }
  return true;
}

void stopBlocking() {
  setVelocity(0);
  uint32_t t0 = millis();
  while (mVel != 0 && millis() - t0 < 2000) delay(1);
  setTarget(mPos);
}

// ------------------------------------------------------------------
//  Homing against the mechanical stop
// ------------------------------------------------------------------
volatile bool homed = false;
volatile bool busy  = false;        // homing / sweeping (shown in the app)
volatile bool ignOn  = true;        // debounced ignition sense (D9)
volatile bool parked = false;       // ignition off: needle sent to 0, input ignored

// Drive downscale at a constant speed for a set distance (needle may sit on the stop part of it).
void driveDown(float dps, float distDeg) {
  float start = mPos;
  setVelocity(-dps);
  while (start - mPos < distDeg) delay(1);
  stopBlocking();
}

bool homeNeedle() {
  homed = false;
  busy = true;
  Serial.println(F("Homing against the stop..."));
  // Gentle acceleration so the motor never loses sync on the way down.
  setLimits(cfg.homeSpeedDps, fminf(cfg.accelDps2, 600.0f));

  // 1. Drive downscale further than the whole dial, so wherever the needle
  //    started it ends up on the stop.
  driveDown(cfg.homeSpeedDps, cfg.sweepDeg + cfg.overtravelDeg + cfg.zeroOffsetDeg);
  delay(150);

  // 2. Back off, then 3. re-seat slowly. A slow, short final approach means
  //    the needle meets the stop softly and the motor only slips briefly,
  //    so it settles in the same place every time.
  setLimits(60.0f, 600.0f);
  float p = mPos;
  moveToBlocking(p + 10.0f);
  driveDown(30.0f, 14.0f);
  delay(200);                                    // let the needle settle on the stop

  // The stop is zeroOffsetDeg below the 0 mark.
  relabelPosition(-cfg.zeroOffsetDeg);
  useGaugeLimits();
  moveToBlocking(0);
  homed = true;
  busy = false;
  Serial.println(F("Homed."));
  return true;
}

// ------------------------------------------------------------------
//  Tach input
// ------------------------------------------------------------------
portMUX_TYPE tachMux = portMUX_INITIALIZER_UNLOCKED;
volatile uint32_t t_lastUs   = 0;
volatile uint32_t t_per[8];
volatile uint8_t  t_idx      = 0;
volatile uint8_t  t_count    = 0;
volatile uint32_t t_minPerUs = 1000;
constexpr uint32_t TACH_TIMEOUT_US = 400000;   // no pulse for 0.4 s = engine stopped

void ARDUINO_ISR_ATTR onTachEdge() {
  uint32_t now = micros();
  uint32_t p = now - t_lastUs;
  if (p < t_minPerUs) return;                  // points bounce / coil ringing
  portENTER_CRITICAL_ISR(&tachMux);
  t_lastUs = now;
  if (p < TACH_TIMEOUT_US) {
    t_per[t_idx] = p;
    t_idx = (t_idx + 1) & 7;
    if (t_count < 8) t_count++;
  }
  portEXIT_CRITICAL_ISR(&tachMux);
}

void updateTachLockout() {
  t_minPerUs = (uint32_t)(60.0e6f / (cfg.maxInputRpm * cfg.pulsesPerRev));
}

// Average over one crank revolution so cylinder-to-cylinder and dwell
// variation doesn't make the needle shiver.
float measureRpm() {
  uint32_t per[8], last;
  uint8_t idx, cnt;
  portENTER_CRITICAL(&tachMux);
  for (int i = 0; i < 8; i++) per[i] = t_per[i];
  idx = t_idx; cnt = t_count; last = t_lastUs;
  portEXIT_CRITICAL(&tachMux);

  uint32_t age = micros() - last;
  if (cnt == 0) return 0;
  if (age > TACH_TIMEOUT_US) {
    portENTER_CRITICAL(&tachMux);
    t_count = 0;
    portEXIT_CRITICAL(&tachMux);
    return 0;
  }
  int k = constrain((int)lroundf(cfg.pulsesPerRev), 1, 8);
  if (k > cnt) k = cnt;
  uint32_t sum = 0;
  for (int i = 1; i <= k; i++) sum += per[(idx - i) & 7];
  float avg = sum / (float)k;
  float period = fmaxf(avg, (float)age);       // engine slowing: let the needle fall
  return 60.0e6f / (period * cfg.pulsesPerRev);
}

// ------------------------------------------------------------------
//  Gauge mapping and update
// ------------------------------------------------------------------
float valueToDeg(float v) {
  uint8_t n = cfg.calN;
  if (n < 2) return 0;
  if (v <= cfg.calVal[0]) return cfg.calDeg[0];
  for (uint8_t i = 1; i < n; i++) {
    if (v <= cfg.calVal[i]) {
      float f = (v - cfg.calVal[i - 1]) / (cfg.calVal[i] - cfg.calVal[i - 1]);
      return cfg.calDeg[i - 1] + f * (cfg.calDeg[i] - cfg.calDeg[i - 1]);
    }
  }
  return cfg.calDeg[n - 1];
}

volatile float simRpm     = -1;      // >= 0 overrides the real input
volatile bool  manualHold = false;   // goto/jog: needle ignores input
volatile float rpmFilt    = 0;
volatile float rpmRaw     = 0;

void gaugeUpdate() {
  static uint32_t lastMs = 0;
  uint32_t now = millis();
  uint32_t el = now - lastMs;
  if (el < 5) return;
  lastMs = now;
  float dt = el / 1000.0f;

  rpmRaw = (simRpm >= 0) ? simRpm : measureRpm();
  float a = dt / (cfg.filterMs / 1000.0f + dt);
  rpmFilt += a * (rpmRaw - rpmFilt);
  if (rpmRaw == 0 && rpmFilt < 5) rpmFilt = 0;

  if (!homed || manualHold || parked) return;
  float deg = constrain(valueToDeg(rpmFilt), 0.0f, cfg.sweepDeg);
  if (mMode != MoveMode::Position || fabsf(deg - mTarget) >= cfg.deadbandDeg ||
      (deg == 0 && mTarget != 0)) setTarget(deg);
}

// ------------------------------------------------------------------
//  Apply / save config
// ------------------------------------------------------------------
void applyConfig() {
  useGaugeLimits();
  updateTachLockout();
}

void saveConfig() {
  prefs.begin("gaugex27", false);
  prefs.putBytes("cfg", &cfg, sizeof(cfg));
  prefs.end();
  memcpy(&savedCfg, &cfg, sizeof(cfg));
  cfgUnsaved = false;
  Serial.println(F("Saved."));
}

void wakeSweep() {
  moveToBlocking(cfg.sweepDeg);
  delay(150);
  moveToBlocking(0);
}

// ------------------------------------------------------------------
//  Serial console (the BLE app will drive the same settings later)
// ------------------------------------------------------------------
void printHelp() {
  Serial.println(F(
    "\nCommands:\n"
    "  show                  print settings\n"
    "  mon                   toggle live readout\n"
    "  rpm <n>               simulate input (e.g. rpm 3000)\n"
    "  live                  back to real input\n"
    "  sweep [n]             sweep 0 -> full -> 0 (n times; step-loss test)\n"
    "  home                  re-zero against the stop\n"
    "  goto <deg>            hold needle at angle\n"
    "  jog <deg>             nudge needle (+/-)\n"
    "  setzero               needle is on the 0 mark now -> store offset from stop\n"
    "  motor off | motor on  de-energize / re-energize coils\n"
    "  set <key> <value>     keys: sweep max ppr maxrpm filter speed accel deadband\n"
    "                              drive invert zerooffset homespeed overtravel wake\n"
    "                              ign (1 = park needle when ignition turns off)\n"
    "  cal clear | cal add <rpm> <deg> | cal show\n"
    "  save | defaults\n"));
}

void printConfig() {
  Serial.printf("\nsweep %.1f deg | ppr %.2f | maxrpm %.0f | filter %.0f ms\n",
                cfg.sweepDeg, cfg.pulsesPerRev, cfg.maxInputRpm, cfg.filterMs);
  Serial.printf("speed %.0f deg/s | accel %.0f deg/s2 | deadband %.2f deg | drive %u%% | invert %u\n",
                cfg.maxSpeedDps, cfg.accelDps2, cfg.deadbandDeg, cfg.drivePct, cfg.invertDir);
  Serial.printf("zerooffset %.2f deg | homespeed %.0f | overtravel %.0f | wake %u | homed %u\n",
                cfg.zeroOffsetDeg, cfg.homeSpeedDps, cfg.overtravelDeg, cfg.wakeSweep, homed);
  Serial.print(F("cal:"));
  for (uint8_t i = 0; i < cfg.calN; i++) Serial.printf(" [%.0f rpm -> %.1f deg]", cfg.calVal[i], cfg.calDeg[i]);
  Serial.println();
}

bool monitorOn = false;

void handleSet(const char* key, float v) {
  bool rehome = false;
  if      (!strcmp(key, "sweep")) {
    float old = cfg.sweepDeg;
    cfg.sweepDeg = constrain(v, 10.0f, 359.0f);
    if (cfg.calN == 2 && cfg.calDeg[1] == old) cfg.calDeg[1] = cfg.sweepDeg;
  }
  else if (!strcmp(key, "max"))       { cfg.calN = 2; cfg.calVal[0] = 0; cfg.calDeg[0] = 0;
                                        cfg.calVal[1] = v; cfg.calDeg[1] = cfg.sweepDeg; }
  else if (!strcmp(key, "ppr"))       cfg.pulsesPerRev  = constrain(v, 0.25f, 16.0f);
  else if (!strcmp(key, "maxrpm"))    cfg.maxInputRpm   = constrain(v, 1000.0f, 20000.0f);
  else if (!strcmp(key, "filter"))    cfg.filterMs      = constrain(v, 0.0f, 2000.0f);
  else if (!strcmp(key, "speed"))     cfg.maxSpeedDps   = constrain(v, 10.0f, 1000.0f);
  else if (!strcmp(key, "accel"))     cfg.accelDps2     = constrain(v, 50.0f, 20000.0f);
  else if (!strcmp(key, "deadband"))  cfg.deadbandDeg   = constrain(v, 0.0f, 5.0f);
  else if (!strcmp(key, "drive"))     cfg.drivePct      = (uint8_t)constrain(v, 10.0f, 100.0f);
  else if (!strcmp(key, "invert"))    { cfg.invertDir = v != 0; rehome = true; }
  else if (!strcmp(key, "zerooffset")) { cfg.zeroOffsetDeg = v; rehome = true; }
  else if (!strcmp(key, "homespeed")) cfg.homeSpeedDps  = constrain(v, 20.0f, 600.0f);
  else if (!strcmp(key, "overtravel")) cfg.overtravelDeg = constrain(v, 0.0f, 60.0f);
  else if (!strcmp(key, "wake"))      cfg.wakeSweep = v != 0;
  else if (!strcmp(key, "ign"))       cfg.ignSense = v != 0;
  else { Serial.println(F("Unknown key.")); return; }

  applyConfig();
  if (rehome) homeNeedle();
  Serial.println(F("OK (type 'save' to keep)."));
}

void handleLine(char* line) {
  char* cmd = strtok(line, " \t");
  if (!cmd) return;
  char* a1 = strtok(nullptr, " \t");
  char* a2 = strtok(nullptr, " \t");
  char* a3 = strtok(nullptr, " \t");

  if      (!strcmp(cmd, "help") || !strcmp(cmd, "?")) printHelp();
  else if (!strcmp(cmd, "show"))     printConfig();
  else if (!strcmp(cmd, "mon"))      monitorOn = !monitorOn;
  else if (!strcmp(cmd, "rpm") && a1) { simRpm = fmaxf(0.0f, atof(a1)); manualHold = false; }
  else if (!strcmp(cmd, "live"))     { simRpm = -1; manualHold = false; }
  else if (!strcmp(cmd, "sweep"))    {
    // "sweep 10" repeats it: if the needle isn't back on the 0 mark
    // afterwards, the motor is losing steps at the current speed/accel.
    int n = a1 ? constrain(atoi(a1), 1, 50) : 1;
    manualHold = true; busy = true;
    for (int i = 0; i < n; i++) wakeSweep();
    manualHold = false; busy = false;
    Serial.printf("Did %d sweep(s). Needle should be exactly on 0.\n", n);
  }
  else if (!strcmp(cmd, "home"))     homeNeedle();
  else if (!strcmp(cmd, "goto") && a1) { manualHold = true; setTarget(atof(a1)); }
  else if (!strcmp(cmd, "jog") && a1)  { manualHold = true; setTarget(mTarget + atof(a1)); }
  else if (!strcmp(cmd, "setzero")) {
    if (!homed) { Serial.println(F("Home first.")); return; }
    moveToBlocking(mTarget);
    float p = mPos;
    cfg.zeroOffsetDeg += p;
    relabelPosition(0);
    Serial.printf("Zero set. Offset from stop is now %.2f deg (type 'save').\n", cfg.zeroOffsetDeg);
  }
  else if (!strcmp(cmd, "motor") && a1) {
    if (!strcmp(a1, "off")) { coilsEnabled = false; homed = false;
                              Serial.println(F("Coils off. 'motor on' re-homes.")); }
    else                    { coilsEnabled = true; homeNeedle(); }
  }
  else if (!strcmp(cmd, "set") && a1 && a2) handleSet(a1, atof(a2));
  else if (!strcmp(cmd, "cal") && a1) {
    if (!strcmp(a1, "clear")) { cfg.calN = 0; Serial.println(F("Cal cleared; add at least 2 points.")); }
    else if (!strcmp(a1, "add") && a2 && a3 && cfg.calN < CAL_MAX) {
      float v = atof(a2), d = atof(a3);
      uint8_t i = cfg.calN;                    // insert sorted by value
      while (i > 0 && cfg.calVal[i - 1] > v) { cfg.calVal[i] = cfg.calVal[i - 1]; cfg.calDeg[i] = cfg.calDeg[i - 1]; i--; }
      cfg.calVal[i] = v; cfg.calDeg[i] = d; cfg.calN++;
      printConfig();
    }
    else printConfig();
  }
  else if (!strcmp(cmd, "save"))     saveConfig();
  else if (!strcmp(cmd, "defaults")) { setDefaults(); applyConfig(); homeNeedle(); Serial.println(F("Defaults loaded (not saved).")); }
  else Serial.println(F("? (type help)"));
}

void pollSerial() {
  void processCommand(char* line);
  static char buf[96];
  static uint8_t len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') { buf[len] = 0; processCommand(buf); len = 0; }
    else if (len < sizeof(buf) - 1) buf[len++] = c;
  }
}

void printMonitor() {
  static uint32_t lastMs = 0;
  if (!monitorOn || millis() - lastMs < 250) return;
  lastMs = millis();
  Serial.printf("%s rpm %5.0f (raw %5.0f)  needle %6.2f deg  target %6.2f  v %6.0f deg/s\n",
                simRpm >= 0 ? "SIM " : "LIVE", rpmFilt, rpmRaw,
                (float)mPos, (float)mTarget, (float)mVel);
}

// ------------------------------------------------------------------
//  Bluetooth LE  (the setup app)
//
//  Service 6a1f0001-...
//    config  6a1f0002  READ    JSON of every setting (+ "rev")
//    command 6a1f0003  WRITE   the same text commands as the serial console,
//                              one per line ("set speed 300\nsave")
//    status  6a1f0004  NOTIFY  live JSON 10x/s: rpm, needle, homed, rev, ota...
//    otactl  6a1f0005  WRITE   OTA control: "begin <bytes>", "end", "abort"
//    otadata 6a1f0006  WRITE   OTA firmware chunks (raw bytes, in order)
//
//  Commands from the app are queued and run in loop(), exactly as if they
//  were typed in the Serial Monitor, so there's only one code path.
// ------------------------------------------------------------------
#define GAUGE_SVC_UUID    "6a1f0001-3c2d-4b8e-9f10-7e5a2c9d4b01"
#define GAUGE_CFG_UUID    "6a1f0002-3c2d-4b8e-9f10-7e5a2c9d4b01"
#define GAUGE_CMD_UUID    "6a1f0003-3c2d-4b8e-9f10-7e5a2c9d4b01"
#define GAUGE_STATUS_UUID "6a1f0004-3c2d-4b8e-9f10-7e5a2c9d4b01"
#define GAUGE_OTACTL_UUID "6a1f0005-3c2d-4b8e-9f10-7e5a2c9d4b01"
#define GAUGE_OTADAT_UUID "6a1f0006-3c2d-4b8e-9f10-7e5a2c9d4b01"

constexpr size_t BLE_CMD_LEN = 100;
QueueHandle_t      bleCmdQ    = nullptr;
BLECharacteristic* cfgChar    = nullptr;
BLECharacteristic* statusChar = nullptr;
volatile bool      bleConnected = false;
char               bleName[24];

// ---- OTA (firmware update from the app) ----
// 0 idle, 1 receiving, 2 done (rebooting), 3 failed
volatile uint8_t  otaState    = 0;
volatile uint32_t otaSize     = 0;
volatile uint32_t otaRecv     = 0;
volatile uint32_t otaRebootAt = 0;
char              otaErr[48]  = "";

void otaFail(const char* why) {
  if (Update.isRunning()) Update.abort();
  strncpy(otaErr, why, sizeof(otaErr) - 1);
  otaErr[sizeof(otaErr) - 1] = 0;
  otaState = 3;
  manualHold = false;
}

void otaAbortIfRunning() {
  if (otaState == 1) { otaFail("connection lost"); }
}

// Runs in the BLE task. The write is acknowledged only after this returns,
// so the app can't send data faster than flash can take it.
class OtaCtlCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    String v = c->getValue();
    v.trim();
    if (v.startsWith("begin ")) {
      if (Update.isRunning()) Update.abort();
      uint32_t size = (uint32_t)strtoul(v.c_str() + 6, nullptr, 10);
      otaErr[0] = 0;
      otaRecv = 0;
      otaSize = size;
      if (size < 1024) { otaFail("bad size"); return; }
      if (!Update.begin(size)) { otaFail(Update.errorString()); return; }
      otaState = 1;
      manualHold = true;                         // park the needle at zero while updating
      setTarget(0);
    } else if (v == "end") {
      if (otaState != 1) return;
      if (otaRecv != otaSize) { otaFail("incomplete"); return; }
      if (!Update.end(false)) { otaFail(Update.errorString()); return; }  // verifies the image
      otaState = 2;
      otaRebootAt = millis() + 1500;            // give the app time to see "done"
    } else if (v == "abort") {
      if (Update.isRunning()) Update.abort();
      otaState = 0;
      manualHold = false;
    }
  }
};

class OtaDataCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    if (otaState != 1) return;
    size_t len = c->getLength();
    if (len == 0) return;
    if (otaRecv + len > otaSize) { otaFail("too much data"); return; }
    if (Update.write(c->getData(), len) != len) { otaFail(Update.errorString()); return; }
    otaRecv = otaRecv + len;
  }
};

class GaugeServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer*) override    { bleConnected = true; }
  void onDisconnect(BLEServer*) override {
    bleConnected = false;
    otaAbortIfRunning();
    BLEDevice::startAdvertising();
  }
};

class GaugeCmdCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) override {
    String v = c->getValue();
    int start = 0;
    while (start < (int)v.length()) {             // split on newlines
      int end = v.indexOf('\n', start);
      if (end < 0) end = v.length();
      String line = v.substring(start, end);
      line.trim();
      if (line.length() && line.length() < BLE_CMD_LEN) {
        char buf[BLE_CMD_LEN];
        strncpy(buf, line.c_str(), BLE_CMD_LEN - 1);
        buf[BLE_CMD_LEN - 1] = 0;
        xQueueSend(bleCmdQ, buf, 0);               // drop if full; never block the BLE stack
      }
      start = end + 1;
    }
  }
};

void publishConfig() {
  if (!cfgChar) return;
  static char b[640];
  int n = snprintf(b, sizeof(b),
    "{\"fw\":\"%s\",\"type\":\"tach\",\"driver\":\"%s\",\"rev\":%lu,"
    "\"sweep\":%.1f,\"ppr\":%.2f,\"maxrpm\":%.0f,\"filter\":%.0f,"
    "\"speed\":%.0f,\"accel\":%.0f,\"deadband\":%.2f,\"drive\":%u,\"invert\":%u,"
    "\"zerooffset\":%.2f,\"homespeed\":%.0f,\"overtravel\":%.0f,\"wake\":%u,\"ign\":%u,\"cal\":[",
    FW_VERSION, USE_TB6612 ? "tb6612" : "gpio", (unsigned long)cfgRev,
    cfg.sweepDeg, cfg.pulsesPerRev, cfg.maxInputRpm, cfg.filterMs,
    cfg.maxSpeedDps, cfg.accelDps2, cfg.deadbandDeg, cfg.drivePct, cfg.invertDir,
    cfg.zeroOffsetDeg, cfg.homeSpeedDps, cfg.overtravelDeg, cfg.wakeSweep, cfg.ignSense);
  for (uint8_t i = 0; i < cfg.calN && n < (int)sizeof(b) - 40; i++)
    n += snprintf(b + n, sizeof(b) - n, "%s[%.0f,%.2f]", i ? "," : "", cfg.calVal[i], cfg.calDeg[i]);
  n += snprintf(b + n, sizeof(b) - n, "]}");
  cfgChar->setValue((uint8_t*)b, n);
}

// While parked the coils may be off, so anything that moves the needle would
// lose track of where it is. Those commands wait until the ignition is back on.
bool isMotionCommand(const char* line) {
  static const char* const motion[] = { "home", "sweep", "goto", "jog", "setzero", "motor", "rpm", "defaults" };
  for (const char* m : motion) {
    size_t n = strlen(m);
    if (strncmp(line, m, n) == 0 && (line[n] == 0 || line[n] == ' ')) return true;
  }
  return strncmp(line, "set invert", 10) == 0 || strncmp(line, "set zerooffset", 14) == 0;
}

// Every command (serial or BLE) goes through here, so the app always
// sees settings changes no matter where they came from.
void processCommand(char* line) {
  if (parked && isMotionCommand(line)) {
    Serial.println(F("Ignition is off - needle parked. Turn the key on first."));
    return;
  }
  static Config before;
  memcpy(&before, &cfg, sizeof(cfg));
  handleLine(line);
  if (memcmp(&before, &cfg, sizeof(cfg)) != 0) {
    cfgRev = cfgRev + 1;
    publishConfig();
  }
  cfgUnsaved = memcmp(&cfg, &savedCfg, sizeof(cfg)) != 0;
}

void bleStatusTask(void*) {
  char b[300];
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(100));
    if (!bleConnected || !statusChar) continue;
    unsigned otaPct = otaSize ? (unsigned)((uint64_t)otaRecv * 100 / otaSize) : 0;
    int n = snprintf(b, sizeof(b),
      "{\"rpm\":%.0f,\"raw\":%.0f,\"needle\":%.2f,\"target\":%.2f,\"homed\":%u,\"busy\":%u,"
      "\"hold\":%u,\"sim\":%d,\"unsaved\":%u,\"rev\":%lu,\"ota\":%u,\"otapct\":%u,\"otaerr\":\"%s\","
      "\"ignon\":%u,\"parked\":%u}",
      (float)rpmFilt, (float)rpmRaw, (float)mPos, (float)mTarget, homed ? 1 : 0, busy ? 1 : 0,
      manualHold ? 1 : 0, simRpm >= 0 ? (int)simRpm : -1, cfgUnsaved ? 1 : 0, (unsigned long)cfgRev,
      (unsigned)otaState, otaPct, otaErr, ignOn ? 1 : 0, parked ? 1 : 0);
    statusChar->setValue((uint8_t*)b, n);
    statusChar->notify();
  }
}

void bleBegin() {
  bleCmdQ = xQueueCreate(16, BLE_CMD_LEN);
  uint64_t mac = ESP.getEfuseMac();
  snprintf(bleName, sizeof(bleName), "GaugeTach-%04X", (unsigned)((mac >> 32) & 0xFFFF));

  BLEDevice::init(bleName);
  BLEDevice::setMTU(517);
  BLEServer* srv = BLEDevice::createServer();
  srv->setCallbacks(new GaugeServerCB());
  BLEService* svc = srv->createService(GAUGE_SVC_UUID);

  cfgChar = svc->createCharacteristic(GAUGE_CFG_UUID, BLECharacteristic::PROPERTY_READ);
  BLECharacteristic* cmdChar = svc->createCharacteristic(GAUGE_CMD_UUID, BLECharacteristic::PROPERTY_WRITE);
  cmdChar->setCallbacks(new GaugeCmdCB());
  statusChar = svc->createCharacteristic(GAUGE_STATUS_UUID,
                 BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
#if defined(CONFIG_BLUEDROID_ENABLED)
  statusChar->addDescriptor(new BLE2902());
#endif
  svc->createCharacteristic(GAUGE_OTACTL_UUID, BLECharacteristic::PROPERTY_WRITE)
     ->setCallbacks(new OtaCtlCB());
  svc->createCharacteristic(GAUGE_OTADAT_UUID, BLECharacteristic::PROPERTY_WRITE)
     ->setCallbacks(new OtaDataCB());
  publishConfig();
  svc->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(GAUGE_SVC_UUID);
  adv->setScanResponse(true);                    // name goes in the scan response
  BLEDevice::startAdvertising();

  xTaskCreate(bleStatusTask, "bleStatus", 4096, nullptr, 1, nullptr);
  Serial.printf("BLE advertising as %s\n", bleName);
}

void pollBle() {
  char buf[BLE_CMD_LEN];
  while (bleCmdQ && xQueueReceive(bleCmdQ, buf, 0) == pdTRUE) {
    if (strncmp(buf, "rpm ", 4) != 0) Serial.printf("[app] %s\n", buf);   // slider spam stays quiet
    processCommand(buf);
  }
}

// ------------------------------------------------------------------
//  Ignition sense: park the needle when the key is turned off
//
//  D9 sees ~2.4-2.9 V with the key on (12-14.4 V through 2.2k / 560) and
//  0 V with it off. The supercap keeps the gauge alive for a few seconds,
//  long enough to drive the needle home and switch the coils off.
//
//  To avoid parking by mistake (battery sag while cranking can pull D9
//  below the logic threshold), it parks only when D9 has been LOW for
//  100 ms AND no spark has arrived for 100 ms. With the key off there's no
//  spark, so this costs nothing; with the engine running it can't trigger.
// ------------------------------------------------------------------
constexpr uint32_t IGN_OFF_MS      = 100;   // D9 low this long = key off
constexpr uint32_t IGN_ON_MS       = 300;   // D9 high this long = key back on
constexpr uint32_t IGN_QUIET_US    = 100000;// no spark for 100 ms
constexpr uint32_t PARK_TIMEOUT_MS = 4000;  // coils off by now, no matter what

uint32_t parkStartMs = 0;
bool     parkCoilsOff = false;

void parkNeedle() {
  parked = true;
  parkCoilsOff = false;
  parkStartMs = millis();
  simRpm = -1;
  manualHold = false;
  setTarget(0);                                  // at the normal speed / accel (no lost steps)
  Serial.println(F("Ignition off: parking needle."));
}

void unparkNeedle() {
  bool lostTrack = parkCoilsOff && !homed;
  coilsEnabled = true;                           // same coil phase as when switched off,
  parked = false;                                // so the needle doesn't move
  parkCoilsOff = false;
  Serial.println(F("Ignition on."));
  if (lostTrack) homeNeedle();
}

void ignitionUpdate() {
  static uint32_t lastHighMs = 0, lastLowMs = 0;
  uint32_t now = millis();
  if (digitalRead(PIN_IGN) == HIGH) lastHighMs = now; else lastLowMs = now;
  if (ignOn  && now - lastHighMs >= IGN_OFF_MS) ignOn = false;
  if (!ignOn && now - lastLowMs  >= IGN_ON_MS)  ignOn = true;

  if (!cfg.ignSense) {                           // feature switched off in the app
    if (parked) unparkNeedle();
    return;
  }

  if (!parked) {
    bool sparkQuiet = (uint32_t)(micros() - t_lastUs) > IGN_QUIET_US;
    if (!ignOn && sparkQuiet && homed && !busy && otaState == 0) parkNeedle();
    return;
  }

  // Parked
  if (ignOn) { unparkNeedle(); return; }
  if (!parkCoilsOff) {
    bool atZero = settled() && mPos == 0;
    if (atZero || millis() - parkStartMs > PARK_TIMEOUT_MS) {
      if (!atZero) homed = false;                // didn't make it: re-home next key-on
      coilsEnabled = false;                      // save the supercap
      parkCoilsOff = true;
      Serial.println(atZero ? F("Needle parked, coils off.") : F("Park timed out, coils off."));
    }
  }
}

// ------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
#if ARDUINO_USB_MODE && ARDUINO_USB_CDC_ON_BOOT
  // If USB is plugged in but nothing is reading the port, the core waits up
  // to 2 s per print before giving up. Drop the text instead, so printing
  // can never stall the gauge.
  Serial.setTxTimeoutMs(0);
#endif
  delay(300);
  Serial.println(F("\nGaugeTach X27 v" FW_VERSION));
  Serial.println(USE_TB6612 ? F("Drive: TB6612FNG (5 V)") : F("Drive: direct GPIO (3.3 V)"));

  loadConfig();
  applyConfig();
  buildCosTable();

  const uint8_t mpins[4] = { PIN_M1, PIN_M2, PIN_M3, PIN_M4 };
  for (uint8_t p : mpins) {
    if (!ledcAttach(p, PWM_FREQ, PWM_BITS)) Serial.printf("PWM attach failed on GPIO%u\n", p);
    gpio_set_drive_capability((gpio_num_t)p, GPIO_DRIVE_CAP_3);   // strongest output
    ledcWrite(p, 0);
  }

  pinMode(PIN_TACH, INPUT_PULLUP);
  pinMode(PIN_IGN, INPUT_PULLDOWN);              // unwired = "ignition off"
  attachInterrupt(digitalPinToInterrupt(PIN_TACH), onTachEdge, FALLING);

  coilsEnabled = true;
  esp_timer_create_args_t targs = {};
  targs.callback = &motionTick;
  targs.name = "motion";
  esp_timer_create(&targs, &motionTimer);
  esp_timer_start_periodic(motionTimer, MOTION_PERIOD_US);

  bleBegin();                                    // app can connect while it homes
  homeNeedle();
  if (cfg.wakeSweep) wakeSweep();
  printHelp();
}

void loop() {
  gaugeUpdate();
  pollSerial();
  pollBle();
  ignitionUpdate();
  printMonitor();
  if (otaState == 2 && (int32_t)(millis() - otaRebootAt) >= 0) {
    Serial.println(F("Firmware updated, restarting."));
    delay(100);
    ESP.restart();
  }
  delay(1);
}
