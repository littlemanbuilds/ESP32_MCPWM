# ESP32_MCPWM

Arduino-friendly dual H-bridge motor driver for ESP32 using the **MCPWM** peripheral (not LEDC).  
Smooth, predictable control with configurable **freewheel modes**, **hard/soft braking**, optional **center-aligned PWM**, and ISR fallbacks for **fault** and **capture**.

- ✅ Arduino Library Manager ready  
- ✅ Beginner alias: `Motor` → `HBridgeMotor`  
- ✅ Works with **BTS7960 / IBT-2** and **DRV8871** (and similar)  
- ✅ FreeRTOS-friendly, ISR-safe design  
- ✅ MIT Licensed  

---

## Install

**Arduino IDE (recommended):** search **ESP32_MCPWM** in Library Manager.  
**Manual:** download the release ZIP → *Sketch → Include Library → Add .ZIP Library…*  
**PlatformIO:** add to `lib_deps` or install from ZIP.

---

## Quick Start

```cpp
#include <ESP32_MCPWM.h>  // Brings IMotorDriver/HBridgeMotor and alias: using Motor = HBridgeMotor;

// DRV8871 example (EN unused = -1)
MotorMCPWMConfig cfg{
  2,              // LPWM pin
  1,              // RPWM pin
  -1,             // EN pin (-1 if unused)
  MCPWM_UNIT_0,   // MCPWM unit
  MCPWM_TIMER_0,  // MCPWM timer
  MCPWM0A,        // LPWM signal route
  MCPWM0B         // RPWM signal route
};

Motor motor;

void setup() {
  Serial.begin(115200);
  motor.setup(cfg);   // If EN is unused (`-1`), the driver defaults to **HiZ_Awake** so outputs remain stable.
}

void loop() {
  motor.setSpeed(512, Dir::CW);   // Half speed CW
  delay(750);
  motor.setSpeed(0,   Dir::CW);   // Stop → freewheel/soft-brake per behavior
  delay(750);

  motor.setSpeed(512, Dir::CCW);  // Half speed CCW
  delay(750);
  motor.setSpeed(0,   Dir::CW);   // Stop
  delay(750);
}
```

---

## Features

- **Concrete driver:** `HBridgeMotor` on ESP32 MCPWM (two PWM outputs A/B).  
- **Unified interface:** `IMotorDriver` (clean, testable API).  
- **Freewheel modes:** `HiZ`, `HiZ_Awake`, `DitherBrake`.  
- **Soft-brake dithering:** periodic **Brake ↔ Coast/Hi-Z** at `soft_brake_hz`.  
- **Hard brake:** dynamic short (A=100%, B=100%) for fast stops.  
- **Center-aligned option:** set `counter = MCPWM_UP_DOWN_COUNTER`.  
- **Runtime frequency retune:** `reconfigureFrequency(new_hz)`.  
- **Safety fallback:** fault input ISR (oneshot or level-follow).  
- **Capture fallback:** edge-timed period measurement callback.  
- **Performance niceties:** duty caching, small critical section, wrap-safe time math.  

---

## API Overview

### Types

- `Motor` — beginner alias of `HBridgeMotor`  
- `HBridgeMotor` — concrete driver  
- `IMotorDriver` — abstract base class  
- `MotorMCPWMConfig` — MCPWM pins/timer/signal/dead-time/counter setup  
- `MotorBehaviorConfig` — freewheel + soft-brake tuning (see below)  
- `MotorSafetyConfig` — optional fault input config  
- `MotorCaptureConfig` — optional capture input config  
- `Dir` — `CW` or `CCW`  
- `FreewheelMode` — `HiZ`, `HiZ_Awake`, `DitherBrake`
- `CaptureCallback` — ISR callback for period measurements
- `FaultCallback` — ISR callback for fault events

### Methods (common surface)

```cpp
// Setup
void setup(const MotorMCPWMConfig& hw);
void setup(const MotorMCPWMConfig& hw, const MotorBehaviorConfig& beh);
void setup(const MotorMCPWMConfig& hw, const MotorBehaviorConfig& beh,
           const MotorSafetyConfig& sfty, const MotorCaptureConfig& cap);

// Control
void setSpeed(int speed, Dir dir) noexcept;   // 0 => stop per behavior
void setSpeedPercent(float percent, Dir dir) noexcept; // 0..100
void setFreewheel() noexcept;                 // Apply current freewheel mode
void setHardBrake() noexcept;                 // Dynamic brake (A/B = 100%)
void setSoftBrakePWM(uint16_t pwm) noexcept;  // Tune dither strength
void softBrakeNow(uint16_t pwm) noexcept;     // Start dither immediately
void pollFaults() noexcept;                      // Process deferred fault actions (call in loop)

// Behavior
void setFreewheelMode(FreewheelMode m) noexcept;
void applyFreewheel(FreewheelMode m) noexcept;

// Lifecycle
void start() noexcept;
void stop() noexcept;
bool reconfigureFrequency(int new_hz) noexcept;

// Safety & raw outputs
bool hasFault() const noexcept;
void clearFault() noexcept;
void forceOutputs(bool a_high, bool b_high) noexcept;
void setFaultCallback(FaultCallback cb, void* ctx) noexcept;

// Info
int getMaxPwmInput() const noexcept;  // Mirrors cfg.input_max (default 1023)
```

---

## Configuration

### `MotorMCPWMConfig`

```cpp
MotorMCPWMConfig hw{
  /* lpwm_pin   */ 2,
  /* rpwm_pin   */ 1,
  /* en_pin     */ -1,          // -1 if unused
  /* unit       */ MCPWM_UNIT_0,
  /* timer      */ MCPWM_TIMER_0,
  /* sig_l      */ MCPWM0A,
  /* sig_r      */ MCPWM0B
};

// Optional extras (defaults shown)
hw.pwm_freq_hz     = 20000;                        // Hz for MCPWM timer
hw.input_max       = 1023;                         // logical max input
hw.counter         = MCPWM_UP_COUNTER;             // or MCPWM_UP_DOWN_COUNTER
hw.use_deadtime    = false;
hw.deadtime_type   = MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE;
hw.deadtime_red_ns = 500;
hw.deadtime_fed_ns = 500;
```

### `MotorBehaviorConfig` (Freewheel + Dither)

```cpp
MotorBehaviorConfig beh{
  /* freewheel_mode    */ FreewheelMode::HiZ, // or HiZ_Awake / DitherBrake
  /* soft_brake_hz     */ 60,                 // dither frequency (Hz). Internally bounded to [50, 2000 Hz] for stability.
  /* dither_pwm        */ 0,                  // set at runtime with setSoftBrakePWM()
  /* default_soft      */ 0,                  // dither automatically when speed→0
  /* min_phase_us      */ 600,                // minimum slice per half-cycle (µs)
  /* dither_coast_hi_z */ true                // coast half truly Hi-Z (EN LOW)
};
```

**Fields**

- `freewheel_mode` — which strategy to apply when coasting.  
- `soft_brake_hz` — dither frequency. Lower is gentler for a fixed minimum pulse length.  
- `dither_pwm` — base “strength” for dither (0..`getMaxPwmInput()`).  
- `default_soft` — dither to apply automatically when `setSpeed(0, …)` is called.  
- `min_phase_us` — minimum brake/coast slice per half-cycle. Affects effective minimum fraction.  
- `dither_coast_hi_z` — if true, *coast half* of dither pulls EN LOW (true Hi-Z). Critical on IBT-2.  

### `MotorSafetyConfig` (optional)

```cpp
MotorSafetyConfig sfty;
sfty.fault_gpio        = 4;       // -1 to disable
sfty.fault_active_high = true;    // true: high = fault
sfty.oneshot           = true;    // latch until clearFault()
```

### `MotorCaptureConfig` (optional)

```cpp
MotorCaptureConfig cap;
cap.cap_gpio   = 5;                      // -1 to disable
cap.edge       = CaptureEdge::Rising;    // Rising/Falling/Both
cap.on_capture = [](uint32_t period_us, void* user){
  // ISR context: keep it short.
};
cap.user = nullptr;
```

---

## Dither Braking — Deep Dive

**What it is:** alternate between **Brake** and **Coast** (or **Hi-Z**) at `soft_brake_hz`.  
**Why:** gentle drag, stabilize roll-off, keep modules awake, or demonstrate braking without a full hard stop.

### Phases

- **Brake phase** — both sides asserted (dynamic short).  
- **Coast phase** — either A=0,B=0 (*coast*) or **true Hi-Z** (EN LOW) when `dither_coast_hi_z=true`.

> On many **IBT-2/BTS7960** boards, A=0,B=0 behaves as **short-brake**. Set `dither_coast_hi_z=true` to ensure the coast half really floats (Hi-Z).

### Effective Brake Fraction

Let `T = 1/soft_brake_hz`. With a minimum slice `min_phase_us`, the smallest fraction per half is:

```
f_min = (min_phase_us × soft_brake_hz) / 1,000,000
```

If you request a very small dither (e.g. 5%), but `f_min` is ~50%, it will feel like **hard brake**.  
**Fix**: lower `soft_brake_hz` and/or `min_phase_us` to make `f_min` small.

### Tuning Recipes

- **Gentle, audible:** `soft_brake_hz = 60–100`, `min_phase_us = 500–800`, `setSoftBrakePWM ≈ 5–10%`  
- **Feather-light:** `soft_brake_hz = 10–40`, `min_phase_us = 100–300`, `setSoftBrakePWM ≈ 1–4%`, `dither_coast_hi_z=true`  
- **Crisp stop:** `soft_brake_hz = 200–400`, larger `setSoftBrakePWM` (beware min slice; can feel like hard brake)

### Pulse Skipping (optional, API-only)

You can make dither nearly imperceptible by skipping coast windows between pulses (no extra API needed):

```cpp
const uint32_t period_ms = 1000UL / beh.soft_brake_hz;
const int skip_periods = 9; // coast between pulses
uint32_t elapsed = 0, obs_ms = 3000;

while (elapsed < obs_ms) {
  motor.setFreewheelMode(FreewheelMode::DitherBrake);
  motor.setSoftBrakePWM(/*~1–2% of input_max*/);
  motor.setSpeed(0, Dir::CW);   // engage dither for one period
  delay(period_ms);
  elapsed += period_ms;

  // long Hi-Z coast between pulses
  motor.setSoftBrakePWM(0);
  motor.setFreewheelMode(FreewheelMode::HiZ);
  motor.setFreewheel();
  uint32_t coast_ms = period_ms * skip_periods;
  delay(min(coast_ms, obs_ms - elapsed));
  elapsed += min(coast_ms, obs_ms - elapsed);
}
```

---

## Dead-Time (IMPORTANT with IBT-2/BTS7960)

**Dead-time modes in MCPWM couple A/B as complementary outputs** (B is a delayed complement of A). That’s correct for a **single half-bridge (HS/LS)** gate-drive, but it **breaks dual-input H-bridge modules** like IBT-2/BTS7960 which expect **independent LPWM/RPWM** signals.

- **Symptom:** motor surges then behaves strangely when `use_deadtime = true`.  
- **Fix:** keep `hw.use_deadtime = false` for IBT-2/BTS7960 (you can still use **center-aligned** with `MCPWM_UP_DOWN_COUNTER`).  
- **When to use dead-time:** only in designs where **hardware** should generate the complementary low-side from the high-side PWM (not the case here).

Example (safe for IBT-2):
```cpp
hw.use_deadtime = false;
hw.counter = MCPWM_UP_DOWN_COUNTER; // center-aligned OK
```

---

## Fault Handling (Software Fallback)

The library can watch a **fault GPIO** and either **latch** on first event or **follow level**.

- **Oneshot (latched):** set `sfty.oneshot = true`. First active edge → **hard brake** and `hasFault() == true` until you call `clearFault()`.
- **Level-follow:** set `sfty.oneshot = false`. When the input is active → **hard brake**; when inactive → cleared automatically.

Wiring & setup:
```cpp
MotorSafetyConfig sfty;
sfty.fault_gpio        = 4;     // -1 to disable
sfty.fault_active_high = true;  // set false if your signal is active-low
sfty.oneshot           = true;  // latch

motor.setup(hw, beh, sfty, MotorCaptureConfig{});

// Later:
if (motor.hasFault()) {
  Serial.println("Fault latched, clearing…");
  motor.clearFault();     // returns to a safe idle (Awake-HiZ)
}
```

**Tip:** debounce at the source if your fault source is noisy; the ISR is intentionally minimal.

---

## Capture (Period Measurement, Software Fallback)

Capture measures the **time between edges** on a GPIO using `micros()` from an ISR. It’s lightweight and good for tachs or timing marks.

- Choose edge: `Rising`, `Falling`, or `Both`.  
- Provide a tiny ISR callback (keep it short).  
- Read your last period in the main loop.

Example:
```cpp
volatile uint32_t last_period_us = 0;
static void onCap(uint32_t us, void*) { last_period_us = us; }  // ISR context

MotorCaptureConfig cap;
cap.cap_gpio   = 5;
cap.edge       = CaptureEdge::Rising;
cap.on_capture = &onCap;
cap.user       = nullptr;

motor.setup(hw, beh, MotorSafetyConfig{}, cap);

// In loop():
if (last_period_us) {
  const float hz = 1e6f / last_period_us;
  Serial.printf("Capture: %lu us  (~%.2f Hz)\n", (unsigned long)last_period_us, hz);
}
```

**Note:** the software path avoids MCPWM’s internal capture unit for portability; if you need hardware capture, you can swap in an IDF-native implementation behind the same config later.

---

## Practical Examples

### Basic Drive

```cpp
MotorMCPWMConfig hw{ 2,1,-1, MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, MCPWM0B };
Motor motor; motor.setup(hw);
motor.setSpeed(800, Dir::CW);
delay(500);
motor.setSpeed(0, Dir::CW);   // Applies behavior
```

### Braking Overview (continuous + feather-light)

```cpp
// Behavior (example): gentle dither and true Hi-Z coast on IBT-2
MotorBehaviorConfig beh{ FreewheelMode::HiZ, 25, 0, 0, 100, true };
Motor motor; motor.setup(hw, beh);
motor.start();

// Continuous gentle
motor.setFreewheelMode(FreewheelMode::DitherBrake);
motor.setSoftBrakePWM(motor.getMaxPwmInput() * 3 / 100);  // ~3%
motor.setSpeed(0, Dir::CW);                                // engage dither

// Feather-light (pulse skip)
const uint32_t period_ms = 1000UL / 25;
motor.setSoftBrakePWM(motor.getMaxPwmInput() * 1 / 100);  // ~1%
// Alternate one dither period with several Hi-Z periods (see code in Dither section)
```

### Center-Aligned (without Dead-Time)

```cpp
hw.use_deadtime = false;                 // keep off for IBT-2
hw.counter      = MCPWM_UP_DOWN_COUNTER; // center-aligned
```

### Fault + Capture

```cpp
MotorSafetyConfig sfty; sfty.fault_gpio = 4; sfty.fault_active_high = true; sfty.oneshot = true;
MotorCaptureConfig cap; cap.cap_gpio = 5; cap.edge = CaptureEdge::Rising; cap.on_capture = &onCap;
motor.setup(hw, beh, sfty, cap);
```

---

## Notes & Tips

- Pins **34–39 on a standard ESP32 are input-only**. Use an output-capable GPIO for EN or set `en_pin=-1` and hard-wire EN high. I have tested the code with an **ESP32-S3!  
- **Input range:** `getMaxPwmInput()` mirrors your configured `input_max` (default **1023**).  
- **Time types:** prefer fixed-width types like `uint32_t` for `millis()`/`micros()` math.  
- **Threading:** small critical section around soft-brake state; keep ISRs minimal.  
- **Audible noise:** dither audibility depends on `soft_brake_hz` and `min_phase_us`—tune them together.

---

## Troubleshooting

- **Coast feels like brake on IBT-2:** Use `FreewheelMode::HiZ` and `dither_coast_hi_z=true`. Ensure EN truly goes LOW in coast.  
- **Dither feels like hard brake:** Your `f_min` is too large. Lower `soft_brake_hz` and/or `min_phase_us`, or reduce `setSoftBrakePWM`.  
- **Ramp shows no change:** Disable dither (`setSoftBrakePWM(0)`), use `HiZ_Awake` while ramping, verify EN pin is output-capable, try lower drive PWM (1–3 kHz) for a clearer effect.  
- **Weird surges when enabling dead-time:** Turn **dead-time OFF** for IBT-2/BTS7960; it couples A/B and breaks dual-input control.  
- **Nothing happens:** sanity-check with `forceOutputs(true,false)` for a short pulse; if no motion, EN wiring or LPWM/RPWM mapping is wrong.

---

## License

MIT © Darren Osborne (Little Man Builds)

Contributions welcome! Please keep Doxygen comments consistent with the interface and follow the established naming conventions.
