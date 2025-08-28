/**
 * @file Advanced features.ino
 *
 * @brief Advanced features:
 *    - Configure timer: center-aligned PWM (dead-time DISABLED for IBT-2/BTS7960).
 *    - (Optional) Configure capture ISR and safety fault latch (if pins wired).
 *    - Initialize and start the motor driver.
 *    - Attempt runtime PWM-frequency retune.
 *    - Loop: smooth duty ramp up/down (with floor to overcome stiction).
 *    - (Optional) ForceOutputs diagnostic pulse (read comments before using).
 *    - Loop: fault handling (if wired).
 *    - Loop: capture readout (if wired).
 */

#include <ESP32_MCPWM.h>

/**
 * @brief Note: GPIO 34-39 on a standard ESP32 are input only!
 *  Tested with an ESP32-S3.
 */
#define LPWM_PIN 37
#define RPWM_PIN 38
#define EN_PIN 39    ///< Set to -1 if your driver has no EN pin.
#define FAULT_PIN -1 ///< Set a valid GPIO to demo software fault latch (optional).
#define CAP_PIN -1   ///< Set a valid GPIO to demo capture callback (optional).

MotorMCPWMConfig hw{LPWM_PIN, RPWM_PIN, EN_PIN, MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, MCPWM0B};

MotorBehaviorConfig beh{
    FreewheelMode::HiZ,
    /*soft_brake_hz=*/50,
    /*dither_pwm=*/0,
    /*default_soft=*/0,
    /*min_phase_us=*/100,
    /*dither_coast_hi_z=*/true};

static void configureTimerAdvanced()
{
  hw.use_deadtime = false;            ///< Dead-time modes are for driving a single half-bridge (HS/LS).
  hw.counter = MCPWM_UP_DOWN_COUNTER; ///< Center-aligned.
  hw.pwm_freq_hz = 20000;             ///< 20 kHz drive PWM.
  hw.input_max = 1023;                ///< Logical max.
}

// ---- Safety (optional) : assign in setup ----
MotorSafetyConfig safety; ///< Defaults - wire if you want to test oneshot latch.

// ---- Capture (optional) : assign in setup ----
volatile uint32_t g_lastPeriodUs = 0;
static void onCapture(uint32_t us, void *) { g_lastPeriodUs = us; }
MotorCaptureConfig cap; ///< Set CAP_PIN if enabled.

Motor motor;

static inline int pwmFromPct(float pct)
{
  if (pct < 0)
    pct = 0;
  if (pct > 100)
    pct = 100;
  const int maxIn = motor.getMaxPwmInput();
  return (pct * maxIn + 50) / 100;
}

/**
 * @brief Smooth ramp with a small floor to overcome static friction.
 */
static void rampPercent(int startPct, int endPct, uint32_t total_ms, Dir dir,
                        int steps = 40, int minEffPct = 30)
{
  if (steps < 1)
    steps = 1;
  const float inc = (endPct - startPct) / static_cast<float>(steps);
  const uint32_t dwell = total_ms / static_cast<uint32_t>(steps ? steps : 1);
  for (int i = 0; i <= steps; ++i)
  {
    float pf = startPct + inc * i;
    int p = static_cast<int>(pf + (pf >= 0 ? 0.5f : -0.5f));
    if (p > 0 && p < minEffPct)
      p = minEffPct;
    Serial.print("PWM: ");
    Serial.println(p);
    motor.setSpeed(pwmFromPct(p), dir);
    delay(dwell);
  }
}

/**
 * @brief Print capture frequency once (if wired).
 */
static void printCaptureOnce()
{
  uint32_t us = g_lastPeriodUs;
  if (us)
  {
    const float hz = 1e6f / static_cast<float>(us);
    Serial.print("[Capture] Period_us = ");
    Serial.print(us);
    Serial.print(", freq_hz ≈ ");
    Serial.println(hz, 2);
  }
  else
  {
    Serial.println("[Capture] No edges yet.");
  }
}

void setup()
{
  Serial.begin(115200);
  delay(50);

  configureTimerAdvanced();

  // Optional wiring: set only if pins are connected.
  if (CAP_PIN >= 0)
  {
    cap.cap_gpio = CAP_PIN;
    cap.edge = CaptureEdge::Rising;
    cap.on_capture = &onCapture;
    cap.user = nullptr;
  }
  if (FAULT_PIN >= 0)
  {
    safety.fault_gpio = FAULT_PIN;
    safety.fault_active_high = true;
    safety.oneshot = true;
  }

  motor.setup(hw, beh, safety, cap);
  motor.start();

  // Try runtime freq retune (drive PWM).
  if (motor.reconfigureFrequency(25000))
    Serial.println("Drive PWM retuned to 25 kHz.");
  else
    Serial.println("Drive PWM retune not supported on this core.");
}

void loop()
{
  const Dir dir = Dir::CW;

  // Smooth ramp up/down once per loop.
  motor.setSoftBrakePWM(0);
  motor.setFreewheelMode(FreewheelMode::HiZ_Awake);
  rampPercent(30, 85, 2000, dir);
  delay(1000);
  rampPercent(85, 0, 2000, dir);
  delay(1000);

  /*
   * @brief ForceOutputs diagnostics pulse - SAFE TO USE CHECKLIST
   *        - Keep pulses short (100-500ms) and bench test with wheels off the ground.
   *        - Don't spam it under heavy load - it's 100% duty instantly.
   *        - Never use for direction changes.
   *        - After a force pulse, return to normal control.
   */
  // motor.forceOutputs(true, false);
  // delay(100);
  // motor.forceOutputs(false, false);
  // delay(100);

  // Fault handling (if wired).
  if (motor.hasFault())
  {
    Serial.println("[Fault] Latched. Clearing to safe idle.");
    motor.clearFault();
  }

  // Capture read (if wired).
  if (CAP_PIN >= 0)
    printCaptureOnce();

  delay(1000);
}