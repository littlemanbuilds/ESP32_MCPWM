/**
 * @file Braking Overview.ino
 *
 * @brief Braking comparison:
 *    - True coast (Hi-Z, EN LOW).
 *    - Hard brake.
 *    - Dither brake (gentle, continuous).
 *    - Dither brake (feather-light, pulse-skipped).
 */

#include <ESP32_MCPWM.h>

/**
 * @brief Note: GPIO 34-39 on a standard ESP32 are input only!
 *  Tested with an ESP32-S3.
 */
#define LPWM_PIN 37
#define RPWM_PIN 38
#define EN_PIN 39 ///< Set to -1 if your driver has no EN pin.

MotorMCPWMConfig hw{LPWM_PIN, RPWM_PIN, EN_PIN, MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, MCPWM0B};

// Behavior tuning.
static constexpr int kDitherHz = 25;      // Low Hz → gentle; visible but not grabby.
static constexpr int kMinPhaseUs = 100;   // Allows sub-1% slices at these Hz.
static constexpr int kDitherPctSoft = 3;  // Continuous dither strength (~3%).
static constexpr int kDitherPctFeath = 1; // Feather-light dither strength (~1%).

// Default to true coast; dither uses Brake <-> Hi-Z (EN LOW during coast half-cycle).
MotorBehaviorConfig beh{
    FreewheelMode::HiZ,
    /*soft_brake_hz=*/kDitherHz,
    /*dither_pwm=*/0,
    /*default_soft=*/0,
    /*min_phase_us=*/kMinPhaseUs,
    /*dither_coast_hi_z=*/true};

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

void setup()
{
    Serial.begin(115200);
    delay(50);

    motor.setup(hw, beh);
    motor.start();

    // Keep the drive PWM quiet; dither frequency is independent (25 Hz from beh).
    motor.reconfigureFrequency(20000); ///< 20 kHz on LPWM/RPWM drive.

    // Safety: Ensure coast is truly Hi-Z (EN LOW) when we freewheel.
    motor.setFreewheelMode(FreewheelMode::HiZ);
    motor.setSoftBrakePWM(0);
}

void loop()
{
    const uint32_t spin_ms = 2000; ///< Time to reach a steady speed.
    const uint32_t obs_ms = 3000;  ///< Time to observe spin-down.
    const int runPct = 100;        ///< Drive level (%).
    const Dir dir = Dir::CW;

    // Coast (true Hi-Z, EN LOW).
    Serial.println("Coast (Hi-Z)...");
    motor.setSpeedPercent(runPct, dir);
    delay(spin_ms);
    motor.setSoftBrakePWM(0);                   ///< No dither at zero.
    motor.setFreewheelMode(FreewheelMode::HiZ); ///< Ensure EN LOW during coast.
    motor.setFreewheel();
    delay(obs_ms);

    // Hard brake (short brake).
    Serial.println("Hard brake...");
    motor.setSpeedPercent(runPct, dir);
    delay(spin_ms);
    motor.setHardBrake(); ///< EN HIGH + both sides asserted.
    delay(obs_ms);
    motor.setFreewheel(); ///< Release.

    // Dither brake (continuous, gentle).
    Serial.println("Dither brake (continuous, gentle)...");
    motor.setSpeedPercent(runPct, dir);
    delay(spin_ms);
    motor.setFreewheelMode(FreewheelMode::DitherBrake);
    motor.setSoftBrakePWM(pwmFromPct(kDitherPctSoft)); ///< ~3% at kDitherHz.
    motor.setSpeed(0, dir);                            ///< Engage dither at zero.
    delay(obs_ms);

    // Dither brake (feather-light, pulse-skipped).
    Serial.println("Dither brake (feather-light, pulse-skipped)...");
    motor.setSpeedPercent(runPct, dir);
    delay(spin_ms);

    const uint32_t period_ms = 1000UL / kDitherHz; ///< E.g., 40 ms @ 25 Hz.
    const int skip_periods = 9;                    ///< Skip 9 → ~1 pulse each ~400 ms.
    uint32_t elapsed = 0;

    while (elapsed < obs_ms)
    {
        // One gentle dither period ON (Brake <-> Hi-Z with ~1% brake).
        motor.setFreewheelMode(FreewheelMode::DitherBrake);
        motor.setSoftBrakePWM(pwmFromPct(kDitherPctFeath));
        motor.setSpeed(0, dir);
        delay(period_ms);
        elapsed += period_ms;
        if (elapsed >= obs_ms)
            break;

        // Skip N periods with pure Hi-Z coast.
        motor.setSoftBrakePWM(0);
        motor.setFreewheelMode(FreewheelMode::HiZ);
        motor.setFreewheel();
        const uint32_t coast_ms = period_ms * (uint32_t)skip_periods;
        const uint32_t slice = (elapsed + coast_ms > obs_ms) ? (obs_ms - elapsed) : coast_ms;
        delay(slice);
        elapsed += slice;
    }

    // Reset for next cycle.
    motor.setSoftBrakePWM(0);
    motor.setFreewheelMode(FreewheelMode::HiZ);
    motor.setFreewheel();
    delay(1500);
}