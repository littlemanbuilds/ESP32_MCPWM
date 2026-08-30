/**
 * MIT License
 *
 * @brief Compare Hi-Z coast, hard brake, and gentle dither brake.
 *
 * @file 02_FreewheelAndDitherBrake.ino
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-06-22
 * @copyright Copyright (c) 2026 Little Man Builds
 *
 * Wiring (ESP32-S3 DevKitC-1):
 * GPIO 4 -> LPWM, GPIO 5 -> RPWM, GPIO 6 -> EN, and common GND.
 * Dither brake alternates short brake and coast phases. With Hi-Z coast,
 * EN is low during coast. Confirm the module truth table and brake current on
 * the bench before using hard or dither braking on a loaded mechanism.
 */

#include <ESP32_MCPWM.h>

// ---- Hardware and behavior configuration ---- //

const int LPWM_PIN = 4;
const int RPWM_PIN = 5;
const int EN_PIN = 6;
const int DITHER_HZ = 100;
const int MIN_PHASE_US = 50;

MotorMCPWMConfig hardware{LPWM_PIN, RPWM_PIN, EN_PIN, MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, MCPWM0B};
// Start with explicit Hi-Z coast; softBrakeNow() supplies each demonstrated
// dither strength while this group owns timing and coast-phase behavior.
MotorBehaviorConfig behavior{FreewheelMode::HiZ, DITHER_HZ, 0, 0, MIN_PHASE_US, true};
Motor motor;

// ---- Helpers ---- //

void runMotor()
{
    motor.drivePercent(60, Dir::CW);
    delay(2000);
}

// ---- Setup ---- //

void setup()
{
    Serial.begin(115200);
    const MotorSetupResult setup_result = motor.setup(hardware, behavior);

    if (!setup_result.ok())
    {
        Serial.println("Motor setup failed. Check the configured pins and timing.");
        while (true)
            delay(1000);
    }
}

// ---- Main loop ---- //

void loop()
{
    Serial.println("Hi-Z coast: EN goes low.");
    runMotor();
    motor.applyFreewheel(FreewheelMode::HiZ);
    delay(2500);

    Serial.println("Hard brake: both bridge inputs are asserted.");
    runMotor();
    motor.setHardBrake();
    delay(1000);
    motor.coast();
    delay(1500);

    Serial.println("Dither brake at about 8% strength.");
    runMotor();
    motor.softBrakeNow(static_cast<uint16_t>(motor.getMaxPwmInput() * 8 / 100));
    delay(2500);

    Serial.println("Zero dither: ordinary Hi-Z coast, with no timer running.");
    motor.softBrakeNow(0);
    delay(2500);
}
