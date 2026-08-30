/**
 * MIT License
 *
 * @brief Drive two motors forward, reverse, and turn in place.
 *
 * @file 06_TwoMotors.ino
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-06-22
 * @copyright Copyright (c) 2026 Little Man Builds
 *
 * Wiring (ESP32-S3 DevKitC-1):
 * Left bridge: GPIO 4 -> LPWM, GPIO 5 -> RPWM, GPIO 6 -> EN.
 * Right bridge: GPIO 9 -> LPWM, GPIO 10 -> RPWM, GPIO 11 -> EN.
 * Connect the ESP32 and both bridge logic grounds together.
 */

#include <ESP32_MCPWM.h>

// ---- Motor configuration ---- //

// Each Motor instance owns one MCPWM timer and one bridge. The application is
// the single command owner for both instances in this sketch.
MotorMCPWMConfig left_hardware{4, 5, 6, MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, MCPWM0B};
MotorMCPWMConfig right_hardware{9, 10, 11, MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM1A, MCPWM1B};
Motor left_motor;
Motor right_motor;

static constexpr Dir LEFT_FORWARD = Dir::CW;
static constexpr Dir RIGHT_FORWARD = Dir::CW;

// ---- Helpers ---- //

void coastBoth()
{
    left_motor.coast();
    right_motor.coast();
    delay(1000);
}

// ---- Setup ---- //

void setup()
{
    Serial.begin(115200);
    const MotorSetupResult left_setup = left_motor.setup(left_hardware);
    const MotorSetupResult right_setup = right_motor.setup(right_hardware);

    if (!left_setup.ok() || !right_setup.ok())
    {
        Serial.println("Motor setup failed. Check both pin and timer configurations.");
        while (true)
            delay(1000);
    }
}

// ---- Main loop ---- //

void loop()
{
    Serial.println("Forward.");
    left_motor.drivePercent(50, LEFT_FORWARD);
    right_motor.drivePercent(50, RIGHT_FORWARD);
    delay(2000);
    coastBoth();

    Serial.println("Reverse.");
    left_motor.drivePercent(50, IMotorDriver::changeDir(LEFT_FORWARD));
    right_motor.drivePercent(50, IMotorDriver::changeDir(RIGHT_FORWARD));
    delay(2000);
    coastBoth();

    Serial.println("Turn left in place.");
    left_motor.drivePercent(40, IMotorDriver::changeDir(LEFT_FORWARD));
    right_motor.drivePercent(40, RIGHT_FORWARD);
    delay(1500);
    coastBoth();
}
