/**
 * MIT License
 *
 * @brief Drive one motor forward, coast, reverse, then disable its outputs.
 *
 * @file 01_BasicMotorControl.ino
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-06-22
 * @copyright Copyright (c) 2026 Little Man Builds
 *
 * Wiring (ESP32-S3 DevKitC-1 -> H-bridge logic input):
 * GPIO 4 -> LPWM, GPIO 5 -> RPWM, GPIO 6 -> EN, and GND -> logic GND.
 * Confirm the voltage and truth table for your own driver module.
 *
 * What you should see: the motor runs in each direction for two seconds,
 * coasts between directions, then remains disabled.
 */

#include <ESP32_MCPWM.h>

// ---- Hardware configuration ---- //

const int LPWM_PIN = 4;
const int RPWM_PIN = 5;
const int EN_PIN = 6;

// This group tells the driver which bridge pins and MCPWM resources it owns.
MotorMCPWMConfig hardware{LPWM_PIN, RPWM_PIN, EN_PIN, MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, MCPWM0B};
Motor motor;

// ---- Setup ---- //

void setup()
{
    Serial.begin(115200);
    const MotorSetupResult setup_result = motor.setup(hardware);

    // Never command the bridge when its hardware setup did not complete.
    if (!setup_result.ok())
    {
        Serial.println("Motor setup failed. Check the configured pins and frequency.");
        while (true)
            delay(1000);
    }
}

// ---- Main loop ---- //

void loop()
{
    Serial.println("Forward at 50%.");
    motor.drivePercent(50, Dir::CW);
    delay(2000);

    Serial.println("Coast.");
    // Coast releases the bridge using the configured freewheel semantics.
    // Zero drive demand is not a stop command in v2, so choose this explicitly.
    motor.coast();
    delay(2000);

    Serial.println("Reverse at 50%.");
    motor.drivePercent(50, Dir::CCW);
    delay(2000);

    Serial.println("Disable outputs.");
    // Disable is the explicit containment/lifecycle action: it deasserts EN
    // when configured and stops MCPWM output generation.
    motor.disableOutputs();

    while (true)
        delay(1000);
}
