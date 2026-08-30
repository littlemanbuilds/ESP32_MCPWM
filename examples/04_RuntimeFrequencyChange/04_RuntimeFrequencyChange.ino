/**
 * MIT License
 *
 * @brief Change the drive PWM frequency and check whether it succeeded.
 *
 * @file 04_RuntimeFrequencyChange.ino
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-06-22
 * @copyright Copyright (c) 2026 Little Man Builds
 *
 * Wiring (ESP32-S3 DevKitC-1):
 * GPIO 4 -> LPWM, GPIO 5 -> RPWM, GPIO 6 -> EN, and common GND.
 * Reconfiguration first brings the bridge to a quiet zero-duty coast state.
 * Submit a new drive command after a successful change.
 */

#include <ESP32_MCPWM.h>

// ---- Hardware configuration ---- //

const int LPWM_PIN = 4;
const int RPWM_PIN = 5;
const int EN_PIN = 6;

MotorMCPWMConfig hardware{LPWM_PIN, RPWM_PIN, EN_PIN, MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, MCPWM0B};
Motor motor;

// ---- Setup ---- //

void setup()
{
    Serial.begin(115200);
    hardware.pwm_freq_hz = 20000;
    const MotorSetupResult setup_result = motor.setup(hardware);

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
    Serial.println("Running with 20 kHz drive PWM.");
    motor.drivePercent(50, Dir::CW);
    delay(2500);

    const MotorOperationResult frequency_result = motor.reconfigureFrequency(25000);
    if (frequency_result.ok())
    {
        Serial.println("Changed to 25 kHz. Sending a new drive command.");
        // Frequency changes do not restore old drive intent automatically.
        motor.drivePercent(50, Dir::CW);
    }
    else
    {
        Serial.println("Frequency change failed; inspect status before recovery.");
    }
    delay(2500);

    motor.coast();
    delay(1500);

    if (!motor.reconfigureFrequency(20000).ok())
        Serial.println("Could not restore 20 kHz.");
}
