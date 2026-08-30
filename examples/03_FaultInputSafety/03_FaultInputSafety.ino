/**
 * MIT License
 *
 * @brief Use a software-observed active-low fault to latch a coast command.
 *
 * @file 03_FaultInputSafety.ino
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-06-22
 * @copyright Copyright (c) 2026 Little Man Builds
 *
 * Wiring (ESP32-S3 DevKitC-1):
 * GPIO 4 -> LPWM, GPIO 5 -> RPWM, GPIO 6 -> EN, and common GND.
 * GPIO 7 is the active-low software fault observer. For a low-current bench
 * test, connect GPIO 7 to GND with a pushbutton. This path requires pollFaults()
 * and is not a hardware emergency stop. Do not use a motor power wire here.
 */

#include <ESP32_MCPWM.h>

// ---- Hardware and fault configuration ---- //

const int LPWM_PIN = 4;
const int RPWM_PIN = 5;
const int EN_PIN = 6;
const int FAULT_PIN = 7;

MotorMCPWMConfig hardware{LPWM_PIN, RPWM_PIN, EN_PIN, MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, MCPWM0B};
// This is a software observer configuration, not a hardware emergency stop.
MotorSafetyConfig safety;
Motor motor;
bool fault_reported = false;
bool drive_armed = true;

// ---- Setup ---- //

void setup()
{
    Serial.begin(115200);

    safety.fault_gpio = FAULT_PIN;
    safety.fault_active_high = false;
    safety.oneshot = true;
    safety.fault_action = FaultAction::Coast;

    const MotorSetupResult setup_result = motor.setup(hardware, MotorBehaviorConfig{}, safety, MotorCaptureConfig{});
    if (!setup_result.ok())
    {
        Serial.println("Motor setup failed. Check the configured pins.");
        while (true)
            delay(1000);
    }

    motor.pollFaults();
    if (!motor.hasFault())
    {
        Serial.println("Motor running at 30%. Ground GPIO 7 to assert the fault.");
        motor.drivePercent(30, Dir::CW);
    }
}

// ---- Main loop ---- //

void loop()
{
    // The GPIO ISR only records fault state; polling owns the bridge action.
    motor.pollFaults();

    if (motor.hasFault())
    {
        drive_armed = false;
        if (!fault_reported)
        {
            Serial.println("Fault latched. Release GPIO 7, then send c to clear.");
            fault_reported = true;
        }

        if (Serial.available() && Serial.read() == 'c')
        {
            motor.clearFault();
            if (!motor.hasFault())
                Serial.println("Cleared at zero output. Send d for a new drive command.");
        }
    }
    else if (!drive_armed && Serial.available() && Serial.read() == 'd')
    {
        motor.drivePercent(30, Dir::CW);
        drive_armed = true;
        fault_reported = false;
        Serial.println("New drive command accepted.");
    }

    delay(10); // Keeps deferred fault response prompt without busy-waiting.
}
