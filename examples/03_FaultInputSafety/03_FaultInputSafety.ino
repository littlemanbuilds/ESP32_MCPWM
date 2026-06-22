/**
 * @file 03_FaultInputSafety.ino
 *
 * @brief Use an active-low fault input to latch a safe coast command.
 *
 * Wiring (ESP32-S3 DevKitC-1):
 * GPIO 4 -> LPWM, GPIO 5 -> RPWM, GPIO 6 -> EN, and common GND.
 * GPIO 7 is the active-low fault input. For a low-current bench test, connect
 * GPIO 7 to GND with a pushbutton. Do not use a motor power wire for this test.
 */

#include <ESP32_MCPWM.h>

static constexpr int LPWM_PIN = 4;
static constexpr int RPWM_PIN = 5;
static constexpr int EN_PIN = 6;
static constexpr int FAULT_PIN = 7;

MotorMCPWMConfig hardware{LPWM_PIN, RPWM_PIN, EN_PIN,
                          MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, MCPWM0B};
MotorSafetyConfig safety;
Motor motor;
bool fault_reported = false;
bool drive_armed = true;

void setup()
{
    Serial.begin(115200);

    safety.fault_gpio = FAULT_PIN;
    safety.fault_active_high = false;
    safety.oneshot = true;
    safety.fault_action = FaultAction::Coast;

    motor.setup(hardware, MotorBehaviorConfig{}, safety, MotorCaptureConfig{});
    if (!motor.isSetupComplete())
    {
        Serial.println("Motor setup failed. Check the configured pins.");
        while (true)
            delay(1000);
    }

    motor.pollFaults();
    if (!motor.hasFault())
    {
        Serial.println("Motor running at 30%. Ground GPIO 7 to assert the fault.");
        motor.setSpeedPercent(30, Dir::CW);
    }
}

void loop()
{
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
        motor.setSpeedPercent(30, Dir::CW);
        drive_armed = true;
        fault_reported = false;
        Serial.println("New drive command accepted.");
    }

    delay(10); // Keeps deferred fault response prompt without busy-waiting.
}
