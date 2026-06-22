/**
 * @file 01_BasicMotorControl.ino
 *
 * @brief Drive one motor forward, coast, reverse, and coast again.
 *
 * Wiring (ESP32-S3 DevKitC-1 -> H-bridge logic input):
 * GPIO 4 -> LPWM, GPIO 5 -> RPWM, GPIO 6 -> EN, and GND -> logic GND.
 * Confirm the voltage and truth table for your own driver module.
 *
 * What you should see: the motor runs in each direction for two seconds and
 * coasts to a stop between directions.
 */

#include <ESP32_MCPWM.h>

static constexpr int LPWM_PIN = 4;
static constexpr int RPWM_PIN = 5;
static constexpr int EN_PIN = 6;

MotorMCPWMConfig hardware{LPWM_PIN, RPWM_PIN, EN_PIN,
                          MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, MCPWM0B};
Motor motor;

void setup()
{
    Serial.begin(115200);
    motor.setup(hardware);

    if (!motor.isSetupComplete())
    {
        Serial.println("Motor setup failed. Check the configured pins and frequency.");
        while (true)
            delay(1000);
    }
}

void loop()
{
    Serial.println("Forward at 50%.");
    motor.setSpeedPercent(50, Dir::CW);
    delay(2000);

    Serial.println("Coast.");
    motor.setFreewheel();
    delay(2000);

    Serial.println("Reverse at 50%.");
    motor.setSpeedPercent(50, Dir::CCW);
    delay(2000);

    Serial.println("Coast.");
    motor.setFreewheel();
    delay(2000);
}
