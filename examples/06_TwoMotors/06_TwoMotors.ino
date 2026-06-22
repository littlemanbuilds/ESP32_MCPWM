/**
 * @file 06_TwoMotors.ino
 *
 * @brief Drive two motors forward, reverse, and turn in place.
 *
 * Wiring (ESP32-S3 DevKitC-1):
 * Left bridge: GPIO 4 -> LPWM, GPIO 5 -> RPWM, GPIO 6 -> EN.
 * Right bridge: GPIO 9 -> LPWM, GPIO 10 -> RPWM, GPIO 11 -> EN.
 * Connect the ESP32 and both bridge logic grounds together.
 */

#include <ESP32_MCPWM.h>

MotorMCPWMConfig left_hardware{4, 5, 6, MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, MCPWM0B};
MotorMCPWMConfig right_hardware{9, 10, 11, MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM1A, MCPWM1B};
Motor left_motor;
Motor right_motor;

static constexpr Dir LEFT_FORWARD = Dir::CW;
static constexpr Dir RIGHT_FORWARD = Dir::CW;

void coastBoth()
{
    left_motor.setFreewheel();
    right_motor.setFreewheel();
    delay(1000);
}

void setup()
{
    Serial.begin(115200);
    left_motor.setup(left_hardware);
    right_motor.setup(right_hardware);

    if (!left_motor.isSetupComplete() || !right_motor.isSetupComplete())
    {
        Serial.println("Motor setup failed. Check both pin and timer configurations.");
        while (true)
            delay(1000);
    }
}

void loop()
{
    Serial.println("Forward.");
    left_motor.setSpeedPercent(50, LEFT_FORWARD);
    right_motor.setSpeedPercent(50, RIGHT_FORWARD);
    delay(2000);
    coastBoth();

    Serial.println("Reverse.");
    left_motor.setSpeedPercent(50, IMotorDriver::changeDir(LEFT_FORWARD));
    right_motor.setSpeedPercent(50, IMotorDriver::changeDir(RIGHT_FORWARD));
    delay(2000);
    coastBoth();

    Serial.println("Turn left in place.");
    left_motor.setSpeedPercent(40, IMotorDriver::changeDir(LEFT_FORWARD));
    right_motor.setSpeedPercent(40, RIGHT_FORWARD);
    delay(1500);
    coastBoth();
}
