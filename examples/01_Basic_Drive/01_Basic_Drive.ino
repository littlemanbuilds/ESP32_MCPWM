/**
 * @file Basic_Drive.ino
 *
 * @brief Set speed CW/CCW and stop in between.
 */

#include <ESP32_MCPWM.h>

/**
 * @brief Note: GPIO 34-39 on the original ESP32 are input only.
 *  Tested with an ESP32-S3 DevKitC-1; adjust pins for your board.
 */
#define LPWM_PIN 37
#define RPWM_PIN 38
#define EN_PIN 39 ///< Set to -1 if your driver has no EN pin.

MotorMCPWMConfig hw{LPWM_PIN, RPWM_PIN, EN_PIN, MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, MCPWM0B};
Motor motor;

void setup()
{
    Serial.begin(115200);
    delay(50);

    motor.setup(hw); ///< Default behavior: if EN is unused, library prefers HiZ_Awake for stability.
}

void loop()
{
    Serial.println("CW half speed...");
    motor.setSpeedPercent(50, Dir::CW);
    delay(3000);

    Serial.println("Stop (behavior-defined freewheel/soft-brake)...");
    motor.setSpeedPercent(0, Dir::CW);
    delay(2000);

    Serial.println("CCW half speed...");
    motor.setSpeedPercent(50, Dir::CCW);
    delay(3000);

    Serial.println("Stop...");
    motor.setSpeedPercent(0, Dir::CW);
    delay(2000);
}
