/**
 * @file 02_FreewheelAndDitherBrake.ino
 *
 * @brief Compare Hi-Z coast, hard brake, and gentle dither brake.
 *
 * Wiring (ESP32-S3 DevKitC-1):
 * GPIO 4 -> LPWM, GPIO 5 -> RPWM, GPIO 6 -> EN, and common GND.
 * Dither brake alternates short brake and coast phases. With Hi-Z coast,
 * EN is low during coast. Confirm that behavior in your module's truth table.
 */

#include <ESP32_MCPWM.h>

static constexpr int LPWM_PIN = 4;
static constexpr int RPWM_PIN = 5;
static constexpr int EN_PIN = 6;
static constexpr int DITHER_HZ = 100;
static constexpr int MIN_PHASE_US = 50;

MotorMCPWMConfig hardware{LPWM_PIN, RPWM_PIN, EN_PIN,
                          MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, MCPWM0B};
MotorBehaviorConfig behavior{FreewheelMode::HiZ, DITHER_HZ,
                             0, 0, MIN_PHASE_US, true};
Motor motor;

void runMotor()
{
    motor.setSpeedPercent(60, Dir::CW);
    delay(2000);
}

void setup()
{
    Serial.begin(115200);
    motor.setup(hardware, behavior);

    if (!motor.isSetupComplete())
    {
        Serial.println("Motor setup failed. Check the configured pins and timing.");
        while (true)
            delay(1000);
    }
}

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
    motor.setFreewheel();
    delay(1500);

    Serial.println("Dither brake at about 8% strength.");
    runMotor();
    motor.softBrakeNow(motor.getMaxPwmInput() * 8 / 100);
    delay(2500);

    Serial.println("Zero dither: ordinary Hi-Z coast, with no timer running.");
    motor.softBrakeNow(0);
    delay(2500);
}
