/**
 * @file 04_RuntimeFrequencyChange.ino
 *
 * @brief Change the drive PWM frequency and check whether it succeeded.
 *
 * Wiring (ESP32-S3 DevKitC-1):
 * GPIO 4 -> LPWM, GPIO 5 -> RPWM, GPIO 6 -> EN, and common GND.
 * Reconfiguration first brings the bridge to disabled zero output. Submit a
 * new drive command after a successful change.
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
    hardware.pwm_freq_hz = 20000;
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
    Serial.println("Running with 20 kHz drive PWM.");
    motor.setSpeedPercent(50, Dir::CW);
    delay(2500);

    if (motor.reconfigureFrequency(25000))
    {
        Serial.println("Changed to 25 kHz. Sending a new drive command.");
        motor.setSpeedPercent(50, Dir::CW);
    }
    else
    {
        Serial.println("Frequency change failed; outputs remain in a safe state.");
    }
    delay(2500);

    motor.setFreewheel();
    delay(1500);

    if (!motor.reconfigureFrequency(20000))
        Serial.println("Could not restore 20 kHz.");
}
