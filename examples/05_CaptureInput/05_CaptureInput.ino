/**
 * @file 05_CaptureInput.ino
 *
 * @brief Measure time between signal edges with the capture input.
 *
 * Motor wiring (ESP32-S3 DevKitC-1):
 * GPIO 4 -> LPWM, GPIO 5 -> RPWM, GPIO 6 -> EN, and common GND.
 * Test wiring: connect GPIO 7 directly to GPIO 8. GPIO 7 generates a 1 kHz
 * square wave and GPIO 8 captures both edges.
 *
 * Both-edge capture reports adjacent-edge intervals. A symmetrical 1 kHz
 * square wave therefore reads about 500 us, half of its 1000 us full period.
 */

#include <ESP32_MCPWM.h>

static constexpr int LPWM_PIN = 4;
static constexpr int RPWM_PIN = 5;
static constexpr int EN_PIN = 6;
static constexpr int TEST_SIGNAL_PIN = 7;
static constexpr int CAPTURE_PIN = 8;
static constexpr int TEST_SIGNAL_HZ = 1000;

MotorMCPWMConfig hardware{LPWM_PIN, RPWM_PIN, EN_PIN,
                          MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, MCPWM0B};
MotorCaptureConfig capture;
Motor motor;

void setup()
{
    Serial.begin(115200);

    capture.cap_gpio = CAPTURE_PIN;
    capture.edge = CaptureEdge::Both;
    motor.setup(hardware, MotorBehaviorConfig{}, MotorSafetyConfig{}, capture);

    if (!motor.isSetupComplete())
    {
        Serial.println("Motor setup failed. Check the configured pins.");
        while (true)
            delay(1000);
    }

    tone(TEST_SIGNAL_PIN, TEST_SIGNAL_HZ);
    Serial.println("Capturing both edges of the 1 kHz test signal.");
}

void loop()
{
    const uint32_t edge_interval_us = motor.getLastCapturePeriodUs();
    if (edge_interval_us == 0)
    {
        Serial.println("No capture yet. Check the GPIO 7 to GPIO 8 jumper.");
    }
    else
    {
        Serial.print("Edge interval: ");
        Serial.print(edge_interval_us);
        Serial.println(" us (about 500 us expected with Both).");
    }
    delay(500);
}
