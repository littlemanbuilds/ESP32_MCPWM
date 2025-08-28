/**
 * @file Two_Motors_Demo.ino
 *
 * @brief Minimal demonstration using two H-bridge motors.
 */

#include <ESP32_MCPWM.h>

/**
 * @brief Note: GPIO 34-39 on a standard ESP32 are input only!
 *  Tested with an ESP32-S3.
 */

// Left motor pins.
#define L_LPWM 37
#define L_RPWM 38
#define L_EN 39 ///< Set to -1 if your driver has no EN pin.

// Right motor pins.
#define R_LPWM 5
#define R_RPWM 6
#define R_EN 7 ///< Set to -1 if your driver has no EN pin.

MotorMCPWMConfig hwL{L_LPWM, L_RPWM, L_EN, MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, MCPWM0B};
MotorMCPWMConfig hwR{R_LPWM, R_RPWM, R_EN, MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM1A, MCPWM1B};

Motor motorL, motorR;

// If one motor spins opposite to your “forward”, flip its FWD constant.
static constexpr Dir L_FWD = Dir::CW;
static constexpr Dir R_FWD = Dir::CW;

// Convenience: Stop (coast) both motors.
static void stopCoast(uint32_t ms)
{
    motorL.setSpeed(0, L_FWD);
    motorR.setSpeed(0, R_FWD);
    motorL.setFreewheel();
    motorR.setFreewheel();
    delay(ms);
}

// Convenience: Drive both forward at % for ms.
static void driveForward(int pct, uint32_t ms)
{
    motorL.setSpeedPercent(pct, L_FWD);
    motorR.setSpeedPercent(pct, R_FWD);
    delay(ms);
}

// Convenience: drive both in reverse at % for ms.
static void driveReverse(int pct, uint32_t ms)
{
    motorL.setSpeedPercent(pct, IMotorDriver::changeDir(L_FWD));
    motorR.setSpeedPercent(pct, IMotorDriver::changeDir(R_FWD));
    delay(ms);
}

// Pivot left: left reverse, right forward (turn-in-place).
static void pivotLeft(int pct, uint32_t ms)
{
    motorL.setSpeedPercent(pct, IMotorDriver::changeDir(L_FWD));
    motorR.setSpeedPercent(pct, R_FWD);
    delay(ms);
}

// Pivot right: left forward, right reverse (turn-in-place).
static void pivotRight(int pct, uint32_t ms)
{
    motorL.setSpeedPercent(pct, L_FWD);
    motorR.setSpeedPercent(pct, IMotorDriver::changeDir(R_FWD));
    delay(ms);
}

void setup()
{
    Serial.begin(115200);
    delay(50);

    // Optional: keep the drive PWM quiet while running.
    hwL.pwm_freq_hz = 20000;
    hwR.pwm_freq_hz = 20000;

    // Setup both motors.
    motorL.setup(hwL);
    motorR.setup(hwR);
    motorL.start();
    motorR.start();
}

void loop()
{
    // Forward.
    Serial.println("Move forward...");
    driveForward(60, 2000);
    stopCoast(800);

    // Reverse.
    Serial.println("Move in reverse...");
    driveReverse(60, 2000);
    stopCoast(800);

    // Pivot left (turn-in-place).
    Serial.println("Pivot left...");
    pivotLeft(45, 1500);
    stopCoast(600);

    // Pivot right (turn-in-place).
    Serial.println("Pivot right...");
    pivotRight(45, 1500);
    stopCoast(600);

    delay(1000);
}