/**
 * MIT License
 *
 * @file compile_smoke.ino
 * @brief Small target-compile gate for the public ESP32_MCPWM surface.
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-08-07
 * @copyright Copyright (c) 2026 Little Man Builds
 */

#include <ESP32_MCPWM.h>

MotorMCPWMConfig hardware{4, 5, 6, MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, MCPWM0B};
Motor motor;

void setup()
{
    const MotorSetupResult result = motor.setup(hardware);
    if (result.ok())
        (void)motor.coast();
}

void loop()
{
}
