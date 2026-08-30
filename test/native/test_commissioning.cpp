/**
 * MIT License
 *
 * @brief Build-gate regression for the raw commissioning output API.
 *
 * @file test_commissioning.cpp
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-08-07
 * @copyright Copyright (c) 2026 Little Man Builds
 */

#include <HBridgeMotor.h>
#include <mock_hal.h>
#include <cmath>
#include <iostream>

int main()
{
    mock_hal::reset();
    MotorMCPWMConfig hardware{2, 3, 8, MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, MCPWM0B};
    HBridgeMotor motor;
    if (!motor.setup(hardware).ok())
        return 1;

    const MotorOperationResult forced = motor.forceOutputs(true, false);
    if (!forced.ok())
        return 2;
    if (std::fabs(mock_hal::duties[MCPWM_UNIT_0][MCPWM_TIMER_0][MCPWM_OPR_A] - 100.0f) > 0.001f)
        return 3;
    if (std::fabs(mock_hal::duties[MCPWM_UNIT_0][MCPWM_TIMER_0][MCPWM_OPR_B]) > 0.001f)
        return 4;

    std::cout << "[PASS] commissioning API build gate\n";
    return 0;
}
