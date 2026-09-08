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
    if (motor.forceOutputs(true, true).error != MotorOperationError::NotSetup || !mock_hal::events.empty()) return 19;
    if (!motor.setup(hardware).ok())
        return 1;

    if (!motor.stop().ok()) return 5;
    mock_hal::events.clear();
    if (motor.forceOutputs(true, false).error != MotorOperationError::InvalidCommand ||
        !mock_hal::events.empty() || mock_hal::pin_levels[8] != LOW) return 6;
    if (!motor.start().ok()) return 7;
    const MotorOperationResult forced = motor.forceOutputs(true, false);
    if (!forced.ok())
        return 2;
    if (std::fabs(mock_hal::duties[MCPWM_UNIT_0][MCPWM_TIMER_0][MCPWM_OPR_A] - 100.0f) > 0.001f)
        return 3;
    if (std::fabs(mock_hal::duties[MCPWM_UNIT_0][MCPWM_TIMER_0][MCPWM_OPR_B]) > 0.001f)
        return 4;

    if (!forced.changed || motor.status().output_mode != MotorOutputMode::Commissioning ||
        !motor.status().enable_asserted || mock_hal::pin_levels[8] != HIGH) return 8;
    if (motor.forceOutputs(true, false).changed) return 9;
    mock_hal::duty_result = -1;
    if (motor.forceOutputs(false, true).ok() || motor.status().output_mode != MotorOutputMode::Uncertain) return 10;
    mock_hal::duty_result = ESP_OK;
    if (!motor.stop().ok() || motor.status().output_mode != MotorOutputMode::Disabled) return 11;
    if (!motor.start().ok() || !motor.drive(500, Dir::CW).ok() ||
        motor.status().output_mode != MotorOutputMode::DriveCW) return 12;
    MotorSafetyConfig safety{};
    safety.fault_gpio = 10;
    safety.fault_action = FaultAction::HardBrake;
    if (!motor.setup(hardware, MotorBehaviorConfig{}, safety, MotorCaptureConfig{}).ok()) return 13;
    mock_hal::pin_levels[10] = HIGH;
    mock_hal::invokeInterrupt(10);
    if (!motor.pollFaults().ok()) return 14;
    mock_hal::events.clear();
    if (motor.forceOutputs(true, true).error != MotorOperationError::FaultActive || !mock_hal::events.empty()) return 15;
    mock_hal::pin_levels[10] = LOW;
    mock_hal::invokeInterrupt(10);
    mock_hal::duty_result = -1;
    if (motor.clearFault().ok()) return 16;
    mock_hal::duty_result = ESP_OK;
    mock_hal::events.clear();
    if (motor.forceOutputs(true, true).error != MotorOperationError::FaultActive || !mock_hal::events.empty()) return 17;
    if (!motor.clearFault().ok() || !motor.forceOutputs(false, false).ok()) return 18;
    std::cout << "[PASS] commissioning API build gate\n";
    return 0;
}
