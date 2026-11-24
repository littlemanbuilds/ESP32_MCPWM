/**
 * MIT License
 *
 * @brief Deterministic startup-state and control regression tests.
 *
 * @file test_startup_state.cpp
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-06-21
 * @copyright Copyright (c) 2026 Little Man Builds
 */

#include <HBridgeMotor.h>
#include <mock_hal.h>
#include <soc/gpio_sig_map.h>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
class TestFailure : public std::runtime_error
{
public:
    explicit TestFailure(const std::string &message) : std::runtime_error(message) {}
};

size_t g_assertions = 0;

void check(bool condition, const char *expression, int line)
{
    ++g_assertions;
    if (!condition)
        throw TestFailure("line " + std::to_string(line) + ": " + expression);
}

#define CHECK(expression) check((expression), #expression, __LINE__)

void checkNear(float actual, float expected, float tolerance, int line)
{
    ++g_assertions;
    if (std::fabs(actual - expected) > tolerance)
    {
        throw TestFailure("line " + std::to_string(line) + ": expected " +
                          std::to_string(expected) + ", got " + std::to_string(actual));
    }
}

#define CHECK_NEAR(actual, expected, tolerance) checkNear((actual), (expected), (tolerance), __LINE__)

struct OutputState
{
    int en;
    float a;
    float b;
    bool timer_active;
};

MotorMCPWMConfig hardware(int en_pin = 8)
{
    return MotorMCPWMConfig{2, 3, en_pin, MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, MCPWM0B};
}

MotorBehaviorConfig behavior(FreewheelMode mode, uint16_t dither_pwm = 256,
                             bool dither_coast_hi_z = false)
{
    return MotorBehaviorConfig{mode, 300, dither_pwm, 50, 100, dither_coast_hi_z};
}

MotorSafetyConfig faultConfig(FaultAction action, bool oneshot = true, int pin = 10)
{
    MotorSafetyConfig safety{};
    safety.fault_gpio = pin;
    safety.oneshot = oneshot;
    safety.fault_action = action;
    return safety;
}

MotorHardwareFaultConfig hardwareFaultConfig(HardwareFaultMode mode, int pin = 12)
{
    MotorHardwareFaultConfig fault{};
    fault.fault_gpio = pin;
    fault.mode = mode;
    fault.input = HardwareFaultInput::Fault0;
    fault.active_high = true;
    fault.action_a = HardwareFaultOutputAction::ForceLow;
    fault.action_b = HardwareFaultOutputAction::ForceLow;
    return fault;
}

OutputState outputState(int en_pin = 8)
{
    const auto en_it = mock_hal::pin_levels.find(en_pin);
    return {
        (en_it == mock_hal::pin_levels.end()) ? -1 : en_it->second,
        mock_hal::duties[MCPWM_UNIT_0][MCPWM_TIMER_0][MCPWM_OPR_A],
        mock_hal::duties[MCPWM_UNIT_0][MCPWM_TIMER_0][MCPWM_OPR_B],
        mock_hal::last_timer && mock_hal::last_timer->active};
}

void checkState(const OutputState &actual, const OutputState &expected)
{
    CHECK(actual.en == expected.en);
    CHECK_NEAR(actual.a, expected.a, 0.001f);
    CHECK_NEAR(actual.b, expected.b, 0.001f);
    CHECK(actual.timer_active == expected.timer_active);
}

void checkFaultActionState(FaultAction action, int en_pin = 8)
{
    switch (action)
    {
    case FaultAction::Coast:
        checkState(outputState(en_pin), {en_pin >= 0 ? LOW : -1, 0.0f, 0.0f, false});
        CHECK(mock_hal::running[MCPWM_UNIT_0][MCPWM_TIMER_0]);
        break;
    case FaultAction::DisableOutputs:
        checkState(outputState(en_pin), {en_pin >= 0 ? LOW : -1, 0.0f, 0.0f, false});
        CHECK(!mock_hal::running[MCPWM_UNIT_0][MCPWM_TIMER_0]);
        break;
    case FaultAction::HardBrake:
        checkState(outputState(en_pin), {en_pin >= 0 ? HIGH : -1, 100.0f, 100.0f, false});
        CHECK(mock_hal::running[MCPWM_UNIT_0][MCPWM_TIMER_0]);
        break;
    }
}

void checkOutputCommandsInhibited(HBridgeMotor &motor, FaultAction action)
{
    const auto checkAction = [&]() { checkFaultActionState(action); };

    motor.setSpeed(700, Dir::CCW);
    checkAction();
    motor.setSpeedPercent(60.0f, Dir::CW);
    checkAction();
    motor.setSpeed(0, Dir::CW);
    checkAction();
    motor.setFreewheel();
    checkAction();
    motor.applyFreewheel(FreewheelMode::HiZ_Awake);
    checkAction();
    motor.setHardBrake();
    checkAction();
    motor.softBrakeNow(512);
    checkAction();
    motor.forceOutputs(true, false);
    checkAction();
    motor.start();
    checkAction();

    // Configuration-only setters remain available without changing outputs.
    motor.setSoftBrakePWM(256);
    motor.setFreewheelMode(FreewheelMode::HiZ);
    checkAction();

    // stop() is deliberately allowed during a fault so callers can always
    // move the peripheral toward an electrically disabled state.
    const MotorOperationResult stopped = motor.stop();
    CHECK(stopped.ok());
    checkState(outputState(), {LOW, 0.0f, 0.0f, false});
    CHECK(!mock_hal::running[MCPWM_UNIT_0][MCPWM_TIMER_0]);
}

int findEvent(mock_hal::EventKind kind, int first = -1, int second = -1)
{
    for (size_t i = 0; i < mock_hal::events.size(); ++i)
    {
        const auto &event = mock_hal::events[i];
        if (event.kind == kind && (first < 0 || event.first == first) &&
            (second < 0 || event.second == second))
            return static_cast<int>(i);
    }
    return -1;
}

bool hasEvent(mock_hal::EventKind kind, int first = -1, int second = -1)
{
    return findEvent(kind, first, second) >= 0;
}

void testHiZSetupIsInactiveWithoutEnabledWindow()
{
    mock_hal::reset();
    HBridgeMotor motor;

    motor.setup(hardware(), behavior(FreewheelMode::HiZ));

    checkState(outputState(), {LOW, 0.0f, 0.0f, false});
    const int en_low = findEvent(mock_hal::EventKind::GpioLatch, 8, LOW);
    const int en_output = findEvent(mock_hal::EventKind::PinMode, 8, OUTPUT);
    CHECK(en_low >= 0);
    CHECK(en_output >= 0);
    CHECK(en_low < en_output);
    CHECK(!hasEvent(mock_hal::EventKind::DigitalWrite, 8, HIGH));
    CHECK(!hasEvent(mock_hal::EventKind::TimerStart));
    CHECK(hasEvent(mock_hal::EventKind::Duty, MCPWM_OPR_A));
    CHECK(hasEvent(mock_hal::EventKind::Duty, MCPWM_OPR_B));
}

void testSetupOverloadsApplyConfiguredFreewheel()
{
    OutputState default_state{};
    {
        mock_hal::reset();
        HBridgeMotor motor;
        motor.setup(hardware());
        default_state = outputState();
    }
    checkState(default_state, {LOW, 0.0f, 0.0f, false});

    OutputState behavior_state{};
    {
        mock_hal::reset();
        HBridgeMotor motor;
        motor.setup(hardware(), behavior(FreewheelMode::HiZ_Awake));
        behavior_state = outputState();
    }

    OutputState full_state{};
    {
        mock_hal::reset();
        HBridgeMotor motor;
        motor.setup(hardware(), behavior(FreewheelMode::HiZ_Awake),
                    MotorSafetyConfig{}, MotorCaptureConfig{});
        full_state = outputState();
    }
    checkState(behavior_state, {HIGH, 0.0f, 0.0f, false});
    checkState(full_state, behavior_state);

    mock_hal::reset();
    HBridgeMotor no_enable_motor;
    no_enable_motor.setup(hardware(-1));
    checkState(outputState(-1), {-1, 0.0f, 0.0f, false});
    CHECK(!hasEvent(mock_hal::EventKind::DigitalWrite));
}

void testHiZAwakeMatchesExplicitFreewheel()
{
    mock_hal::reset();
    HBridgeMotor motor;
    motor.setup(hardware(), behavior(FreewheelMode::HiZ_Awake));
    const OutputState setup_state = outputState();

    motor.setSpeed(700, Dir::CW);
    motor.setFreewheel();

    checkState(setup_state, {HIGH, 0.0f, 0.0f, false});
    checkState(outputState(), setup_state);
}

void testDitherBrakeAppliesConfiguredBehavior()
{
    {
        mock_hal::reset();
        HBridgeMotor motor;
        motor.setup(hardware(), behavior(FreewheelMode::DitherBrake));

        checkState(outputState(), {HIGH, 0.0f, 0.0f, true});
        CHECK(mock_hal::last_timer->timeout_us > 0);
        const int timer_create = findEvent(mock_hal::EventKind::TimerCreate);
        const int en_high = findEvent(mock_hal::EventKind::DigitalWrite, 8, HIGH);
        const int timer_start = findEvent(mock_hal::EventKind::TimerStart);
        CHECK(timer_create >= 0);
        CHECK(en_high > timer_create);
        CHECK(timer_start > en_high);
        mock_hal::events.clear();
        mock_hal::fireTimer(mock_hal::last_timer);
        checkState(outputState(), {HIGH, 100.0f, 100.0f, true});
        const int transition_disable = findEvent(mock_hal::EventKind::DigitalWrite, 8, LOW);
        const int transition_a = findEvent(mock_hal::EventKind::Duty, MCPWM_OPR_A);
        const int transition_b = findEvent(mock_hal::EventKind::Duty, MCPWM_OPR_B);
        const int transition_enable = findEvent(mock_hal::EventKind::DigitalWrite, 8, HIGH);
        CHECK(transition_disable >= 0);
        CHECK(transition_disable < transition_a);
        CHECK(transition_a < transition_b);
        CHECK(transition_b < transition_enable);
    }

    {
        mock_hal::reset();
        HBridgeMotor hi_z_coast_motor;
        hi_z_coast_motor.setup(hardware(), behavior(FreewheelMode::DitherBrake, 256, true));
        checkState(outputState(), {LOW, 0.0f, 0.0f, true});
    }

    {
        mock_hal::reset();
        HBridgeMotor zero_dither_motor;
        zero_dither_motor.setup(hardware(), behavior(FreewheelMode::DitherBrake, 0, true));
        checkState(outputState(), {LOW, 0.0f, 0.0f, false});
    }

    mock_hal::reset();
    HBridgeMotor full_dither_motor;
    full_dither_motor.setup(hardware(), behavior(FreewheelMode::DitherBrake, 1023, true));
    checkState(outputState(), {HIGH, 100.0f, 100.0f, false});
}

void testZeroDriveIsExplicitAndDitherZeroUsesConfiguredCoast()
{
    mock_hal::reset();
    HBridgeMotor hi_z_motor;
    hi_z_motor.setup(hardware(), behavior(FreewheelMode::DitherBrake, 256, true));
    CHECK(mock_hal::last_timer->active);

    hi_z_motor.setSoftBrakePWM(0);
    checkState(outputState(), {LOW, 0.0f, 0.0f, false});
    hi_z_motor.softBrakeNow(0);
    checkState(outputState(), {LOW, 0.0f, 0.0f, false});

    // A zero drive request is not a hidden stop-mode selector in v2.
    hi_z_motor.setSpeed(700, Dir::CW);
    const OutputState before_zero = outputState();
    const MotorOperationResult zero_counts = hi_z_motor.setSpeed(0, Dir::CW);
    CHECK(!zero_counts.ok());
    CHECK(zero_counts.error == MotorOperationError::InvalidCommand);
    checkState(outputState(), before_zero);
    const MotorOperationResult zero_percent = hi_z_motor.setSpeedPercent(0.0f, Dir::CCW);
    CHECK(!zero_percent.ok());
    CHECK(zero_percent.error == MotorOperationError::InvalidCommand);
    checkState(outputState(), before_zero);

    CHECK(hi_z_motor.setup(MotorMCPWMConfig{}).error == MotorSetupError::InvalidPwmPin);
    mock_hal::reset();
    HBridgeMotor freewheel_motor;
    freewheel_motor.setup(hardware(), behavior(FreewheelMode::DitherBrake, 0, true));
    freewheel_motor.setSpeed(700, Dir::CW);
    freewheel_motor.setFreewheel();
    checkState(outputState(), {LOW, 0.0f, 0.0f, false});

    CHECK(freewheel_motor.setup(MotorMCPWMConfig{}).error == MotorSetupError::InvalidPwmPin);
    mock_hal::reset();
    HBridgeMotor enabled_coast_motor;
    enabled_coast_motor.setup(hardware(), behavior(FreewheelMode::DitherBrake, 0, false));
    checkState(outputState(), {HIGH, 0.0f, 0.0f, false});
    enabled_coast_motor.softBrakeNow(0);
    checkState(outputState(), {HIGH, 0.0f, 0.0f, false});
}

void testSameFreewheelModeIsTrueNoOp()
{
    mock_hal::reset();
    HBridgeMotor motor;
    motor.setup(hardware(), behavior(FreewheelMode::DitherBrake, 256, true));
    const OutputState before = outputState();
    const int64_t timeout_before = mock_hal::last_timer->timeout_us;
    mock_hal::events.clear();

    motor.setFreewheelMode(FreewheelMode::DitherBrake);

    checkState(outputState(), before);
    CHECK(mock_hal::last_timer->timeout_us == timeout_before);
    CHECK(mock_hal::events.empty());

    CHECK(motor.setup(MotorMCPWMConfig{}).error == MotorSetupError::InvalidPwmPin);
    mock_hal::reset();
    HBridgeMotor unconfigured;
    unconfigured.setFreewheelMode(FreewheelMode::HiZ);
    CHECK(mock_hal::events.empty());
    CHECK(!unconfigured.isSetupComplete());
}

void testChangedFreewheelModeStopsDitherInCoast()
{
    mock_hal::reset();
    HBridgeMotor motor;
    motor.setup(hardware(), behavior(FreewheelMode::DitherBrake, 256, true));
    mock_hal::fireTimer(mock_hal::last_timer);
    checkState(outputState(), {HIGH, 100.0f, 100.0f, true});

    motor.setFreewheelMode(FreewheelMode::HiZ);
    checkState(outputState(), {LOW, 0.0f, 0.0f, false});
}

void checkDitherPeriod(uint16_t pwm, uint16_t min_phase_us)
{
    mock_hal::reset();
    HBridgeMotor motor;
    MotorBehaviorConfig dither{FreewheelMode::DitherBrake, 300, pwm, 0,
                               min_phase_us, true};
    motor.setup(hardware(), dither);
    CHECK(motor.isSetupComplete());
    CHECK(mock_hal::last_timer->active);
    const int64_t coast_us = mock_hal::last_timer->timeout_us;
    mock_hal::fireTimer(mock_hal::last_timer);
    const int64_t brake_us = mock_hal::last_timer->timeout_us;

    CHECK(brake_us > 0);
    CHECK(coast_us > 0);
    CHECK(brake_us + coast_us == 1000000 / dither.soft_brake_hz);
}

void testDitherPeriodIsPreserved()
{
    checkDitherPeriod(1, 50);
    checkDitherPeriod(1022, 50);
    checkDitherPeriod(1, 2000); // The effective minimum is reduced so the period still fits.
}

int capture_count = 0;
void onCapture(uint32_t, void *) { ++capture_count; }

void testRepeatedSetupCleansPreviousState()
{
    mock_hal::reset();
    capture_count = 0;
    HBridgeMotor motor;

    MotorSafetyConfig old_safety{};
    old_safety.fault_gpio = 10;
    MotorCaptureConfig old_capture{};
    old_capture.cap_gpio = 11;
    old_capture.on_capture = &onCapture;
    motor.setup(hardware(8), behavior(FreewheelMode::DitherBrake), old_safety, old_capture);
    CHECK(mock_hal::last_timer->active);

    mock_hal::pin_levels[10] = HIGH;
    mock_hal::invokeInterrupt(10);
    CHECK(motor.hasFault());
    mock_hal::micros_value = 100;
    mock_hal::invokeInterrupt(11);
    mock_hal::micros_value = 200;
    mock_hal::invokeInterrupt(11);
    CHECK(capture_count == 1);

    MotorSafetyConfig new_safety{};
    new_safety.fault_gpio = 12;
    MotorCaptureConfig new_capture{};
    new_capture.cap_gpio = 13;
    new_capture.on_capture = &onCapture;
    motor.setup(hardware(9), behavior(FreewheelMode::HiZ), new_safety, new_capture);

    checkState(outputState(9), {LOW, 0.0f, 0.0f, false});
    CHECK(mock_hal::pin_levels[8] == LOW);
    CHECK(!motor.hasFault());
    CHECK(mock_hal::timer_create_count == 1);
    CHECK(mock_hal::interrupts.find(10) == mock_hal::interrupts.end());
    CHECK(mock_hal::interrupts.find(11) == mock_hal::interrupts.end());
    CHECK(mock_hal::interrupts.find(12) != mock_hal::interrupts.end());
    CHECK(mock_hal::interrupts.find(13) != mock_hal::interrupts.end());
    CHECK(hasEvent(mock_hal::EventKind::McpwmStop, MCPWM_UNIT_0, MCPWM_TIMER_0));

    mock_hal::micros_value = 300;
    mock_hal::invokeInterrupt(13);
    CHECK(capture_count == 1);
}

void testRepeatedSetupDisablesPreviousDeadtime()
{
    mock_hal::reset();
    HBridgeMotor motor;
    MotorMCPWMConfig with_deadtime = hardware();
    with_deadtime.use_deadtime = true;
    motor.setup(with_deadtime, behavior(FreewheelMode::HiZ));
    CHECK(hasEvent(mock_hal::EventKind::DeadtimeEnable));

    mock_hal::events.clear();
    motor.setup(hardware(), behavior(FreewheelMode::HiZ));
    CHECK(hasEvent(mock_hal::EventKind::DeadtimeDisable));
    CHECK(!hasEvent(mock_hal::EventKind::DeadtimeEnable));
    CHECK(motor.isSetupComplete());
}

void testDriveBrakeFreewheelAndLifecycle()
{
    mock_hal::reset();
    HBridgeMotor motor;
    motor.setup(hardware(), behavior(FreewheelMode::HiZ));

    motor.setSpeed(512, Dir::CW);
    OutputState state = outputState();
    CHECK(state.en == HIGH);
    CHECK_NEAR(state.a, 512.0f * 100.0f / 1023.0f, 0.001f);
    CHECK_NEAR(state.b, 0.0f, 0.001f);

    motor.setFreewheel();
    checkState(outputState(), {LOW, 0.0f, 0.0f, false});

    motor.softBrakeNow(512);
    CHECK(mock_hal::last_timer->active);
    mock_hal::fireTimer(mock_hal::last_timer);
    checkState(outputState(), {HIGH, 100.0f, 100.0f, true});

    motor.setHardBrake();
    checkState(outputState(), {HIGH, 100.0f, 100.0f, false});

    motor.stop();
    checkState(outputState(), {LOW, 0.0f, 0.0f, false});
    CHECK(!mock_hal::running[MCPWM_UNIT_0][MCPWM_TIMER_0]);
    motor.start();
    CHECK(mock_hal::running[MCPWM_UNIT_0][MCPWM_TIMER_0]);

    motor.setSpeed(1023, Dir::CCW);
    checkState(outputState(), {HIGH, 0.0f, 100.0f, false});
}

void testDestructorDisablesOutputs()
{
    mock_hal::reset();
    {
        HBridgeMotor motor;
        motor.setup(hardware(), behavior(FreewheelMode::HiZ_Awake),
                    faultConfig(FaultAction::HardBrake), MotorCaptureConfig{});
        motor.setSpeed(700, Dir::CW);
        CHECK(outputState().a > 0.0f);
    }

    checkState(outputState(), {LOW, 0.0f, 0.0f, false});
    CHECK(!mock_hal::running[MCPWM_UNIT_0][MCPWM_TIMER_0]);
    CHECK(mock_hal::interrupts.find(10) == mock_hal::interrupts.end());
    CHECK(mock_hal::last_timer == nullptr);
}

int fault_callback_count = 0;
bool last_fault_active = false;
void onFault(bool active, void *)
{
    ++fault_callback_count;
    last_fault_active = active;
}

void testDefaultFaultDisablesOutputsAndLatches()
{
    mock_hal::reset();
    fault_callback_count = 0;
    last_fault_active = false;
    HBridgeMotor motor;
    motor.setFaultCallback(&onFault, nullptr);

    MotorSafetyConfig safety{};
    CHECK(safety.fault_action == FaultAction::DisableOutputs);
    safety.fault_gpio = 10;
    motor.setup(hardware(), behavior(FreewheelMode::HiZ), safety, MotorCaptureConfig{});
    motor.setSpeed(700, Dir::CW);
    mock_hal::pin_levels[10] = HIGH;
    mock_hal::invokeInterrupt(10);
    CHECK(motor.hasFault());

    OutputState state = outputState();
    CHECK(state.en == HIGH);
    CHECK(state.a > 0.0f && state.a < 100.0f);
    CHECK_NEAR(state.b, 0.0f, 0.001f);

    motor.pollFaults();
    checkFaultActionState(FaultAction::DisableOutputs);
    CHECK(fault_callback_count == 1);
    CHECK(last_fault_active);
    motor.pollFaults();
    CHECK(fault_callback_count == 1);

    checkOutputCommandsInhibited(motor, FaultAction::DisableOutputs);

    // A still-active input cannot be cleared without a new edge.
    motor.clearFault();
    CHECK(motor.hasFault());
    checkState(outputState(), {LOW, 0.0f, 0.0f, false});

    mock_hal::pin_levels[10] = LOW;
    mock_hal::invokeInterrupt(10);
    motor.clearFault();
    CHECK(!motor.hasFault());
    checkState(outputState(), {LOW, 0.0f, 0.0f, false});
    CHECK(mock_hal::running[MCPWM_UNIT_0][MCPWM_TIMER_0]);

    // Clear does not restore the previous command; a new request is required.
    motor.clearFault();
    checkState(outputState(), {LOW, 0.0f, 0.0f, false});
    motor.setSpeed(400, Dir::CCW);
    state = outputState();
    CHECK(state.en == HIGH);
    CHECK_NEAR(state.a, 0.0f, 0.001f);
    CHECK(state.b > 0.0f);
}

void testOneShotCoastFaultAndLatch()
{
    mock_hal::reset();
    HBridgeMotor motor;
    motor.setup(hardware(), behavior(FreewheelMode::HiZ),
                faultConfig(FaultAction::Coast), MotorCaptureConfig{});
    CHECK(motor.hasEnableControl());
    motor.setSpeed(700, Dir::CW);

    mock_hal::events.clear();
    mock_hal::pin_levels[10] = HIGH;
    mock_hal::invokeInterrupt(10);
    CHECK(motor.hasFault());

    // The software fallback remains unchanged until task-context polling.
    OutputState state = outputState();
    CHECK(state.en == HIGH);
    CHECK(state.a > 0.0f);
    motor.pollFaults();
    checkFaultActionState(FaultAction::Coast);

    const int en_low = findEvent(mock_hal::EventKind::DigitalWrite, 8, LOW);
    const int duty_a = findEvent(mock_hal::EventKind::Duty, MCPWM_OPR_A);
    CHECK(en_low >= 0);
    CHECK(duty_a >= 0);
    CHECK(en_low < duty_a);

    checkOutputCommandsInhibited(motor, FaultAction::Coast);
    mock_hal::pin_levels[10] = LOW;
    mock_hal::invokeInterrupt(10);
    motor.clearFault();
    CHECK(!motor.hasFault());
    checkState(outputState(), {LOW, 0.0f, 0.0f, false});
}

void testOneShotDisableOutputsFaultAndLatch()
{
    mock_hal::reset();
    HBridgeMotor motor;
    motor.setup(hardware(), behavior(FreewheelMode::HiZ),
                faultConfig(FaultAction::DisableOutputs), MotorCaptureConfig{});
    motor.setSpeed(700, Dir::CCW);

    mock_hal::events.clear();
    mock_hal::pin_levels[10] = HIGH;
    mock_hal::invokeInterrupt(10);
    motor.pollFaults();
    CHECK(motor.hasFault());
    checkFaultActionState(FaultAction::DisableOutputs);

    const int en_low = findEvent(mock_hal::EventKind::DigitalWrite, 8, LOW);
    const int duty_b = findEvent(mock_hal::EventKind::Duty, MCPWM_OPR_B);
    const int stopped = findEvent(mock_hal::EventKind::McpwmStop, MCPWM_UNIT_0, MCPWM_TIMER_0);
    CHECK(en_low >= 0);
    CHECK(duty_b >= 0);
    CHECK(stopped >= 0);
    CHECK(en_low < duty_b);
    CHECK(duty_b < stopped);

    checkOutputCommandsInhibited(motor, FaultAction::DisableOutputs);
    mock_hal::pin_levels[10] = LOW;
    mock_hal::invokeInterrupt(10);
    motor.clearFault();
    CHECK(!motor.hasFault());
    checkState(outputState(), {LOW, 0.0f, 0.0f, false});
    CHECK(mock_hal::running[MCPWM_UNIT_0][MCPWM_TIMER_0]);
}

void testFaultStopsDither()
{
    mock_hal::reset();
    HBridgeMotor motor;
    motor.setup(hardware(), behavior(FreewheelMode::DitherBrake),
                faultConfig(FaultAction::Coast), MotorCaptureConfig{});
    CHECK(mock_hal::last_timer->active);

    mock_hal::pin_levels[10] = HIGH;
    mock_hal::invokeInterrupt(10);
    CHECK(mock_hal::last_timer->active);
    motor.pollFaults();
    CHECK(!mock_hal::last_timer->active);
    checkFaultActionState(FaultAction::Coast);

    mock_hal::fireTimer(mock_hal::last_timer);
    checkFaultActionState(FaultAction::Coast);
}

void testCoastPreservesStoppedMcpwmState()
{
    mock_hal::reset();
    HBridgeMotor motor;
    motor.setup(hardware(), behavior(FreewheelMode::HiZ),
                faultConfig(FaultAction::Coast), MotorCaptureConfig{});
    motor.stop();
    CHECK(!mock_hal::running[MCPWM_UNIT_0][MCPWM_TIMER_0]);

    mock_hal::pin_levels[10] = HIGH;
    mock_hal::invokeInterrupt(10);
    motor.pollFaults();
    checkState(outputState(), {LOW, 0.0f, 0.0f, false});
    CHECK(!mock_hal::running[MCPWM_UNIT_0][MCPWM_TIMER_0]);
}

void testClearPreservesPendingFaultNotification()
{
    mock_hal::reset();
    fault_callback_count = 0;
    HBridgeMotor motor;
    motor.setFaultCallback(&onFault, nullptr);
    motor.setup(hardware(), behavior(FreewheelMode::HiZ),
                faultConfig(FaultAction::HardBrake), MotorCaptureConfig{});
    motor.setSpeed(600, Dir::CW);

    mock_hal::pin_levels[10] = HIGH;
    mock_hal::invokeInterrupt(10);
    mock_hal::pin_levels[10] = LOW;
    mock_hal::invokeInterrupt(10);
    motor.clearFault();
    CHECK(!motor.hasFault());
    checkState(outputState(), {LOW, 0.0f, 0.0f, false});

    motor.pollFaults();
    CHECK(fault_callback_count == 1);
    checkState(outputState(), {LOW, 0.0f, 0.0f, false});
}

void testLevelFollowAllFaultActions()
{
    const FaultAction actions[]{FaultAction::Coast, FaultAction::DisableOutputs,
                                FaultAction::HardBrake};

    for (const FaultAction action : actions)
    {
        mock_hal::reset();
        fault_callback_count = 0;
        last_fault_active = false;
        HBridgeMotor motor;
        motor.setFaultCallback(&onFault, nullptr);
        motor.setup(hardware(), behavior(FreewheelMode::HiZ),
                    faultConfig(action, false), MotorCaptureConfig{});
        motor.setSpeed(600, Dir::CW);

        mock_hal::pin_levels[10] = HIGH;
        mock_hal::invokeInterrupt(10);
        motor.pollFaults();
        CHECK(motor.hasFault());
        checkFaultActionState(action);
        CHECK(fault_callback_count == 1);
        CHECK(last_fault_active);

        motor.pollFaults();
        CHECK(fault_callback_count == 1);

        mock_hal::pin_levels[10] = LOW;
        mock_hal::invokeInterrupt(10);
        CHECK(motor.hasFault()); // Recovery remains inhibited until quiet polling succeeds.
        motor.pollFaults();
        CHECK(!motor.hasFault());
        checkState(outputState(), {LOW, 0.0f, 0.0f, false});
        CHECK(mock_hal::running[MCPWM_UNIT_0][MCPWM_TIMER_0]);
        CHECK(fault_callback_count == 2);
        CHECK(!last_fault_active);

        motor.pollFaults();
        CHECK(fault_callback_count == 2);
        motor.setSpeed(300, Dir::CCW);
        CHECK(outputState().b > 0.0f);
    }
}

void testActiveFaultAtSetupAllActions()
{
    const FaultAction actions[]{FaultAction::Coast, FaultAction::DisableOutputs,
                                FaultAction::HardBrake};

    for (const FaultAction action : actions)
    {
        mock_hal::reset();
        fault_callback_count = 0;
        last_fault_active = false;
        mock_hal::pin_levels[10] = HIGH;
        HBridgeMotor motor;
        motor.setFaultCallback(&onFault, nullptr);
        motor.setup(hardware(), behavior(FreewheelMode::HiZ_Awake),
                    faultConfig(action), MotorCaptureConfig{});

        CHECK(motor.hasFault());
        checkFaultActionState(action);
        if (action != FaultAction::HardBrake)
            CHECK(!hasEvent(mock_hal::EventKind::DigitalWrite, 8, HIGH));
        CHECK(fault_callback_count == 0);
        motor.pollFaults();
        checkFaultActionState(action);
        CHECK(fault_callback_count == 1);
        CHECK(last_fault_active);
    }

    mock_hal::reset();
    HBridgeMotor active_low_motor;
    MotorSafetyConfig active_low = faultConfig(FaultAction::Coast);
    active_low.fault_active_high = false;
    active_low_motor.setup(hardware(), behavior(FreewheelMode::HiZ_Awake),
                           active_low, MotorCaptureConfig{});
    CHECK(active_low_motor.hasFault());
    checkFaultActionState(FaultAction::Coast);
}

void testEnAbsentFaultCapabilities()
{
    const FaultAction actions[]{FaultAction::Coast, FaultAction::DisableOutputs};

    for (const FaultAction action : actions)
    {
        mock_hal::reset();
        HBridgeMotor motor;
        motor.setup(hardware(-1), behavior(FreewheelMode::HiZ_Awake),
                    faultConfig(action), MotorCaptureConfig{});
        CHECK(!motor.hasEnableControl());
        motor.setSpeed(1023, Dir::CW);
        mock_hal::pin_levels[10] = HIGH;
        mock_hal::invokeInterrupt(10);
        motor.pollFaults();
        checkFaultActionState(action, -1);
        CHECK(!hasEvent(mock_hal::EventKind::DigitalWrite));

        mock_hal::pin_levels[10] = LOW;
        mock_hal::invokeInterrupt(10);
        motor.clearFault();
        checkState(outputState(-1), {-1, 0.0f, 0.0f, false});
        CHECK(mock_hal::running[MCPWM_UNIT_0][MCPWM_TIMER_0]);
    }
}

void testFaultDisabledAndLegacyConfigCompatibility()
{
    MotorSafetyConfig legacy{10, true, false};
    CHECK(legacy.fault_gpio == 10);
    CHECK(!legacy.oneshot);
    CHECK(legacy.fault_action == FaultAction::DisableOutputs);
    MotorSafetyConfig configured{11, false, true, FaultAction::Coast};
    CHECK(configured.fault_gpio == 11);
    CHECK(!configured.fault_active_high);
    CHECK(configured.oneshot);
    CHECK(configured.fault_action == FaultAction::Coast);

    mock_hal::reset();
    HBridgeMotor motor;
    MotorSafetyConfig disabled{};
    disabled.fault_action = FaultAction::DisableOutputs;
    motor.setup(hardware(), behavior(FreewheelMode::HiZ), disabled, MotorCaptureConfig{});
    CHECK(mock_hal::interrupts.find(10) == mock_hal::interrupts.end());
    CHECK(!motor.hasFault());
    motor.setSpeed(500, Dir::CW);
    CHECK(outputState().a > 0.0f);
}

void testRuntimeFrequencyReconfiguration()
{
    mock_hal::reset();
    HBridgeMotor unconfigured;
    CHECK(!unconfigured.reconfigureFrequency(20000));
    CHECK(!unconfigured.reconfigureFrequency(0));
    CHECK(mock_hal::events.empty());

    mock_hal::reset();
    HBridgeMotor motor;
    motor.setup(hardware(), behavior(FreewheelMode::DitherBrake, 256, true));
    CHECK(mock_hal::last_timer->active);
    CHECK(!motor.reconfigureFrequency(0));
    CHECK(mock_hal::last_timer->active);
    CHECK(!motor.reconfigureFrequency(1000001));
    CHECK(mock_hal::last_timer->active);

    const MotorOperationResult changed = motor.reconfigureFrequency(25000);
    CHECK(changed.ok());
    CHECK(changed.changed);
    checkState(outputState(), {LOW, 0.0f, 0.0f, true});
    CHECK(motor.status().output_mode == MotorOutputMode::DitherBrake);

    const MotorOperationResult unchanged = motor.reconfigureFrequency(25000);
    CHECK(unchanged.ok());
    CHECK(!unchanged.changed);
    CHECK(mock_hal::last_timer->active);

    mock_hal::frequency_result = -1;
    const MotorOperationResult failed = motor.reconfigureFrequency(30000);
    CHECK(!failed.ok());
    CHECK(failed.changed);
    checkState(outputState(), {LOW, 0.0f, 0.0f, false});
    CHECK(motor.status().output_mode == MotorOutputMode::Disabled);
    CHECK(!motor.status().mcpwm_running);
    CHECK(motor.status().pwm_frequency_hz == 25000);
}

void testDriveFrequencyReconfigurationPublishesQuietState()
{
    for (const Dir direction : {Dir::CW, Dir::CCW})
    {
        mock_hal::reset();
        HBridgeMotor motor;
        CHECK(motor.setup(hardware(), behavior(FreewheelMode::HiZ)).ok());
        CHECK(motor.drive(600, direction).ok());

        const MotorOperationResult changed = motor.reconfigureFrequency(25000);
        CHECK(changed.ok());
        CHECK(changed.changed);
        CHECK(motor.status().output_mode == MotorOutputMode::Coast);
        checkState(outputState(), {LOW, 0.0f, 0.0f, false});
    }
}

void testFrequencyReconfigurationFailureStatusIsTruthful()
{
    mock_hal::reset();
    HBridgeMotor contained;
    CHECK(contained.setup(hardware(), behavior(FreewheelMode::HiZ)).ok());
    CHECK(contained.drive(600, Dir::CW).ok());
    mock_hal::frequency_result = -1;
    const MotorOperationResult frequency_failed =
        contained.reconfigureFrequency(25000);
    CHECK(frequency_failed.error == MotorOperationError::FrequencyChangeFailed);
    CHECK(frequency_failed.changed);
    CHECK(contained.status().output_mode == MotorOutputMode::Disabled);
    CHECK(contained.status().pwm_frequency_hz == 20000);
    CHECK(contained.readback().frequency_hz == 0U);
    checkState(outputState(), {LOW, 0.0f, 0.0f, false});

    CHECK(contained.setup(MotorMCPWMConfig{}).error == MotorSetupError::InvalidPwmPin);
    mock_hal::reset();
    HBridgeMotor uncertain;
    CHECK(uncertain.setup(hardware(), behavior(FreewheelMode::HiZ)).ok());
    CHECK(uncertain.drive(600, Dir::CCW).ok());
    mock_hal::duty_result = -1;
    const MotorOperationResult output_failed = uncertain.reconfigureFrequency(25000);
    CHECK(output_failed.error == MotorOperationError::HardwareWriteFailed);
    CHECK(output_failed.changed);
    CHECK(uncertain.status().output_mode == MotorOutputMode::Disabled);
    CHECK(!uncertain.status().mcpwm_running);
    CHECK(mock_hal::pwm_routes.empty());
    CHECK(!uncertain.drive(500, Dir::CW).ok());

    mock_hal::duty_result = ESP_OK;
    CHECK(uncertain.stop().ok());
    CHECK(uncertain.start().ok());
    CHECK(uncertain.drive(500, Dir::CW).ok());
    CHECK(uncertain.status().output_mode == MotorOutputMode::DriveCW);
}

void expectSetupError(const MotorMCPWMConfig &hw, const MotorBehaviorConfig &beh,
                      const MotorSafetyConfig &safety, const MotorCaptureConfig &capture,
                      MotorSetupError expected)
{
    mock_hal::reset();
    HBridgeMotor motor;
    motor.setup(hw, beh, safety, capture);
    CHECK(!motor.isSetupComplete());
    CHECK(motor.getLastSetupError() == expected);
    CHECK(!hasEvent(mock_hal::EventKind::McpwmInit));
    CHECK(!hasEvent(mock_hal::EventKind::TimerCreate));
}

void testInvalidHardwareConfigurationFailsSafely()
{
    MotorMCPWMConfig hw = hardware();
    hw.lpwm_pin = -1;
    expectSetupError(hw, behavior(FreewheelMode::HiZ), {}, {}, MotorSetupError::InvalidPwmPin);

    hw = hardware();
    hw.rpwm_pin = hw.lpwm_pin;
    expectSetupError(hw, behavior(FreewheelMode::HiZ), {}, {}, MotorSetupError::DuplicatePwmPin);

    hw = hardware();
    hw.en_pin = hw.lpwm_pin;
    expectSetupError(hw, behavior(FreewheelMode::HiZ), {}, {}, MotorSetupError::PinConflict);

    MotorSafetyConfig safety{};
    safety.fault_gpio = 3;
    expectSetupError(hardware(), behavior(FreewheelMode::HiZ), safety, {},
                     MotorSetupError::PinConflict);

    hw = hardware();
    hw.pwm_freq_hz = 0;
    expectSetupError(hw, behavior(FreewheelMode::HiZ), {}, {},
                     MotorSetupError::InvalidPwmFrequency);

    MotorBehaviorConfig invalid_dither = behavior(FreewheelMode::HiZ);
    invalid_dither.soft_brake_hz = 0;
    expectSetupError(hardware(), invalid_dither, {}, {}, MotorSetupError::InvalidDitherConfig);

    hw = hardware();
    hw.input_max = 0;
    expectSetupError(hw, behavior(FreewheelMode::HiZ), {}, {}, MotorSetupError::InvalidInputRange);
}

void testFailedRepeatedSetupClearsPreviousConfiguration()
{
    mock_hal::reset();
    mock_hal::pin_levels[12] = LOW;
    HBridgeMotor motor;

    MotorSafetyConfig safety = faultConfig(FaultAction::DisableOutputs);
    MotorCaptureConfig capture{};
    capture.cap_gpio = 11;
    MotorHardwareFaultConfig hardware_fault = hardwareFaultConfig(HardwareFaultMode::OneShot);

    CHECK(motor.setup(hardware(), behavior(FreewheelMode::HiZ), safety,
                      capture, hardware_fault)
              .ok());
    CHECK(motor.status().software_fault_configured);
    CHECK(motor.status().hardware_fault_configured);

    MotorMCPWMConfig invalid = hardware();
    invalid.lpwm_pin = -1;
    const MotorSetupResult failed = motor.setup(invalid, behavior(FreewheelMode::HiZ),
                                                safety, capture, hardware_fault);

    CHECK(!failed.ok());
    CHECK(failed.error == MotorSetupError::InvalidPwmPin);
    CHECK(!failed.software_fault_enabled);
    CHECK(!failed.hardware_fault_enabled);
    CHECK(!motor.isSetupComplete());
    CHECK(!motor.hasFault());

    const MotorDriverStatus status = motor.status();
    CHECK(!status.setup_complete);
    CHECK(!status.software_fault_configured);
    CHECK(!status.hardware_fault_configured);
    CHECK(status.output_mode == MotorOutputMode::Unconfigured);
    CHECK(!motor.readback().valid);
    CHECK(mock_hal::interrupts.find(10) == mock_hal::interrupts.end());
    CHECK(mock_hal::interrupts.find(11) == mock_hal::interrupts.end());
    CHECK(mock_hal::interrupts.find(12) == mock_hal::interrupts.end());
}

void testSetupHardwareFailuresRemainControlled()
{
    mock_hal::reset();
    mock_hal::mcpwm_init_result = -1;
    HBridgeMotor hardware_failure;
    hardware_failure.setup(hardware(), behavior(FreewheelMode::HiZ));
    CHECK(!hardware_failure.isSetupComplete());
    CHECK(hardware_failure.getLastSetupError() == MotorSetupError::HardwareInitFailed);
    CHECK(mock_hal::pin_levels[8] == LOW);
    CHECK(!mock_hal::running[MCPWM_UNIT_0][MCPWM_TIMER_0]);

    mock_hal::reset();
    mock_hal::timer_create_result = -1;
    HBridgeMotor timer_failure;
    timer_failure.setup(hardware(), behavior(FreewheelMode::HiZ));
    CHECK(!timer_failure.isSetupComplete());
    CHECK(timer_failure.getLastSetupError() == MotorSetupError::TimerInitFailed);
    CHECK(mock_hal::pin_levels[8] == LOW);
    CHECK(!mock_hal::running[MCPWM_UNIT_0][MCPWM_TIMER_0]);

    mock_hal::reset();
    mock_hal::timer_start_result = -1;
    HBridgeMotor timer_start_failure;
    timer_start_failure.setup(hardware(), behavior(FreewheelMode::DitherBrake, 256, true));
    CHECK(!timer_start_failure.isSetupComplete());
    CHECK(timer_start_failure.getLastSetupError() == MotorSetupError::TimerInitFailed);
    checkState(outputState(), {LOW, 0.0f, 0.0f, false});
    CHECK(!mock_hal::running[MCPWM_UNIT_0][MCPWM_TIMER_0]);
}

enum class RaceAction
{
    Drive,
    Coast,
    Disable,
    HardBrake
};

HBridgeMotor *race_motor = nullptr;
RaceAction race_action = RaceAction::Drive;
MotorOperationResult race_result{};
std::size_t race_return_event_count = 0;

void issueRaceCommand()
{
    switch (race_action)
    {
    case RaceAction::Drive:
        race_result = race_motor->drive(700, Dir::CW);
        break;
    case RaceAction::Coast:
        race_result = race_motor->coast();
        break;
    case RaceAction::Disable:
        race_result = race_motor->disableOutputs();
        break;
    case RaceAction::HardBrake:
        race_result = race_motor->setHardBrake();
        break;
    }
    race_return_event_count = mock_hal::events.size();
}

void armRaceCommandOnDuty()
{
    mock_hal::duty_hook = &issueRaceCommand;
}

void testStaleDitherCallbackCannotOverwriteDrive()
{
    const RaceAction actions[]{RaceAction::Drive, RaceAction::Coast,
                               RaceAction::Disable, RaceAction::HardBrake};
    for (const RaceAction action : actions)
    {
        mock_hal::reset();
        HBridgeMotor motor;
        race_motor = &motor;
        race_action = action;
        CHECK(motor.setup(hardware(), behavior(FreewheelMode::DitherBrake, 256, true)).ok());
        mock_hal::duty_hook = &issueRaceCommand;

        mock_hal::fireTimer(mock_hal::last_timer);
        CHECK(race_result.ok());

        // The mock hook runs immediately after the first old-generation duty
        // commit. No second old brake write may occur after the newer command
        // has returned, even if final-state restoration would hide it.
        for (std::size_t i = race_return_event_count; i < mock_hal::events.size(); ++i)
        {
            const mock_hal::Event &event = mock_hal::events[i];
            CHECK(event.kind != mock_hal::EventKind::Duty ||
                  std::fabs(event.value - 100.0f) > 0.001f);
        }
        CHECK((action == RaceAction::Coast) == mock_hal::last_timer->active);
    }
    race_motor = nullptr;
}

void testStaleDitherSchedulingPreservesNewerDiagnostics()
{
    mock_hal::reset();
    HBridgeMotor motor;
    race_motor = &motor;
    race_action = RaceAction::Drive;
    CHECK(motor.setup(hardware(), behavior(FreewheelMode::DitherBrake, 256, true)).ok());

    // Resume the old callback's timer start only after a newer drive has
    // completed, reproducing the post-phase-commit scheduling window.
    mock_hal::timer_start_hook = &issueRaceCommand;
    mock_hal::fireTimer(mock_hal::last_timer);

    CHECK(race_result.ok());
    const MotorDriverStatus status = motor.status();
    CHECK(!status.dither_active);
    CHECK(!mock_hal::last_timer->active);
    CHECK(status.output_mode == MotorOutputMode::DriveCW);
    CHECK(status.last_operation == MotorOperation::Drive);
    CHECK(status.last_error == MotorOperationError::None);
    CHECK_NEAR(outputState().a, 700.0f * 100.0f / 1023.0f, 0.001f);
    CHECK_NEAR(outputState().b, 0.0f, 0.001f);

    // No obsolete phase remains queued after the stale scheduler unwinds.
    const OutputState before = outputState();
    mock_hal::fireTimer(mock_hal::last_timer);
    checkState(outputState(), before);
    race_motor = nullptr;
}

void testRepeatedSetupContainmentFailuresTakePrecedence()
{
    MotorMCPWMConfig without_en = hardware(-1);

    mock_hal::reset();
    HBridgeMotor duty_failure;
    CHECK(duty_failure.setup(without_en, behavior(FreewheelMode::HiZ_Awake)).ok());
    CHECK(duty_failure.drive(700, Dir::CW).ok());
    mock_hal::duty_result = -1;
    MotorMCPWMConfig invalid = without_en;
    invalid.lpwm_pin = -1;
    const MotorSetupResult duty_result =
        duty_failure.setup(invalid, behavior(FreewheelMode::HiZ_Awake));
    CHECK(duty_result.error == MotorSetupError::ContainmentFailed);
    CHECK(duty_failure.status().setup_complete);
    CHECK(duty_failure.status().output_mode == MotorOutputMode::Disabled);
    CHECK(duty_failure.readback().valid);
    CHECK(mock_hal::pwm_routes.empty());
    CHECK(!duty_failure.drive(500, Dir::CW).ok());
    mock_hal::duty_result = ESP_OK;

    CHECK(duty_failure.setup(MotorMCPWMConfig{}).error == MotorSetupError::InvalidPwmPin);
    mock_hal::reset();
    HBridgeMotor stop_failure;
    CHECK(stop_failure.setup(without_en, behavior(FreewheelMode::HiZ_Awake)).ok());
    CHECK(stop_failure.drive(700, Dir::CW).ok());
    mock_hal::stop_result = -1;
    const MotorSetupResult stop_setup =
        stop_failure.setup(invalid, behavior(FreewheelMode::HiZ_Awake));
    CHECK(stop_setup.error == MotorSetupError::ContainmentFailed);
    CHECK(stop_failure.status().setup_complete);
    CHECK(stop_failure.status().mcpwm_running);
    CHECK(stop_failure.status().recovery_inhibited);
    CHECK(stop_failure.status().output_mode == MotorOutputMode::Disabled);
    mock_hal::stop_result = ESP_OK;

    CHECK(stop_failure.setup(MotorMCPWMConfig{}).error == MotorSetupError::InvalidPwmPin);
    mock_hal::reset();
    HBridgeMotor en_contained;
    CHECK(en_contained.setup(hardware(), behavior(FreewheelMode::HiZ_Awake)).ok());
    CHECK(en_contained.drive(700, Dir::CW).ok());
    mock_hal::duty_result = -1;
    const MotorSetupResult en_result =
        en_contained.setup(invalid, behavior(FreewheelMode::HiZ_Awake));
    CHECK(en_result.error == MotorSetupError::ContainmentFailed);
    CHECK(en_contained.status().output_mode == MotorOutputMode::Disabled);
    CHECK(mock_hal::pin_levels[8] == LOW);
    mock_hal::duty_result = ESP_OK;

    CHECK(en_contained.setup(MotorMCPWMConfig{}).error == MotorSetupError::InvalidPwmPin);
    mock_hal::reset();
    HBridgeMotor deadtime_failure;
    MotorMCPWMConfig with_deadtime = hardware();
    with_deadtime.use_deadtime = true;
    CHECK(deadtime_failure.setup(with_deadtime, behavior(FreewheelMode::HiZ)).ok());
    mock_hal::deadtime_disable_result = -1;
    const MotorSetupResult deadtime_result =
        deadtime_failure.setup(invalid, behavior(FreewheelMode::HiZ));
    CHECK(deadtime_result.error == MotorSetupError::ContainmentFailed);
    CHECK(deadtime_failure.status().setup_complete);
    CHECK(deadtime_failure.status().output_mode == MotorOutputMode::Disabled);
    mock_hal::deadtime_disable_result = ESP_OK;

    CHECK(deadtime_failure.setup(MotorMCPWMConfig{}).error == MotorSetupError::InvalidPwmPin);
    mock_hal::reset();
    mock_hal::pin_levels[12] = LOW;
    HBridgeMotor fault_teardown_failure;
    CHECK(fault_teardown_failure
              .setup(hardware(), behavior(FreewheelMode::HiZ), MotorSafetyConfig{},
                     MotorCaptureConfig{}, hardwareFaultConfig(HardwareFaultMode::OneShot))
              .ok());
    mock_hal::fault_deinit_result = -1;
    const MotorSetupResult fault_teardown_result =
        fault_teardown_failure.setup(invalid, behavior(FreewheelMode::HiZ));
    CHECK(fault_teardown_result.error == MotorSetupError::ContainmentFailed);
    CHECK(fault_teardown_failure.status().setup_complete);
    CHECK(fault_teardown_failure.status().hardware_fault_configured);
    CHECK(fault_teardown_failure.status().output_mode == MotorOutputMode::Disabled);
    mock_hal::fault_deinit_result = ESP_OK;
}

void testDitherWriteFailureTerminatesAndContains()
{
    mock_hal::reset();
    HBridgeMotor motor;
    CHECK(motor.setup(hardware(), behavior(FreewheelMode::DitherBrake, 256, true)).ok());
    mock_hal::duty_results = {ESP_OK, -1, -1, -1, -1};
    mock_hal::fireTimer(mock_hal::last_timer);
    MotorDriverStatus failed = motor.status();
    CHECK(!failed.dither_active);
    CHECK(!mock_hal::last_timer->active);
    CHECK(failed.last_error == MotorOperationError::HardwareWriteFailed);
    CHECK(failed.output_mode == MotorOutputMode::Disabled);

    mock_hal::duty_results.clear();
    CHECK(motor.drive(500, Dir::CCW).ok());
    CHECK(motor.status().output_mode == MotorOutputMode::DriveCCW);

    CHECK(motor.setup(MotorMCPWMConfig{}).error == MotorSetupError::InvalidPwmPin);
    mock_hal::reset();
    HBridgeMotor no_en;
    CHECK(no_en.setup(hardware(-1), behavior(FreewheelMode::DitherBrake, 256, true)).ok());
    mock_hal::duty_results = {ESP_OK, -1, -1, -1, -1};
    mock_hal::fireTimer(mock_hal::last_timer);
    failed = no_en.status();
    CHECK(!failed.dither_active);
    CHECK(failed.last_error == MotorOperationError::HardwareWriteFailed);
    CHECK(failed.output_mode == MotorOutputMode::Uncertain);
    mock_hal::duty_results.clear();
    CHECK(no_en.drive(500, Dir::CW).ok());
}

void testDitherTimerFailureTerminatesAndContains()
{
    mock_hal::reset();
    HBridgeMotor contained;
    CHECK(contained
              .setup(hardware(), behavior(FreewheelMode::DitherBrake, 256, true))
              .ok());
    mock_hal::timer_start_result = -1;
    mock_hal::fireTimer(mock_hal::last_timer);
    MotorDriverStatus failed = contained.status();
    CHECK(!failed.dither_active);
    CHECK(!mock_hal::last_timer->active);
    CHECK(failed.last_error == MotorOperationError::TimerFailed);
    CHECK(failed.output_mode == MotorOutputMode::Coast);

    mock_hal::timer_start_result = ESP_OK;
    CHECK(contained.drive(500, Dir::CCW).ok());
    CHECK(contained.status().output_mode == MotorOutputMode::DriveCCW);

    CHECK(contained.setup(MotorMCPWMConfig{}).error == MotorSetupError::InvalidPwmPin);
    mock_hal::reset();
    HBridgeMotor uncertain;
    CHECK(uncertain
              .setup(hardware(-1), behavior(FreewheelMode::DitherBrake, 256, true))
              .ok());
    mock_hal::timer_start_result = -1;
    mock_hal::duty_results = {ESP_OK, ESP_OK, -1, -1, -1};
    mock_hal::fireTimer(mock_hal::last_timer);
    failed = uncertain.status();
    CHECK(!failed.dither_active);
    CHECK(!mock_hal::last_timer->active);
    CHECK(failed.last_error == MotorOperationError::TimerFailed);
    CHECK(failed.output_mode == MotorOutputMode::Uncertain);

    mock_hal::timer_start_result = ESP_OK;
    mock_hal::duty_results.clear();
    CHECK(uncertain.drive(500, Dir::CW).ok());
    CHECK(uncertain.status().output_mode == MotorOutputMode::DriveCW);
}

void testSupersededDitherTimerFailurePreservesNewerDrive()
{
    mock_hal::reset();
    HBridgeMotor motor;
    race_motor = &motor;
    race_action = RaceAction::Drive;
    CHECK(motor
              .setup(hardware(), behavior(FreewheelMode::DitherBrake, 256, true))
              .ok());

    // Arm the nested drive only after the timer start has produced its genuine
    // failure. The drive completes while the old failure path is paused in its
    // first containment write, before that path can publish diagnostics.
    mock_hal::timer_start_result = -1;
    mock_hal::timer_start_hook = &armRaceCommandOnDuty;
    mock_hal::fireTimer(mock_hal::last_timer);

    CHECK(race_result.ok());
    const MotorDriverStatus status = motor.status();
    CHECK(!status.dither_active);
    CHECK(!mock_hal::last_timer->active);
    CHECK(status.output_mode == MotorOutputMode::DriveCW);
    CHECK(status.last_operation == MotorOperation::Drive);
    CHECK(status.last_error == MotorOperationError::None);
    CHECK_NEAR(outputState().a, 700.0f * 100.0f / 1023.0f, 0.001f);
    CHECK_NEAR(outputState().b, 0.0f, 0.001f);

    for (std::size_t i = race_return_event_count; i < mock_hal::events.size(); ++i)
    {
        const mock_hal::EventKind kind = mock_hal::events[i].kind;
        CHECK(kind != mock_hal::EventKind::Duty &&
              kind != mock_hal::EventKind::DigitalWrite);
    }
    race_motor = nullptr;
}

void testChangedSemanticsAreLiteral()
{
    mock_hal::reset();
    HBridgeMotor motor;
    CHECK(motor.setup(hardware(), behavior(FreewheelMode::HiZ)).ok());

    CHECK(!motor.coast().changed);
    CHECK(motor.drive(500, Dir::CW).changed);
    CHECK(!motor.drive(500, Dir::CW).changed);
    CHECK(motor.coast().changed);
    CHECK(!motor.coast().changed);
    CHECK(motor.setHardBrake().changed);
    CHECK(!motor.setHardBrake().changed);
    CHECK(motor.stop().changed);
    CHECK(!motor.stop().changed);
    CHECK(motor.start().changed);
    CHECK(!motor.start().changed);
    CHECK(!motor.setFreewheelMode(FreewheelMode::HiZ).changed);
    CHECK(motor.setFreewheelMode(FreewheelMode::HiZ_Awake).changed);
    CHECK(!motor.setFreewheelMode(FreewheelMode::HiZ_Awake).changed);
}

void testStoppedCoastContract()
{
    const FreewheelMode modes[]{FreewheelMode::HiZ, FreewheelMode::HiZ_Awake};
    for (const FreewheelMode mode : modes)
    {
        mock_hal::reset();
        HBridgeMotor motor;
        CHECK(motor.setup(hardware(), behavior(mode)).ok());
        CHECK(motor.stop().ok());
        CHECK(motor.coast().ok());
        CHECK(!motor.status().mcpwm_running);
        CHECK(motor.status().output_mode == MotorOutputMode::Coast);
    }

    mock_hal::reset();
    HBridgeMotor dither;
    CHECK(dither.setup(hardware(), behavior(FreewheelMode::DitherBrake, 256, true)).ok());
    CHECK(dither.stop().ok());
    const MotorOperationResult result = dither.coast();
    CHECK(result.error == MotorOperationError::InvalidCommand);
    CHECK(!dither.status().dither_active);
    CHECK(!dither.status().mcpwm_running);
}

uint32_t last_capture_period = 0;
void recordCapture(uint32_t period, void *) { last_capture_period = period; }

void testCaptureBehaviorRemainsUnchanged()
{
    mock_hal::reset();
    last_capture_period = 0;
    HBridgeMotor motor;
    MotorCaptureConfig capture{};
    capture.cap_gpio = 11;
    capture.edge = CaptureEdge::Both;
    capture.on_capture = &recordCapture;
    motor.setup(hardware(), behavior(FreewheelMode::HiZ), MotorSafetyConfig{}, capture);

    IMotorDriver *driver = &motor;
    CHECK(driver->getLastCapturePeriodUs() == 0);

    mock_hal::micros_value = 0;
    mock_hal::invokeInterrupt(11);
    mock_hal::micros_value = 40;
    mock_hal::invokeInterrupt(11);
    CHECK(last_capture_period == 40u);
    CHECK(driver->getLastCapturePeriodUs() == 40u);

    mock_hal::micros_value = 0xFFFFFFF0u;
    mock_hal::invokeInterrupt(11);
    mock_hal::micros_value = 20;
    mock_hal::invokeInterrupt(11);
    CHECK(last_capture_period == 36u);
    CHECK(driver->getLastCapturePeriodUs() == 36u);
}


void testStructuredResultsAndReadback()
{
    mock_hal::reset();
    HBridgeMotor motor;
    const MotorSetupResult setup = motor.setup(hardware(), behavior(FreewheelMode::HiZ));
    CHECK(setup.ok());
    CHECK(!setup.software_fault_enabled);
    CHECK(!setup.hardware_fault_enabled);

    const MotorOperationResult drive = motor.drive(512, Dir::CW);
    CHECK(drive.ok());
    CHECK(drive.operation == MotorOperation::Drive);
    CHECK(drive.changed);
    CHECK(drive.sequence > 0U);

    const MotorDriverStatus status = motor.status();
    CHECK(status.setup_complete);
    CHECK(status.mcpwm_running);
    CHECK(status.output_mode == MotorOutputMode::DriveCW);
    CHECK(status.commanded_a_percent > 0.0f);
    CHECK_NEAR(status.commanded_b_percent, 0.0f, 0.001f);
    CHECK(status.last_operation == MotorOperation::Drive);
    CHECK(status.last_error == MotorOperationError::None);

    const MotorHardwareReadback readback = motor.readback();
    CHECK(readback.valid);
    CHECK(readback.frequency_hz == 20000U);
    CHECK_NEAR(readback.duty_a_percent, 50.0f, 0.001f);
    CHECK_NEAR(readback.duty_b_percent, status.commanded_b_percent, 0.001f);
    CHECK(readback.enable_control);
    CHECK(readback.enable_asserted);
    CHECK(readback.running_cached);
}

void testLifecycleAndHardwareWriteFailuresAreReported()
{
    mock_hal::reset();
    HBridgeMotor motor;
    CHECK(motor.setup(hardware(), behavior(FreewheelMode::HiZ)).ok());

    mock_hal::duty_result = -1;
    const MotorOperationResult failed_drive = motor.drive(500, Dir::CW);
    CHECK(!failed_drive.ok());
    CHECK(failed_drive.error == MotorOperationError::HardwareWriteFailed);
    CHECK(motor.status().last_error == MotorOperationError::HardwareWriteFailed);
    mock_hal::duty_result = ESP_OK;

    CHECK(motor.stop().ok());
    mock_hal::start_result = -1;
    const MotorOperationResult failed_start = motor.start();
    CHECK(!failed_start.ok());
    CHECK(failed_start.error == MotorOperationError::HardwareStartFailed);
    CHECK(!motor.status().mcpwm_running);
    mock_hal::start_result = ESP_OK;
    CHECK(motor.start().ok());

    mock_hal::stop_result = -1;
    const MotorOperationResult failed_stop = motor.stop();
    CHECK(!failed_stop.ok());
    CHECK(failed_stop.error == MotorOperationError::HardwareStopFailed);
    CHECK(motor.status().mcpwm_running);
    mock_hal::stop_result = ESP_OK;
}

void testHardwareFaultOneShotConfigurationAndRearm()
{
    mock_hal::reset();
    mock_hal::pin_levels[12] = LOW;
    HBridgeMotor motor;
    MotorHardwareFaultConfig fault = hardwareFaultConfig(HardwareFaultMode::OneShot);
    fault.action_a = HardwareFaultOutputAction::ForceLow;
    fault.action_b = HardwareFaultOutputAction::ForceHigh;

    const MotorSetupResult setup = motor.setup(hardware(), behavior(FreewheelMode::HiZ),
                                                MotorSafetyConfig{}, MotorCaptureConfig{}, fault);
    CHECK(setup.ok());
    CHECK(setup.hardware_fault_enabled);
    CHECK(hasEvent(mock_hal::EventKind::McpwmGpioInit, MCPWM_UNIT_0, MCPWM_FAULT_0));
    CHECK(hasEvent(mock_hal::EventKind::HardwareFaultInit, MCPWM_UNIT_0, MCPWM_SELECT_F0));
    CHECK(hasEvent(mock_hal::EventKind::HardwareFaultMode, MCPWM_UNIT_0, MCPWM_TIMER_0));

    bool exact_action_seen = false;
    for (const auto &event : mock_hal::events)
    {
        if (event.kind == mock_hal::EventKind::HardwareFaultMode &&
            event.first == MCPWM_UNIT_0 && event.second == MCPWM_TIMER_0 &&
            std::fabs(event.value - static_cast<float>(MCPWM_FORCE_MCPWMXA_LOW * 10 +
                                                       MCPWM_FORCE_MCPWMXB_HIGH)) < 0.001f)
            exact_action_seen = true;
    }
    CHECK(exact_action_seen);
    CHECK(motor.drive(500, Dir::CW).ok());
    CHECK(outputState().a > 0.0f);

    mock_hal::pin_levels[12] = HIGH;
    mock_hal::invokeInterrupt(12);
    CHECK(motor.hasFault());
    MotorDriverStatus status = motor.status();
    CHECK(status.hardware_fault_configured);
    CHECK(status.hardware_fault_input_active);
    CHECK(status.hardware_fault_latched);
    CHECK(status.hardware_fault_sequence > 0U);
    CHECK(motor.drive(500, Dir::CW).error == MotorOperationError::FaultActive);

    mock_hal::pin_levels[12] = LOW;
    mock_hal::invokeInterrupt(12);
    CHECK(motor.hasFault()); // one-shot remains latched after the input clears
    mock_hal::events.clear();
    const MotorOperationResult cleared = motor.clearFault();
    CHECK(cleared.ok());

    // Zero compare values must be staged before fault de-initialization/re-arm,
    // otherwise a pre-fault drive duty could briefly reappear.
    const int zero_a = findEvent(mock_hal::EventKind::Duty, MCPWM_OPR_A);
    const int fault_deinit = findEvent(mock_hal::EventKind::HardwareFaultDeinit,
                                       MCPWM_UNIT_0, MCPWM_SELECT_F0);
    CHECK(zero_a >= 0);
    CHECK(fault_deinit >= 0);
    CHECK(zero_a < fault_deinit);
    CHECK(!motor.hasFault());
    CHECK(hasEvent(mock_hal::EventKind::HardwareFaultDeinit, MCPWM_UNIT_0, MCPWM_SELECT_F0));
    CHECK(motor.status().hardware_fault_sequence > status.hardware_fault_sequence);
}

void testHardwareFaultCycleByCycleStatusFollowsInput()
{
    mock_hal::reset();
    mock_hal::pin_levels[12] = LOW;
    HBridgeMotor motor;
    const MotorSetupResult setup = motor.setup(hardware(), behavior(FreewheelMode::HiZ),
                                                MotorSafetyConfig{}, MotorCaptureConfig{},
                                                hardwareFaultConfig(HardwareFaultMode::CycleByCycle));
    CHECK(setup.ok());

    mock_hal::pin_levels[12] = HIGH;
    mock_hal::invokeInterrupt(12);
    CHECK(motor.hasFault());
    CHECK(motor.status().hardware_fault_input_active);

    mock_hal::pin_levels[12] = LOW;
    mock_hal::invokeInterrupt(12);
    CHECK(!motor.hasFault());
    CHECK(!motor.status().hardware_fault_input_active);
    CHECK(!motor.status().hardware_fault_latched);
}

void testInvalidAndFailedHardwareFaultSetupFailsSafe()
{
    MotorHardwareFaultConfig active_low = hardwareFaultConfig(HardwareFaultMode::OneShot);
    active_low.active_high = false;
    mock_hal::reset();
    HBridgeMotor motor;
    MotorSetupResult setup = motor.setup(hardware(), behavior(FreewheelMode::HiZ),
                                          MotorSafetyConfig{}, MotorCaptureConfig{}, active_low);
    CHECK(!setup.ok());
    CHECK(setup.error == MotorSetupError::UnsupportedHardwareFaultLevel);
    CHECK(!hasEvent(mock_hal::EventKind::McpwmInit));

    MotorHardwareFaultConfig conflict = hardwareFaultConfig(HardwareFaultMode::OneShot, 8);
    mock_hal::reset();
    HBridgeMotor conflict_motor;
    setup = conflict_motor.setup(hardware(), behavior(FreewheelMode::HiZ),
                                 MotorSafetyConfig{}, MotorCaptureConfig{}, conflict);
    CHECK(!setup.ok());
    CHECK(setup.error == MotorSetupError::PinConflict);

    mock_hal::reset();
    mock_hal::fault_init_result = -1;
    HBridgeMotor init_failure;
    setup = init_failure.setup(hardware(), behavior(FreewheelMode::HiZ),
                               MotorSafetyConfig{}, MotorCaptureConfig{},
                               hardwareFaultConfig(HardwareFaultMode::OneShot));
    CHECK(!setup.ok());
    CHECK(setup.error == MotorSetupError::HardwareFaultInitFailed);
    CHECK(mock_hal::pin_levels[8] == LOW);
    CHECK(!mock_hal::running[MCPWM_UNIT_0][MCPWM_TIMER_0]);

    mock_hal::reset();
    mock_hal::fault_mode_result = -1;
    HBridgeMotor mode_failure;
    setup = mode_failure.setup(hardware(), behavior(FreewheelMode::HiZ),
                               MotorSafetyConfig{}, MotorCaptureConfig{},
                               hardwareFaultConfig(HardwareFaultMode::OneShot));
    CHECK(!setup.ok());
    CHECK(setup.error == MotorSetupError::HardwareFaultInitFailed);
    CHECK(hasEvent(mock_hal::EventKind::HardwareFaultDeinit, MCPWM_UNIT_0, MCPWM_SELECT_F0));
}

void testCommissioningOutputApiIsDisabledByDefault()
{
    mock_hal::reset();
    HBridgeMotor motor;
    CHECK(motor.setup(hardware(), behavior(FreewheelMode::HiZ)).ok());
    const OutputState before = outputState();
    const MotorOperationResult forced = motor.forceOutputs(true, true);
    CHECK(!forced.ok());
    CHECK(forced.error == MotorOperationError::CommissioningDisabled);
    checkState(outputState(), before);
}

void testFrequencyReadbackTracksSuccessfulReconfiguration()
{
    mock_hal::reset();
    HBridgeMotor motor;
    CHECK(motor.setup(hardware(), behavior(FreewheelMode::HiZ)).ok());
    CHECK(motor.readback().frequency_hz == 20000U);
    CHECK(motor.reconfigureFrequency(25000).ok());
    CHECK(motor.readback().frequency_hz == 25000U);
    CHECK(motor.status().pwm_frequency_hz == 25000);
}

void testCenterAlignedWaveformUsesRequestedFrequency()
{
    mock_hal::reset();
    HBridgeMotor motor;
    auto hw = hardware();
    hw.counter = MCPWM_UP_DOWN_COUNTER;
    const auto &timer = mock_hal::pwm_timers[MCPWM_UNIT_0][MCPWM_TIMER_0];
    CHECK(motor.setup(hw, behavior(FreewheelMode::HiZ), faultConfig(FaultAction::Coast),
                      MotorCaptureConfig{}).ok());
    CHECK(timer.mode == MCPWM_UP_DOWN_COUNTER);
    CHECK(timer.active_resolution_hz == 20000U * timer.cycleTicks());
    CHECK(timer.resolution_hz == 2000000U);
    CHECK(timer.period_register == 50U);
    CHECK(motor.readback().frequency_hz == 20000U);
    CHECK(timer.highTicks(mock_hal::duties[0][0][MCPWM_OPR_A]) == 0U);
    CHECK(timer.highTicks(mock_hal::duties[0][0][MCPWM_OPR_B]) == 0U);
    CHECK(!motor.readback().enable_asserted);

    CHECK(motor.setSpeed(hw.input_max, Dir::CW).ok());
    CHECK(timer.highTicks(mock_hal::duties[0][0][MCPWM_OPR_A]) == timer.cycleTicks());
    CHECK(timer.highTicks(mock_hal::duties[0][0][MCPWM_OPR_B]) == 0U);
    CHECK(motor.setSpeed(hw.input_max / 2 + 1, Dir::CW).ok());
    CHECK(timer.highTicks(mock_hal::duties[0][0][MCPWM_OPR_A]) == 50U);

    CHECK(motor.reconfigureFrequency(25000).ok());
    CHECK(timer.period_register == 40U);
    CHECK(timer.active_resolution_hz == 25000U * timer.cycleTicks());
    CHECK(motor.readback().frequency_hz == 25000U);
    CHECK(motor.status().pwm_frequency_hz == 25000);
    CHECK(timer.highTicks(mock_hal::duties[0][0][MCPWM_OPR_A]) == 0U);
    CHECK(!motor.readback().enable_asserted);

    CHECK(motor.setSpeed(hw.input_max, Dir::CW).ok());
    mock_hal::pin_levels[10] = HIGH;
    mock_hal::invokeInterrupt(10);
    motor.pollFaults();
    CHECK(timer.highTicks(mock_hal::duties[0][0][MCPWM_OPR_A]) == 0U);
    CHECK(!motor.readback().enable_asserted);
    CHECK(motor.disableOutputs().ok());
    CHECK(!motor.readback().enable_asserted);
}

void testFrequencyModeBoundariesAndResolutionFailure()
{
    mock_hal::reset();
    HBridgeMotor motor;
    auto hw = hardware();
    const auto &timer = mock_hal::pwm_timers[MCPWM_UNIT_0][MCPWM_TIMER_0];
    hw.counter = MCPWM_UP_DOWN_COUNTER;
    hw.pwm_freq_hz = 15;
    CHECK(motor.setup(hw, behavior(FreewheelMode::HiZ)).error == MotorSetupError::InvalidPwmFrequency);
    CHECK(!hasEvent(mock_hal::EventKind::McpwmInit));

    hw.pwm_freq_hz = 16;
    CHECK(motor.setup(hw, behavior(FreewheelMode::HiZ)).ok());
    CHECK(timer.period_register == 62499U);
    CHECK(motor.readback().frequency_hz == 16U);
    CHECK(motor.reconfigureFrequency(500000).ok());
    CHECK(timer.cycleTicks() == 4U);
    CHECK(motor.reconfigureFrequency(500001).error == MotorOperationError::InvalidCommand);
    CHECK(timer.cycleTicks() == 4U);

    // The nearest waveform can be on either side of a non-exact request.
    CHECK(motor.reconfigureFrequency(19000).ok());
    CHECK(timer.period_register == 53U);
    CHECK(motor.readback().frequency_hz == 18867U);
    CHECK(motor.reconfigureFrequency(19150).ok());
    CHECK(timer.period_register == 52U);
    CHECK(motor.readback().frequency_hz == 19230U);
    CHECK(motor.reconfigureFrequency(333334).ok());
    CHECK(timer.period_register == 3U);
    CHECK(motor.readback().frequency_hz == 333333U);

    for (const auto counter : {MCPWM_UP_COUNTER, MCPWM_DOWN_COUNTER})
    {
        hw.counter = counter;
        hw.pwm_freq_hz = 20000;
        CHECK(motor.setup(hw, behavior(FreewheelMode::HiZ)).ok());
        CHECK(timer.mode == counter);
        CHECK(timer.resolution_hz == 1000000U);
        CHECK(timer.active_resolution_hz == 1000000U);
        CHECK(timer.cycleTicks() == 50U);
        CHECK(motor.reconfigureFrequency(25000).ok());
        CHECK(timer.cycleTicks() == 40U);
    }

    hw.counter = MCPWM_FREEZE_COUNTER;
    CHECK(motor.setup(hw, behavior(FreewheelMode::HiZ)).error == MotorSetupError::InvalidPwmFrequency);

    hw.counter = MCPWM_UP_DOWN_COUNTER;
    mock_hal::resolution_result = -1;
    CHECK(motor.setup(hw, behavior(FreewheelMode::HiZ)).error == MotorSetupError::HardwareInitFailed);
    CHECK(!mock_hal::running[0][0]);
    CHECK(mock_hal::pin_levels[hw.en_pin] == LOW);

    mock_hal::resolution_result = ESP_OK;
    mock_hal::start_result = -1;
    CHECK(motor.setup(hw, behavior(FreewheelMode::HiZ)).error == MotorSetupError::HardwareInitFailed);
    CHECK(!mock_hal::running[0][0]);
    CHECK(mock_hal::pin_levels[hw.en_pin] == LOW);
}

void testImmediateLowFrequencySetupAndReuse()
{
    mock_hal::reset();
    HBridgeMotor owner;
    auto hw = hardware(-1);
    hw.pwm_freq_hz = 16;
    hw.counter = MCPWM_UP_DOWN_COUNTER;
    const auto options = behavior(FreewheelMode::HiZ_Awake);
    CHECK(owner.setup(hw, options).ok());
    auto &timer = mock_hal::pwm_timers[0][0];
    for (const auto mode : {MCPWM_UP_DOWN_COUNTER, MCPWM_DOWN_COUNTER, MCPWM_UP_COUNTER})
    {
        CHECK(owner.drive(600, Dir::CW).ok());
        delayMicroseconds(100);
        CHECK(timer.active_compare[0] != timer.shadow_compare[0] ||
              timer.highTicksForCompare(timer.active_compare[0]) > 0U);
        timer.phase_ticks = 1U;
        timer.events |= mock_hal::stopped_event; // Stale event from an earlier cycle.
        const uint32_t before = micros();
        hw.counter = mode;
        CHECK(owner.setup(hw, options).ok());
        CHECK(micros() - before > 60000U);
        CHECK(timer.mode == mode);
        CHECK(mock_hal::active_reinitializations == 0U);
        CHECK(mock_hal::unsafe_routes == 0U);
        CHECK(owner.readback().frequency_hz == 16U);
    }

    mock_hal::suppress_timer_stop = true;
    CHECK(owner.setup(hw, options).error == MotorSetupError::ContainmentFailed);
    CHECK(owner.status().mcpwm_running);
    CHECK(owner.status().recovery_inhibited);
    CHECK(!owner.drive(600, Dir::CCW).ok());
    CHECK(mock_hal::pwm_routes.empty());
    CHECK(mock_hal::pin_levels[hw.lpwm_pin] == LOW);
    CHECK(mock_hal::pin_levels[hw.rpwm_pin] == LOW);
    CHECK(timer.stop_pending);
    HBridgeMotor replacement;
    CHECK(replacement.setup(hw, options).error == MotorSetupError::ResourceConflict);

    mock_hal::suppress_timer_stop = false;
    delayMicroseconds(63000U); // Event arrives after timeout, before retry.
    CHECK(!mock_hal::running[0][0]);
    CHECK(!timer.stop_pending);
    auto invalid = hw;
    invalid.lpwm_pin = -1;
    CHECK(owner.setup(invalid, options).error == MotorSetupError::InvalidPwmPin);
    CHECK(replacement.setup(hw, options).ok());
    CHECK(mock_hal::active_reinitializations == 0U);
    CHECK(mock_hal::unsafe_routes == 0U);
}

void testRetuneSeparatesActiveAndPendingOutput()
{
    for (const int enable_pin : {-1, 8})
    {
        mock_hal::reset();
        HBridgeMotor motor;
        auto hw = hardware(enable_pin);
        hw.counter = MCPWM_UP_DOWN_COUNTER;
        hw.pwm_freq_hz = 16;
        CHECK(motor.setup(hw, behavior(FreewheelMode::HiZ_Awake)).ok());
        auto &timer = mock_hal::pwm_timers[0][0];
        CHECK(motor.drive(700, Dir::CW).ok());
        delayMicroseconds(63000U);
        CHECK(timer.active_compare[0] > 0U);
        CHECK(motor.drive(700, Dir::CCW).ok());
        CHECK(timer.shadow_compare[1] > 0U);
        CHECK(timer.active_compare[1] == 0U);
        if (enable_pin < 0)
        {
            const auto event_count = mock_hal::events.size();
            CHECK(motor.reconfigureFrequency(25000).error == MotorOperationError::Unsupported);
            CHECK(mock_hal::events.size() == event_count);
            CHECK(timer.shadow_compare[1] > 0U);
            CHECK(motor.status().pwm_frequency_hz == 16);
            CHECK(motor.stop().ok());
            CHECK(motor.reconfigureFrequency(25000).ok());
            CHECK(!motor.status().mcpwm_running);
            CHECK(mock_hal::pwm_routes.empty());
            CHECK(motor.start().ok());
            CHECK(motor.readback().frequency_hz == 25000U);
            CHECK(mock_hal::unsafe_routes == 0U);
            continue;
        }
        mock_hal::observed_enable_pin = enable_pin;
        CHECK(motor.reconfigureFrequency(25000).ok());
        CHECK(mock_hal::live_period_writes == 0U);
        CHECK(mock_hal::unsafe_routes == 0U);
        CHECK(timer.active_compare[0] == 0U);
        CHECK(timer.active_compare[1] == 0U);
        CHECK(!motor.readback().enable_asserted);
        CHECK(motor.readback().frequency_hz == 25000U);

        CHECK(motor.drive(400, Dir::CCW).ok());
        delayMicroseconds(50);
        CHECK(timer.highTicksForCompare(timer.active_compare[1]) > 0U);
        mock_hal::start_result = -1;
        CHECK(motor.reconfigureFrequency(16).error == MotorOperationError::HardwareStartFailed);
        CHECK(motor.status().pwm_frequency_hz == 16);
        CHECK(motor.readback().frequency_hz == 0U);
        CHECK(!motor.status().mcpwm_running);
        CHECK(mock_hal::pwm_routes.empty());
        mock_hal::start_result = ESP_OK;
        CHECK(motor.start().ok());
        CHECK(motor.readback().frequency_hz == 16U);
        CHECK(timer.active_compare[0] == 0U);
        CHECK(timer.active_compare[1] == 0U);
        CHECK(mock_hal::unsafe_routes == 0U);
        CHECK(mock_hal::live_period_writes == 0U);

        CHECK(motor.stop().ok());
        CHECK(motor.reconfigureFrequency(25000).ok());
        CHECK(!motor.status().mcpwm_running);
        CHECK(mock_hal::pwm_routes.empty());
        CHECK(motor.start().ok());
        CHECK(motor.readback().frequency_hz == 25000U);

        CHECK(motor.setHardBrake().ok());
        delayMicroseconds(50);
        CHECK(timer.highTicksForCompare(timer.active_compare[0]) == timer.cycleTicks());
        CHECK(timer.highTicksForCompare(timer.active_compare[1]) == timer.cycleTicks());
        CHECK(motor.reconfigureFrequency(20000).ok());
        CHECK(mock_hal::remaps_while_enabled == 0U);
    }
}

void testRetunePreservesFaultOwnership()
{
    mock_hal::reset();
    HBridgeMotor motor;
    CHECK(motor.setup(hardware(), behavior(FreewheelMode::HiZ), faultConfig(FaultAction::Coast),
                      MotorCaptureConfig{}).ok());
    CHECK(motor.drive(512, Dir::CW).ok());
    mock_hal::pin_levels[10] = HIGH;
    mock_hal::invokeInterrupt(10);
    const auto events = mock_hal::events.size();
    CHECK(motor.reconfigureFrequency(25000).error == MotorOperationError::FaultActive);
    CHECK(mock_hal::events.size() == events);
    CHECK(motor.pollFaults().ok());
    mock_hal::pin_levels[10] = LOW;
    CHECK(motor.clearFault().ok());

    mock_hal::pwm_wait_hook = []() {
        mock_hal::pin_levels[10] = HIGH;
        mock_hal::invokeInterrupt(10);
    };
    CHECK(motor.reconfigureFrequency(25000).error == MotorOperationError::FaultActive);
    CHECK(motor.status().pwm_frequency_hz == 20000);
    CHECK(mock_hal::pwm_routes.empty());
    CHECK(!motor.readback().enable_asserted);

    CHECK(motor.setup(hardware(), behavior(FreewheelMode::HiZ), MotorSafetyConfig{},
                      MotorCaptureConfig{}, hardwareFaultConfig(HardwareFaultMode::OneShot)).ok());
    const auto events_with_fault_config = mock_hal::events.size();
    CHECK(motor.reconfigureFrequency(25000).error == MotorOperationError::Unsupported);
    CHECK(mock_hal::events.size() == events_with_fault_config);
    mock_hal::pin_levels[12] = HIGH;
    mock_hal::invokeInterrupt(12);
    CHECK(motor.reconfigureFrequency(25000).error == MotorOperationError::FaultActive);
    CHECK(motor.status().pwm_frequency_hz == 20000);
}

void testCounterModesAndElectricalEndpointsInModel()
{
    for (const auto mode : {MCPWM_UP_COUNTER, MCPWM_DOWN_COUNTER, MCPWM_UP_DOWN_COUNTER})
    {
        mock_hal::reset();
        HBridgeMotor motor;
        auto hw = hardware();
        hw.counter = mode;
        hw.pwm_freq_hz = 500000;
        CHECK(motor.setup(hw, behavior(FreewheelMode::HiZ_Awake)).ok());
        auto &timer = mock_hal::pwm_timers[0][0];
        CHECK(timer.highTicksForCompare(timer.active_compare[0]) == 0U);
        CHECK(timer.highTicksForCompare(timer.active_compare[1]) == 0U);
        CHECK(motor.drive(hw.input_max, Dir::CW).ok());
        delayMicroseconds(3);
        CHECK(timer.highTicksForCompare(timer.active_compare[0]) == timer.cycleTicks());
        CHECK(timer.highTicksForCompare(timer.active_compare[1]) == 0U);
        CHECK(motor.readback().duty_a_percent == 100.0f);
        CHECK(motor.readback().duty_b_percent == 0.0f);
        CHECK(motor.reconfigureFrequency(500001).error == MotorOperationError::InvalidCommand);
        CHECK(motor.reconfigureFrequency(15).error == MotorOperationError::InvalidCommand);
        CHECK(motor.reconfigureFrequency(400001).ok());
        CHECK(motor.readback().frequency_hz == 333333U);
        CHECK(motor.reconfigureFrequency(440000).ok());
        CHECK(motor.readback().frequency_hz == 500000U);
        CHECK(motor.reconfigureFrequency(20000).ok());
        CHECK(motor.drive(hw.input_max / 2 + 1, Dir::CCW).ok());
        delayMicroseconds(60);
        CHECK(timer.highTicksForCompare(timer.active_compare[1]) * 2U == timer.cycleTicks());
        CHECK(motor.readback().duty_b_percent == 50.0f);
        CHECK(mock_hal::unsafe_routes == 0U);
    }
}

void checkPercentMatchesRaw(HBridgeMotor &motor, Dir direction, float percent, int expected_count)
{
    const bool cw = direction == Dir::CW;
    const auto channel = cw ? MCPWM_OPR_A : MCPWM_OPR_B;
    const auto &timer = mock_hal::pwm_timers[0][0];
    CHECK(motor.drivePercent(percent, direction).ok());
    delayMicroseconds(63000U);
    const auto status = motor.status();
    const float demand = cw ? status.commanded_a_percent : status.commanded_b_percent;
    const auto mapped_count = std::lround(static_cast<double>(demand) * motor.getMaxPwmInput() / 100.0);
    CHECK(mapped_count == expected_count);
    const auto percent_compares = timer.active_compare;
    const auto percent_readback = motor.readback();

    if (status.pwm_frequency_hz == 500000 && (expected_count == 511 || expected_count == 512))
    {
        const bool half_output = timer.mode == MCPWM_UP_DOWN_COUNTER || expected_count == 512;
        const uint32_t expected_compare = (timer.mode == MCPWM_DOWN_COUNTER) != half_output ? 1U : 0U;
        const auto idle_channel = cw ? MCPWM_OPR_B : MCPWM_OPR_A;
        CHECK(percent_compares[channel] == expected_compare);
        CHECK(timer.highTicksForCompare(percent_compares[channel]) ==
              (half_output ? timer.cycleTicks() / 2U : 0U));
        CHECK(timer.highTicksForCompare(percent_compares[idle_channel]) == 0U);
        CHECK_NEAR(cw ? percent_readback.duty_a_percent : percent_readback.duty_b_percent,
                   half_output ? 50.0f : 0.0f, 0.001f);
    }

    CHECK(motor.coast().ok());
    CHECK(motor.drive(expected_count, direction).ok());
    delayMicroseconds(63000U);
    CHECK(timer.active_compare == percent_compares);
    const auto raw_readback = motor.readback();
    CHECK_NEAR(raw_readback.duty_a_percent, percent_readback.duty_a_percent, 0.001f);
    CHECK_NEAR(raw_readback.duty_b_percent, percent_readback.duty_b_percent, 0.001f);
}

void testPercentMidpointAndRawEquivalence()
{
    for (const auto mode : {MCPWM_UP_COUNTER, MCPWM_DOWN_COUNTER, MCPWM_UP_DOWN_COUNTER})
    {
        for (const int hz : {500000, 20000, 16})
        {
            mock_hal::reset();
            HBridgeMotor motor;
            auto hw = hardware();
            hw.counter = mode;
            hw.pwm_freq_hz = hz;
            CHECK(motor.setup(hw).ok());
            CHECK(motor.readback().frequency_hz == static_cast<uint32_t>(hz));

            for (const auto direction : {Dir::CW, Dir::CCW})
            {
                checkPercentMatchesRaw(motor, direction, 50.0f, 512);
                checkPercentMatchesRaw(motor, direction, std::nextafter(50.0f, 0.0f), 511);
                checkPercentMatchesRaw(motor, direction, std::nextafter(50.0f, 100.0f), 512);
                checkPercentMatchesRaw(motor, direction, 25.0f, 256);
                checkPercentMatchesRaw(motor, direction, 75.0f, 767);
            }
        }
    }
}

void testPercentRangeAndOtherInputScales()
{
    mock_hal::reset();
    HBridgeMotor motor;
    auto hw = hardware();
    hw.input_max = 10;
    CHECK(motor.setup(hw).ok());
    checkPercentMatchesRaw(motor, Dir::CW, std::nextafter(25.0f, 0.0f), 2);
    checkPercentMatchesRaw(motor, Dir::CW, 25.0f, 3);
    checkPercentMatchesRaw(motor, Dir::CW, std::nextafter(25.0f, 100.0f), 3);
    checkPercentMatchesRaw(motor, Dir::CCW, std::numeric_limits<float>::min(), 1);
    checkPercentMatchesRaw(motor, Dir::CCW, 100.0f, 10);
    checkPercentMatchesRaw(motor, Dir::CCW, std::numeric_limits<float>::max(), 10);
    const auto compares = mock_hal::pwm_timers[0][0].shadow_compare;
    const auto state = outputState();
    for (const float invalid : {0.0f, -1.0f, std::numeric_limits<float>::infinity(),
                               std::numeric_limits<float>::quiet_NaN()})
    {
        CHECK(motor.drivePercent(invalid, Dir::CW).error == MotorOperationError::InvalidCommand);
        CHECK(mock_hal::pwm_timers[0][0].shadow_compare == compares);
        checkState(outputState(), state);
    }
    CHECK(motor.coast().ok());
    delayMicroseconds(100U);
    CHECK(motor.readback().duty_a_percent == 0.0f && motor.readback().duty_b_percent == 0.0f);

    hw.input_max = 65535;
    CHECK(motor.setup(hw).ok());
    checkPercentMatchesRaw(motor, Dir::CW, 50.0f, 32768);
    checkPercentMatchesRaw(motor, Dir::CW, 101.0f, 65535);
}

void testEnFirstPwmSuppression()
{
    using Kind = mock_hal::EventKind;
    bool modeled_release_violation = false;
    for (const float duty : {15.0f, 40.0f})
    {
        for (const auto direction : {Dir::CW, Dir::CCW})
        {
            for (uint32_t phase = 0; phase < 100U; phase += 5U)
            {
                mock_hal::reset();
                HBridgeMotor motor;
                auto hw = hardware();
                hw.counter = MCPWM_UP_DOWN_COUNTER;
                CHECK(motor.setup(hw, behavior(FreewheelMode::HiZ),
                                  faultConfig(FaultAction::Coast), MotorCaptureConfig{}).ok());
                mock_hal::observed_enable_pin = hw.en_pin;
                CHECK(motor.drivePercent(duty, direction).ok());
                delayMicroseconds(50);
                auto &timer = mock_hal::pwm_timers[0][0];
                timer.phase_ticks = phase;
                const std::size_t route = direction == Dir::CW ? 0U : 1U;
                const auto old_compare = timer.active_compare[route];
                CHECK(old_compare != 0U);
                CHECK(motor.drivePercent(60.0f, direction).ok());
                CHECK(timer.shadow_compare[route] != old_compare);

                // Model an old centered pulse recurring while a zero write
                // is delayed. The injected interval is a counterexample to
                // an aperture guarantee, not a measured SDK latency.
                uint32_t last_rise = 0;
                unsigned rises = 0;
                for (uint32_t elapsed = 1; elapsed <= 138U; ++elapsed)
                {
                    if ((phase + elapsed) % timer.cycleTicks() == timer.cycleTicks() - old_compare)
                    {
                        last_rise = elapsed;
                        ++rises;
                    }
                }
                modeled_release_violation |= rises > 1U && last_rise > timer.cycleTicks();

                mock_hal::events.clear();
                const bool stop = phase % 15U == 0U;
                const bool fault = phase % 15U == 5U;
                if (stop)
                    CHECK(motor.stop().ok());
                else if (fault)
                {
                    mock_hal::pin_levels[10] = HIGH;
                    mock_hal::invokeInterrupt(10);
                    CHECK(motor.pollFaults().ok());
                }
                else
                    CHECK(motor.coast().ok());

                bool en_low = false;
                unsigned disconnected = 0;
                bool wrote_zero = false;
                for (const auto &event : mock_hal::events)
                {
                    if (event.kind == Kind::DigitalWrite && event.first == hw.en_pin && event.second == LOW)
                        en_low = true;
                    if (event.kind == Kind::PwmMatrix && event.second == static_cast<int>(SIG_GPIO_OUT_IDX))
                    {
                        CHECK(en_low);
                        CHECK(!wrote_zero);
                        CHECK(event.first == (disconnected == 0U ? hw.lpwm_pin : hw.rpwm_pin));
                        ++disconnected;
                    }
                    if (event.kind == Kind::Duty)
                    {
                        CHECK(en_low && disconnected == 2U);
                        wrote_zero = true;
                    }
                }
                CHECK(en_low && disconnected == 2U && wrote_zero);
                CHECK(mock_hal::pwm_routes.empty());
                CHECK(mock_hal::pin_levels[hw.lpwm_pin] == LOW);
                CHECK(mock_hal::pin_levels[hw.rpwm_pin] == LOW);

                if (stop)
                {
                    CHECK(motor.stop().ok());
                    CHECK(motor.start().ok());
                }
                else if (fault)
                {
                    mock_hal::pin_levels[10] = LOW;
                    mock_hal::invokeInterrupt(10);
                    CHECK(motor.clearFault().ok());
                }
                else
                {
                    CHECK(timer.active_compare[route] == old_compare);
                    CHECK(motor.coast().ok());
                }
                CHECK(mock_hal::pin_levels[hw.en_pin] == LOW);
                CHECK(mock_hal::pwm_routes.empty());
                CHECK(motor.readback().frequency_hz == 20000U);
                const auto fresh = direction == Dir::CW ? Dir::CCW : Dir::CW;
                CHECK(motor.drivePercent(24.0f, fresh).ok());
                CHECK(mock_hal::pwm_routes.size() == 2U);
                CHECK(mock_hal::unsafe_routes == 0U);
                CHECK(mock_hal::remaps_while_enabled == 0U);
                CHECK(timer.active_compare[route] == 0U);
                CHECK(timer.shadow_compare[route] == 0U);
                CHECK(timer.shadow_compare[1U - route] != 0U);
            }
        }
    }
    CHECK(modeled_release_violation);
}

void testCoastRestoreWaitRetainsSuppression()
{
    mock_hal::reset();
    HBridgeMotor motor;
    const auto hw = hardware();
    CHECK(motor.setup(hw, behavior(FreewheelMode::HiZ),
                      faultConfig(FaultAction::Coast), MotorCaptureConfig{}).ok());
    CHECK(motor.drivePercent(40.0f, Dir::CW).ok());
    delayMicroseconds(50);
    CHECK(motor.coast().ok());
    mock_hal::suppress_timer_empty = true;
    CHECK(!motor.drivePercent(16.0f, Dir::CCW).ok());
    CHECK(mock_hal::pwm_routes.empty());
    CHECK(mock_hal::pin_levels[hw.en_pin] == LOW);

    mock_hal::suppress_timer_empty = false;
    mock_hal::pwm_wait_hook = [] {
        mock_hal::pin_levels[10] = HIGH;
        mock_hal::invokeInterrupt(10);
    };
    CHECK(!motor.drivePercent(16.0f, Dir::CCW).ok());
    CHECK(mock_hal::pwm_routes.empty());
    CHECK(mock_hal::pin_levels[hw.en_pin] == LOW);
    CHECK(motor.pollFaults().ok());
    mock_hal::pin_levels[10] = LOW;
    mock_hal::invokeInterrupt(10);
    CHECK(motor.clearFault().ok());
    CHECK(motor.drivePercent(16.0f, Dir::CCW).ok());
    CHECK(mock_hal::unsafe_routes == 0U);
}

void testCoastRestoresOwnedRoutes()
{
    for (const auto mode : {MCPWM_UP_COUNTER, MCPWM_DOWN_COUNTER, MCPWM_UP_DOWN_COUNTER})
    {
        mock_hal::reset();
        HBridgeMotor first;
        HBridgeMotor second;
        auto hw = MotorMCPWMConfig{4, 5, 9, MCPWM_UNIT_1, MCPWM_TIMER_2, MCPWM2A, MCPWM2B};
        hw.counter = mode;
        const auto options = behavior(FreewheelMode::HiZ, 128, true);
        CHECK(first.setup(hardware(), options).ok());
        CHECK(second.setup(hw, options).ok());
        CHECK(first.drivePercent(30.0f, Dir::CW).ok());
        CHECK(second.drivePercent(40.0f, Dir::CCW).ok());
        delayMicroseconds(50);
        CHECK(second.coast().ok());
        CHECK(mock_hal::pwm_routes.count(2) == 1U && mock_hal::pwm_routes.count(3) == 1U);
        CHECK(mock_hal::pwm_routes.count(4) == 0U && mock_hal::pwm_routes.count(5) == 0U);
        CHECK(mock_hal::pin_levels[8] == HIGH);
        CHECK(second.softBrakeNow(128).ok());
        CHECK(mock_hal::pwm_routes.at(4) == std::make_pair(MCPWM_UNIT_1, MCPWM2A));
        CHECK(mock_hal::pwm_routes.at(5) == std::make_pair(MCPWM_UNIT_1, MCPWM2B));
        CHECK(mock_hal::unsafe_routes == 0U);
        CHECK(mock_hal::last_timer->active);
        CHECK(mock_hal::pin_levels[9] == LOW);
        mock_hal::fireTimer(mock_hal::last_timer);
        CHECK(mock_hal::pin_levels[9] == HIGH);
    }
}

void testExactDutyEndpointsAcrossStopAndRestart()
{
    mock_hal::reset();
    HBridgeMotor motor;
    auto hw = hardware(-1);
    hw.pwm_freq_hz = 16;
    hw.input_max = 65535;
    const auto options = behavior(FreewheelMode::HiZ_Awake);
    CHECK(motor.setup(hw, options).ok());
    auto &timer = mock_hal::pwm_timers[0][0];
    CHECK(motor.drive(3, Dir::CW).ok());
    delayMicroseconds(63000U);
    CHECK(timer.active_compare[0] > 0U);
    CHECK(motor.stop().ok());
    CHECK(timer.shadow_compare[0] == 0U);
    CHECK(motor.start().ok());
    CHECK(timer.active_compare[0] == 0U);
    CHECK(mock_hal::unsafe_routes == 0U);

    hw.input_max = 97; // Multiplication by the cached ratio rounds below 100.
    hw.pwm_freq_hz = 500000;
    CHECK(motor.setup(hw, options).ok());
    CHECK(motor.drive(hw.input_max, Dir::CW).ok());
    delayMicroseconds(3);
    CHECK(timer.highTicksForCompare(timer.active_compare[0]) == timer.cycleTicks());
    CHECK(motor.readback().duty_a_percent == 100.0f);
    CHECK(motor.status().commanded_a_percent == 100.0f);
}

void testFailedIsolationNeverClaimsDisabledOrReleasesOwnership()
{
    mock_hal::reset();
    HBridgeMotor owner;
    HBridgeMotor contender;
    const auto hw = hardware(-1);
    mock_hal::gpio_level_result = -1;
    CHECK(owner.setup(hw).error == MotorSetupError::ContainmentFailed);
    CHECK(owner.status().output_mode == MotorOutputMode::Uncertain);
    CHECK(owner.setup(hw).error == MotorSetupError::ContainmentFailed);
    CHECK(owner.status().output_mode == MotorOutputMode::Uncertain);
    CHECK(contender.setup(hw).error == MotorSetupError::ResourceConflict);
    mock_hal::gpio_level_result = ESP_OK;
    CHECK(owner.setup(hw).ok());
}

void testRestartRequiresNewTimerEventsAndFaultFreeEnable()
{
    mock_hal::reset();
    HBridgeMotor motor;
    const auto options = behavior(FreewheelMode::HiZ_Awake);
    CHECK(motor.setup(hardware(), options, faultConfig(FaultAction::Coast), MotorCaptureConfig{}).ok());
    CHECK(motor.stop().ok());
    mock_hal::suppress_timer_empty = true;
    CHECK(motor.start().error == MotorOperationError::HardwareStartFailed);
    CHECK(!motor.status().mcpwm_running);
    CHECK(mock_hal::pwm_routes.empty());
    CHECK(!motor.readback().enable_asserted);

    mock_hal::suppress_timer_empty = false;
    mock_hal::pwm_wait_hook = []() {
        mock_hal::pin_levels[10] = HIGH;
        mock_hal::invokeInterrupt(10);
    };
    CHECK(motor.start().error == MotorOperationError::FaultActive);
    CHECK(!motor.readback().enable_asserted);
    CHECK(!motor.status().mcpwm_running);
    CHECK(mock_hal::pwm_routes.empty());
    CHECK(motor.status().output_mode == MotorOutputMode::Disabled);
    CHECK(motor.pollFaults().ok());
}

void testPollFaultsIsAvailableThroughInterface()
{
    mock_hal::reset();
    HBridgeMotor motor;
    motor.setup(hardware(), behavior(FreewheelMode::HiZ),
                faultConfig(FaultAction::Coast), MotorCaptureConfig{});
    IMotorDriver *driver = &motor;
    CHECK(driver->isSetupComplete());

    motor.setSpeed(700, Dir::CW);
    mock_hal::pin_levels[10] = HIGH;
    mock_hal::invokeInterrupt(10);
    driver->pollFaults();
    checkFaultActionState(FaultAction::Coast);
}
void checkReleasedPin(int pin)
{
    CHECK(!mock_hal::pin_pulls.at(pin).first);
    CHECK(mock_hal::pin_modes.at(pin) == GPIO_MODE_INPUT);
    CHECK(mock_hal::pin_pulls.at(pin).second);
    CHECK(mock_hal::pwm_routes.count(pin) == 0);
}

void testReleasedMotorPins(bool with_enable)
{
    mock_hal::reset();
    const auto hw = hardware(with_enable ? 8 : -1);
    for (const bool reuse : {false, true})
    {
        {
            HBridgeMotor motor;
            CHECK(motor.setup(hw).ok());
            CHECK(!mock_hal::pin_pulls.at(hw.lpwm_pin).second);
            CHECK(!mock_hal::pin_pulls.at(hw.rpwm_pin).second);
            CHECK(motor.drive(600, Dir::CW).ok());
            if (reuse)
            {
                CHECK(mock_hal::pwm_routes.at(hw.lpwm_pin).second == hw.sig_l);
                CHECK(mock_hal::pwm_routes.at(hw.rpwm_pin).second == hw.sig_r);
                if (with_enable)
                {
                    CHECK(mock_hal::pin_modes.at(hw.en_pin) == OUTPUT);
                    CHECK(mock_hal::pin_levels.at(hw.en_pin) == HIGH);
                    CHECK(!mock_hal::pin_pulls.at(hw.en_pin).second);
                }
            }
            CHECK(motor.stop().ok());
            for (const int pin : {hw.lpwm_pin, hw.rpwm_pin, hw.en_pin})
            {
                if (pin < 0)
                    continue;
                CHECK(mock_hal::pin_modes.at(pin) == OUTPUT);
                CHECK(mock_hal::pin_levels.at(pin) == LOW);
                CHECK(!mock_hal::pin_pulls.at(pin).first);
                CHECK(!mock_hal::pin_pulls.at(pin).second);
                CHECK(mock_hal::pwm_routes.count(pin) == 0);
            }
            HBridgeMotor contender;
            CHECK(contender.setup(hw).error == MotorSetupError::ResourceConflict);
            mock_hal::events.clear();
        }
        checkReleasedPin(hw.lpwm_pin);
        checkReleasedPin(hw.rpwm_pin);
        if (with_enable)
        {
            checkReleasedPin(hw.en_pin);
            std::vector<int> released;
            for (const auto &event : mock_hal::events)
                if (event.kind == mock_hal::EventKind::GpioInputRelease)
                    released.push_back(event.first);
            CHECK((released == std::vector<int>{hw.lpwm_pin, hw.rpwm_pin, hw.en_pin}));
        }
    }
}

void testFailedTeardownRetainsOwnedPins()
{
    for (const bool with_enable : {false, true})
    {
        mock_hal::reset();
        const auto hw = hardware(with_enable ? 8 : -1);
        HBridgeMotor owner;
        HBridgeMotor contender;
        CHECK(owner.setup(hw).ok());
        CHECK(owner.drive(600, Dir::CW).ok());
        mock_hal::stop_result = -1;
        CHECK(owner.stop().error == MotorOperationError::HardwareStopFailed);
        CHECK(owner.setup(hw).error == MotorSetupError::ContainmentFailed);
        CHECK(contender.setup(hw).error == MotorSetupError::ResourceConflict);
        CHECK(mock_hal::pwm_routes.empty());
        CHECK(mock_hal::pin_modes.at(hw.lpwm_pin) == OUTPUT);
        CHECK(mock_hal::pin_modes.at(hw.rpwm_pin) == OUTPUT);
        CHECK(mock_hal::pin_levels.at(hw.lpwm_pin) == LOW);
        CHECK(mock_hal::pin_levels.at(hw.rpwm_pin) == LOW);
        if (with_enable)
            CHECK(mock_hal::pin_modes.at(hw.en_pin) == OUTPUT && mock_hal::pin_levels.at(hw.en_pin) == LOW);
        mock_hal::stop_result = ESP_OK;

        for (const int pin : {hw.lpwm_pin, hw.en_pin})
        {
            if (pin < 0)
                continue;
            mock_hal::gpio_release_failure_pin = pin;
            CHECK(owner.setup(hw).error == MotorSetupError::ContainmentFailed);
            CHECK(contender.setup(hw).error == MotorSetupError::ResourceConflict);
            CHECK(mock_hal::pwm_routes.empty());
            CHECK(mock_hal::pin_modes.at(hw.lpwm_pin) == OUTPUT);
            CHECK(mock_hal::pin_modes.at(hw.rpwm_pin) == OUTPUT);
            CHECK(mock_hal::pin_levels.at(hw.lpwm_pin) == LOW);
            CHECK(mock_hal::pin_levels.at(hw.rpwm_pin) == LOW);
            if (with_enable)
                CHECK(mock_hal::pin_modes.at(hw.en_pin) == OUTPUT && mock_hal::pin_levels.at(hw.en_pin) == LOW);
        }
        mock_hal::gpio_release_failure_pin = -1;
        CHECK(owner.setup(MotorMCPWMConfig{}).error == MotorSetupError::InvalidPwmPin);
        checkReleasedPin(hw.lpwm_pin);
        checkReleasedPin(hw.rpwm_pin);
        if (with_enable)
            checkReleasedPin(hw.en_pin);
        CHECK(contender.setup(hw).ok());
    }
}

void testResourceOwnershipAndMapping()
{
    mock_hal::reset();
    MotorMCPWMConfig other = hardware();
    other.lpwm_pin = 4;
    other.rpwm_pin = 5;
    other.en_pin = 9;
    {
        HBridgeMotor owner;
        CHECK(owner.setup(hardware()).ok());
        CHECK(owner.drive(600, Dir::CW).ok());
        const MotorDriverStatus before = owner.status();
        const OutputState outputs = outputState();
        mock_hal::events.clear();
        {
            HBridgeMotor contender;
            CHECK(contender.setup(other).error == MotorSetupError::ResourceConflict);
            CHECK(mock_hal::events.empty());
        }
        CHECK(mock_hal::events.empty());
        checkState(outputState(), outputs);
        CHECK(owner.status().commanded_a_percent == before.commanded_a_percent);
        CHECK(owner.stop().ok());
        HBridgeMotor contender;
        CHECK(contender.setup(other).error == MotorSetupError::ResourceConflict);
        other.timer = MCPWM_TIMER_1;
        CHECK(contender.setup(other).error == MotorSetupError::InvalidTimerSignals);
        other.sig_l = MCPWM1A;
        other.sig_r = MCPWM1B;
        CHECK(contender.setup(other).ok());
        CHECK(contender.drive(500, Dir::CCW).ok());
        CHECK(owner.start().ok());
        CHECK(owner.drive(400, Dir::CW).ok());
        CHECK(mock_hal::running[0][0] && mock_hal::running[0][1]);
        CHECK(contender.setup(MotorMCPWMConfig{}).error == MotorSetupError::InvalidPwmPin);
        CHECK(mock_hal::running[0][0]);
    }
    checkReleasedPin(2);
    checkReleasedPin(3);
    checkReleasedPin(8);
    for (int invalid = 0; invalid < 4; ++invalid)
    {
        HBridgeMotor motor;
        MotorMCPWMConfig bad = hardware();
        if (invalid == 0) bad.timer = MCPWM_TIMER_1;
        if (invalid == 1) { bad.sig_l = MCPWM0B; bad.sig_r = MCPWM0A; }
        if (invalid == 2) bad.unit = static_cast<mcpwm_unit_t>(2);
        if (invalid == 3) bad.timer = static_cast<mcpwm_timer_t>(3);
        mock_hal::events.clear();
        CHECK(motor.setup(bad).error == MotorSetupError::InvalidTimerSignals);
        CHECK(mock_hal::events.empty());
    }
    {
        HBridgeMotor motor;
        mock_hal::timer_create_result = -1;
        CHECK(!motor.setup(hardware()).ok());
        mock_hal::timer_create_result = ESP_OK;
        HBridgeMotor replacement;
        CHECK(replacement.setup(hardware()).ok());
    }
    {
        HBridgeMotor owner;
        CHECK(owner.setup(hardware()).ok());
        mock_hal::stop_result = -1;
        CHECK(owner.setup(other).error == MotorSetupError::ContainmentFailed);
        HBridgeMotor contender;
        CHECK(contender.setup(hardware()).error == MotorSetupError::ResourceConflict);
        mock_hal::stop_result = ESP_OK;
        CHECK(owner.setup(other).ok());
        CHECK(contender.setup(hardware()).ok());
    }
    {
        HBridgeMotor owner;
        CHECK(owner.setup(hardware()).ok());
        mock_hal::gpio_release_failure_pin = hardware().lpwm_pin;
        CHECK(owner.setup(other).error == MotorSetupError::ContainmentFailed);
        HBridgeMotor contender;
        CHECK(contender.setup(hardware()).error == MotorSetupError::ResourceConflict);
        mock_hal::gpio_release_failure_pin = -1;
        CHECK(owner.setup(other).ok());
        CHECK(contender.setup(hardware()).ok());
    }
}

void testHardwareFaultUnitIsolationAndPins()
{
    mock_hal::reset();
    MotorMCPWMConfig other{4, 5, 9, MCPWM_UNIT_0, MCPWM_TIMER_1, MCPWM1A, MCPWM1B};
    {
        HBridgeMotor owner;
        CHECK(owner.setup(hardware(), behavior(FreewheelMode::HiZ), MotorSafetyConfig{},
                          MotorCaptureConfig{}, hardwareFaultConfig(HardwareFaultMode::OneShot)).ok());
        HBridgeMotor contender;
        mock_hal::events.clear();
        CHECK(contender.setup(other).error == MotorSetupError::ResourceConflict);
        CHECK(mock_hal::events.empty());
        other.unit = MCPWM_UNIT_1;
        CHECK(contender.setup(other).ok());
    }
    other.unit = MCPWM_UNIT_0;
    {
        HBridgeMotor owner;
        CHECK(owner.setup(hardware()).ok());
        HBridgeMotor contender;
        CHECK(contender.setup(other, behavior(FreewheelMode::HiZ), MotorSafetyConfig{},
                               MotorCaptureConfig{}, hardwareFaultConfig(HardwareFaultMode::OneShot)).error ==
              MotorSetupError::ResourceConflict);
        MotorCaptureConfig capture{};
        capture.cap_gpio = 2;
        CHECK(contender.setup(other, behavior(FreewheelMode::HiZ), MotorSafetyConfig{}, capture).error ==
              MotorSetupError::ResourceConflict);
    }
}

void assertRecoveryBarrier(HBridgeMotor &motor)
{
    CHECK(motor.hasFault() && motor.status().recovery_inhibited);
    CHECK(motor.drive(400, Dir::CW).error == MotorOperationError::FaultActive);
    CHECK(motor.setHardBrake().error == MotorOperationError::FaultActive);
    CHECK(motor.softBrakeNow(300).error == MotorOperationError::FaultActive);
    CHECK(motor.start().error == MotorOperationError::FaultActive);
    CHECK(motor.disableOutputs().ok());
}

void testRecoveryFailuresAndNewAssertions()
{
    for (int boundary = 0; boundary < 5; ++boundary)
    {
        mock_hal::reset();
        HBridgeMotor motor;
        CHECK(motor.setup(hardware(), behavior(FreewheelMode::HiZ), faultConfig(FaultAction::HardBrake),
                          MotorCaptureConfig{}, hardwareFaultConfig(HardwareFaultMode::OneShot)).ok());
        mock_hal::pin_levels[10] = HIGH;
        mock_hal::invokeInterrupt(10);
        CHECK(motor.pollFaults().ok());
        CHECK(motor.clearFault().error == MotorOperationError::FaultActive);
        mock_hal::pin_levels[10] = LOW;
        mock_hal::invokeInterrupt(10);
        if (boundary == 0) mock_hal::duty_result = -1;
        if (boundary == 1) mock_hal::fault_init_result = -1;
        if (boundary == 2) { CHECK(motor.stop().ok()); mock_hal::start_result = -1; }
        if (boundary == 3) mock_hal::fault_deinit_result = -1;
        if (boundary == 4) mock_hal::fault_mode_result = -1;
        CHECK(!motor.clearFault().ok());
        // Restore HAL availability without clearing the library barrier.
        mock_hal::duty_result = ESP_OK;
        mock_hal::fault_init_result = ESP_OK;
        mock_hal::start_result = ESP_OK;
        mock_hal::fault_deinit_result = ESP_OK;
        mock_hal::fault_mode_result = ESP_OK;
        assertRecoveryBarrier(motor);
        CHECK(motor.clearFault().ok());
        CHECK(!motor.hasFault());
        checkState(outputState(), {LOW, 0.0f, 0.0f, false});
        CHECK(motor.drive(400, Dir::CW).ok());
    }
    mock_hal::reset();
    HBridgeMotor motor;
    CHECK(motor.setup(hardware(), behavior(FreewheelMode::HiZ), faultConfig(FaultAction::HardBrake),
                      MotorCaptureConfig{}).ok());
    mock_hal::pin_levels[10] = HIGH;
    mock_hal::invokeInterrupt(10);
    CHECK(motor.pollFaults().ok());
    mock_hal::pin_levels[10] = LOW;
    mock_hal::invokeInterrupt(10);
    mock_hal::duty_hook = []() {
        mock_hal::pin_levels[10] = HIGH;
        mock_hal::invokeInterrupt(10);
        mock_hal::pin_levels[10] = LOW;
        mock_hal::invokeInterrupt(10);
    };
    CHECK(motor.clearFault().error == MotorOperationError::FaultActive);
    CHECK(motor.hasFault() && motor.status().software_fault_pending);
    assertRecoveryBarrier(motor);
    CHECK(motor.clearFault().ok());
    CHECK(!motor.hasFault());
}

void testHardwareFaultArrivesDuringRecovery()
{
    for (int boundary = 0; boundary < 2; ++boundary)
    {
        mock_hal::reset();
        HBridgeMotor motor;
        CHECK(motor.setup(hardware(), behavior(FreewheelMode::HiZ), MotorSafetyConfig{},
                          MotorCaptureConfig{}, hardwareFaultConfig(HardwareFaultMode::OneShot)).ok());
        CHECK(motor.drive(400, Dir::CW).ok());
        mock_hal::pin_levels[12] = HIGH;
        mock_hal::invokeInterrupt(12);
        CHECK(motor.clearFault().error == MotorOperationError::FaultActive);
        mock_hal::pin_levels[12] = LOW;
        mock_hal::invokeInterrupt(12);
        const auto assertion = []() {
            mock_hal::pin_levels[12] = HIGH;
            mock_hal::invokeInterrupt(12);
            mock_hal::pin_levels[12] = LOW;
            mock_hal::invokeInterrupt(12);
        };
        if (boundary == 0) mock_hal::fault_mode_hook = assertion;
        else { CHECK(motor.stop().ok()); mock_hal::start_hook = assertion; }
        CHECK(motor.clearFault().error == MotorOperationError::FaultActive);
        CHECK(motor.status().hardware_fault_latched);
        assertRecoveryBarrier(motor);
        CHECK(motor.clearFault().ok());
        CHECK(!motor.hasFault());
        checkState(outputState(), {LOW, 0.0f, 0.0f, false});
    }
}

void testDeadtimeForwardingAndRange()
{
    for (uint32_t count : {0U, 500U, 65535U, 65536U})
    {
        mock_hal::reset();
        HBridgeMotor motor;
        MotorMCPWMConfig hw = hardware();
        hw.use_deadtime = true;
        hw.deadtime_red_ns = count;
        hw.deadtime_fed_ns = count == 0 ? 123U : count;
        const MotorSetupResult setup = motor.setup(hw);
        if (count > 65535U)
        {
            CHECK(setup.error == MotorSetupError::InvalidDeadtime);
            CHECK(mock_hal::events.empty());
        }
        else
        {
            CHECK(setup.ok());
            CHECK(mock_hal::deadtime_red == hw.deadtime_red_ns);
            CHECK(mock_hal::deadtime_fed == hw.deadtime_fed_ns);
        }
    }
}

// Last test deliberately leaves one reservation quarantined until process exit.
void testDestructionQuarantinesFailedTeardown()
{
    mock_hal::reset();
    {
        HBridgeMotor owner;
        CHECK(owner.setup(hardware()).ok());
        CHECK(owner.drive(600, Dir::CW).ok());
        mock_hal::stop_result = -1;
    }
    CHECK(mock_hal::pwm_routes.empty());
    for (const int pin : {hardware().lpwm_pin, hardware().rpwm_pin, hardware().en_pin})
    {
        CHECK(mock_hal::pin_modes.at(pin) == OUTPUT);
        CHECK(mock_hal::pin_levels.at(pin) == LOW);
    }
    mock_hal::stop_result = ESP_OK;
    mock_hal::events.clear();
    HBridgeMotor contender;
    CHECK(contender.setup(hardware()).error == MotorSetupError::ResourceConflict);
    CHECK(mock_hal::events.empty());
}
} // namespace

int main()
{
    const std::vector<std::pair<const char *, std::function<void()>>> tests{
        {"HiZ setup is inactive without enabled window", testHiZSetupIsInactiveWithoutEnabledWindow},
        {"setup overloads apply configured freewheel", testSetupOverloadsApplyConfiguredFreewheel},
        {"HiZ_Awake matches explicit freewheel", testHiZAwakeMatchesExplicitFreewheel},
        {"DitherBrake applies configured behavior", testDitherBrakeAppliesConfiguredBehavior},
        {"zero drive is explicit and dither zero uses configured coast", testZeroDriveIsExplicitAndDitherZeroUsesConfiguredCoast},
        {"same freewheel mode is a true no-op", testSameFreewheelModeIsTrueNoOp},
        {"changed freewheel mode stops dither in coast", testChangedFreewheelModeStopsDitherInCoast},
        {"dither period is preserved", testDitherPeriodIsPreserved},
        {"repeated setup cleans previous state", testRepeatedSetupCleansPreviousState},
        {"repeated setup disables previous deadtime", testRepeatedSetupDisablesPreviousDeadtime},
        {"drive, brake, freewheel, and lifecycle", testDriveBrakeFreewheelAndLifecycle},
        {"destructor disables outputs", testDestructorDisablesOutputs},
        {"default fault disables outputs and latches", testDefaultFaultDisablesOutputsAndLatches},
        {"one-shot coast fault and latch", testOneShotCoastFaultAndLatch},
        {"one-shot disable-output fault and latch", testOneShotDisableOutputsFaultAndLatch},
        {"fault stops dither", testFaultStopsDither},
        {"coast preserves stopped MCPWM state", testCoastPreservesStoppedMcpwmState},
        {"clear preserves pending fault notification", testClearPreservesPendingFaultNotification},
        {"level-follow supports every fault action", testLevelFollowAllFaultActions},
        {"active fault at setup supports every action", testActiveFaultAtSetupAllActions},
        {"EN-absent fault capabilities", testEnAbsentFaultCapabilities},
        {"fault-disabled and legacy config compatibility", testFaultDisabledAndLegacyConfigCompatibility},
        {"runtime frequency reconfiguration", testRuntimeFrequencyReconfiguration},
        {"drive frequency reconfiguration publishes quiet state", testDriveFrequencyReconfigurationPublishesQuietState},
        {"frequency reconfiguration failure status is truthful", testFrequencyReconfigurationFailureStatusIsTruthful},
        {"invalid hardware configuration fails safely", testInvalidHardwareConfigurationFailsSafely},
        {"failed repeated setup clears previous configuration", testFailedRepeatedSetupClearsPreviousConfiguration},
        {"setup hardware failures remain controlled", testSetupHardwareFailuresRemainControlled},
        {"stale dither callback cannot overwrite drive", testStaleDitherCallbackCannotOverwriteDrive},
        {"stale dither scheduling preserves newer diagnostics", testStaleDitherSchedulingPreservesNewerDiagnostics},
        {"repeated setup containment failures take precedence", testRepeatedSetupContainmentFailuresTakePrecedence},
        {"dither write failure terminates and contains", testDitherWriteFailureTerminatesAndContains},
        {"dither timer failure terminates and contains", testDitherTimerFailureTerminatesAndContains},
        {"superseded dither timer failure preserves newer drive", testSupersededDitherTimerFailurePreservesNewerDrive},
        {"changed semantics are literal", testChangedSemanticsAreLiteral},
        {"stopped coast contract", testStoppedCoastContract},
        {"capture behavior remains unchanged", testCaptureBehaviorRemainsUnchanged},
        {"structured results and readback", testStructuredResultsAndReadback},
        {"lifecycle and hardware-write failures are reported", testLifecycleAndHardwareWriteFailuresAreReported},
        {"hardware one-shot fault config and rearm", testHardwareFaultOneShotConfigurationAndRearm},
        {"hardware cycle-by-cycle status follows input", testHardwareFaultCycleByCycleStatusFollowsInput},
        {"invalid and failed hardware fault setup fails safe", testInvalidAndFailedHardwareFaultSetupFailsSafe},
        {"commissioning output API disabled by default", testCommissioningOutputApiIsDisabledByDefault},
        {"center-aligned waveform frequency and safety", testCenterAlignedWaveformUsesRequestedFrequency},
        {"frequency boundaries, modes and resolution failure", testFrequencyModeBoundariesAndResolutionFailure},
        {"low-frequency reuse waits and retains ownership on timeout", testImmediateLowFrequencySetupAndReuse},
        {"retune isolates pending and active compares", testRetuneSeparatesActiveAndPendingOutput},
        {"retune preserves fault ownership", testRetunePreservesFaultOwnership},
        {"counter modes and modeled electrical endpoints", testCounterModesAndElectricalEndpointsInModel},
        {"500 kHz percentage midpoint and raw equivalence", testPercentMidpointAndRawEquivalence},
        {"percentage range and input scales", testPercentRangeAndOtherInputScales},
        {"EN-first pad suppression and quiet restore across PWM phases", testEnFirstPwmSuppression},
        {"coast restore stays suppressed during timeout and fault", testCoastRestoreWaitRetainsSuppression},
        {"coast restores only its owned routes across counter modes", testCoastRestoresOwnedRoutes},
        {"exact duty endpoints across stop and restart", testExactDutyEndpointsAcrossStopAndRestart},
        {"failed GPIO isolation retains uncertain ownership", testFailedIsolationNeverClaimsDisabledOrReleasesOwnership},
        {"restart requires events and a fault-free enable", testRestartRequiresNewTimerEventsAndFaultFreeEnable},
        {"frequency readback tracks reconfiguration", testFrequencyReadbackTracksSuccessfulReconfiguration},
        {"pollFaults is available through interface", testPollFaultsIsAvailableThroughInterface},
        {"EN-equipped stop, destruction and GPIO reuse", [] { testReleasedMotorPins(true); }},
        {"EN-less stop, destruction and GPIO reuse", [] { testReleasedMotorPins(false); }},
        {"failed motor pin teardown retains and recovers ownership", testFailedTeardownRetainsOwnedPins},
        {"resource ownership and mapping", testResourceOwnershipAndMapping},
        {"hardware fault unit and pin isolation", testHardwareFaultUnitIsolationAndPins},
        {"recovery barriers and assertions", testRecoveryFailuresAndNewAssertions},
        {"hardware assertion during recovery", testHardwareFaultArrivesDuringRecovery},
        {"deadtime forwarding and range", testDeadtimeForwardingAndRange},
        {"destructor quarantine", testDestructionQuarantinesFailedTeardown},
    };

    for (const auto &test : tests)
    {
        try
        {
            test.second();
            std::cout << "[PASS] " << test.first << '\n';
        }
        catch (const std::exception &error)
        {
            std::cerr << "[FAIL] " << test.first << ": " << error.what() << '\n';
            return 1;
        }
    }

    std::cout << "[SUMMARY] " << tests.size() << " tests / "
              << g_assertions << " assertions\n";
    return 0;
}
