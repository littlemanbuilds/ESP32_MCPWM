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
#include <cmath>
#include <functional>
#include <iostream>
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
    const int en_low = findEvent(mock_hal::EventKind::DigitalWrite, 8, LOW);
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

    mock_hal::reset();
    HBridgeMotor freewheel_motor;
    freewheel_motor.setup(hardware(), behavior(FreewheelMode::DitherBrake, 0, true));
    freewheel_motor.setSpeed(700, Dir::CW);
    freewheel_motor.setFreewheel();
    checkState(outputState(), {LOW, 0.0f, 0.0f, false});

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

void testClearDiscardsPendingFaultWork()
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
    CHECK(fault_callback_count == 0);
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
        CHECK(!motor.hasFault());
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
    CHECK(motor.status().output_mode == MotorOutputMode::Coast);
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
    CHECK(contained.status().output_mode == MotorOutputMode::Coast);
    checkState(outputState(), {LOW, 0.0f, 0.0f, false});

    mock_hal::reset();
    HBridgeMotor uncertain;
    CHECK(uncertain.setup(hardware(-1), behavior(FreewheelMode::HiZ)).ok());
    CHECK(uncertain.drive(600, Dir::CCW).ok());
    mock_hal::duty_result = -1;
    const MotorOperationResult output_failed = uncertain.reconfigureFrequency(25000);
    CHECK(output_failed.error == MotorOperationError::HardwareWriteFailed);
    CHECK(output_failed.changed);
    CHECK(uncertain.status().output_mode == MotorOutputMode::Uncertain);

    mock_hal::duty_result = ESP_OK;
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
    CHECK(duty_failure.status().output_mode == MotorOutputMode::Uncertain);
    CHECK(duty_failure.readback().valid);
    mock_hal::duty_result = ESP_OK;

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
    CHECK(stop_failure.status().output_mode == MotorOutputMode::Uncertain);
    mock_hal::stop_result = ESP_OK;

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
    CHECK_NEAR(readback.duty_a_percent, status.commanded_a_percent, 0.001f);
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
        {"clear discards pending fault work", testClearDiscardsPendingFaultWork},
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
        {"frequency readback tracks reconfiguration", testFrequencyReadbackTracksSuccessfulReconfiguration},
        {"pollFaults is available through interface", testPollFaultsIsAvailableThroughInterface},
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
