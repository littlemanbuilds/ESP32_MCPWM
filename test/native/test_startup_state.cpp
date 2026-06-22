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

void check(bool condition, const char *expression, int line)
{
    if (!condition)
        throw TestFailure("line " + std::to_string(line) + ": " + expression);
}

#define CHECK(expression) check((expression), #expression, __LINE__)

void checkNear(float actual, float expected, float tolerance, int line)
{
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
    motor.stop();
    checkAction();

    // Configuration-only setters remain available without changing outputs.
    motor.setSoftBrakePWM(256);
    motor.setFreewheelMode(FreewheelMode::HiZ);
    checkAction();
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

void testZeroDitherUsesConfiguredCoastEverywhere()
{
    mock_hal::reset();
    HBridgeMotor hi_z_motor;
    hi_z_motor.setup(hardware(), behavior(FreewheelMode::DitherBrake, 256, true));
    CHECK(mock_hal::last_timer->active);

    hi_z_motor.setSoftBrakePWM(0);
    checkState(outputState(), {LOW, 0.0f, 0.0f, false});
    hi_z_motor.softBrakeNow(0);
    checkState(outputState(), {LOW, 0.0f, 0.0f, false});
    hi_z_motor.setSpeed(0, Dir::CW);
    checkState(outputState(), {LOW, 0.0f, 0.0f, false});

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

void testDefaultHardBrakeFaultAndLatch()
{
    mock_hal::reset();
    fault_callback_count = 0;
    last_fault_active = false;
    HBridgeMotor motor;
    motor.setFaultCallback(&onFault, nullptr);

    MotorSafetyConfig safety{};
    CHECK(safety.fault_action == FaultAction::HardBrake);
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
    checkFaultActionState(FaultAction::HardBrake);
    CHECK(fault_callback_count == 1);
    CHECK(last_fault_active);
    motor.pollFaults();
    CHECK(fault_callback_count == 1);

    checkOutputCommandsInhibited(motor, FaultAction::HardBrake);

    // A still-active input cannot be cleared without a new edge.
    motor.clearFault();
    CHECK(motor.hasFault());
    checkFaultActionState(FaultAction::HardBrake);

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
    CHECK(legacy.fault_action == FaultAction::HardBrake);
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

    CHECK(motor.reconfigureFrequency(25000));
    checkState(outputState(), {LOW, 0.0f, 0.0f, true});

    mock_hal::frequency_result = -1;
    CHECK(!motor.reconfigureFrequency(30000));
    checkState(outputState(), {LOW, 0.0f, 0.0f, false});
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
    CHECK(timer_start_failure.isSetupComplete());
    checkState(outputState(), {LOW, 0.0f, 0.0f, false});
}

HBridgeMotor *race_motor = nullptr;
void issueDriveDuringDitherWrite()
{
    race_motor->setSpeed(700, Dir::CW);
}

void testStaleDitherCallbackCannotOverwriteDrive()
{
    mock_hal::reset();
    HBridgeMotor motor;
    race_motor = &motor;
    motor.setup(hardware(), behavior(FreewheelMode::DitherBrake, 256, true));
    mock_hal::duty_hook = &issueDriveDuringDitherWrite;

    mock_hal::fireTimer(mock_hal::last_timer);

    const OutputState state = outputState();
    CHECK(state.en == HIGH);
    CHECK_NEAR(state.a, 700.0f * 100.0f / 1023.0f, 0.001f);
    CHECK_NEAR(state.b, 0.0f, 0.001f);
    CHECK(!state.timer_active);
    race_motor = nullptr;
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
        {"zero dither uses configured coast everywhere", testZeroDitherUsesConfiguredCoastEverywhere},
        {"same freewheel mode is a true no-op", testSameFreewheelModeIsTrueNoOp},
        {"changed freewheel mode stops dither in coast", testChangedFreewheelModeStopsDitherInCoast},
        {"dither period is preserved", testDitherPeriodIsPreserved},
        {"repeated setup cleans previous state", testRepeatedSetupCleansPreviousState},
        {"repeated setup disables previous deadtime", testRepeatedSetupDisablesPreviousDeadtime},
        {"drive, brake, freewheel, and lifecycle", testDriveBrakeFreewheelAndLifecycle},
        {"destructor disables outputs", testDestructorDisablesOutputs},
        {"default hard-brake fault and latch", testDefaultHardBrakeFaultAndLatch},
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
        {"invalid hardware configuration fails safely", testInvalidHardwareConfigurationFailsSafely},
        {"setup hardware failures remain controlled", testSetupHardwareFailuresRemainControlled},
        {"stale dither callback cannot overwrite drive", testStaleDitherCallbackCannotOverwriteDrive},
        {"capture behavior remains unchanged", testCaptureBehaviorRemainsUnchanged},
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

    return 0;
}
