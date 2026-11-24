/**
 * MIT License
 *
 * @brief ESP32 MCPWM H-bridge driver implementation.
 *
 * @file HBridgeMotor.cpp
 * @author Little Man Builds (Darren Osborne)
 * @date 2025-08-28
 * @copyright Copyright (c) 2026 Little Man Builds
 *
 */

#include "HBridgeMotor.h"

#include <driver/gpio.h>
#include <esp_rom_gpio.h>
#include <soc/gpio_sig_map.h>
#include <soc/mcpwm_reg.h>
#include <soc/soc.h>
#include <soc/soc_caps.h>

#include <cmath>

/**
 * @brief Clamp a value to the inclusive requested range.
 *
 * @tparam T Comparable value type.
 * @param value Value to clamp.
 * @param low Inclusive lower bound.
 * @param high Inclusive upper bound.
 * @return Value constrained to the requested range.
 */
template <typename T> static inline T clampValue(T value, T low, T high)
{
    return (value < low) ? low : (value > high) ? high : value;
}

namespace
{
constexpr uint32_t edge_timer_hz = 1000000U;
// At 20 kHz this gives 50 ticks per half cycle, retaining 2-point duty resolution.
// Both supported targets divide the default 10 MHz group clock by five exactly.
constexpr uint32_t center_timer_hz = 2000000U;
// Initialize the SDK's module with a small nondegenerate period while the pins
// are isolated. Changing the cached resolution before init also writes an
// unclocked peripheral on first use in this SDK, so start with its preset clock.
constexpr uint32_t bootstrap_hz = edge_timer_hz / 64U;
// At the selected clocks even the largest register period lasts at most 65,536 us.
// The extra time allows synchronization and polling overhead, never proves a stop.
constexpr uint32_t timer_event_timeout_us = 70000U;

/// @brief Read the owned timer's programmed period register.
uint32_t timerPeriod(mcpwm_unit_t unit, mcpwm_timer_t timer)
{
    const uint32_t stride = MCPWM_TIMER1_CFG0_REG(unit) - MCPWM_TIMER0_CFG0_REG(unit);
    const uint32_t config = REG_READ(MCPWM_TIMER0_CFG0_REG(unit) + static_cast<uint32_t>(timer) * stride);
    return (config & MCPWM_TIMER0_PERIOD_M) >> MCPWM_TIMER0_PERIOD_S;
}

/// @brief Wait for the selected raw event without taking a driver lock.
bool waitForTimerEvent(mcpwm_unit_t unit, uint32_t mask)
{
    const uint32_t started = static_cast<uint32_t>(micros());
    while ((REG_READ(MCPWM_INT_RAW_REG(unit)) & mask) == 0U)
    {
        const uint32_t elapsed = static_cast<uint32_t>(micros()) - started;
        if (elapsed >= timer_event_timeout_us)
            return false;
        if (elapsed < 200U)
            delayMicroseconds(10);
        else
            delay(1);
    }
    return true;
}

/// @brief Return the resolution required for a drive counter mode.
///
/// @param mode Selected legacy MCPWM mode.
/// @return Timer clock in Hz after the timer divider.
uint32_t timerResolution(mcpwm_counter_type_t mode)
{
    return mode == MCPWM_UP_DOWN_COUNTER ? center_timer_hz : edge_timer_hz;
}

/// @brief Translate physical PWM frequency into the legacy timer API argument.
///
/// @param mode Selected legacy MCPWM mode.
/// @param requested_hz Desired GPIO waveform frequency in Hz.
/// @return Legacy frequency argument, or zero if outside the usable timer range.
uint32_t legacyFrequency(mcpwm_counter_type_t mode, int requested_hz)
{
    if (requested_hz <= 0)
        return 0;
    const auto hz = static_cast<uint32_t>(requested_hz);
    const bool centered = mode == MCPWM_UP_DOWN_COUNTER;
    if ((!centered && mode != MCPWM_UP_COUNTER && mode != MCPWM_DOWN_COUNTER) ||
        hz < 16U || hz > 500000U)
        return 0;

    // Use P+1 for edge cycles and P for a centered half cycle. The legacy
    // argument can encode only divisors of the form floor(clock / argument).
    const uint32_t clock = timerResolution(mode);
    const uint32_t rate = centered ? 2U * hz : hz;
    const uint32_t offset = centered ? 1U : 0U;
    const uint32_t target = (clock + rate - 1U) / rate;
    const uint32_t slower_arg = clock / (target + offset);
    const uint32_t slower = clock / slower_arg - offset;
    const uint32_t slower_error = slower * rate - clock;
    if (slower_error == 0U)
        return slower_arg;

    const uint32_t faster_arg = slower_arg + 1U;
    const uint32_t faster = clock / faster_arg - offset;
    if (slower + offset > UINT16_MAX + 1U)
        return faster_arg;
    const uint32_t faster_error = clock - faster * rate;
    return static_cast<uint64_t>(slower_error) * faster <= static_cast<uint64_t>(faster_error) * slower
               ? slower_arg
               : faster_arg;
}

/// @brief Select the comparator command for the generator's waveform.
float comparatorDuty(mcpwm_counter_type_t mode, uint32_t period, float duty)
{
    if (duty >= 100.0f)
        return duty;
    if (mode == MCPWM_UP_DOWN_COUNTER)
    {
        // The compare at the turn-around peak is direction-sensitive. Keep
        // fractional commands below it; full duty uses the no-match bin.
        const float upper_bin = 100.0f * (static_cast<float>(period) - 0.5f) /
                                static_cast<float>(period + 1U);
        return (static_cast<float>(period + 1U) * duty / 100.0f < static_cast<float>(period))
                   ? duty : upper_bin;
    }
    if (mode != MCPWM_DOWN_COUNTER)
        return duty;
    // In down mode the generator goes high at peak and low on compare.
    // A compare at peak wins over the peak action and produces true zero.
    const auto high = static_cast<uint32_t>(static_cast<float>(period + 1U) * duty / 100.0f);
    const uint32_t compare = period - high;
    // Encode a point inside the intended integer bin, away from truncation.
    return 100.0f * (static_cast<float>(compare) + 0.5f) / static_cast<float>(period + 1U);
}

/// @brief Decode the legacy comparator ratio into generator duty.
float generatorDuty(mcpwm_counter_type_t mode, uint32_t period, float compare)
{
    const float sdk_ticks = static_cast<float>(period + 1U);
    if (mode == MCPWM_DOWN_COUNTER)
    {
        if (compare >= 100.0f)
            return 100.0f;
        const float ticks = std::round(compare * sdk_ticks / 100.0f);
        return 100.0f * (static_cast<float>(period) - ticks) / sdk_ticks;
    }
    if (mode == MCPWM_UP_DOWN_COUNTER && period != 0U)
        return clampValue(compare * sdk_ticks / static_cast<float>(period), 0.0f, 100.0f);
    return compare;
}

// Value-only reservations survive a failed destructor without dangling owners.
struct ResourceReservation
{
    bool occupied = false;
    bool whole_unit = false;
    int pins[6] = {-1, -1, -1, -1, -1, -1};
};
portMUX_TYPE resource_mux = portMUX_INITIALIZER_UNLOCKED;
ResourceReservation reservations[SOC_MCPWM_GROUPS][SOC_MCPWM_TIMERS_PER_GROUP];
}

bool HBridgeMotor::acquireResources(const MotorMCPWMConfig &hw, const MotorSafetyConfig &safety,
                                    const MotorCaptureConfig &cap, const MotorHardwareFaultConfig &fault) noexcept
{
    const bool whole_unit = fault.mode != HardwareFaultMode::Disabled;
    const int pins[] = {hw.lpwm_pin, hw.rpwm_pin, hw.en_pin, safety.fault_gpio, cap.cap_gpio, fault.fault_gpio};
    // Registry lock never nests with an instance lock or spans peripheral calls.
    portENTER_CRITICAL(&resource_mux);
    for (int unit = 0; unit < SOC_MCPWM_GROUPS; ++unit)
    {
        for (int timer = 0; timer < SOC_MCPWM_TIMERS_PER_GROUP; ++timer)
        {
            const ResourceReservation &entry = reservations[unit][timer];
            if (!entry.occupied)
                continue;
            bool conflict = unit == hw.unit && (timer == hw.timer || whole_unit || entry.whole_unit);
            for (int pin : pins)
                for (int reserved : entry.pins)
                    conflict = conflict || (pin >= 0 && pin == reserved);
            if (conflict)
            {
                portEXIT_CRITICAL(&resource_mux);
                return false;
            }
        }
    }
    ResourceReservation &entry = reservations[hw.unit][hw.timer];
    entry.occupied = true;
    entry.whole_unit = whole_unit;
    for (int i = 0; i < 6; ++i)
        entry.pins[i] = pins[i];
    resources_owned_ = true;
    portEXIT_CRITICAL(&resource_mux);
    return true;
}

bool HBridgeMotor::releaseResources() noexcept
{
    if (!resources_owned_)
        return true;
    if (mcpwm_running_)
        return false;
    if (!pwm_isolated_ && !isolatePwm())
        return false;

    // Hand off the isolated pads without driving them or pulling them high.
    // Keep EN driven low until both PWM pads have been handed off.
    gpio_config_t released{};
    released.pin_bit_mask = (UINT64_C(1) << static_cast<uint32_t>(lpwm_pin_)) |
                            (UINT64_C(1) << static_cast<uint32_t>(rpwm_pin_));
    released.mode = GPIO_MODE_INPUT;
    released.pull_up_en = GPIO_PULLUP_DISABLE;
    released.pull_down_en = GPIO_PULLDOWN_ENABLE;
    if (gpio_config(&released) != ESP_OK)
    {
        (void)isolatePwm();
        return false;
    }
    if (use_en_)
    {
        released.pin_bit_mask = UINT64_C(1) << static_cast<uint32_t>(en_pin_);
        if (gpio_config(&released) != ESP_OK)
        {
            (void)isolatePwm();
            return false;
        }
    }
    portENTER_CRITICAL(&resource_mux);
    reservations[mcpwm_unit_][mcpwm_timer_] = ResourceReservation{};
    resources_owned_ = false;
    portEXIT_CRITICAL(&resource_mux);
    return true;
}

HBridgeMotor::~HBridgeMotor() noexcept
{
    // Failed teardown retains a value-only quarantine until reboot. Detach
    // callbacks even on failure so the registry never outlives object storage.
    (void)prepareForSetup();
    detachFaultInterrupt();
    detachCaptureInterrupt();
    if (hardware_fault_irq_pin_ >= 0)
        detachInterrupt(hardware_fault_irq_pin_);

    if (soft_timer_)
    {
        (void)esp_timer_stop(soft_timer_);
        (void)esp_timer_delete(soft_timer_);
        soft_timer_ = nullptr;
    }
}

// ---- Setup ---- //

MotorSetupResult HBridgeMotor::setup(const MotorMCPWMConfig &hw)
{
    MotorBehaviorConfig behavior{};
    if (hw.en_pin < 0 && behavior.freewheel_mode == FreewheelMode::HiZ)
        behavior.freewheel_mode = FreewheelMode::HiZ_Awake;

    return setup(hw, behavior, MotorSafetyConfig{}, MotorCaptureConfig{}, MotorHardwareFaultConfig{});
}

MotorSetupResult HBridgeMotor::setup(const MotorMCPWMConfig &hw, const MotorBehaviorConfig &beh)
{
    return setup(hw, beh, MotorSafetyConfig{}, MotorCaptureConfig{}, MotorHardwareFaultConfig{});
}

MotorSetupResult HBridgeMotor::setup(const MotorMCPWMConfig &hw, const MotorBehaviorConfig &beh,
                                     const MotorSafetyConfig &safety, const MotorCaptureConfig &cap)
{
    return setup(hw, beh, safety, cap, MotorHardwareFaultConfig{});
}

MotorSetupResult HBridgeMotor::setup(const MotorMCPWMConfig &hw, const MotorBehaviorConfig &beh,
                                     const MotorSafetyConfig &safety, const MotorCaptureConfig &cap,
                                     const MotorHardwareFaultConfig &hardware_fault)
{
    const MotorSetupError teardown_error = prepareForSetup();
    if (teardown_error != MotorSetupError::None)
        return setupResult();

    setup_error_ = validateConfig(hw, beh, safety, cap, hardware_fault);
    if (setup_error_ != MotorSetupError::None)
        return setupResult();

    if (!acquireResources(hw, safety, cap, hardware_fault))
    {
        setup_error_ = MotorSetupError::ResourceConflict;
        return setupResult();
    }

    lpwm_pin_ = hw.lpwm_pin;
    rpwm_pin_ = hw.rpwm_pin;
    en_pin_ = hw.en_pin;
    mcpwm_unit_ = hw.unit;
    mcpwm_timer_ = hw.timer;
    mcpwm_sig_l_ = hw.sig_l;
    mcpwm_sig_r_ = hw.sig_r;

    beh_ = beh;
    safety_ = safety;
    cap_ = cap;
    hardware_fault_ = hardware_fault;
    min_phase_us_ = beh_.min_phase_us;
    dither_coast_hi_z_ = beh_.dither_coast_hi_z;
    soft_hz_ = beh_.soft_brake_hz;
    pwm_freq_hz_ = hw.pwm_freq_hz;
    input_max_ = hw.input_max;
    counter_mode_ = hw.counter;
    percent_per_count_ = 100.0f / static_cast<float>(input_max_);
    soft_brake_pwm_ =
        static_cast<uint16_t>(clampValue<int>(static_cast<int>(beh_.default_soft_brake_pwm), 0, input_max_));

    if (en_pin_ >= 0)
    {
        // Preload the GPIO latch even before Arduino has registered this pin.
        if (gpio_set_level(static_cast<gpio_num_t>(en_pin_), 0) != ESP_OK)
        {
            failSetup(MotorSetupError::HardwareInitFailed);
            return setupResult();
        }
        pinMode(en_pin_, OUTPUT);
        use_en_ = true;
        en_state_ = false;
    }

    if (!isolatePwm())
    {
        failSetup(MotorSetupError::HardwareInitFailed);
        return setupResult();
    }

    mcpwm_config_t cfg{};
    cfg.frequency = bootstrap_hz;
    cfg.counter_mode = counter_mode_;
    cfg.duty_mode = MCPWM_DUTY_MODE_0;
    if (mcpwm_init(mcpwm_unit_, mcpwm_timer_, &cfg) != ESP_OK)
    {
        failSetup(MotorSetupError::HardwareInitFailed);
        return setupResult();
    }
    mcpwm_initialized_ = true;
    mcpwm_running_ = true;

    if (!stopTimer() ||
        mcpwm_timer_set_resolution(mcpwm_unit_, mcpwm_timer_, timerResolution(counter_mode_)) != ESP_OK ||
        mcpwm_set_frequency(mcpwm_unit_, mcpwm_timer_, legacyFrequency(counter_mode_, pwm_freq_hz_)) != ESP_OK)
    {
        failSetup(MotorSetupError::HardwareInitFailed);
        return setupResult();
    }

    if (hw.use_deadtime)
    {
        if (mcpwm_deadtime_enable(mcpwm_unit_, mcpwm_timer_, hw.deadtime_type, hw.deadtime_red_ns,
                                  hw.deadtime_fed_ns) != ESP_OK)
        {
            failSetup(MotorSetupError::HardwareInitFailed);
            return setupResult();
        }
        deadtime_enabled_ = true;
    }

    // Clear the previous owner's timer/compare action table before installing
    // the new count direction. Fault actions live in separate registers.
    const uint32_t generator_offset = static_cast<uint32_t>(mcpwm_timer_) *
        (MCPWM_GEN1_A_REG(mcpwm_unit_) - MCPWM_GEN0_A_REG(mcpwm_unit_));
    REG_WRITE(MCPWM_GEN0_A_REG(mcpwm_unit_) + generator_offset, 0U);
    REG_WRITE(MCPWM_GEN0_B_REG(mcpwm_unit_) + generator_offset, 0U);
    const auto duty_mode = counter_mode_ == MCPWM_DOWN_COUNTER ? MCPWM_DUTY_MODE_1 : MCPWM_DUTY_MODE_0;
    if (mcpwm_set_duty_type(mcpwm_unit_, mcpwm_timer_, MCPWM_OPR_A, duty_mode) != ESP_OK ||
        mcpwm_set_duty_type(mcpwm_unit_, mcpwm_timer_, MCPWM_OPR_B, duty_mode) != ESP_OK)
    {
        failSetup(MotorSetupError::HardwareInitFailed);
        return setupResult();
    }

    if (!soft_timer_)
    {
        esp_timer_create_args_t args{};
        args.callback = [](void *context) { static_cast<HBridgeMotor *>(context)->softBrakeTimerTask(); };
        args.arg = this;
        args.dispatch_method = ESP_TIMER_TASK;
        args.name = "soft_brake";
        if (esp_timer_create(&args, &soft_timer_) != ESP_OK)
        {
            failSetup(MotorSetupError::TimerInitFailed);
            return setupResult();
        }
    }

    if (!configureHardwareFault())
    {
        failSetup(MotorSetupError::HardwareFaultInitFailed);
        return setupResult();
    }

    if (safety_.fault_gpio >= 0)
    {
        pinMode(safety_.fault_gpio, safety_.fault_active_high ? INPUT_PULLDOWN : INPUT_PULLUP);
        attachInterruptArg(safety_.fault_gpio, &HBridgeMotor::faultISRThunk, this, CHANGE);
        fault_irq_pin_ = safety_.fault_gpio;

        const bool active = faultInputActive();
        lockState();
        fault_latched_ = active;
        fault_pending_ = active;
        if (active)
            ++fault_sequence_;
        unlockState();
    }

    if (cap_.cap_gpio >= 0)
    {
        pinMode(cap_.cap_gpio, INPUT);
        const int mode = (cap_.edge == CaptureEdge::Rising)    ? RISING
                         : (cap_.edge == CaptureEdge::Falling) ? FALLING
                                                               : CHANGE;
        attachInterruptArg(cap_.cap_gpio, &HBridgeMotor::capISRThunk, this, mode);
        cap_irq_pin_ = cap_.cap_gpio;
    }

    if (!commandOutput(false, 0.0f, 0.0f) || startTimer() != MotorOperationError::None)
    {
        failSetup(MotorSetupError::HardwareInitFailed);
        return setupResult();
    }

    lockState();
    setup_done_ = true;
    setup_error_ = MotorSetupError::None;
    output_mode_ = MotorOutputMode::Coast;
    unlockState();

    if (softwareFaultActiveSnapshot())
    {
        if (applyFaultAction() != MotorOperationError::None)
        {
            failSetup(MotorSetupError::HardwareInitFailed);
            return setupResult();
        }
    }
    else if (hardware_fault_latched_ || hardware_fault_active_)
    {
        setOutputMode(MotorOutputMode::FaultContainment);
    }
    else if (beh_.freewheel_mode == FreewheelMode::HiZ)
    {
        // startTimer() already proved a quiet generator cycle with EN low.
        (void)result(MotorOperation::Coast, MotorOperationError::None, false);
    }
    else
    {
        const MotorOperationResult coast_result = coast();
        if (!coast_result.ok())
        {
            failSetup((coast_result.error == MotorOperationError::TimerFailed) ? MotorSetupError::TimerInitFailed
                                                                               : MotorSetupError::HardwareInitFailed);
            return setupResult();
        }
    }

    return setupResult();
}

bool HBridgeMotor::isSetupComplete() const noexcept
{
    lockState();
    const bool complete = setup_done_;
    unlockState();
    return complete;
}

MotorSetupError HBridgeMotor::getLastSetupError() const noexcept
{
    lockState();
    const MotorSetupError error = setup_error_;
    unlockState();
    return error;
}

MotorSetupResult HBridgeMotor::setupResult() const noexcept
{
    lockState();
    const MotorSetupResult out{setup_error_, setup_done_ && fault_irq_pin_ >= 0,
                               setup_done_ && hardware_fault_enabled_};
    unlockState();
    return out;
}

// ---- Explicit drive / coast / brake ---- //

MotorOperationResult HBridgeMotor::drive(int speed, Dir dir) noexcept
{
    if (!isSetupComplete())
        return reject(MotorOperation::Drive, MotorOperationError::NotSetup);
    if (hasFault())
        return reject(MotorOperation::Drive, MotorOperationError::FaultActive);
    if (speed <= 0)
        return reject(MotorOperation::Drive, MotorOperationError::InvalidCommand);

    lockState();
    const bool running = mcpwm_running_;
    unlockState();
    if (!running)
        return reject(MotorOperation::Drive, MotorOperationError::InvalidCommand);

    const uint16_t request = static_cast<uint16_t>(clampValue<int>(speed, 1, input_max_));
    const float duty = request == input_max_ ? 100.0f : static_cast<float>(request) * percent_per_count_;
    lockState();
    const MotorOutputMode requested_mode = (dir == Dir::CW) ? MotorOutputMode::DriveCW : MotorOutputMode::DriveCCW;
    const bool changed = output_mode_ != requested_mode || soft_active_ || !commanded_enable_ ||
                         commanded_a_percent_ != ((dir == Dir::CW) ? duty : 0.0f) ||
                         commanded_b_percent_ != ((dir == Dir::CCW) ? duty : 0.0f);
    unlockState();
    stopSoftBrake();

    const bool ok = (dir == Dir::CW) ? commandOutput(true, duty, 0.0f) : commandOutput(true, 0.0f, duty);
    if (!ok)
        return reject(MotorOperation::Drive, MotorOperationError::HardwareWriteFailed);

    setOutputMode(requested_mode);
    return result(MotorOperation::Drive, MotorOperationError::None, changed);
}

MotorOperationResult HBridgeMotor::drivePercent(float percent, Dir dir) noexcept
{
    if (!std::isfinite(percent) || percent <= 0.0f)
        return reject(MotorOperation::Drive, MotorOperationError::InvalidCommand);

    const float clamped = clampValue<float>(percent, 0.0f, 100.0f);
    // Multiply before dividing and keep enough precision for exact half counts.
    const double scaled = static_cast<double>(clamped) * getMaxPwmInput() / 100.0;
    const int speed = static_cast<int>(scaled + 0.5);
    return drive((speed > 0) ? speed : 1, dir);
}

MotorOperationResult HBridgeMotor::setSpeed(int speed, Dir dir) noexcept
{
    if (speed <= 0)
        return reject(MotorOperation::Drive, MotorOperationError::InvalidCommand);
    return drive(speed, dir);
}

MotorOperationResult HBridgeMotor::setSpeedPercent(float percent, Dir dir) noexcept
{
    if (!std::isfinite(percent) || percent <= 0.0f)
        return reject(MotorOperation::Drive, MotorOperationError::InvalidCommand);
    return drivePercent(percent, dir);
}

MotorOperationResult HBridgeMotor::coast() noexcept
{
    // Coast follows the configured electrical policy; DitherBrake deliberately
    // alternates braking and coast rather than treating zero drive as a stop.
    if (!isSetupComplete())
        return reject(MotorOperation::Coast, MotorOperationError::NotSetup);
    if (hasFault())
        return reject(MotorOperation::Coast, MotorOperationError::FaultActive);

    lockState();
    const bool was_coast = output_mode_ == MotorOutputMode::Coast;
    const bool was_dither = output_mode_ == MotorOutputMode::DitherBrake;
    const uint16_t previous_soft_pwm = soft_brake_pwm_;
    unlockState();

    stopSoftBrake();

    bool ok = true;
    switch (beh_.freewheel_mode)
    {
    case FreewheelMode::HiZ:
        ok = commandOutput(false, 0.0f, 0.0f, canSuppressCoast());
        break;
    case FreewheelMode::HiZ_Awake:
        ok = commandOutput(true, 0.0f, 0.0f);
        break;
    case FreewheelMode::DitherBrake:
    {
        const MotorOperationResult configured = setSoftBrakePWM(beh_.dither_pwm);
        if (!configured.ok())
            return configured;
        const MotorOperationError brake_error = startSoftBrake();
        if (brake_error != MotorOperationError::None)
            return reject(MotorOperation::Coast, brake_error);
        lockState();
        const bool active = soft_active_;
        const uint16_t pwm = soft_brake_pwm_;
        unlockState();
        setOutputMode((active || pwm > 0) ? MotorOutputMode::DitherBrake : MotorOutputMode::Coast);
        const MotorOutputMode desired_mode =
            (beh_.dither_pwm > 0U) ? MotorOutputMode::DitherBrake : MotorOutputMode::Coast;
        const bool changed = (desired_mode == MotorOutputMode::DitherBrake)
                                 ? (!was_dither || previous_soft_pwm != beh_.dither_pwm)
                                 : !was_coast;
        return result(MotorOperation::Coast, MotorOperationError::None, changed);
    }
    }

    if (!ok)
        return reject(MotorOperation::Coast, MotorOperationError::HardwareWriteFailed);

    setOutputMode(MotorOutputMode::Coast);
    return result(MotorOperation::Coast, MotorOperationError::None, !was_coast);
}

MotorOperationResult HBridgeMotor::setHardBrake() noexcept
{
    if (!isSetupComplete())
        return reject(MotorOperation::HardBrake, MotorOperationError::NotSetup);
    if (hasFault())
        return reject(MotorOperation::HardBrake, MotorOperationError::FaultActive);

    lockState();
    const bool running = mcpwm_running_;
    unlockState();
    if (!running)
        return reject(MotorOperation::HardBrake, MotorOperationError::InvalidCommand);

    lockState();
    const bool changed = output_mode_ != MotorOutputMode::HardBrake || soft_active_;
    unlockState();
    stopSoftBrake();
    if (!commandOutput(true, 100.0f, 100.0f))
        return reject(MotorOperation::HardBrake, MotorOperationError::HardwareWriteFailed);

    setOutputMode(MotorOutputMode::HardBrake);
    return result(MotorOperation::HardBrake, MotorOperationError::None, changed);
}

MotorOperationResult HBridgeMotor::setSoftBrakePWM(uint16_t pwm) noexcept
{
    if (!isSetupComplete())
        return reject(MotorOperation::SoftBrake, MotorOperationError::NotSetup);

    const uint16_t clamped = static_cast<uint16_t>(clampValue<int>(static_cast<int>(pwm), 0, input_max_));

    lockState();
    const bool unchanged = (clamped == soft_brake_pwm_);
    soft_brake_pwm_ = clamped;
    const bool was_active = soft_active_;
    unlockState();

    if (unchanged)
        return result(MotorOperation::SoftBrake, MotorOperationError::None, false);

    if (was_active)
    {
        const MotorOperationError error = startSoftBrake();
        if (error != MotorOperationError::None)
            return reject(MotorOperation::SoftBrake, error);
    }

    return result(MotorOperation::SoftBrake, MotorOperationError::None, true);
}

MotorOperationResult HBridgeMotor::softBrakeNow(uint16_t pwm) noexcept
{
    if (!isSetupComplete())
        return reject(MotorOperation::SoftBrake, MotorOperationError::NotSetup);
    if (hasFault())
        return reject(MotorOperation::SoftBrake, MotorOperationError::FaultActive);

    lockState();
    const uint16_t requested = static_cast<uint16_t>(clampValue<int>(static_cast<int>(pwm), 0, input_max_));
    const MotorOutputMode desired_mode = (requested > 0U) ? MotorOutputMode::DitherBrake : MotorOutputMode::Coast;
    const bool changed = output_mode_ != desired_mode || soft_brake_pwm_ != requested;
    unlockState();

    const MotorOperationResult configured = setSoftBrakePWM(pwm);
    if (!configured.ok())
        return configured;

    const MotorOperationError error = startSoftBrake();
    if (error != MotorOperationError::None)
        return reject(MotorOperation::SoftBrake, error);

    lockState();
    const bool active = soft_active_;
    const uint16_t level = soft_brake_pwm_;
    unlockState();
    setOutputMode((active || level > 0) ? MotorOutputMode::DitherBrake : MotorOutputMode::Coast);
    return result(MotorOperation::SoftBrake, MotorOperationError::None, changed);
}

// ---- Software fault observer ---- //

MotorOperationResult HBridgeMotor::pollFaults() noexcept
{
    if (!isSetupComplete())
        return reject(MotorOperation::PollFaults, MotorOperationError::NotSetup);

    bool pending = false;
    bool active = false;
    FaultCallback callback = nullptr;
    void *context = nullptr;

    lockState();
    pending = fault_pending_;
    active = fault_latched_;
    if (pending)
        fault_pending_ = false;
    callback = fault_cb_;
    context = fault_ctx_;
    unlockState();

    if (!pending)
        return result(MotorOperation::PollFaults, MotorOperationError::None, false);

    MotorOperationError error = MotorOperationError::None;
    if (active)
    {
        error = applyFaultAction();
    }
    else
    {
        error = clearFault().error;
    }

    if (callback)
        callback(active, context);

    return (error == MotorOperationError::None) ? result(MotorOperation::PollFaults, error, true)
                                                : reject(MotorOperation::PollFaults, error);
}

bool HBridgeMotor::hasFault() const noexcept
{
    lockState();
    const bool active = recovery_inhibited_ || timer_recovery_required_ || fault_latched_ || hardware_fault_active_ || hardware_fault_latched_;
    unlockState();
    return active;
}

MotorOperationResult HBridgeMotor::clearFault() noexcept
{
    if (!isSetupComplete())
        return reject(MotorOperation::ClearFault, MotorOperationError::NotSetup);

    // Deliver pre-existing assertion work before attempting recovery; a later
    // ISR assertion remains pending and invalidates the generation snapshot.
    if (softwareFaultPendingSnapshot() && softwareFaultActiveSnapshot())
        (void)pollFaults();

    // Keep an admission barrier until every fallible step succeeds. Generation
    // snapshots prevent recovery from erasing an ISR assertion during HAL work.
    lockState();
    if (timer_recovery_required_)
    {
        unlockState();
        return reject(MotorOperation::ClearFault, MotorOperationError::HardwareStopFailed);
    }
    recovery_inhibited_ = true;
    const uint32_t software_sequence = fault_sequence_;
    const uint32_t hardware_sequence = hardware_fault_sequence_;
    const bool input_active = (safety_.fault_gpio >= 0 && faultInputActive()) ||
                              (hardware_fault_enabled_ && hardwareFaultInputActive());
    output_mode_ = MotorOutputMode::FaultContainment;
    unlockState();
    if (input_active)
        return reject(MotorOperation::ClearFault, MotorOperationError::FaultActive);

    stopSoftBrake();
    if (!commandOutput(false, 0.0f, 0.0f))
        return reject(MotorOperation::ClearFault, MotorOperationError::HardwareWriteFailed);
    if (!rearmHardwareFault())
        return reject(MotorOperation::ClearFault, MotorOperationError::HardwareFaultClearFailed);

    lockState();
    const bool running = mcpwm_running_;
    unlockState();
    if ((!running || pwm_isolated_) && startTimer() != MotorOperationError::None)
        return reject(MotorOperation::ClearFault, MotorOperationError::HardwareStartFailed);

    lockState();
    const bool superseded = software_sequence != fault_sequence_ || hardware_sequence != hardware_fault_sequence_ ||
                            (safety_.fault_gpio >= 0 && faultInputActive()) ||
                            (hardware_fault_enabled_ && hardwareFaultInputActive());
    if (!superseded)
    {
        ++fault_sequence_;
        ++hardware_fault_sequence_;
        fault_latched_ = false;
        hardware_fault_active_ = false;
        hardware_fault_latched_ = false;
        recovery_inhibited_ = false;
        output_mode_ = MotorOutputMode::Coast;
    }
    unlockState();
    // Preserve pending callback work. A superseding software assertion must
    // retain its configured containment action as well as its notification.
    if (superseded)
    {
        if (softwareFaultActiveSnapshot())
            (void)applyFaultAction();
        return reject(MotorOperation::ClearFault, MotorOperationError::FaultActive);
    }
    return result(MotorOperation::ClearFault, MotorOperationError::None, true);
}

void HBridgeMotor::setFaultCallback(FaultCallback cb, void *ctx) noexcept
{
    lockState();
    fault_cb_ = cb;
    fault_ctx_ = ctx;
    unlockState();
}

// ---- Runtime behavior ---- //

int HBridgeMotor::getMaxPwmInput() const noexcept
{
    lockState();
    const int value = input_max_;
    unlockState();
    return value;
}

MotorOperationResult HBridgeMotor::setFreewheelMode(FreewheelMode mode) noexcept
{
    if (!isSetupComplete())
        return reject(MotorOperation::Coast, MotorOperationError::NotSetup);

    lockState();
    if (beh_.freewheel_mode == mode)
    {
        unlockState();
        return result(MotorOperation::Coast, MotorOperationError::None, false);
    }
    beh_.freewheel_mode = mode;
    const bool was_active = soft_active_;
    unlockState();

    if (was_active)
    {
        stopSoftBrake();
        if (!applyPhase(BrakePhase::Coast))
            return reject(MotorOperation::Coast, MotorOperationError::HardwareWriteFailed);
        setOutputMode(MotorOutputMode::Coast);
    }

    return result(MotorOperation::Coast, MotorOperationError::None, true);
}

MotorOperationResult HBridgeMotor::applyFreewheel(FreewheelMode mode) noexcept
{
    const MotorOperationResult configured = setFreewheelMode(mode);
    if (!configured.ok())
        return configured;
    return coast();
}

MotorOperationResult HBridgeMotor::start() noexcept
{
    // Stage zero duty with EN low before restarting the generator so a prior
    // drive compare value is not intentionally restored by this lifecycle call.
    if (!isSetupComplete())
        return reject(MotorOperation::Start, MotorOperationError::NotSetup);
    if (hasFault())
        return reject(MotorOperation::Start, MotorOperationError::FaultActive);

    lockState();
    const bool was_running = mcpwm_running_;
    const MotorOutputMode old_mode = output_mode_;
    unlockState();

    stopSoftBrake();
    if (!commandOutput(false, 0.0f, 0.0f))
        return reject(MotorOperation::Start, MotorOperationError::HardwareWriteFailed);
    const MotorOperationError started = startTimer(true);
    if (started != MotorOperationError::None)
        return result(MotorOperation::Start, started, was_running);

    if (hasFault() || (hardware_fault_enabled_ && hardwareFaultInputActive()))
    {
        setOutputMode(MotorOutputMode::FaultContainment);
        return result(MotorOperation::Start, MotorOperationError::FaultActive, true);
    }

    bool ok = true;
    switch (beh_.freewheel_mode)
    {
    case FreewheelMode::HiZ:
        ok = commandOutput(false, 0.0f, 0.0f, canSuppressCoast());
        break;
    case FreewheelMode::HiZ_Awake:
    {
        const bool fault = hasFault() || (hardware_fault_enabled_ && hardwareFaultInputActive());
        if (!fault)
            ok = commandOutput(true, 0.0f, 0.0f);
        if (fault)
        {
            setOutputMode(MotorOutputMode::FaultContainment);
            return result(MotorOperation::Start, MotorOperationError::FaultActive, true);
        }
        break;
    }
    case FreewheelMode::DitherBrake:
    {
        lockState();
        soft_brake_pwm_ = beh_.dither_pwm;
        unlockState();
        const MotorOperationError brake_error = startSoftBrake();
        if (brake_error != MotorOperationError::None)
            return reject(MotorOperation::Start, brake_error);
        lockState();
        const bool active = soft_active_;
        const uint16_t pwm = soft_brake_pwm_;
        unlockState();
        setOutputMode((active || pwm > 0U) ? MotorOutputMode::DitherBrake : MotorOutputMode::Coast);
        const MotorOutputMode desired_mode =
            (beh_.dither_pwm > 0U) ? MotorOutputMode::DitherBrake : MotorOutputMode::Coast;
        const bool changed = !was_running || old_mode != desired_mode;
        return result(MotorOperation::Start, MotorOperationError::None, changed);
    }
    }

    if (!ok)
        return reject(MotorOperation::Start, MotorOperationError::HardwareWriteFailed);

    setOutputMode(MotorOutputMode::Coast);
    return result(MotorOperation::Start, MotorOperationError::None, !was_running || old_mode != MotorOutputMode::Coast);
}

MotorOperationResult HBridgeMotor::stop() noexcept
{
    if (!isSetupComplete())
        return reject(MotorOperation::Stop, MotorOperationError::NotSetup);

    lockState();
    const bool changed = mcpwm_running_ || output_mode_ != MotorOutputMode::Disabled || soft_active_;
    unlockState();

    stopSoftBrake();
    const MotorOperationError error = quiesceTimer();
    return result(MotorOperation::Stop, error, changed);
}

MotorOperationResult HBridgeMotor::reconfigureFrequency(int new_hz) noexcept
{
    if (!isSetupComplete())
        return reject(MotorOperation::ReconfigureFrequency, MotorOperationError::NotSetup);
    const uint32_t sdk_hz = legacyFrequency(counter_mode_, new_hz);
    if (sdk_hz == 0U)
        return reject(MotorOperation::ReconfigureFrequency, MotorOperationError::InvalidCommand);
    if (hasFault())
        return reject(MotorOperation::ReconfigureFrequency, MotorOperationError::FaultActive);

    lockState();
    const bool was_running = mcpwm_running_;
    const bool restart_dither = soft_active_;
    const int previous_hz = pwm_freq_hz_;
    unlockState();
    if (new_hz == previous_hz)
        return result(MotorOperation::ReconfigureFrequency, MotorOperationError::None, false);
    // A running bridge needs EN to inhibit it before the two pads are
    // remapped separately. Peripheral fault and dead-time output actions
    // must also retain control of their configured output behavior.
    if ((was_running && !use_en_) || hardware_fault_enabled_ || deadtime_enabled_)
        return reject(MotorOperation::ReconfigureFrequency, MotorOperationError::Unsupported);

    stopSoftBrake();
    const MotorOperationError stopped = quiesceTimer();
    if (stopped != MotorOperationError::None)
        return result(MotorOperation::ReconfigureFrequency, stopped, true);
    if (hasFault())
        return result(MotorOperation::ReconfigureFrequency, MotorOperationError::FaultActive, true);

    if (mcpwm_set_frequency(mcpwm_unit_, mcpwm_timer_, sdk_hz) != ESP_OK)
        return result(MotorOperation::ReconfigureFrequency, MotorOperationError::FrequencyChangeFailed, true);

    lockState();
    pwm_freq_hz_ = new_hz;
    unlockState();

    // The legacy setter rescales shadow compare values. Stage zero again at
    // the new period and keep the pins isolated through a fresh timer boundary.
    lockState();
    last_a_percent_ = last_b_percent_ = -1.0f;
    unlockState();
    if (!commandOutput(false, 0.0f, 0.0f))
        return result(MotorOperation::ReconfigureFrequency, MotorOperationError::HardwareWriteFailed, true);
    if (!was_running)
        return result(MotorOperation::ReconfigureFrequency, MotorOperationError::None, true);
    const MotorOperationError started = startTimer(true);
    if (started != MotorOperationError::None)
        return result(MotorOperation::ReconfigureFrequency, started, true);
    if (hasFault())
    {
        setOutputMode(MotorOutputMode::FaultContainment);
        return result(MotorOperation::ReconfigureFrequency, MotorOperationError::FaultActive, true);
    }
    setOutputMode(MotorOutputMode::Coast);

    if (restart_dither)
    {
        const MotorOperationError error = startSoftBrake();
        if (error != MotorOperationError::None)
        {
            containFailedDither();
            return result(MotorOperation::ReconfigureFrequency, error, true);
        }
        setOutputMode(MotorOutputMode::DitherBrake);
    }

    return result(MotorOperation::ReconfigureFrequency, MotorOperationError::None, true);
}

// ---- Capture ---- //

uint32_t HBridgeMotor::getLastCapturePeriodUs() const noexcept
{
    lockState();
    const uint32_t period = period_us_;
    unlockState();
    return period;
}

// ---- Diagnostics ---- //

bool HBridgeMotor::hasEnableControl() const noexcept
{
    lockState();
    const bool available = use_en_;
    unlockState();
    return available;
}

MotorDriverStatus HBridgeMotor::status() const noexcept
{
    MotorDriverStatus out{};
    lockState();
    out.setup_complete = setup_done_;
    out.mcpwm_running = mcpwm_running_;
    out.enable_control = use_en_;
    out.enable_asserted = en_state_;
    out.software_fault_configured = fault_irq_pin_ >= 0;
    out.software_fault_active = fault_latched_;
    out.recovery_inhibited = recovery_inhibited_ || timer_recovery_required_;
    out.software_fault_pending = fault_pending_;
    out.hardware_fault_configured = hardware_fault_enabled_;
    out.hardware_fault_input_active = hardware_fault_active_;
    out.hardware_fault_latched = hardware_fault_latched_;
    out.dither_active = soft_active_;
    out.output_mode = output_mode_;
    out.commanded_a_percent = commanded_a_percent_;
    out.commanded_b_percent = commanded_b_percent_;
    out.pwm_frequency_hz = pwm_freq_hz_;
    out.operation_sequence = operation_sequence_;
    out.fault_sequence = fault_sequence_;
    out.hardware_fault_sequence = hardware_fault_sequence_;
    out.capture_sequence = capture_sequence_;
    out.last_operation = last_operation_;
    out.last_error = last_operation_error_;
    unlockState();
    return out;
}

MotorHardwareReadback HBridgeMotor::readback() const noexcept
{
    MotorHardwareReadback out{};
    lockState();
    const bool setup = setup_done_;
    const bool use_en = use_en_;
    const int en_pin = en_pin_;
    const bool running = mcpwm_running_;
    const bool hw_fault = hardware_fault_enabled_;
    const auto counter = counter_mode_;
    const bool stopping = timer_stop_pending_ || timer_recovery_required_ || pwm_isolated_;
    const auto unit = mcpwm_unit_;
    const auto timer = mcpwm_timer_;
    unlockState();

    if (!setup)
        return out;

    out.valid = true;
    // This is derived from the hardware period and the configured clock.
    // Neither this register nor the legacy getter measures the active divider.
    const uint32_t period = timerPeriod(unit, timer);
    const uint32_t ticks = (counter == MCPWM_UP_DOWN_COUNTER) ? 2U * period : period + 1U;
    const uint32_t clock = timerResolution(counter);
    if (running && !stopping && ticks != 0U &&
        mcpwm_get_frequency(unit, timer) == clock / (period + 1U))
        out.frequency_hz = clock / ticks;
    out.duty_a_percent = generatorDuty(counter, period, mcpwm_get_duty(unit, timer, MCPWM_OPR_A));
    out.duty_b_percent = generatorDuty(counter, period, mcpwm_get_duty(unit, timer, MCPWM_OPR_B));
    out.enable_control = use_en;
    out.enable_asserted = use_en ? (gpio_get_level(static_cast<gpio_num_t>(en_pin)) != 0) : false;
    out.running_cached = running;
    out.hardware_fault_configured = hw_fault;
    out.hardware_fault_input_active = hw_fault ? hardwareFaultInputActive() : false;
    return out;
}

MotorOperationResult HBridgeMotor::forceOutputs(bool a_high, bool b_high) noexcept
{
#if ESP32_MCPWM_ENABLE_COMMISSIONING_API
    if (!isSetupComplete())
        return reject(MotorOperation::ForceOutputs, MotorOperationError::NotSetup);
    if (hasFault())
        return reject(MotorOperation::ForceOutputs, MotorOperationError::FaultActive);

    lockState();
    const bool running = mcpwm_running_;
    const float a = a_high ? 100.0f : 0.0f;
    const float b = b_high ? 100.0f : 0.0f;
    const bool changed = output_mode_ != MotorOutputMode::Commissioning || soft_active_ ||
                         !commanded_enable_ || commanded_a_percent_ != a || commanded_b_percent_ != b;
    unlockState();
    if (!running)
        return reject(MotorOperation::ForceOutputs, MotorOperationError::InvalidCommand);
    stopSoftBrake();
    if (!commandOutput(true, a, b))
    {
        setOutputMode(MotorOutputMode::Uncertain);
        return reject(MotorOperation::ForceOutputs, MotorOperationError::HardwareWriteFailed);
    }
    setOutputMode(MotorOutputMode::Commissioning);
    return result(MotorOperation::ForceOutputs, MotorOperationError::None, changed);
#else
    (void)a_high;
    (void)b_high;
    return reject(MotorOperation::ForceOutputs, MotorOperationError::CommissioningDisabled);
#endif
}

// ---- Result helpers ---- //

MotorOperationResult HBridgeMotor::result(MotorOperation operation, MotorOperationError error, bool changed) noexcept
{
    lockState();
    last_operation_ = operation;
    last_operation_error_ = error;
    if (error == MotorOperationError::None)
        ++operation_sequence_;
    const uint32_t sequence = operation_sequence_;
    unlockState();
    return MotorOperationResult{operation, error, changed, sequence};
}

MotorOperationResult HBridgeMotor::reject(MotorOperation operation, MotorOperationError error) noexcept
{
    recordOperation(operation, error);
    lockState();
    const uint32_t sequence = operation_sequence_;
    unlockState();
    return MotorOperationResult{operation, error, false, sequence};
}

void HBridgeMotor::recordOperation(MotorOperation operation, MotorOperationError error) noexcept
{
    lockState();
    last_operation_ = operation;
    last_operation_error_ = error;
    unlockState();
}

void HBridgeMotor::setOutputMode(MotorOutputMode mode) noexcept
{
    lockState();
    output_mode_ = mode;
    unlockState();
}

// ---- Soft brake ---- //

void HBridgeMotor::softBrakeTimerTask() noexcept
{
    lockState();
    if (!setup_done_ || !soft_active_ || recovery_inhibited_ || fault_latched_ || hardware_fault_active_ || hardware_fault_latched_)
    {
        soft_active_ = false;
        unlockState();
        return;
    }

    soft_phase_ = (soft_phase_ == BrakePhase::Coast) ? BrakePhase::Brake : BrakePhase::Coast;
    const BrakePhase phase = soft_phase_;
    const uint32_t sequence = soft_sequence_;
    unlockState();

    const OutputCommitResult phase_result = applyDitherPhase(phase, sequence);
    if (phase_result == OutputCommitResult::Stale)
        return;
    if (phase_result == OutputCommitResult::HardwareFailed)
    {
        containFailedDither();
        recordOperation(MotorOperation::SoftBrake, MotorOperationError::HardwareWriteFailed);
        return;
    }

    (void)scheduleNextPhase(sequence);
}

bool HBridgeMotor::applyPhase(BrakePhase phase) noexcept
{
    if (!isSetupComplete() || hasFault())
        return false;

    return (phase == BrakePhase::Brake) ? commandOutput(true, 100.0f, 100.0f)
                                        : commandOutput(!dither_coast_hi_z_, 0.0f, 0.0f);
}

HBridgeMotor::OutputCommitResult HBridgeMotor::applyDitherPhase(BrakePhase phase, uint32_t sequence) noexcept
{
    const bool enable = (phase == BrakePhase::Brake) || !dither_coast_hi_z_;
    const float duty = (phase == BrakePhase::Brake) ? 100.0f : 0.0f;

    lockState();
    if (!setup_done_ || !soft_active_ || recovery_inhibited_ || fault_latched_ || hardware_fault_active_ || hardware_fault_latched_ ||
        soft_sequence_ != sequence)
    {
        unlockState();
        return OutputCommitResult::Stale;
    }
    commanded_enable_ = enable;
    commanded_a_percent_ = duty;
    commanded_b_percent_ = duty;
    const uint32_t output_sequence = ++output_sequence_;
    const OutputCommitResult committed = writeHardwareOutput(output_sequence, enable, duty, duty);
    unlockState();
    return committed;
}

HBridgeMotor::TimerCommandResult HBridgeMotor::scheduleNextPhase(uint32_t sequence) noexcept
{
    lockState();
    if (!soft_active_ || recovery_inhibited_ || fault_latched_ || hardware_fault_active_ || hardware_fault_latched_ ||
        soft_sequence_ != sequence)
    {
        unlockState();
        return TimerCommandResult::StaleOrCancelled;
    }

    const int64_t timeout = (soft_phase_ == BrakePhase::Brake) ? soft_us_brake_ : soft_us_coast_;
    commanded_timer_active_ = true;
    commanded_timer_us_ = timeout;
    const uint32_t timer_sequence = ++timer_sequence_;
    unlockState();

    const TimerCommandResult timer_result = writeTimerUntilCurrent(timer_sequence, true, timeout);
    if (timer_result == TimerCommandResult::TimerFailed)
        return containDitherTimerFailure(sequence);
    return timer_result;
}

MotorOperationError HBridgeMotor::startSoftBrake() noexcept
{
    // Zero/full strength are steady endpoints, not zero-duration timer phases.
    // Intermediate strength starts in coast before scheduling the first brake.
    if (!isSetupComplete())
        return MotorOperationError::NotSetup;
    if (hasFault())
        return MotorOperationError::FaultActive;

    lockState();
    const bool running = mcpwm_running_;
    unlockState();
    if (!running)
        return MotorOperationError::InvalidCommand;

    stopSoftBrake();
    if (!recomputeSoftDurations())
    {
        if (!applyPhase(BrakePhase::Coast))
            return MotorOperationError::HardwareWriteFailed;
        return MotorOperationError::None;
    }

    lockState();
    const bool pure_coast = (soft_us_brake_ == 0);
    const bool pure_brake = (soft_us_coast_ == 0);

    if (pure_coast || pure_brake)
    {
        soft_active_ = false;
        ++soft_sequence_;
        unlockState();
        return applyPhase(pure_coast ? BrakePhase::Coast : BrakePhase::Brake)
                   ? MotorOperationError::None
                   : MotorOperationError::HardwareWriteFailed;
    }

    // Reconnect at zero in the caller's task before arming the dither
    // timer. Its shared callback can then commit phases without waiting.
    if (restoreCoastRoutes(++output_sequence_) != OutputCommitResult::Committed)
    {
        unlockState();
        return MotorOperationError::HardwareWriteFailed;
    }
    soft_phase_ = BrakePhase::Coast;
    soft_active_ = true;
    const uint32_t sequence = ++soft_sequence_;
    unlockState();

    if (applyDitherPhase(BrakePhase::Coast, sequence) != OutputCommitResult::Committed)
        return MotorOperationError::HardwareWriteFailed;
    const TimerCommandResult timer_result = scheduleNextPhase(sequence);
    if (timer_result != TimerCommandResult::Applied)
        return (timer_result == TimerCommandResult::TimerFailed) ? MotorOperationError::TimerFailed
                                                                 : MotorOperationError::None;
    return MotorOperationError::None;
}

void HBridgeMotor::stopSoftBrake() noexcept
{
    lockState();
    soft_active_ = false;
    ++soft_sequence_;
    commanded_timer_active_ = false;
    commanded_timer_us_ = 0;
    const uint32_t timer_sequence = ++timer_sequence_;
    unlockState();

    (void)writeTimerUntilCurrent(timer_sequence, false, 0);
}

void HBridgeMotor::containFailedDither() noexcept
{
    stopSoftBrake();
    const bool zeroed = commandOutput(false, 0.0f, 0.0f);

    lockState();
    const bool independently_disabled = use_en_ && !en_state_;
    output_mode_ = zeroed                   ? MotorOutputMode::Coast
                   : independently_disabled ? MotorOutputMode::Disabled
                                            : MotorOutputMode::Uncertain;
    unlockState();
}

HBridgeMotor::TimerCommandResult HBridgeMotor::containDitherTimerFailure(uint32_t sequence) noexcept
{
    lockState();
    if (soft_sequence_ != sequence)
    {
        unlockState();
        return TimerCommandResult::StaleOrCancelled;
    }

    soft_active_ = false;
    ++soft_sequence_;
    commanded_timer_active_ = false;
    commanded_timer_us_ = 0;

    commanded_enable_ = false;
    commanded_a_percent_ = 0.0f;
    commanded_b_percent_ = 0.0f;
    const uint32_t output_sequence = ++output_sequence_;
    const OutputCommitResult contained = writeHardwareOutput(output_sequence, false, 0.0f, 0.0f);
    if (contained == OutputCommitResult::Stale)
    {
        unlockState();
        return TimerCommandResult::StaleOrCancelled;
    }

    const bool independently_disabled = use_en_ && !en_state_;
    output_mode_ = (contained == OutputCommitResult::Committed) ? MotorOutputMode::Coast
                   : independently_disabled                     ? MotorOutputMode::Disabled
                                                                : MotorOutputMode::Uncertain;
    last_operation_ = MotorOperation::SoftBrake;
    last_operation_error_ = MotorOperationError::TimerFailed;
    unlockState();
    return TimerCommandResult::TimerFailed;
}

bool HBridgeMotor::recomputeSoftDurations() noexcept
{
    if (input_max_ <= 0 || soft_hz_ <= 0)
        return false;

    const int64_t period_us = static_cast<int64_t>(kMicrosPerSec / static_cast<uint32_t>(soft_hz_));
    if (period_us < 2)
        return false;

    lockState();
    const uint32_t pwm = clampValue<uint32_t>(soft_brake_pwm_, 0U, static_cast<uint32_t>(input_max_));
    unlockState();

    int64_t brake_us = 0;
    if (pwm >= static_cast<uint32_t>(input_max_))
    {
        brake_us = period_us;
    }
    else if (pwm > 0U)
    {
        // Widen the product before rounding; reserve a nonzero coast phase
        // even when the requested minimum exceeds half of the period.
        brake_us =
            static_cast<int64_t>((static_cast<uint64_t>(period_us) * pwm + static_cast<uint32_t>(input_max_ / 2)) /
                                 static_cast<uint32_t>(input_max_));
        const int64_t minimum = clampValue<int64_t>(static_cast<int64_t>(min_phase_us_), 1, period_us / 2);
        brake_us = clampValue<int64_t>(brake_us, minimum, period_us - minimum);
    }
    const int64_t coast_us = period_us - brake_us;

    lockState();
    soft_us_brake_ = brake_us;
    soft_us_coast_ = coast_us;
    unlockState();
    return true;
}

// ---- Hardware output ---- //

HBridgeMotor::OutputCommitResult HBridgeMotor::writeHardwareOutput(uint32_t sequence, bool enable, float a_percent,
                                                                   float b_percent, bool suppress_pwm) noexcept
{
    // state_mux_ is deliberately held by the caller from the freshness check
    // through each physical commit. This is the linearization boundary between
    // deferred dither work and task-context commands. The ESP32 critical
    // section and host mock are recursive, so nested adversarial hooks can
    // publish and commit a newer command without deadlocking.
    if (sequence != output_sequence_)
        return OutputCommitResult::Stale;

    a_percent = std::isfinite(a_percent) ? clampValue<float>(a_percent, 0.0f, 100.0f) : 0.0f;
    b_percent = std::isfinite(b_percent) ? clampValue<float>(b_percent, 0.0f, 100.0f) : 0.0f;

    const bool enable_controlled = use_en_;
    const bool enable_asserted = en_state_;
    const float last_a = last_a_percent_;
    const float last_b = last_b_percent_;

    const bool both_change = a_percent != last_a && b_percent != last_b;

    if (!enable || (enable_controlled && enable_asserted && both_change))
    {
        if (enable_controlled && en_state_)
        {
            digitalWrite(en_pin_, LOW);
            en_state_ = false;
        }
    }

    if (sequence != output_sequence_)
        return OutputCommitResult::Stale;

    if (suppress_pwm && !pwm_isolated_ && !pwm_coast_suppressed_)
    {
        if (!routePwmLow())
            return OutputCommitResult::HardwareFailed;
        pwm_coast_suppressed_ = true;
    }

    const uint32_t period = counter_mode_ == MCPWM_UP_COUNTER ? 0U : timerPeriod(mcpwm_unit_, mcpwm_timer_);
    const auto writeDuty = [&](mcpwm_operator_t channel, float percent) {
        return mcpwm_set_duty(mcpwm_unit_, mcpwm_timer_, channel, comparatorDuty(counter_mode_, period, percent));
    };

    const auto contain_after_failure = [this, sequence, &writeDuty]() -> OutputCommitResult
    {
        if (use_en_ && en_state_)
        {
            digitalWrite(en_pin_, LOW);
            en_state_ = false;
        }

        if (sequence != output_sequence_)
            return OutputCommitResult::Stale;
        if (writeDuty(MCPWM_OPR_A, 0.0f) == ESP_OK)
            last_a_percent_ = 0.0f;
        if (sequence != output_sequence_)
            return OutputCommitResult::Stale;
        if (writeDuty(MCPWM_OPR_B, 0.0f) == ESP_OK)
            last_b_percent_ = 0.0f;
        return (sequence == output_sequence_) ? OutputCommitResult::HardwareFailed : OutputCommitResult::Stale;
    };

    if (a_percent != last_a_percent_)
    {
        const esp_err_t error = writeDuty(MCPWM_OPR_A, a_percent);
        if (sequence != output_sequence_)
            return OutputCommitResult::Stale;
        if (error != ESP_OK)
            return contain_after_failure();
        last_a_percent_ = a_percent;
    }

    if (sequence != output_sequence_)
        return OutputCommitResult::Stale;

    if (b_percent != last_b_percent_)
    {
        const esp_err_t error = writeDuty(MCPWM_OPR_B, b_percent);
        if (sequence != output_sequence_)
            return OutputCommitResult::Stale;
        if (error != ESP_OK)
            return contain_after_failure();
        last_b_percent_ = b_percent;
    }

    if (enable)
    {
        if (sequence != output_sequence_)
            return OutputCommitResult::Stale;
        if (use_en_ && !en_state_)
        {
            digitalWrite(en_pin_, HIGH);
            en_state_ = true;
        }
    }

    return OutputCommitResult::Committed;
}

bool HBridgeMotor::commandOutput(bool enable, float a_percent, float b_percent, bool suppress_pwm,
                                 bool allow_fault) noexcept
{
    lockState();
    if (enable && !allow_fault && (fault_latched_ || recovery_inhibited_ ||
                                  hardware_fault_active_ || hardware_fault_latched_ ||
                                  (safety_.fault_gpio >= 0 && faultInputActive()) ||
                                  (hardware_fault_enabled_ && hardwareFaultInputActive())))
    {
        unlockState();
        return false;
    }
    commanded_enable_ = enable;
    commanded_a_percent_ = a_percent;
    commanded_b_percent_ = b_percent;
    const uint32_t sequence = ++output_sequence_;
    OutputCommitResult committed = enable ? restoreCoastRoutes(sequence, allow_fault) : OutputCommitResult::Committed;
    if (committed == OutputCommitResult::Committed)
        committed = writeHardwareOutput(sequence, enable, a_percent, b_percent, suppress_pwm);
    unlockState();

    return committed == OutputCommitResult::Committed;
}

bool HBridgeMotor::canSuppressCoast() const noexcept
{
    return use_en_ && !hardware_fault_enabled_ && !deadtime_enabled_;
}

HBridgeMotor::OutputCommitResult HBridgeMotor::restoreCoastRoutes(uint32_t sequence, bool allow_fault) noexcept
{
    if (!pwm_coast_suppressed_ || pwm_isolated_ || !mcpwm_running_)
        return OutputCommitResult::Committed;

    // A failed zero write can leave an old shadow compare queued. Retry zero
    // staging before starting the event proof if a compare is still pending.
    if (last_a_percent_ != 0.0f || last_b_percent_ != 0.0f)
    {
        const OutputCommitResult zeroed = writeHardwareOutput(sequence, false, 0.0f, 0.0f);
        if (zeroed != OutputCommitResult::Committed)
            return zeroed;
    }

    const uint32_t fault_sequence = fault_sequence_;
    const uint32_t timer = static_cast<uint32_t>(mcpwm_timer_);
    const uint32_t peak = MCPWM_TIMER0_TEP_INT_RAW << timer;
    const uint32_t zero = MCPWM_TIMER0_TEZ_INT_RAW << timer;
    const mcpwm_unit_t unit = mcpwm_unit_;
    REG_WRITE(MCPWM_INT_CLR_REG(unit), peak);
    unlockState();
    const bool reached_peak = waitForTimerEvent(unit, peak);
    REG_WRITE(MCPWM_INT_CLR_REG(unit), zero);
    const bool reached_zero = reached_peak && waitForTimerEvent(unit, zero);
    lockState();

    if (sequence != output_sequence_ || fault_sequence != fault_sequence_ ||
        (!allow_fault && (fault_latched_ || recovery_inhibited_ ||
                          (safety_.fault_gpio >= 0 && faultInputActive()))))
        return OutputCommitResult::Stale;
    if (!reached_zero || !mcpwm_running_ || !pwm_coast_suppressed_ || pwm_isolated_ || en_state_)
        return OutputCommitResult::HardwareFailed;

    // The original MCPWM GPIO initialization configured these pads as
    // matrix outputs. Change only their signal selection while EN is low.
    const uint32_t base = unit == MCPWM_UNIT_0 ? PWM0_OUT0A_IDX : PWM1_OUT0A_IDX;
    esp_rom_gpio_connect_out_signal(static_cast<uint32_t>(lpwm_pin_), base + static_cast<uint32_t>(mcpwm_sig_l_),
                                    false, false);
    esp_rom_gpio_connect_out_signal(static_cast<uint32_t>(rpwm_pin_), base + static_cast<uint32_t>(mcpwm_sig_r_),
                                    false, false);
    pwm_coast_suppressed_ = false;
    return OutputCommitResult::Committed;
}

HBridgeMotor::TimerCommandResult HBridgeMotor::writeTimerUntilCurrent(uint32_t sequence, bool active,
                                                                      int64_t timeout_us) noexcept
{
    bool superseded = false;
    for (;;)
    {
        if (soft_timer_)
            (void)esp_timer_stop(soft_timer_);

        bool start_succeeded = true;
        if (active)
        {
            start_succeeded = soft_timer_ && timeout_us > 0 && esp_timer_start_once(soft_timer_, timeout_us) == ESP_OK;
        }

        lockState();
        if (sequence == timer_sequence_)
        {
            if (!start_succeeded)
            {
                commanded_timer_active_ = false;
                commanded_timer_us_ = 0;
                ++timer_sequence_;
                soft_active_ = false;
            }
            unlockState();
            if (superseded)
                return TimerCommandResult::StaleOrCancelled;
            return start_succeeded ? TimerCommandResult::Applied : TimerCommandResult::TimerFailed;
        }
        superseded = true;
        sequence = timer_sequence_;
        active = commanded_timer_active_;
        timeout_us = commanded_timer_us_;
        unlockState();
    }
}

// ---- Software fault ISR ---- //

void HBridgeMotor::faultISRThunk(void *arg)
{
    static_cast<HBridgeMotor *>(arg)->faultISR();
}

void HBridgeMotor::faultISR() noexcept
{
    const bool active = faultInputActive();

    lockStateISR();
    if (active)
        recovery_inhibited_ = true;
    if (safety_.oneshot)
    {
        if (active && (!fault_latched_ || recovery_inhibited_))
        {
            fault_latched_ = true;
            fault_pending_ = true;
            ++fault_sequence_;
        }
    }
    else if (fault_latched_ != active)
    {
        fault_latched_ = active;
        fault_pending_ = true;
        ++fault_sequence_;
    }
    unlockStateISR();
}

bool HBridgeMotor::faultInputActive() const noexcept
{
    const int level = gpio_get_level(static_cast<gpio_num_t>(safety_.fault_gpio));
    return safety_.fault_active_high ? (level != 0) : (level == 0);
}

bool HBridgeMotor::softwareFaultActiveSnapshot() const noexcept
{
    lockState();
    const bool active = fault_latched_;
    unlockState();
    return active;
}

bool HBridgeMotor::softwareFaultPendingSnapshot() const noexcept
{
    lockState();
    const bool pending = fault_pending_;
    unlockState();
    return pending;
}

MotorOperationError HBridgeMotor::applyFaultAction() noexcept
{
    stopSoftBrake();
    setOutputMode(MotorOutputMode::FaultContainment);

    switch (safety_.fault_action)
    {
    case FaultAction::Coast:
        return commandOutput(false, 0.0f, 0.0f, canSuppressCoast()) ? MotorOperationError::None
                                                                    : MotorOperationError::HardwareWriteFailed;

    case FaultAction::DisableOutputs:
    {
        const MotorOperationError error = quiesceTimer();
        if (error == MotorOperationError::None)
            setOutputMode(MotorOutputMode::FaultContainment);
        return error;
    }

    case FaultAction::HardBrake:
        return emergencyBrake();
    }

    return MotorOperationError::InvalidCommand;
}

MotorOperationError HBridgeMotor::emergencyBrake() noexcept
{
    if (startTimer() != MotorOperationError::None)
        return MotorOperationError::HardwareStartFailed;

    return commandOutput(true, 100.0f, 100.0f, false, true) ? MotorOperationError::None
                                                            : MotorOperationError::HardwareWriteFailed;
}

void HBridgeMotor::detachFaultInterrupt() noexcept
{
    if (fault_irq_pin_ >= 0)
    {
        detachInterrupt(fault_irq_pin_);
        fault_irq_pin_ = -1;
    }
}

// ---- MCPWM hardware fault ---- //

mcpwm_io_signals_t HBridgeMotor::hardwareFaultIoSignal(HardwareFaultInput input) const noexcept
{
    switch (input)
    {
    case HardwareFaultInput::Fault0:
        return MCPWM_FAULT_0;
    case HardwareFaultInput::Fault1:
        return MCPWM_FAULT_1;
    case HardwareFaultInput::Fault2:
        return MCPWM_FAULT_2;
    }
    return MCPWM_FAULT_0;
}

mcpwm_fault_signal_t HBridgeMotor::hardwareFaultSignal(HardwareFaultInput input) const noexcept
{
    switch (input)
    {
    case HardwareFaultInput::Fault0:
        return MCPWM_SELECT_F0;
    case HardwareFaultInput::Fault1:
        return MCPWM_SELECT_F1;
    case HardwareFaultInput::Fault2:
        return MCPWM_SELECT_F2;
    }
    return MCPWM_SELECT_F0;
}

void HBridgeMotor::hardwareFaultISRThunk(void *arg)
{
    static_cast<HBridgeMotor *>(arg)->hardwareFaultISR();
}

void HBridgeMotor::hardwareFaultISR() noexcept
{
    const bool active = hardwareFaultInputActive();

    lockStateISR();
    const bool old_active = hardware_fault_active_;
    const bool old_latched = hardware_fault_latched_;
    hardware_fault_active_ = active;
    if (hardware_fault_.mode == HardwareFaultMode::OneShot && active)
        hardware_fault_latched_ = true;
    else if (hardware_fault_.mode == HardwareFaultMode::CycleByCycle)
        hardware_fault_latched_ = active;

    if (old_active != hardware_fault_active_ || old_latched != hardware_fault_latched_ ||
        (active && recovery_inhibited_))
        ++hardware_fault_sequence_;
    unlockStateISR();
}

bool HBridgeMotor::configureHardwareFault() noexcept
{
    // Peripheral actions must be installed before attaching the diagnostic ISR.
    // The ISR observes the fault; MCPWM enforces the configured A/B levels.
    if (hardware_fault_.mode == HardwareFaultMode::Disabled)
        return true;

    const mcpwm_io_signals_t io_signal = hardwareFaultIoSignal(hardware_fault_.input);
    const mcpwm_fault_signal_t fault_signal = hardwareFaultSignal(hardware_fault_.input);

    pinMode(hardware_fault_.fault_gpio, INPUT);
    if (mcpwm_gpio_init(mcpwm_unit_, io_signal, hardware_fault_.fault_gpio) != ESP_OK)
        return false;
    hardware_fault_enabled_ = true;
    if (mcpwm_fault_init(mcpwm_unit_, MCPWM_HIGH_LEVEL_TGR, fault_signal) != ESP_OK)
        return false;

    const auto action_a = (hardware_fault_.action_a == HardwareFaultOutputAction::Hold) ? MCPWM_NO_CHANGE_IN_MCPWMXA
                          : (hardware_fault_.action_a == HardwareFaultOutputAction::ForceHigh)
                              ? MCPWM_FORCE_MCPWMXA_HIGH
                              : MCPWM_FORCE_MCPWMXA_LOW;
    const auto action_b = (hardware_fault_.action_b == HardwareFaultOutputAction::Hold) ? MCPWM_NO_CHANGE_IN_MCPWMXB
                          : (hardware_fault_.action_b == HardwareFaultOutputAction::ForceHigh)
                              ? MCPWM_FORCE_MCPWMXB_HIGH
                              : MCPWM_FORCE_MCPWMXB_LOW;

    const esp_err_t mode_result =
        (hardware_fault_.mode == HardwareFaultMode::OneShot)
            ? mcpwm_fault_set_oneshot_mode(mcpwm_unit_, mcpwm_timer_, fault_signal, action_a, action_b)
            : mcpwm_fault_set_cyc_mode(mcpwm_unit_, mcpwm_timer_, fault_signal, action_a, action_b);
    if (mode_result != ESP_OK)
    {
        return false;
    }

    hardware_fault_mode_configured_ = true;
    attachInterruptArg(hardware_fault_.fault_gpio, &HBridgeMotor::hardwareFaultISRThunk, this, CHANGE);
    hardware_fault_irq_pin_ = hardware_fault_.fault_gpio;

    const bool active = hardwareFaultInputActive();
    lockState();
    hardware_fault_enabled_ = true;
    hardware_fault_active_ = active;
    hardware_fault_latched_ = (hardware_fault_.mode == HardwareFaultMode::OneShot) ? active : active;
    if (active)
        ++hardware_fault_sequence_;
    unlockState();
    return true;
}

bool HBridgeMotor::rearmHardwareFault() noexcept
{
    if (!hardware_fault_enabled_ || hardware_fault_.mode != HardwareFaultMode::OneShot)
        return true;
    if (hardwareFaultInputActive())
        return false;

    const mcpwm_fault_signal_t fault_signal = hardwareFaultSignal(hardware_fault_.input);
    if (mcpwm_fault_deinit(mcpwm_unit_, fault_signal) != ESP_OK)
        return false;
    if (mcpwm_fault_init(mcpwm_unit_, MCPWM_HIGH_LEVEL_TGR, fault_signal) != ESP_OK)
        return false;

    const auto action_a = (hardware_fault_.action_a == HardwareFaultOutputAction::Hold) ? MCPWM_NO_CHANGE_IN_MCPWMXA
                          : (hardware_fault_.action_a == HardwareFaultOutputAction::ForceHigh)
                              ? MCPWM_FORCE_MCPWMXA_HIGH
                              : MCPWM_FORCE_MCPWMXA_LOW;
    const auto action_b = (hardware_fault_.action_b == HardwareFaultOutputAction::Hold) ? MCPWM_NO_CHANGE_IN_MCPWMXB
                          : (hardware_fault_.action_b == HardwareFaultOutputAction::ForceHigh)
                              ? MCPWM_FORCE_MCPWMXB_HIGH
                              : MCPWM_FORCE_MCPWMXB_LOW;

    if (mcpwm_fault_set_oneshot_mode(mcpwm_unit_, mcpwm_timer_, fault_signal, action_a, action_b) != ESP_OK)
        return false;

    return true;
}

bool HBridgeMotor::detachHardwareFault() noexcept
{
    if (hardware_fault_irq_pin_ >= 0)
    {
        detachInterrupt(hardware_fault_irq_pin_);
        hardware_fault_irq_pin_ = -1;
    }

    bool detached = true;
    if (hardware_fault_enabled_)
    {
        const mcpwm_fault_signal_t signal = hardwareFaultSignal(hardware_fault_.input);
        // Legacy deinit disables detection but leaves per-operator brake actions.
        // Neutralize both action tables before allowing later unit reuse.
        bool neutral = true;
        if (hardware_fault_mode_configured_)
        {
            const bool oneshot = mcpwm_fault_set_oneshot_mode(mcpwm_unit_, mcpwm_timer_, signal,
                                  MCPWM_NO_CHANGE_IN_MCPWMXA, MCPWM_NO_CHANGE_IN_MCPWMXB) == ESP_OK;
            const bool cyclic = mcpwm_fault_set_cyc_mode(mcpwm_unit_, mcpwm_timer_, signal,
                                 MCPWM_NO_CHANGE_IN_MCPWMXA, MCPWM_NO_CHANGE_IN_MCPWMXB) == ESP_OK;
            neutral = oneshot && cyclic;
        }
        const bool disabled = mcpwm_fault_deinit(mcpwm_unit_, signal) == ESP_OK;
        detached = neutral && disabled;
    }

    lockState();
    if (detached)
    {
        hardware_fault_enabled_ = false;
        hardware_fault_mode_configured_ = false;
        hardware_fault_active_ = false;
        hardware_fault_latched_ = false;
    }
    unlockState();
    return detached;
}

bool HBridgeMotor::hardwareFaultInputActive() const noexcept
{
    if (hardware_fault_.fault_gpio < 0)
        return false;
    return gpio_get_level(static_cast<gpio_num_t>(hardware_fault_.fault_gpio)) != 0;
}

// ---- Capture ISR ---- //

void HBridgeMotor::capISRThunk(void *arg)
{
    static_cast<HBridgeMotor *>(arg)->capISR();
}

void HBridgeMotor::capISR() noexcept
{
    const uint32_t now = micros();
    uint32_t interval = 0;
    bool emit = false;

    lockStateISR();
    const uint32_t previous = last_edge_us_;
    const bool had_edge = capture_edge_seen_;
    last_edge_us_ = now;
    capture_edge_seen_ = true;

    if (had_edge)
    {
        interval = now - previous; // Unsigned subtraction is wrap-safe.
        period_us_ = interval;
        ++capture_sequence_;
        emit = true;
    }
    unlockStateISR();

    if (emit)
    {
        CaptureCallback callback = nullptr;
        void *user = nullptr;
        lockStateISR();
        callback = cap_.on_capture;
        user = cap_.user;
        unlockStateISR();
        if (callback)
            callback(interval, user);
    }
}

void HBridgeMotor::detachCaptureInterrupt() noexcept
{
    if (cap_irq_pin_ >= 0)
    {
        detachInterrupt(cap_irq_pin_);
        cap_irq_pin_ = -1;
    }
}

// ---- Setup / teardown helpers ---- //

bool HBridgeMotor::routePwmLow() noexcept
{
    // Take the GPIO matrix route away from MCPWM. A low GPIO latch bypasses
    // generator, compare and dead-time state even without a bridge enable pin.
    const auto left = static_cast<gpio_num_t>(lpwm_pin_);
    const auto right = static_cast<gpio_num_t>(rpwm_pin_);
    if (gpio_set_level(left, 0) != ESP_OK || gpio_set_level(right, 0) != ESP_OK)
        return false;
    esp_rom_gpio_connect_out_signal(static_cast<uint32_t>(left), SIG_GPIO_OUT_IDX, false, false);
    esp_rom_gpio_connect_out_signal(static_cast<uint32_t>(right), SIG_GPIO_OUT_IDX, false, false);
    return true;
}

bool HBridgeMotor::isolatePwm() noexcept
{
    lockState();
    const bool disconnected = pwm_coast_suppressed_ || routePwmLow();
    unlockState();
    bool isolated = false;
    if (disconnected)
    {
        gpio_config_t output{};
        output.pin_bit_mask = (UINT64_C(1) << static_cast<uint32_t>(lpwm_pin_)) |
                              (UINT64_C(1) << static_cast<uint32_t>(rpwm_pin_));
        output.mode = GPIO_MODE_OUTPUT;
        isolated = gpio_config(&output) == ESP_OK;
    }
    lockState();
    pwm_isolated_ = isolated;
    pwm_coast_suppressed_ = false;
    unlockState();
    return isolated;
}

bool HBridgeMotor::stopTimer() noexcept
{
    lockState();
    const bool running = mcpwm_running_;
    const bool pending = timer_stop_pending_;
    unlockState();
    if (!running)
        return true;

    const uint32_t mask = MCPWM_TIMER0_STOP_INT_RAW << static_cast<uint32_t>(mcpwm_timer_);
    if (!pending)
    {
        // W1C only our timer's stale event, before the request. A retry waits
        // for the original event so it can recover even after a late stop.
        REG_WRITE(MCPWM_INT_CLR_REG(mcpwm_unit_), mask);
        if (mcpwm_stop(mcpwm_unit_, mcpwm_timer_) != ESP_OK)
            return false;
        lockState();
        timer_stop_pending_ = true;
        unlockState();
    }
    if (!waitForTimerEvent(mcpwm_unit_, mask))
        return false;
    lockState();
    timer_stop_pending_ = false;
    mcpwm_running_ = false;
    unlockState();
    return true;
}

MotorOperationError HBridgeMotor::startTimer(bool abort_on_fault) noexcept
{
    lockState();
    const bool unsafe = timer_stop_pending_ || timer_recovery_required_;
    unlockState();
    if (unsafe)
        return MotorOperationError::HardwareStartFailed;

    const uint32_t timer = static_cast<uint32_t>(mcpwm_timer_);
    const uint32_t empty = MCPWM_TIMER0_TEZ_INT_RAW << timer;
    const uint32_t full = MCPWM_TIMER0_TEP_INT_RAW << timer;
    REG_WRITE(MCPWM_INT_CLR_REG(mcpwm_unit_), full);
    if (mcpwm_start(mcpwm_unit_, mcpwm_timer_) != ESP_OK)
        return MotorOperationError::HardwareStartFailed;
    lockState();
    mcpwm_running_ = true;
    unlockState();
    if (!pwm_isolated_)
        return MotorOperationError::None;

    const bool reached_full = waitForTimerEvent(mcpwm_unit_, full);
    // Observe zero after peak, even if the initial start at zero also set TEZ.
    // Both the compare shadow update and the generator's zero-duty action
    // have then had a timer event in the new cycle before either pin is routed.
    REG_WRITE(MCPWM_INT_CLR_REG(mcpwm_unit_), empty);
    if (!reached_full || !waitForTimerEvent(mcpwm_unit_, empty))
    {
        const bool stopped = stopTimer();
        lockState();
        timer_recovery_required_ = !stopped;
        unlockState();
        return MotorOperationError::HardwareStartFailed;
    }

    if (abort_on_fault && (hasFault() || (hardware_fault_enabled_ && hardwareFaultInputActive())))
    {
        const bool stopped = stopTimer();
        lockState();
        timer_recovery_required_ = !stopped;
        unlockState();
        return MotorOperationError::FaultActive;
    }

    const bool routed = mcpwm_gpio_init(mcpwm_unit_, mcpwm_sig_l_, lpwm_pin_) == ESP_OK &&
                        mcpwm_gpio_init(mcpwm_unit_, mcpwm_sig_r_, rpwm_pin_) == ESP_OK;
    lockState();
    pwm_isolated_ = false;
    unlockState();
    if (!routed)
    {
        const bool isolated = isolatePwm();
        const bool stopped = stopTimer();
        lockState();
        timer_recovery_required_ = !isolated || !stopped;
        unlockState();
    }
    return routed ? MotorOperationError::None : MotorOperationError::HardwareStartFailed;
}

MotorOperationError HBridgeMotor::quiesceTimer() noexcept
{
    // Withdraw EN and remove both timer routes in the same output commit,
    // ahead of all buffered compare writes and the timer stop request.
    const bool zeroed = commandOutput(false, 0.0f, 0.0f, true);
    const bool isolated = isolatePwm();
    const bool stopped = stopTimer();
    lockState();
    output_mode_ = (isolated || (use_en_ && !en_state_)) ? MotorOutputMode::Disabled : MotorOutputMode::Uncertain;
    // A partial stop inhibits commands until the owner retries stop or setup.
    timer_recovery_required_ = !zeroed || !isolated || !stopped;
    unlockState();
    if (!zeroed || !isolated)
        return MotorOperationError::HardwareWriteFailed;
    return stopped ? MotorOperationError::None : MotorOperationError::HardwareStopFailed;
}

MotorSetupError HBridgeMotor::prepareForSetup() noexcept
{
    // Keep the old resource identity until every containment step succeeds;
    // clearing it early would prevent a later attempt from retrying teardown.
    stopSoftBrake();

    if (mcpwm_initialized_)
    {
        const MotorOperationError stopped = quiesceTimer();
        if (stopped != MotorOperationError::None ||
            (deadtime_enabled_ && mcpwm_deadtime_disable(mcpwm_unit_, mcpwm_timer_) != ESP_OK))
        {
            lockState();
            setup_error_ = MotorSetupError::ContainmentFailed;
            timer_recovery_required_ = true;
            unlockState();
            return MotorSetupError::ContainmentFailed;
        }
        deadtime_enabled_ = false;
    }

    detachFaultInterrupt();
    detachCaptureInterrupt();
    if (!detachHardwareFault() || !releaseResources())
    {
        lockState();
        setup_error_ = MotorSetupError::ContainmentFailed;
        output_mode_ = (pwm_isolated_ || (use_en_ && !en_state_))
                           ? MotorOutputMode::Disabled : MotorOutputMode::Uncertain;
        timer_recovery_required_ = true;
        unlockState();
        return setup_error_;
    }

    lockState();
    recovery_inhibited_ = false;
    fault_latched_ = false;
    fault_pending_ = false;
    fault_sequence_ = 0;
    hardware_fault_active_ = false;
    hardware_fault_latched_ = false;
    hardware_fault_sequence_ = 0;
    last_edge_us_ = 0;
    period_us_ = 0;
    capture_edge_seen_ = false;
    capture_sequence_ = 0;
    last_a_percent_ = -1.0f;
    last_b_percent_ = -1.0f;
    setup_done_ = false;
    mcpwm_initialized_ = false;
    mcpwm_running_ = false;
    timer_stop_pending_ = false;
    timer_recovery_required_ = false;
    pwm_isolated_ = false;
    pwm_coast_suppressed_ = false;
    deadtime_enabled_ = false;
    hardware_fault_enabled_ = false;
    hardware_fault_ = MotorHardwareFaultConfig{};
    use_en_ = false;
    en_state_ = false;
    lpwm_pin_ = -1;
    rpwm_pin_ = -1;
    en_pin_ = -1;
    mcpwm_unit_ = MCPWM_UNIT_0;
    mcpwm_timer_ = MCPWM_TIMER_0;
    mcpwm_sig_l_ = MCPWM0A;
    mcpwm_sig_r_ = MCPWM0B;
    pwm_freq_hz_ = 20000;
    input_max_ = 1023;
    percent_per_count_ = 0.0f;
    min_phase_us_ = 50;
    dither_coast_hi_z_ = false;
    counter_mode_ = MCPWM_UP_COUNTER;
    beh_ = MotorBehaviorConfig{};
    safety_ = MotorSafetyConfig{};
    cap_ = MotorCaptureConfig{};
    soft_hz_ = 300;
    soft_us_brake_ = 0;
    soft_us_coast_ = 0;
    soft_brake_pwm_ = 0;
    setup_error_ = MotorSetupError::None;
    output_mode_ = MotorOutputMode::Unconfigured;
    commanded_enable_ = false;
    commanded_a_percent_ = 0.0f;
    commanded_b_percent_ = 0.0f;
    output_sequence_ = 0;
    operation_sequence_ = 0;
    last_operation_ = MotorOperation::None;
    last_operation_error_ = MotorOperationError::None;
    unlockState();
    return MotorSetupError::None;
}

MotorSetupError HBridgeMotor::validateConfig(const MotorMCPWMConfig &hw, const MotorBehaviorConfig &beh,
                                             const MotorSafetyConfig &safety, const MotorCaptureConfig &cap,
                                             const MotorHardwareFaultConfig &hardware_fault) const noexcept
{
    // Legacy duty writes bind timer x to operator x and generators A/B.
    if (static_cast<int>(hw.unit) < 0 || hw.unit >= SOC_MCPWM_GROUPS ||
        static_cast<int>(hw.timer) < 0 || hw.timer >= SOC_MCPWM_TIMERS_PER_GROUP ||
        hw.timer >= SOC_MCPWM_OPERATORS_PER_GROUP ||
        static_cast<int>(hw.sig_l) != 2 * static_cast<int>(hw.timer) ||
        static_cast<int>(hw.sig_r) != 2 * static_cast<int>(hw.timer) + 1)
        return MotorSetupError::InvalidTimerSignals;
    // Legacy RED/FED arguments are register counts, not nanoseconds.
    if (hw.use_deadtime && (hw.deadtime_red_ns > UINT16_MAX || hw.deadtime_fed_ns > UINT16_MAX))
        return MotorSetupError::InvalidDeadtime;
    if (!GPIO_IS_VALID_OUTPUT_GPIO(hw.lpwm_pin) || !GPIO_IS_VALID_OUTPUT_GPIO(hw.rpwm_pin))
        return MotorSetupError::InvalidPwmPin;
    if (hw.lpwm_pin == hw.rpwm_pin)
        return MotorSetupError::DuplicatePwmPin;
    if (hw.en_pin >= 0 &&
        (!GPIO_IS_VALID_OUTPUT_GPIO(hw.en_pin) || hw.en_pin == hw.lpwm_pin || hw.en_pin == hw.rpwm_pin))
        return MotorSetupError::PinConflict;
    if (hw.sig_l == hw.sig_r)
        return MotorSetupError::PinConflict;
    if (legacyFrequency(hw.counter, hw.pwm_freq_hz) == 0U)
        return MotorSetupError::InvalidPwmFrequency;
    if (hw.input_max <= 0 || hw.input_max > static_cast<int>(UINT16_MAX))
        return MotorSetupError::InvalidInputRange;
    if (beh.soft_brake_hz <= 0 || beh.soft_brake_hz > kSoftHzMax ||
        (kMicrosPerSec / static_cast<uint32_t>(beh.soft_brake_hz)) < 2U)
        return MotorSetupError::InvalidDitherConfig;

    if (safety.fault_gpio >= 0)
    {
        if (!GPIO_IS_VALID_GPIO(safety.fault_gpio) || safety.fault_gpio == hw.lpwm_pin ||
            safety.fault_gpio == hw.rpwm_pin || safety.fault_gpio == hw.en_pin)
            return MotorSetupError::PinConflict;
    }

    if (cap.cap_gpio >= 0)
    {
        if (!GPIO_IS_VALID_GPIO(cap.cap_gpio) || cap.cap_gpio == hw.lpwm_pin || cap.cap_gpio == hw.rpwm_pin ||
            cap.cap_gpio == hw.en_pin || cap.cap_gpio == safety.fault_gpio)
            return MotorSetupError::PinConflict;
    }

    if (hardware_fault.mode != HardwareFaultMode::Disabled)
    {
        if (static_cast<int>(hardware_fault.input) >= SOC_MCPWM_GPIO_FAULTS_PER_GROUP)
            return MotorSetupError::InvalidHardwareFaultConfig;
        if (hardware_fault.fault_gpio < 0 || !GPIO_IS_VALID_GPIO(hardware_fault.fault_gpio))
            return MotorSetupError::InvalidHardwareFaultConfig;
        if (!hardware_fault.active_high)
            return MotorSetupError::UnsupportedHardwareFaultLevel;
        if (hardware_fault.fault_gpio == hw.lpwm_pin || hardware_fault.fault_gpio == hw.rpwm_pin ||
            hardware_fault.fault_gpio == hw.en_pin || hardware_fault.fault_gpio == safety.fault_gpio ||
            hardware_fault.fault_gpio == cap.cap_gpio)
            return MotorSetupError::PinConflict;
    }
    else if (hardware_fault.fault_gpio >= 0)
    {
        return MotorSetupError::InvalidHardwareFaultConfig;
    }

    return MotorSetupError::None;
}

void HBridgeMotor::failSetup(MotorSetupError error) noexcept
{
    // Teardown errors outrank the initiating setup error: callers must know
    // whether partially initialized hardware still needs containment.
    detachFaultInterrupt();
    detachCaptureInterrupt();
    stopSoftBrake();

    const bool stopped = !mcpwm_initialized_ || quiesceTimer() == MotorOperationError::None;
    const bool deadtime_disabled = stopped &&
        (!deadtime_enabled_ || mcpwm_deadtime_disable(mcpwm_unit_, mcpwm_timer_) == ESP_OK);

    // Preserve peripheral fault actions and reservation until the pins and
    // timer are contained. Never detach a brake over an unconfirmed stop.
    const bool hardware_fault_detached = stopped && detachHardwareFault();
    if (!hardware_fault_detached && hardware_fault_irq_pin_ >= 0)
    {
        detachInterrupt(hardware_fault_irq_pin_);
        hardware_fault_irq_pin_ = -1;
    }
    const bool teardown_ok = stopped && deadtime_disabled && hardware_fault_detached && releaseResources();
    lockState();
    const bool contained_by_en = use_en_ && !en_state_;
    const bool contained = contained_by_en || pwm_isolated_;
    setup_done_ = false;
    if (teardown_ok)
        mcpwm_initialized_ = false;
    if (deadtime_disabled)
        deadtime_enabled_ = false;
    if (hardware_fault_detached)
        hardware_fault_enabled_ = false;
    setup_error_ = teardown_ok ? error : MotorSetupError::ContainmentFailed;
    output_mode_ = teardown_ok ? MotorOutputMode::Unconfigured
                   : contained ? MotorOutputMode::Disabled
                               : MotorOutputMode::Uncertain;
    unlockState();
}
