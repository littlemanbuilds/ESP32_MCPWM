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

HBridgeMotor::~HBridgeMotor() noexcept
{
    // Best effort only: the destructor cannot report MCPWM API failures. An
    // independent EN path is the only containment guarantee when duty/stop
    // calls themselves fail during destruction.
    detachFaultInterrupt();
    detachCaptureInterrupt();
    (void)detachHardwareFault();
    stopSoftBrake();

    if (setup_done_ || mcpwm_initialized_)
    {
        (void)commandOutput(false, 0.0f, 0.0f);
        if (deadtime_enabled_)
            (void)mcpwm_deadtime_disable(mcpwm_unit_, mcpwm_timer_);
        (void)mcpwm_stop(mcpwm_unit_, mcpwm_timer_);
    }

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

    use_en_ = (en_pin_ >= 0);
    if (use_en_)
    {
        // Assert the safe level before the pin becomes an output.
        digitalWrite(en_pin_, LOW);
        pinMode(en_pin_, OUTPUT);
        en_state_ = false;
    }

    if (mcpwm_gpio_init(mcpwm_unit_, mcpwm_sig_l_, lpwm_pin_) != ESP_OK ||
        mcpwm_gpio_init(mcpwm_unit_, mcpwm_sig_r_, rpwm_pin_) != ESP_OK)
    {
        failSetup(MotorSetupError::HardwareInitFailed);
        return setupResult();
    }

    mcpwm_config_t cfg{};
    cfg.frequency = pwm_freq_hz_;
    cfg.cmpr_a = 0.0f;
    cfg.cmpr_b = 0.0f;
    cfg.counter_mode = counter_mode_;
    cfg.duty_mode = MCPWM_DUTY_MODE_0;
    if (mcpwm_init(mcpwm_unit_, mcpwm_timer_, &cfg) != ESP_OK)
    {
        failSetup(MotorSetupError::HardwareInitFailed);
        return setupResult();
    }
    mcpwm_initialized_ = true;
    mcpwm_running_ = true;

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

    if (mcpwm_set_duty_type(mcpwm_unit_, mcpwm_timer_, MCPWM_OPR_A, MCPWM_DUTY_MODE_0) != ESP_OK ||
        mcpwm_set_duty_type(mcpwm_unit_, mcpwm_timer_, MCPWM_OPR_B, MCPWM_DUTY_MODE_0) != ESP_OK)
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
    const float duty = static_cast<float>(request) * percent_per_count_;
    lockState();
    const MotorOutputMode requested_mode = (dir == Dir::CW) ? MotorOutputMode::DriveCW : MotorOutputMode::DriveCCW;
    const bool changed = output_mode_ != requested_mode || soft_active_ || !commanded_enable_ ||
                         std::fabs(commanded_a_percent_ - ((dir == Dir::CW) ? duty : 0.0f)) > kDutyEps ||
                         std::fabs(commanded_b_percent_ - ((dir == Dir::CCW) ? duty : 0.0f)) > kDutyEps;
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
    const int speed = static_cast<int>(clamped * (static_cast<float>(getMaxPwmInput()) / 100.0f) + 0.5f);
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
        ok = commandOutput(false, 0.0f, 0.0f);
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
        stopSoftBrake();

        // A level-follow DisableOutputs fault may have stopped MCPWM. When the
        // observed input clears, restore only the peripheral lifecycle and an
        // electrically quiet output; never restore the previous drive command.
        lockState();
        const bool running = mcpwm_running_;
        unlockState();
        if (!running)
        {
            if (mcpwm_start(mcpwm_unit_, mcpwm_timer_) != ESP_OK)
            {
                error = MotorOperationError::HardwareStartFailed;
            }
            else
            {
                lockState();
                mcpwm_running_ = true;
                unlockState();
            }
        }

        if (error == MotorOperationError::None)
        {
            if (!commandOutput(false, 0.0f, 0.0f))
                error = MotorOperationError::HardwareWriteFailed;
            else
                setOutputMode(MotorOutputMode::Coast);
        }
    }

    if (callback)
        callback(active, context);

    return (error == MotorOperationError::None) ? result(MotorOperation::PollFaults, error, true)
                                                : reject(MotorOperation::PollFaults, error);
}

bool HBridgeMotor::hasFault() const noexcept
{
    lockState();
    const bool active = fault_latched_ || hardware_fault_active_ || hardware_fault_latched_;
    unlockState();
    return active;
}

MotorOperationResult HBridgeMotor::clearFault() noexcept
{
    if (!isSetupComplete())
        return reject(MotorOperation::ClearFault, MotorOperationError::NotSetup);

    // Check and clear the software-side latch as one synchronized decision.
    // Reading the GPIO while holding the same state lock prevents an ISR from
    // racing between the physical-level check and the latch update.
    lockState();
    const bool software_input_active = (safety_.fault_gpio >= 0) ? faultInputActive() : false;
    const bool hardware_input_active = hardware_fault_enabled_ ? hardwareFaultInputActive() : false;
    if (software_input_active || hardware_input_active)
    {
        unlockState();
        return reject(MotorOperation::ClearFault, MotorOperationError::FaultActive);
    }

    fault_latched_ = false;
    fault_pending_ = false;
    ++fault_sequence_;
    unlockState();

    // Stage an electrically quiet base command *before* releasing a one-shot
    // peripheral fault. MCPWM fault actions override the generators; they do
    // not necessarily erase the compare values that existed before the fault.
    // Pre-staging zero prevents an old drive duty from reappearing on re-arm.
    stopSoftBrake();
    if (!commandOutput(false, 0.0f, 0.0f))
        return reject(MotorOperation::ClearFault, MotorOperationError::HardwareWriteFailed);

    if (hardware_fault_enabled_ && hardware_fault_.mode == HardwareFaultMode::OneShot)
    {
        if (!rearmHardwareFault())
            return reject(MotorOperation::ClearFault, MotorOperationError::HardwareFaultClearFailed);
    }
    else
    {
        lockState();
        hardware_fault_active_ = false;
        hardware_fault_latched_ = false;
        ++hardware_fault_sequence_;
        unlockState();
    }

    lockState();
    const bool running = mcpwm_running_;
    unlockState();
    if (!running)
    {
        if (mcpwm_start(mcpwm_unit_, mcpwm_timer_) != ESP_OK)
            return reject(MotorOperation::ClearFault, MotorOperationError::HardwareStartFailed);
        lockState();
        mcpwm_running_ = true;
        unlockState();
    }

    bool ok = true;
    switch (beh_.freewheel_mode)
    {
    case FreewheelMode::HiZ:
        ok = commandOutput(false, 0.0f, 0.0f);
        break;
    case FreewheelMode::HiZ_Awake:
        ok = commandOutput(true, 0.0f, 0.0f);
        break;
    case FreewheelMode::DitherBrake:
        // Fault recovery is deliberately quiet. A new explicit coast/dither
        // request is required before dither braking resumes.
        ok = commandOutput(false, 0.0f, 0.0f);
        break;
    }
    if (!ok)
        return reject(MotorOperation::ClearFault, MotorOperationError::HardwareWriteFailed);

    setOutputMode(MotorOutputMode::Coast);
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
    if (mcpwm_start(mcpwm_unit_, mcpwm_timer_) != ESP_OK)
        return reject(MotorOperation::Start, MotorOperationError::HardwareStartFailed);

    lockState();
    mcpwm_running_ = true;
    unlockState();

    bool ok = true;
    switch (beh_.freewheel_mode)
    {
    case FreewheelMode::HiZ:
        ok = commandOutput(false, 0.0f, 0.0f);
        break;
    case FreewheelMode::HiZ_Awake:
        ok = commandOutput(true, 0.0f, 0.0f);
        break;
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
    const bool wrote_zero = commandOutput(false, 0.0f, 0.0f);
    const esp_err_t stop_error = mcpwm_stop(mcpwm_unit_, mcpwm_timer_);

    lockState();
    if (stop_error == ESP_OK)
        mcpwm_running_ = false;
    unlockState();

    if (!wrote_zero)
        return reject(MotorOperation::Stop, MotorOperationError::HardwareWriteFailed);
    if (stop_error != ESP_OK)
        return reject(MotorOperation::Stop, MotorOperationError::HardwareStopFailed);

    setOutputMode(MotorOutputMode::Disabled);
    return result(MotorOperation::Stop, MotorOperationError::None, changed);
}

MotorOperationResult HBridgeMotor::reconfigureFrequency(int new_hz) noexcept
{
    if (!isSetupComplete())
        return reject(MotorOperation::ReconfigureFrequency, MotorOperationError::NotSetup);
    if (new_hz < kPwmHzMin || new_hz > kPwmHzMax)
        return reject(MotorOperation::ReconfigureFrequency, MotorOperationError::InvalidCommand);

    lockState();
    const bool restart_dither = soft_active_;
    const MotorOutputMode previous_mode = output_mode_;
    const int previous_hz = pwm_freq_hz_;
    unlockState();

    if (new_hz == previous_hz)
        return result(MotorOperation::ReconfigureFrequency, MotorOperationError::None, false);

    if (!hasFault())
    {
        stopSoftBrake();
        if (!commandOutput(false, 0.0f, 0.0f))
        {
            containFailedDither();
            lockState();
            const bool changed = output_mode_ != previous_mode || soft_active_;
            unlockState();
            return result(MotorOperation::ReconfigureFrequency, MotorOperationError::HardwareWriteFailed, changed);
        }
        setOutputMode(MotorOutputMode::Coast);
    }

    if (mcpwm_set_frequency(mcpwm_unit_, mcpwm_timer_, new_hz) != ESP_OK)
    {
        lockState();
        const bool changed = output_mode_ != previous_mode || soft_active_;
        unlockState();
        return result(MotorOperation::ReconfigureFrequency, MotorOperationError::FrequencyChangeFailed, changed);
    }

    lockState();
    pwm_freq_hz_ = new_hz;
    unlockState();

    if (restart_dither && !hasFault())
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
    unlockState();

    if (!setup)
        return out;

    out.valid = true;
    out.frequency_hz = mcpwm_get_frequency(mcpwm_unit_, mcpwm_timer_);
    out.duty_a_percent = mcpwm_get_duty(mcpwm_unit_, mcpwm_timer_, MCPWM_OPR_A);
    out.duty_b_percent = mcpwm_get_duty(mcpwm_unit_, mcpwm_timer_, MCPWM_OPR_B);
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

    stopSoftBrake();
    if (!commandOutput(true, a_high ? 100.0f : 0.0f, b_high ? 100.0f : 0.0f))
        return reject(MotorOperation::ForceOutputs, MotorOperationError::HardwareWriteFailed);

    return result(MotorOperation::ForceOutputs, MotorOperationError::None, true);
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
    if (!setup_done_ || !soft_active_ || fault_latched_ || hardware_fault_active_ || hardware_fault_latched_)
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
    if (!setup_done_ || !soft_active_ || fault_latched_ || hardware_fault_active_ || hardware_fault_latched_ ||
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
    if (!soft_active_ || fault_latched_ || hardware_fault_active_ || hardware_fault_latched_ ||
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
                                                                   float b_percent) noexcept
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

    const bool both_change = std::fabs(a_percent - last_a) > kDutyEps && std::fabs(b_percent - last_b) > kDutyEps;

    const auto contain_after_failure = [this, sequence]() -> OutputCommitResult
    {
        if (use_en_ && en_state_)
        {
            digitalWrite(en_pin_, LOW);
            en_state_ = false;
        }

        if (sequence != output_sequence_)
            return OutputCommitResult::Stale;
        if (mcpwm_set_duty(mcpwm_unit_, mcpwm_timer_, MCPWM_OPR_A, 0.0f) == ESP_OK)
            last_a_percent_ = 0.0f;
        if (sequence != output_sequence_)
            return OutputCommitResult::Stale;
        if (mcpwm_set_duty(mcpwm_unit_, mcpwm_timer_, MCPWM_OPR_B, 0.0f) == ESP_OK)
            last_b_percent_ = 0.0f;
        return (sequence == output_sequence_) ? OutputCommitResult::HardwareFailed : OutputCommitResult::Stale;
    };

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

    if (std::fabs(a_percent - last_a_percent_) > kDutyEps)
    {
        const esp_err_t error = mcpwm_set_duty(mcpwm_unit_, mcpwm_timer_, MCPWM_OPR_A, a_percent);
        if (sequence != output_sequence_)
            return OutputCommitResult::Stale;
        if (error != ESP_OK)
            return contain_after_failure();
        last_a_percent_ = a_percent;
    }

    if (sequence != output_sequence_)
        return OutputCommitResult::Stale;

    if (std::fabs(b_percent - last_b_percent_) > kDutyEps)
    {
        const esp_err_t error = mcpwm_set_duty(mcpwm_unit_, mcpwm_timer_, MCPWM_OPR_B, b_percent);
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

bool HBridgeMotor::commandOutput(bool enable, float a_percent, float b_percent) noexcept
{
    lockState();
    commanded_enable_ = enable;
    commanded_a_percent_ = a_percent;
    commanded_b_percent_ = b_percent;
    const uint32_t sequence = ++output_sequence_;
    const OutputCommitResult committed = writeHardwareOutput(sequence, enable, a_percent, b_percent);
    unlockState();

    return committed == OutputCommitResult::Committed;
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
    if (safety_.oneshot)
    {
        if (active && !fault_latched_)
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
        return commandOutput(false, 0.0f, 0.0f) ? MotorOperationError::None : MotorOperationError::HardwareWriteFailed;

    case FaultAction::DisableOutputs:
        if (!commandOutput(false, 0.0f, 0.0f))
            return MotorOperationError::HardwareWriteFailed;
        if (mcpwm_stop(mcpwm_unit_, mcpwm_timer_) != ESP_OK)
            return MotorOperationError::HardwareStopFailed;
        lockState();
        mcpwm_running_ = false;
        unlockState();
        return MotorOperationError::None;

    case FaultAction::HardBrake:
        return emergencyBrake();
    }

    return MotorOperationError::InvalidCommand;
}

MotorOperationError HBridgeMotor::emergencyBrake() noexcept
{
    if (mcpwm_start(mcpwm_unit_, mcpwm_timer_) != ESP_OK)
        return MotorOperationError::HardwareStartFailed;
    lockState();
    mcpwm_running_ = true;
    unlockState();

    return commandOutput(true, 100.0f, 100.0f) ? MotorOperationError::None : MotorOperationError::HardwareWriteFailed;
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

    if (old_active != hardware_fault_active_ || old_latched != hardware_fault_latched_)
        ++hardware_fault_sequence_;
    unlockStateISR();
}

bool HBridgeMotor::configureHardwareFault() noexcept
{
    if (hardware_fault_.mode == HardwareFaultMode::Disabled)
        return true;

    const mcpwm_io_signals_t io_signal = hardwareFaultIoSignal(hardware_fault_.input);
    const mcpwm_fault_signal_t fault_signal = hardwareFaultSignal(hardware_fault_.input);

    pinMode(hardware_fault_.fault_gpio, INPUT);
    if (mcpwm_gpio_init(mcpwm_unit_, io_signal, hardware_fault_.fault_gpio) != ESP_OK)
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

    const esp_err_t mode_result =
        (hardware_fault_.mode == HardwareFaultMode::OneShot)
            ? mcpwm_fault_set_oneshot_mode(mcpwm_unit_, mcpwm_timer_, fault_signal, action_a, action_b)
            : mcpwm_fault_set_cyc_mode(mcpwm_unit_, mcpwm_timer_, fault_signal, action_a, action_b);
    if (mode_result != ESP_OK)
    {
        (void)mcpwm_fault_deinit(mcpwm_unit_, fault_signal);
        return false;
    }

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

    lockState();
    hardware_fault_active_ = false;
    hardware_fault_latched_ = false;
    ++hardware_fault_sequence_;
    unlockState();
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
        detached = mcpwm_fault_deinit(mcpwm_unit_, hardwareFaultSignal(hardware_fault_.input)) == ESP_OK;
    }

    lockState();
    if (detached)
    {
        hardware_fault_enabled_ = false;
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

MotorSetupError HBridgeMotor::prepareForSetup() noexcept
{
    stopSoftBrake();

    bool zeroed = true;
    bool deadtime_disabled = true;
    bool stopped = true;
    if (setup_done_ || mcpwm_initialized_)
    {
        zeroed = commandOutput(false, 0.0f, 0.0f);
        if (deadtime_enabled_)
            deadtime_disabled = mcpwm_deadtime_disable(mcpwm_unit_, mcpwm_timer_) == ESP_OK;
        stopped = mcpwm_stop(mcpwm_unit_, mcpwm_timer_) == ESP_OK;

        lockState();
        if (stopped)
            mcpwm_running_ = false;
        if (deadtime_disabled)
            deadtime_enabled_ = false;
        const bool contained_by_en = use_en_ && !en_state_;
        const bool contained = contained_by_en || (zeroed && stopped);
        if (!zeroed || !deadtime_disabled || !stopped)
        {
            setup_error_ = MotorSetupError::ContainmentFailed;
            output_mode_ = contained ? MotorOutputMode::Disabled : MotorOutputMode::Uncertain;
            unlockState();
            return MotorSetupError::ContainmentFailed;
        }
        unlockState();
    }

    detachFaultInterrupt();
    detachCaptureInterrupt();
    if (!detachHardwareFault())
    {
        lockState();
        setup_error_ = MotorSetupError::ContainmentFailed;
        output_mode_ = MotorOutputMode::Disabled;
        unlockState();
        return MotorSetupError::ContainmentFailed;
    }

    lockState();
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
    if (!GPIO_IS_VALID_OUTPUT_GPIO(hw.lpwm_pin) || !GPIO_IS_VALID_OUTPUT_GPIO(hw.rpwm_pin))
        return MotorSetupError::InvalidPwmPin;
    if (hw.lpwm_pin == hw.rpwm_pin)
        return MotorSetupError::DuplicatePwmPin;
    if (hw.en_pin >= 0 &&
        (!GPIO_IS_VALID_OUTPUT_GPIO(hw.en_pin) || hw.en_pin == hw.lpwm_pin || hw.en_pin == hw.rpwm_pin))
        return MotorSetupError::PinConflict;
    if (hw.sig_l == hw.sig_r)
        return MotorSetupError::PinConflict;
    if (hw.pwm_freq_hz < kPwmHzMin || hw.pwm_freq_hz > kPwmHzMax)
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
    detachFaultInterrupt();
    detachCaptureInterrupt();
    const bool hardware_fault_detached = detachHardwareFault();
    stopSoftBrake();

    bool zeroed = true;
    bool deadtime_disabled = true;
    bool stopped = true;
    if (mcpwm_initialized_)
    {
        zeroed = commandOutput(false, 0.0f, 0.0f);
        if (deadtime_enabled_)
            deadtime_disabled = mcpwm_deadtime_disable(mcpwm_unit_, mcpwm_timer_) == ESP_OK;
        stopped = mcpwm_stop(mcpwm_unit_, mcpwm_timer_) == ESP_OK;
    }

    lockState();
    const bool contained_by_en = use_en_ && !en_state_;
    const bool contained = contained_by_en || (zeroed && stopped);
    const bool teardown_ok = zeroed && deadtime_disabled && stopped && hardware_fault_detached;
    setup_done_ = false;
    if (teardown_ok)
        mcpwm_initialized_ = false;
    if (stopped)
        mcpwm_running_ = false;
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
