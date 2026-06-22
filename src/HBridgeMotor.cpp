/**
 * MIT License
 *
 * @brief MCPWM-based dual H-bridge motor driver implementation.
 *
 * @file HBridgeMotor.cpp
 * @author Little Man Builds (Darren Osborne)
 * @date 2025-08-28
 * @copyright Copyright (c) 2025 Little Man Builds
 */

#include <HBridgeMotor.h>
#include <driver/gpio.h>
#include <cmath>

// Local clamp to avoid external deps.
template <typename T>
static inline T clamp(T v, T lo, T hi)
{
    return (v < lo) ? lo : (v > hi) ? hi
                                    : v;
}

// Virtual destructor for safe polymorphic deletion.
HBridgeMotor::~HBridgeMotor() noexcept
{
    detachFaultInterrupt();
    detachCaptureInterrupt();
    stopSoftBrake();
    if (setup_done_)
    {
        commandOutput(false, 0.0f, 0.0f);
        if (deadtime_enabled_)
            (void)mcpwm_deadtime_disable(mcpwm_unit_, mcpwm_timer_);
        (void)mcpwm_stop(mcpwm_unit_, mcpwm_timer_);
        setup_done_ = false;
        mcpwm_initialized_ = false;
        deadtime_enabled_ = false;
    }
    if (soft_timer_)
    {
        (void)esp_timer_stop(soft_timer_);
        (void)esp_timer_delete(soft_timer_);
        soft_timer_ = nullptr;
    }
}

// Initialize the driver with hardware configuration.
void HBridgeMotor::setup(const MotorMCPWMConfig &hw)
{
    MotorBehaviorConfig def{};
    if (hw.en_pin < 0 && def.freewheel_mode == FreewheelMode::HiZ)
    {
        // Without EN, A/B=0 is the only coast-like state the library can command.
        def.freewheel_mode = FreewheelMode::HiZ_Awake;
    }
    setup(hw, def);
}

// Initialize the driver with hardware and behavior configuration.
void HBridgeMotor::setup(const MotorMCPWMConfig &hw, const MotorBehaviorConfig &beh)
{
    MotorSafetyConfig sfty{};
    MotorCaptureConfig cap{};
    setup(hw, beh, sfty, cap);
}

// Initialize the driver with hardware, behavior, safety, and capture configs.
void HBridgeMotor::setup(const MotorMCPWMConfig &hw, const MotorBehaviorConfig &beh,
                         const MotorSafetyConfig &safety, const MotorCaptureConfig &cap)
{
    prepareForSetup();

    setup_error_ = validateConfig(hw, beh, safety, cap);
    if (setup_error_ != MotorSetupError::None)
        return;

    // Pin & route mirrors.
    lpwm_pin_ = hw.lpwm_pin;
    rpwm_pin_ = hw.rpwm_pin;
    en_pin_ = hw.en_pin;
    mcpwm_unit_ = hw.unit;
    mcpwm_timer_ = hw.timer;
    mcpwm_sig_l_ = hw.sig_l;
    mcpwm_sig_r_ = hw.sig_r;

    // Config mirrors.
    beh_ = beh;
    min_phase_us_ = beh_.min_phase_us;
    dither_coast_hi_z_ = beh_.dither_coast_hi_z;
    safety_ = safety;
    cap_ = cap;
    soft_hz_ = beh_.soft_brake_hz;
    pwm_freq_hz_ = hw.pwm_freq_hz;
    input_max_ = hw.input_max;
    counter_mode_ = hw.counter;
    percent_per_count_ = 100.0f / static_cast<float>(input_max_);
    soft_brake_pwm_ = static_cast<uint16_t>(clamp<int>(beh_.default_soft_brake_pwm, 0, input_max_));

    // EN pin usage.
    use_en_ = (en_pin_ >= 0);
    if (use_en_)
    {
        // Hold the bridge inactive while MCPWM and behavior state are initialized.
        digitalWrite(en_pin_, LOW);
        pinMode(en_pin_, OUTPUT);
        en_state_ = false;
    }

    // GPIO routing.
    if (mcpwm_gpio_init(mcpwm_unit_, mcpwm_sig_l_, lpwm_pin_) != ESP_OK ||
        mcpwm_gpio_init(mcpwm_unit_, mcpwm_sig_r_, rpwm_pin_) != ESP_OK)
    {
        failSetup(MotorSetupError::HardwareInitFailed);
        return;
    }

    // Timer/channel setup.
    mcpwm_config_t cfg{};
    cfg.frequency = pwm_freq_hz_;
    cfg.cmpr_a = 0;
    cfg.cmpr_b = 0;
    cfg.counter_mode = counter_mode_;
    cfg.duty_mode = MCPWM_DUTY_MODE_0;
    if (mcpwm_init(mcpwm_unit_, mcpwm_timer_, &cfg) != ESP_OK)
    {
        failSetup(MotorSetupError::HardwareInitFailed);
        return;
    }
    mcpwm_initialized_ = true;

    // Dead-time (optional).
    if (hw.use_deadtime)
    {
        if (mcpwm_deadtime_enable(mcpwm_unit_, mcpwm_timer_, hw.deadtime_type,
                                  hw.deadtime_red_ns, hw.deadtime_fed_ns) != ESP_OK)
        {
            failSetup(MotorSetupError::HardwareInitFailed);
            return;
        }
        deadtime_enabled_ = true;
    }

    // Duty mode set once.
    if (mcpwm_set_duty_type(mcpwm_unit_, mcpwm_timer_, MCPWM_OPR_A, MCPWM_DUTY_MODE_0) != ESP_OK ||
        mcpwm_set_duty_type(mcpwm_unit_, mcpwm_timer_, MCPWM_OPR_B, MCPWM_DUTY_MODE_0) != ESP_OK)
    {
        failSetup(MotorSetupError::HardwareInitFailed);
        return;
    }

    // Soft-brake scheduler.
    if (!soft_timer_)
    {
        esp_timer_create_args_t args{};
        args.callback = [](void *p)
        { static_cast<HBridgeMotor *>(p)->softBrakeTimerTask(); };
        args.arg = this;
        args.dispatch_method = ESP_TIMER_TASK;
        args.name = "soft_brake";
        if (esp_timer_create(&args, &soft_timer_) != ESP_OK)
        {
            failSetup(MotorSetupError::TimerInitFailed);
            return;
        }
    }

    // Safety (software fallback).
    if (safety_.fault_gpio >= 0)
    {
        pinMode(safety_.fault_gpio, safety_.fault_active_high ? INPUT_PULLDOWN : INPUT_PULLUP);
        attachInterruptArg(safety_.fault_gpio, &HBridgeMotor::faultISRThunk, this, CHANGE);
        fault_irq_pin_ = safety_.fault_gpio;
        if (faultInputActive())
        {
            fault_latched_ = true;
            fault_pending_ = true;
        }
    }

    // Capture (software fallback).
    if (cap_.cap_gpio >= 0)
    {
        pinMode(cap_.cap_gpio, INPUT);
        const int mode = (cap_.edge == CaptureEdge::Rising)
                             ? RISING
                         : (cap_.edge == CaptureEdge::Falling) ? FALLING
                                                               : CHANGE;
        attachInterruptArg(cap_.cap_gpio, &HBridgeMotor::capISRThunk, this, mode);
        cap_irq_pin_ = cap_.cap_gpio;
    }

    setup_done_ = true;
    setup_error_ = MotorSetupError::None;

    if (fault_latched_)
    {
        applyFaultAction();
    }
    else
    {
        // Enabled 0/0 can brake some bridges, so setup must finish through the configured freewheel path.
        setFreewheel();
    }
}

// Set speed and direction.
void HBridgeMotor::setSpeed(int speed, Dir dir) noexcept
{
    if (!setup_done_ || fault_latched_)
        return;

    const uint16_t v = static_cast<uint16_t>(clamp<int>(speed, 0, input_max_));
    if (v == 0)
    {
        startSoftBrake();
        return;
    }

    stopSoftBrake();

    const float duty = static_cast<float>(v) * percent_per_count_;
    if (dir == Dir::CW)
        commandOutput(true, duty, 0.0f);
    else
        commandOutput(true, 0.0f, duty);
}

// Set speed and direction (in percent).
void HBridgeMotor::setSpeedPercent(float percent, Dir dir) noexcept
{
    if (!std::isfinite(percent) || percent < 0.0f)
        percent = 0.0f;
    if (percent > 100.0f)
        percent = 100.0f;

    const int maxIn = getMaxPwmInput(); // e.g., 1023
    const int speed = static_cast<int>(percent * (maxIn / 100.0f) + 0.5f);
    setSpeed(speed, dir);
}

// Enter freewheel (coast) according to current FreewheelMode.
void HBridgeMotor::setFreewheel() noexcept
{
    if (!setup_done_ || fault_latched_)
        return;

    stopSoftBrake();
    switch (beh_.freewheel_mode)
    {
    case FreewheelMode::HiZ:
        commandOutput(false, 0.0f, 0.0f);
        break;
    case FreewheelMode::HiZ_Awake:
        commandOutput(true, 0.0f, 0.0f);
        break;
    case FreewheelMode::DitherBrake:
        setSoftBrakePWM(beh_.dither_pwm);
        startSoftBrake();
        break;
    }
}

// Apply a hard electronic brake (A=100%, B=100%).
void HBridgeMotor::setHardBrake() noexcept
{
    if (!setup_done_ || fault_latched_)
        return;

    stopSoftBrake();
    commandOutput(true, 100.0f, 100.0f);
}

// Set the soft-brake PWM level (0..getMaxPwmInput()).
void HBridgeMotor::setSoftBrakePWM(uint16_t pwm) noexcept
{
    const auto clamped = static_cast<uint16_t>(clamp<int>(pwm, 0, input_max_));
    lockSoft();
    if (clamped == soft_brake_pwm_)
    {
        unlockSoft();
        return; ///< No-op if unchanged.
    }
    soft_brake_pwm_ = clamped;
    const bool was_active = soft_active_;
    unlockSoft();

    if (was_active)
        startSoftBrake();
}

// Convenience to immediately start soft-brake at the given level.
void HBridgeMotor::softBrakeNow(uint16_t pwm) noexcept
{
    if (!setup_done_ || fault_latched_)
        return;

    setSoftBrakePWM(pwm);
    startSoftBrake();
}

// Process deferred fault actions and notify callback.
void HBridgeMotor::pollFaults() noexcept
{
    if (!setup_done_ || !fault_pending_)
        return;
    fault_pending_ = false;
    if (fault_latched_)
    {
        applyFaultAction();
    }
    else
    {
        clearFault(); ///< Back to safe idle.
    }
    if (fault_cb_)
        fault_cb_(fault_latched_, fault_ctx_);
}

// Start the MCPWM outputs (0% duty).
void HBridgeMotor::start() noexcept
{
    if (!setup_done_ || fault_latched_)
        return;

    stopSoftBrake();
    commandOutput(false, 0.0f, 0.0f);
    if (mcpwm_start(mcpwm_unit_, mcpwm_timer_) == ESP_OK)
        setFreewheel();
}

// Stop the MCPWM outputs.
void HBridgeMotor::stop() noexcept
{
    if (!setup_done_ || fault_latched_)
        return;

    stopSoftBrake();
    commandOutput(false, 0.0f, 0.0f);
    (void)mcpwm_stop(mcpwm_unit_, mcpwm_timer_);
}

// Attempt to change the PWM frequency at runtime.
bool HBridgeMotor::reconfigureFrequency(int new_hz) noexcept
{
    if (!setup_done_ || new_hz < kPwmHzMin || new_hz > kPwmHzMax)
        return false;

    lockSoft();
    const bool restart_dither = soft_active_;
    unlockSoft();

    if (!fault_latched_)
    {
        stopSoftBrake();
        // Frequency changes begin from a disabled zero-output state.
        commandOutput(false, 0.0f, 0.0f);
    }

    const esp_err_t err = mcpwm_set_frequency(mcpwm_unit_, mcpwm_timer_, new_hz); // Legacy API path.
    if (err == ESP_OK)
    {
        pwm_freq_hz_ = new_hz;
        if (restart_dither && !fault_latched_)
            startSoftBrake();
        return true;
    }
    return false;
}

// Clear an inactive latched fault and return to zero-output idle.
void HBridgeMotor::clearFault() noexcept
{
    if (!setup_done_)
        return;

    fault_latched_ = false;
    fault_pending_ = false;

    if (safety_.fault_gpio >= 0 && faultInputActive())
    {
        fault_latched_ = true;
        applyFaultAction();
        return;
    }

    stopSoftBrake();
    commandOutput(false, 0.0f, 0.0f);
    (void)mcpwm_start(mcpwm_unit_, mcpwm_timer_);

    // Preserve an assertion posted while the idle state was being applied.
    if (fault_latched_)
        applyFaultAction();
}

// Force raw outputs (100% on the requested sides).
void HBridgeMotor::forceOutputs(bool a_high, bool b_high) noexcept
{
    if (!setup_done_ || fault_latched_)
        return;

    stopSoftBrake();
    commandOutput(true, a_high ? 100.0f : 0.0f, b_high ? 100.0f : 0.0f);
}

// esp_timer task callback → toggle phase and reschedule.
void HBridgeMotor::softBrakeTimerTask() noexcept
{
    lockSoft();
    if (!setup_done_ || !soft_active_ || fault_latched_)
    {
        soft_active_ = false;
        unlockSoft();
        return;
    }
    soft_phase_ = (soft_phase_ == BrakePhase::Coast) ? BrakePhase::Brake : BrakePhase::Coast;
    const BrakePhase p = soft_phase_;
    const uint32_t sequence = soft_sequence_;
    unlockSoft();

    if (applyDitherPhase(p, sequence))
        (void)scheduleNextPhase(sequence);
}

// Apply current soft-brake phase.
void HBridgeMotor::applyPhase(BrakePhase phase) noexcept
{
    if (!setup_done_ || fault_latched_)
        return;

    switch (phase)
    {
    case BrakePhase::Brake:
        commandOutput(true, 100.0f, 100.0f);
        break;
    case BrakePhase::Coast:
        // A zero dither level is ordinary coast, not an enabled zero-PWM brake state.
        commandOutput(!dither_coast_hi_z_, 0.0f, 0.0f);
        break;
    }
}

// Apply a dither phase only while its sequence is still current.
bool HBridgeMotor::applyDitherPhase(BrakePhase phase, uint32_t sequence) noexcept
{
    const bool enable = (phase == BrakePhase::Brake) || !dither_coast_hi_z_;
    const float duty = (phase == BrakePhase::Brake) ? 100.0f : 0.0f;

    lockSoft();
    if (!setup_done_ || !soft_active_ || fault_latched_ || soft_sequence_ != sequence)
    {
        unlockSoft();
        return false;
    }
    commanded_enable_ = enable;
    commanded_a_percent_ = duty;
    commanded_b_percent_ = duty;
    const uint32_t output_sequence = ++output_sequence_;
    unlockSoft();

    writeOutputUntilCurrent(output_sequence, enable, duty, duty);

    lockSoft();
    const bool current = setup_done_ && soft_active_ && !fault_latched_ &&
                         soft_sequence_ == sequence;
    unlockSoft();
    return current;
}

// Schedule next dither phase tick.
bool HBridgeMotor::scheduleNextPhase(uint32_t sequence) noexcept
{
    lockSoft();
    if (!soft_active_ || fault_latched_)
    {
        soft_active_ = false;
        unlockSoft();
        return false;
    }
    if (soft_sequence_ != sequence)
    {
        unlockSoft();
        return false;
    }
    const int64_t use_us = (soft_phase_ == BrakePhase::Brake) ? soft_us_brake_ : soft_us_coast_;
    commanded_timer_active_ = true;
    commanded_timer_us_ = use_us;
    const uint32_t timer_sequence = ++timer_sequence_;
    unlockSoft();

    if (!writeTimerUntilCurrent(timer_sequence, true, use_us))
    {
        stopSoftBrake();
        applyPhase(BrakePhase::Coast);
        return false;
    }
    return true;
}

// Begin soft-brake dither.
void HBridgeMotor::startSoftBrake() noexcept
{
    if (!setup_done_ || fault_latched_)
        return;

    stopSoftBrake();
    if (!recomputeSoftDurations())
    {
        applyPhase(BrakePhase::Coast);
        return;
    }

    lockSoft();
    const bool pure_coast = (soft_us_brake_ == 0);
    const bool pure_brake = (soft_us_coast_ == 0);

    if (pure_coast || pure_brake)
    {
        soft_active_ = false;
        ++soft_sequence_;
        unlockSoft();

        applyPhase(pure_coast ? BrakePhase::Coast : BrakePhase::Brake);
        return;
    }

    soft_phase_ = BrakePhase::Coast;
    soft_active_ = true;
    const uint32_t sequence = ++soft_sequence_;
    unlockSoft();

    if (applyDitherPhase(BrakePhase::Coast, sequence))
        (void)scheduleNextPhase(sequence);
}

// Stop soft-brake dither.
void HBridgeMotor::stopSoftBrake() noexcept
{
    lockSoft();
    soft_active_ = false;
    ++soft_sequence_;
    commanded_timer_active_ = false;
    commanded_timer_us_ = 0;
    const uint32_t timer_sequence = ++timer_sequence_;
    unlockSoft();

    (void)writeTimerUntilCurrent(timer_sequence, false, 0);
}

// Control optional EN pin.
void HBridgeMotor::setEnable(bool enabled) noexcept
{
    if (use_en_ && en_state_ != enabled)
    {
        digitalWrite(en_pin_, enabled ? HIGH : LOW);
        en_state_ = enabled;
    }
}

// Write MCPWM A/B duties (0..100).
void HBridgeMotor::writeAB(float a_percent, float b_percent) noexcept
{
    a_percent = std::isfinite(a_percent) ? clamp(a_percent, 0.0f, 100.0f) : 0.0f;
    b_percent = std::isfinite(b_percent) ? clamp(b_percent, 0.0f, 100.0f) : 0.0f;

    const bool sameA = (std::fabs(a_percent - last_a_percent_) <= kDutyEps);
    const bool sameB = (std::fabs(b_percent - last_b_percent_) <= kDutyEps);

    if (!sameA)
    {
        if (mcpwm_set_duty(mcpwm_unit_, mcpwm_timer_, MCPWM_OPR_A, a_percent) == ESP_OK)
            last_a_percent_ = a_percent;
    }
    if (!sameB)
    {
        if (mcpwm_set_duty(mcpwm_unit_, mcpwm_timer_, MCPWM_OPR_B, b_percent) == ESP_OK)
            last_b_percent_ = b_percent;
    }
}

// Apply one output snapshot in an order that avoids an enabled transition window.
void HBridgeMotor::writeHardwareOutput(bool enable, float a_percent, float b_percent) noexcept
{
    const bool both_channels_change =
        std::fabs(a_percent - last_a_percent_) > kDutyEps &&
        std::fabs(b_percent - last_b_percent_) > kDutyEps;
    if (!enable || (use_en_ && en_state_ && both_channels_change))
    {
        // Avoid a one-sided drive pulse while two MCPWM channels are updated in sequence.
        setEnable(false);
    }
    writeAB(a_percent, b_percent);
    if (enable)
        setEnable(true);
}

// Keep applying the newest published output if a callback loses a command race.
void HBridgeMotor::writeOutputUntilCurrent(uint32_t sequence, bool enable,
                                           float a_percent, float b_percent) noexcept
{
    for (;;)
    {
        writeHardwareOutput(enable, a_percent, b_percent);

        lockSoft();
        if (sequence == output_sequence_)
        {
            unlockSoft();
            return;
        }
        sequence = output_sequence_;
        enable = commanded_enable_;
        a_percent = commanded_a_percent_;
        b_percent = commanded_b_percent_;
        unlockSoft();
    }
}

// Publish a normal task-context output command before touching hardware.
void HBridgeMotor::commandOutput(bool enable, float a_percent, float b_percent) noexcept
{
    lockSoft();
    commanded_enable_ = enable;
    commanded_a_percent_ = a_percent;
    commanded_b_percent_ = b_percent;
    const uint32_t sequence = ++output_sequence_;
    unlockSoft();

    writeOutputUntilCurrent(sequence, enable, a_percent, b_percent);
}

// Keep the shared one-shot timer aligned with the latest start/stop command.
bool HBridgeMotor::writeTimerUntilCurrent(uint32_t sequence, bool active,
                                          int64_t timeout_us) noexcept
{
    for (;;)
    {
        if (soft_timer_)
            (void)esp_timer_stop(soft_timer_);

        bool start_succeeded = true;
        if (active)
        {
            start_succeeded = soft_timer_ && timeout_us > 0 &&
                              esp_timer_start_once(soft_timer_, timeout_us) == ESP_OK;
        }

        lockSoft();
        if (sequence == timer_sequence_)
        {
            if (!start_succeeded)
            {
                commanded_timer_active_ = false;
                commanded_timer_us_ = 0;
                ++timer_sequence_;
                soft_active_ = false;
                ++soft_sequence_;
            }
            unlockSoft();
            return start_succeeded;
        }
        sequence = timer_sequence_;
        active = commanded_timer_active_;
        timeout_us = commanded_timer_us_;
        unlockSoft();
    }
}

// Recompute dither phase durations.
bool HBridgeMotor::recomputeSoftDurations() noexcept
{
    if (input_max_ <= 0 || soft_hz_ <= 0)
        return false;

    const int64_t period_us = static_cast<int64_t>(kMicrosPerSec / static_cast<uint32_t>(soft_hz_));
    if (period_us < 2)
        return false;

    const uint32_t pwm = clamp<uint32_t>(soft_brake_pwm_, 0U, static_cast<uint32_t>(input_max_));
    int64_t br = 0;
    if (pwm >= static_cast<uint32_t>(input_max_))
    {
        br = period_us;
    }
    else if (pwm > 0)
    {
        br = static_cast<int64_t>((static_cast<uint64_t>(period_us) * pwm +
                                   static_cast<uint32_t>(input_max_ / 2)) /
                                  static_cast<uint32_t>(input_max_));
        const int64_t min_us = clamp<int64_t>(static_cast<int64_t>(min_phase_us_),
                                              1, period_us / 2);
        br = clamp<int64_t>(br, min_us, period_us - min_us);
    }
    const int64_t co = period_us - br;

    lockSoft();
    soft_us_brake_ = br;
    soft_us_coast_ = co;
    unlockSoft();
    return true;
}

// Static thunk to instance ISR.
void IRAM_ATTR HBridgeMotor::faultISRThunk(void *arg) { static_cast<HBridgeMotor *>(arg)->faultISR(); }

// Fault ISR.
void IRAM_ATTR HBridgeMotor::faultISR() noexcept
{
    const bool active = faultInputActive();

    if (safety_.oneshot)
    {
        if (active && !fault_latched_)
        {
            fault_latched_ = true;
            fault_pending_ = true; // defer handling to task context
        }
    }
    else
    {
        // Level-following mode.
        fault_latched_ = active;
        fault_pending_ = true;
    }
}

// Read the configured fault level.
bool IRAM_ATTR HBridgeMotor::faultInputActive() const noexcept
{
    const int level = gpio_get_level(static_cast<gpio_num_t>(safety_.fault_gpio));
    return safety_.fault_active_high ? (level != 0) : (level == 0);
}

// Apply the configured low-level fault action.
void HBridgeMotor::applyFaultAction() noexcept
{
    stopSoftBrake();

    switch (safety_.fault_action)
    {
    case FaultAction::Coast:
        commandOutput(false, 0.0f, 0.0f);
        break;
    case FaultAction::DisableOutputs:
        commandOutput(false, 0.0f, 0.0f);
        (void)mcpwm_stop(mcpwm_unit_, mcpwm_timer_);
        break;
    case FaultAction::HardBrake:
        emergencyBrake();
        break;
    }
}

// Apply the full electronic brake.
void HBridgeMotor::emergencyBrake() noexcept
{
    commandOutput(true, 100.0f, 100.0f);
    // Keep MCPWM running so the bridge continues to hold the electronic brake.
    (void)mcpwm_start(mcpwm_unit_, mcpwm_timer_);
}

// Detach prior fault interrupt, if any.
void HBridgeMotor::detachFaultInterrupt() noexcept
{
    if (fault_irq_pin_ >= 0)
    {
        detachInterrupt(fault_irq_pin_);
        fault_irq_pin_ = -1;
    }
}

// Static thunk to instance ISR.
void IRAM_ATTR HBridgeMotor::capISRThunk(void *arg) { static_cast<HBridgeMotor *>(arg)->capISR(); }

// Capture ISR.
void IRAM_ATTR HBridgeMotor::capISR() noexcept
{
    const uint32_t now = micros();
    const uint32_t last = last_edge_us_;
    const bool had_edge = capture_edge_seen_;
    last_edge_us_ = now;
    capture_edge_seen_ = true;

    if (had_edge)
    {
        // Wrap-safe delta (uint32_t micros()).
        period_us_ = (now >= last) ? (now - last) : (now + (0xFFFFFFFFu - last) + 1u);
        if (cap_.on_capture)
        {
            cap_.on_capture(period_us_, cap_.user);
        }
    }
}

// Detach prior capture interrupt, if any.
void HBridgeMotor::detachCaptureInterrupt() noexcept
{
    if (cap_irq_pin_ >= 0)
    {
        detachInterrupt(cap_irq_pin_);
        cap_irq_pin_ = -1;
    }
}

// Make repeated setup() calls safe.
void HBridgeMotor::prepareForSetup() noexcept
{
    detachFaultInterrupt();
    detachCaptureInterrupt();
    stopSoftBrake();

    if (setup_done_)
    {
        commandOutput(false, 0.0f, 0.0f);
        if (deadtime_enabled_)
            (void)mcpwm_deadtime_disable(mcpwm_unit_, mcpwm_timer_);
        (void)mcpwm_stop(mcpwm_unit_, mcpwm_timer_);
    }

    fault_latched_ = false;
    fault_pending_ = false;
    last_edge_us_ = 0;
    period_us_ = 0;
    capture_edge_seen_ = false;
    last_a_percent_ = -1.0f;
    last_b_percent_ = -1.0f;
    setup_done_ = false;
    mcpwm_initialized_ = false;
    deadtime_enabled_ = false;
    use_en_ = false;
    en_state_ = false;
    setup_error_ = MotorSetupError::None;
}

// Check user configuration before MCPWM or interrupt setup begins.
MotorSetupError HBridgeMotor::validateConfig(const MotorMCPWMConfig &hw,
                                             const MotorBehaviorConfig &beh,
                                             const MotorSafetyConfig &safety,
                                             const MotorCaptureConfig &cap) const noexcept
{
    if (!GPIO_IS_VALID_OUTPUT_GPIO(hw.lpwm_pin) || !GPIO_IS_VALID_OUTPUT_GPIO(hw.rpwm_pin))
        return MotorSetupError::InvalidPwmPin;
    if (hw.lpwm_pin == hw.rpwm_pin)
        return MotorSetupError::DuplicatePwmPin;
    if (hw.en_pin >= 0 && (!GPIO_IS_VALID_OUTPUT_GPIO(hw.en_pin) ||
                           hw.en_pin == hw.lpwm_pin || hw.en_pin == hw.rpwm_pin))
        return MotorSetupError::PinConflict;
    if (hw.sig_l == hw.sig_r)
        return MotorSetupError::PinConflict;
    if (hw.pwm_freq_hz < kPwmHzMin || hw.pwm_freq_hz > kPwmHzMax)
        return MotorSetupError::InvalidPwmFrequency;
    if (hw.input_max <= 0 || hw.input_max > UINT16_MAX)
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
        if (!GPIO_IS_VALID_GPIO(cap.cap_gpio) || cap.cap_gpio == hw.lpwm_pin ||
            cap.cap_gpio == hw.rpwm_pin || cap.cap_gpio == hw.en_pin ||
            cap.cap_gpio == safety.fault_gpio)
            return MotorSetupError::PinConflict;
    }

    return MotorSetupError::None;
}

// Leave a failed setup inactive without aborting the application.
void HBridgeMotor::failSetup(MotorSetupError error) noexcept
{
    detachFaultInterrupt();
    detachCaptureInterrupt();
    stopSoftBrake();
    if (use_en_)
        setEnable(false);
    if (mcpwm_initialized_)
    {
        commandOutput(false, 0.0f, 0.0f);
        if (deadtime_enabled_)
            (void)mcpwm_deadtime_disable(mcpwm_unit_, mcpwm_timer_);
        (void)mcpwm_stop(mcpwm_unit_, mcpwm_timer_);
    }
    setup_done_ = false;
    mcpwm_initialized_ = false;
    deadtime_enabled_ = false;
    use_en_ = false;
    setup_error_ = error;
}

// Set the freewheel mode for subsequent setFreewheel() calls.
void HBridgeMotor::setFreewheelMode(FreewheelMode m) noexcept
{
    lockSoft();
    if (beh_.freewheel_mode == m)
    {
        unlockSoft();
        return;
    }
    beh_.freewheel_mode = m;
    const bool was_active = soft_active_;
    unlockSoft();

    if (was_active)
    {
        stopSoftBrake();
        applyPhase(BrakePhase::Coast);
    }
}

// Set the freewheel mode and immediately apply freewheel.
void HBridgeMotor::applyFreewheel(FreewheelMode m) noexcept
{
    setFreewheelMode(m);
    setFreewheel();
}
