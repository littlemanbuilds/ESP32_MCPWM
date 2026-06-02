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
        // Without EN we prefer to keep the driver awake to ensure true Hi-Z is stable.
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
    soft_hz_ = boundSoftHz(beh_.soft_brake_hz);
    pwm_freq_hz_ = (hw.pwm_freq_hz > 0) ? hw.pwm_freq_hz : 20000;
    input_max_ = (hw.input_max > 0) ? hw.input_max : 1023;
    counter_mode_ = hw.counter;
    percent_per_count_ = 100.0f / static_cast<float>(input_max_);
    soft_brake_pwm_ = static_cast<uint16_t>(clamp<int>(beh_.default_soft_brake_pwm, 0, input_max_));

    // EN pin usage.
    use_en_ = (en_pin_ >= 0);

    // GPIO routing.
    ESP_ERROR_CHECK(mcpwm_gpio_init(mcpwm_unit_, mcpwm_sig_l_, lpwm_pin_));
    ESP_ERROR_CHECK(mcpwm_gpio_init(mcpwm_unit_, mcpwm_sig_r_, rpwm_pin_));

    // Timer/channel setup.
    mcpwm_config_t cfg{};
    cfg.frequency = pwm_freq_hz_;
    cfg.cmpr_a = 0;
    cfg.cmpr_b = 0;
    cfg.counter_mode = counter_mode_;
    cfg.duty_mode = MCPWM_DUTY_MODE_0;
    ESP_ERROR_CHECK(mcpwm_init(mcpwm_unit_, mcpwm_timer_, &cfg));

    // Dead-time (optional).
    if (hw.use_deadtime)
    {
        ESP_ERROR_CHECK(mcpwm_deadtime_enable(
            mcpwm_unit_, mcpwm_timer_, hw.deadtime_type, hw.deadtime_red_ns, hw.deadtime_fed_ns));
    }

    // Duty mode set once.
    ESP_ERROR_CHECK(mcpwm_set_duty_type(mcpwm_unit_, mcpwm_timer_, MCPWM_OPR_A, MCPWM_DUTY_MODE_0));
    ESP_ERROR_CHECK(mcpwm_set_duty_type(mcpwm_unit_, mcpwm_timer_, MCPWM_OPR_B, MCPWM_DUTY_MODE_0));

    // EN pin state.
    if (use_en_)
    {
        pinMode(en_pin_, OUTPUT);
        setEnable(true);
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
        ESP_ERROR_CHECK(esp_timer_create(&args, &soft_timer_));
    }

    // Safety (software fallback).
    if (safety_.fault_gpio >= 0)
    {
        pinMode(safety_.fault_gpio, safety_.fault_active_high ? INPUT_PULLDOWN : INPUT_PULLUP);
        attachInterruptArg(safety_.fault_gpio, &HBridgeMotor::faultISRThunk, this, CHANGE);
        fault_irq_pin_ = safety_.fault_gpio;
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
}

// Set speed and direction.
void HBridgeMotor::setSpeed(int speed, Dir dir) noexcept
{
    const uint16_t v = static_cast<uint16_t>(clamp<int>(speed, 0, input_max_));
    if (v == 0)
    {
        startSoftBrake();
        return;
    }

    stopSoftBrake();
    setEnable(true);

    const float duty = static_cast<float>(v) * percent_per_count_;
    if (dir == Dir::CW)
        writeAB(duty, 0.0f);
    else
        writeAB(0.0f, duty);
}

// Set speed and direction (in percent).
void HBridgeMotor::setSpeedPercent(float percent, Dir dir) noexcept
{
    if (percent < 0.0f)
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
    stopSoftBrake();
    switch (beh_.freewheel_mode)
    {
    case FreewheelMode::HiZ:
        setEnable(false);
        writeAB(0.0f, 0.0f);
        break;
    case FreewheelMode::HiZ_Awake:
        setEnable(true);
        writeAB(0.0f, 0.0f);
        break;
    case FreewheelMode::DitherBrake:
        setSoftBrakePWM(beh_.dither_pwm);
        if (soft_brake_pwm_ == 0)
        {
            setEnable(true);
            writeAB(0.0f, 0.0f);
        }
        else
        {
            startSoftBrake();
        }
        break;
    }
}

// Apply a hard electronic brake (A=100%, B=100%).
void HBridgeMotor::setHardBrake() noexcept
{
    stopSoftBrake();
    setEnable(true);
    writeAB(100.0f, 100.0f);
}

// Set the soft-brake PWM level (0..getMaxPwmInput()).
void HBridgeMotor::setSoftBrakePWM(uint16_t pwm) noexcept
{
    const auto clamped = static_cast<uint16_t>(clamp<int>(pwm, 0, input_max_));
    if (clamped == soft_brake_pwm_)
        return; ///< No-op if unchanged.
    soft_brake_pwm_ = clamped;
    if (soft_active_)
        recomputeSoftDurations();
}

// Convenience to immediately start soft-brake at the given level.
void HBridgeMotor::softBrakeNow(uint16_t pwm) noexcept
{
    setSoftBrakePWM(pwm);
    startSoftBrake();
}

// Process deferred fault actions and notify callback.
void HBridgeMotor::pollFaults() noexcept
{
    if (!fault_pending_)
        return;
    fault_pending_ = false;
    if (fault_latched_)
    {
        emergencyBrake(); ///< Hold electronic brake.
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
    writeAB(0.0f, 0.0f);
    ESP_ERROR_CHECK(mcpwm_start(mcpwm_unit_, mcpwm_timer_));
}

// Stop the MCPWM outputs.
void HBridgeMotor::stop() noexcept
{
    writeAB(0.0f, 0.0f);
    ESP_ERROR_CHECK(mcpwm_stop(mcpwm_unit_, mcpwm_timer_));
}

// Attempt to change the PWM frequency at runtime.
bool HBridgeMotor::reconfigureFrequency(int new_hz) noexcept
{
    if (new_hz <= 0)
        return false;

    const esp_err_t err = mcpwm_set_frequency(mcpwm_unit_, mcpwm_timer_, new_hz); // Legacy API path.
    if (err == ESP_OK)
    {
        pwm_freq_hz_ = new_hz;
        return true;
    }
    return false;
}

// Clear a latched fault and return to a safe idle state.
void HBridgeMotor::clearFault() noexcept
{
    fault_latched_ = false;
    stopSoftBrake();
    setEnable(true);
    writeAB(0.0f, 0.0f);
}

// Force raw outputs (100% on the requested sides).
void HBridgeMotor::forceOutputs(bool a_high, bool b_high) noexcept
{
    setEnable(true);
    writeAB(a_high ? 100.0f : 0.0f, b_high ? 100.0f : 0.0f);
}

// esp_timer task callback → toggle phase and reschedule.
void HBridgeMotor::softBrakeTimerTask() noexcept
{
    lockSoft();
    if (!soft_active_)
    {
        unlockSoft();
        return;
    }
    soft_phase_ = (soft_phase_ == BrakePhase::Coast) ? BrakePhase::Brake : BrakePhase::Coast;
    const BrakePhase p = soft_phase_;
    unlockSoft();

    applyPhase(p);
    scheduleNextPhase();
}

// Apply current soft-brake phase.
void HBridgeMotor::applyPhase(BrakePhase p) noexcept
{
    switch (p)
    {
    case BrakePhase::Brake:
        setEnable(true);
        writeAB(100.0f, 100.0f);
        break;
    case BrakePhase::Coast:
        // First ensure duties are zeroed, then optionally Hi-Z the bridge.
        writeAB(0.0f, 0.0f);
        if (dither_coast_hi_z_)
        {
            setEnable(false); ///< True coast on modules where 0/0 is also brake (IBT-2/BTS7960).
        }
        break;
    }
}

// Schedule next dither phase tick.
void HBridgeMotor::scheduleNextPhase() noexcept
{
    lockSoft();
    if (!soft_active_)
    {
        unlockSoft();
        return;
    }
    const int64_t use_us = (soft_phase_ == BrakePhase::Brake) ? soft_us_brake_ : soft_us_coast_;
    unlockSoft();

    (void)esp_timer_stop(soft_timer_);
    ESP_ERROR_CHECK(esp_timer_start_once(soft_timer_, use_us));
}

// Begin soft-brake dither.
void HBridgeMotor::startSoftBrake() noexcept
{
    recomputeSoftDurations();

    lockSoft();
    const float lvl = soft_level_;
    const bool pure_coast = (lvl <= 0.001f) || (soft_us_brake_ == 0);
    const bool pure_brake = (lvl >= 0.999f) || (soft_us_coast_ == 0);

    if (pure_coast || pure_brake)
    {
        soft_active_ = false;
        unlockSoft();

        setEnable(true);
        writeAB(pure_coast ? 0.0f : 100.0f, pure_coast ? 0.0f : 100.0f);
        return;
    }

    soft_phase_ = BrakePhase::Coast;
    soft_active_ = true;
    unlockSoft();

    applyPhase(BrakePhase::Coast);
    (void)esp_timer_stop(soft_timer_);
    scheduleNextPhase();
}

// Stop soft-brake dither.
void HBridgeMotor::stopSoftBrake() noexcept
{
    lockSoft();
    soft_active_ = false;
    unlockSoft();

    if (soft_timer_)
        (void)esp_timer_stop(soft_timer_);
}

// Bound soft-brake frequency.
int HBridgeMotor::boundSoftHz(int hz) noexcept
{
    return (hz < kSoftHzMin) ? kSoftHzMin : (hz > kSoftHzMax) ? kSoftHzMax
                                                              : hz;
}

// Control optional EN pin.
void HBridgeMotor::setEnable(bool on) noexcept
{
    if (use_en_ && en_state_ != on)
    {
        digitalWrite(en_pin_, on ? HIGH : LOW);
        en_state_ = on;
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
        mcpwm_set_duty(mcpwm_unit_, mcpwm_timer_, MCPWM_OPR_A, a_percent);
        last_a_percent_ = a_percent;
    }
    if (!sameB)
    {
        mcpwm_set_duty(mcpwm_unit_, mcpwm_timer_, MCPWM_OPR_B, b_percent);
        last_b_percent_ = b_percent;
    }
}

// Recompute dither phase durations.
void HBridgeMotor::recomputeSoftDurations() noexcept
{
    const float lvl = clamp<float>(
        static_cast<float>(soft_brake_pwm_) / static_cast<float>(input_max_), 0.0f, 1.0f);
    const int hz = (soft_hz_ <= 0) ? kSoftHzMin : soft_hz_; ///< Last-ditch guard.

    // Period in microseconds (integer for esp_timer).
    const double period_us_f = kMicrosPerSec / static_cast<double>(hz);
    const int64_t period_us = static_cast<int64_t>(period_us_f + 0.5);

    // Compute requested phase split (before enforcing minimums).
    int64_t br = static_cast<int64_t>(period_us_f * static_cast<double>(lvl) + 0.5); ///< Brake phase.
    int64_t co = period_us - br;                                                     ///< Coast phase.

    // Cap the per-phase minimum to half the period so two minimums always fit.
    const int64_t half_period = period_us / 2;
    const int64_t min_us_cap = (min_phase_us_ > half_period) ? half_period : static_cast<int64_t>(min_phase_us_);

    // Enforce minimums when a phase is nonzero (keeps duty meaningful at high Hz).
    if (br > 0 && br < min_us_cap)
        br = min_us_cap;
    if (co > 0 && co < min_us_cap)
        co = min_us_cap;

    lockSoft();
    soft_level_ = lvl;
    soft_us_brake_ = br;
    soft_us_coast_ = co;
    unlockSoft();
}

// Static thunk to instance ISR.
void IRAM_ATTR HBridgeMotor::faultISRThunk(void *arg) { static_cast<HBridgeMotor *>(arg)->faultISR(); }

// Fault ISR.
void IRAM_ATTR HBridgeMotor::faultISR() noexcept
{
    const int level = gpio_get_level(static_cast<gpio_num_t>(safety_.fault_gpio));
    const bool active = safety_.fault_active_high ? (level != 0) : (level == 0);

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

// Immediate full electronic brake.
void HBridgeMotor::emergencyBrake() noexcept
{
    stopSoftBrake();
    setEnable(true);
    writeAB(100.0f, 100.0f);
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
    last_edge_us_ = now;

    if (last != 0)
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
        setEnable(false);
        writeAB(0.0f, 0.0f);
        (void)mcpwm_stop(mcpwm_unit_, mcpwm_timer_);
    }

    fault_latched_ = false;
    fault_pending_ = false;
    last_edge_us_ = 0;
    period_us_ = 0;
    last_a_percent_ = -1.0f;
    last_b_percent_ = -1.0f;
    setup_done_ = false;
}

// Set the freewheel mode for subsequent setFreewheel() calls.
void HBridgeMotor::setFreewheelMode(FreewheelMode m) noexcept
{
    lockSoft();
    const bool change = (beh_.freewheel_mode != m);
    beh_.freewheel_mode = m;
    const bool was_active = soft_active_;
    soft_active_ = false;
    unlockSoft();

    if (change && was_active && soft_timer_)
        (void)esp_timer_stop(soft_timer_);
}

// Set the freewheel mode and immediately apply freewheel.
void HBridgeMotor::applyFreewheel(FreewheelMode m) noexcept
{
    setFreewheelMode(m);
    setFreewheel();
}
