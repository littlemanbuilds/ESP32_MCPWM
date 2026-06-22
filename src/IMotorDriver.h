/**
 * MIT License
 *
 * @brief Abstract base class + shared config types for ESP32 MCPWM motor drivers.
 *
 * @file IMotorDriver.h
 * @author Little Man Builds (Darren Osborne)
 * @date 2025-08-28
 * @copyright Copyright (c) 2025 Little Man Builds
 */

#pragma once

#include <cstdint>
#include <driver/mcpwm.h>

// ---- Core Hardware Config ----

/**
 * @brief Hardware configuration for an MCPWM-driven H-bridge motor.
 */
struct MotorMCPWMConfig
{
    // Pins
    int lpwm_pin = -1; ///< GPIO for LPWM (A/IN1).
    int rpwm_pin = -1; ///< GPIO for RPWM (B/IN2).
    int en_pin = -1;   ///< Enable pin; -1 if unused.

    // MCPWM routing
    mcpwm_unit_t unit = MCPWM_UNIT_0;    ///< MCPWM unit.
    mcpwm_timer_t timer = MCPWM_TIMER_0; ///< MCPWM timer.
    mcpwm_io_signals_t sig_l = MCPWM0A;  ///< MCPWM signal for LPWM.
    mcpwm_io_signals_t sig_r = MCPWM0B;  ///< MCPWM signal for RPWM.

    // Timing / scaling
    int pwm_freq_hz = 20000;                         ///< Drive PWM frequency in Hz (validated at setup).
    int input_max = 1023;                            ///< Max logical input; valid range 1..65535.
    mcpwm_counter_type_t counter = MCPWM_UP_COUNTER; ///< MCPWM counter mode.

    // Dead-time (optional). Safe defaults: off.
    bool use_deadtime = false;                                               ///< Enable MCPWM dead-time insertion.
    mcpwm_deadtime_type_t deadtime_type = MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE; ///< Dead-time mode.
    uint32_t deadtime_red_ns = 500;                                          ///< Rising-edge delay (ns).
    uint32_t deadtime_fed_ns = 500;                                          ///< Falling-edge delay (ns).

    /**
     * @brief Construct the default unassigned hardware configuration.
     */
    MotorMCPWMConfig() = default;

    /**
     * @brief Construct a hardware configuration with MCPWM routing.
     * @param lp GPIO for LPWM.
     * @param rp GPIO for RPWM.
     * @param en GPIO for EN, or -1 when unused.
     * @param u MCPWM unit.
     * @param t MCPWM timer.
     * @param sl MCPWM signal routed to LPWM.
     * @param sr MCPWM signal routed to RPWM.
     * @param freq_hz Drive PWM frequency in hertz.
     * @param in_max Maximum logical PWM input.
     */
    MotorMCPWMConfig(int lp, int rp, int en,
                     mcpwm_unit_t u, mcpwm_timer_t t,
                     mcpwm_io_signals_t sl, mcpwm_io_signals_t sr,
                     int freq_hz = 20000, int in_max = 1023)
        : lpwm_pin(lp), rpwm_pin(rp), en_pin(en),
          unit(u), timer(t), sig_l(sl), sig_r(sr),
          pwm_freq_hz(freq_hz), input_max(in_max) {}
};

// ---- Callbacks ----

/**
 * @brief User callback for selected-edge interval measurements (microseconds).
 *
 * Called from GPIO interrupt context on Arduino-ESP32; keep it IRAM-safe,
 * short, and non-blocking (no Serial, allocation, delay, or locks).
 *
 * @param interval_us Measured interval in microseconds between selected edges.
 * @param user Opaque pointer supplied during registration.
 */
using CaptureCallback = void (*)(uint32_t interval_us, void *user);

/**
 * @brief Fault event callback (level or latched).
 *
 * Called from pollFaults() in task context, after the tiny GPIO ISR posts work.
 *
 * @param active True if a fault is currently active, false if it has cleared.
 * @param ctx    Opaque pointer supplied during registration.
 */
using FaultCallback = void (*)(bool active, void *ctx);

// ---- Setup status ----

/**
 * @brief Result of the most recent setup attempt.
 */
enum class MotorSetupError : uint8_t
{
    None,                ///< Setup completed successfully.
    InvalidPwmPin,       ///< LPWM or RPWM is not a usable output pin.
    DuplicatePwmPin,     ///< LPWM and RPWM use the same pin.
    PinConflict,         ///< An optional pin conflicts with another configured pin.
    InvalidPwmFrequency, ///< Drive PWM frequency is outside the supported range.
    InvalidInputRange,   ///< Logical PWM input range is invalid.
    InvalidDitherConfig, ///< Dither timing cannot form a valid period.
    HardwareInitFailed,  ///< MCPWM setup failed.
    TimerInitFailed      ///< The soft-brake timer could not be created.
};

// ---- Optional Modules (software fallbacks) ----

/**
 * @brief Low-level bridge action applied while a fault is active or latched.
 */
enum class FaultAction : uint8_t
{
    Coast,          ///< Remove PWM drive and deassert EN when available.
    DisableOutputs, ///< Deassert EN when available and stop MCPWM output generation.
    HardBrake       ///< Dynamic brake with EN asserted and A/B at 100%.
};

/**
 * @brief Optional safety (fault) configuration.
 *
 * If @p fault_gpio >= 0, a software E-stop ISR (attachInterrupt) is installed.
 */
struct MotorSafetyConfig
{
    int fault_gpio = -1;           ///< Fault input pin; -1 to disable.
    bool fault_active_high = true; ///< Fault level sense (true = active high).
    bool oneshot = true;           ///< Latch until clearFault() if true; otherwise follow level.
    FaultAction fault_action = FaultAction::HardBrake; ///< Fault response; hard brake preserves legacy behavior.

    /**
     * @brief Construct the default safety configuration with fault input disabled.
     */
    constexpr MotorSafetyConfig() = default;

    /**
     * @brief Construct a software fault-input configuration.
     * @param gpio Fault input GPIO, or -1 to disable fault monitoring.
     * @param active_high True when a high input level asserts the fault.
     * @param one_shot True to latch the fault until clearFault() succeeds.
     * @param action Low-level bridge action to apply while faulted.
     */
    constexpr MotorSafetyConfig(int gpio, bool active_high, bool one_shot,
                                FaultAction action = FaultAction::HardBrake)
        : fault_gpio(gpio), fault_active_high(active_high), oneshot(one_shot), fault_action(action) {}
};

/**
 * @brief Edge selection for capture.
 */
enum class CaptureEdge : uint8_t
{
    Rising,  ///< Capture on rising edges.
    Falling, ///< Capture on falling edges.
    Both     ///< Capture adjacent-edge intervals; often a half-cycle for a square wave.
};

/**
 * @brief Optional capture configuration (selected-edge interval measurement).
 *
 * If @p cap_gpio >= 0, an ISR measures the interval between selected edges.
 * With @ref CaptureEdge::Both, a symmetrical square wave usually reports half
 * of its full period.
 * The optional callback also runs in that ISR context and must be IRAM-safe,
 * short, and non-blocking.
 */
struct MotorCaptureConfig
{
    int cap_gpio = -1;                      ///< Capture input pin; -1 to disable.
    CaptureEdge edge = CaptureEdge::Rising; ///< Capture edge selection.
    CaptureCallback on_capture = nullptr;   ///< Optional edge-interval callback.
    void *user = nullptr;                   ///< Opaque user pointer passed to callback.
};

// ---- Behavior Config ----

/**
 * @brief How the driver behaves when setFreewheel() is called.
 */
enum class FreewheelMode : uint8_t
{
    HiZ,        ///< Coast with EN low when library EN control is configured.
    HiZ_Awake,  ///< Enabled A/B=0 state; physical coast/brake is module-dependent.
    DitherBrake ///< Pulsed brake/coast for light drag.
};

/**
 * @brief Per-instance behavior (tunable at setup).
 */
struct MotorBehaviorConfig
{
    FreewheelMode freewheel_mode = FreewheelMode::HiZ; ///< Default freewheel strategy.
    int soft_brake_hz = 300;                           ///< Dither frequency in Hz (valid range 1..10000).
    uint16_t dither_pwm = 30;                          ///< Strength used by DitherBrake freewheel mode.
    uint16_t default_soft_brake_pwm = 50;              ///< Initial soft-brake strength used by setSpeed(0, ...).
    uint16_t min_phase_us = 50;                        ///< Practical minimum brake/coast phase (µs); reduced if two phases cannot fit.
    bool dither_coast_hi_z = false;                    ///< If true, DitherBrake "coast" uses Hi-Z (EN LOW) instead of 0/0.

    /**
     * @brief Construct the default motor behavior configuration.
     */
    constexpr MotorBehaviorConfig() = default;

    /**
     * @brief Construct a motor behavior configuration.
     * @param mode Freewheel strategy.
     * @param hz Soft-brake dither frequency in hertz.
     * @param dither Dither strength used by DitherBrake freewheel mode.
     * @param def_soft Initial soft-brake strength used by setSpeed(0, ...).
     * @param min_phase Minimum requested brake/coast phase in microseconds.
     * @param dither_hi_z True to deassert EN during dither coast phases.
     */
    constexpr MotorBehaviorConfig(FreewheelMode mode, int hz, uint16_t dither, uint16_t def_soft = 50,
                                  uint16_t min_phase = 50, bool dither_hi_z = false)
        : freewheel_mode(mode), soft_brake_hz(hz), dither_pwm(dither), default_soft_brake_pwm(def_soft),
          min_phase_us(min_phase), dither_coast_hi_z(dither_hi_z) {}
};

// ---- Abstract Interface ----

/**
 * @brief Motor direction.
 */
enum class Dir : uint8_t
{
    CW, ///< Clockwise / forward.
    CCW ///< Counter-clockwise / reverse.
};

/**
 * @brief Abstract base class for controlling drive motors.
 */
class IMotorDriver
{
public:
    /**
     * @brief Virtual destructor for safe polymorphic deletion.
     */
    virtual ~IMotorDriver() noexcept = default;

    /**
     * @brief Initialize the driver with hardware configuration.
     *
     * Implementations with setup status should leave outputs inactive on failure.
     * @param hw Hardware configuration for MCPWM and pins.
     */
    virtual void setup(const MotorMCPWMConfig &hw) = 0;

    /**
     * @brief Initialize the driver with hardware and behavior configuration.
     * @param hw Hardware configuration for MCPWM and pins.
     * @param beh Behavior configuration (freewheel, soft-brake).
     */
    virtual void setup(const MotorMCPWMConfig &hw, const MotorBehaviorConfig &beh) { setup(hw); }

    /**
     * @brief Initialize the driver with hardware, behavior, safety, and capture configs.
     * @param hw Hardware configuration for MCPWM and pins.
     * @param beh Behavior configuration (freewheel, soft-brake).
     * @param sfty Optional safety (fault) configuration.
     * @param cap Optional capture configuration.
     */
    virtual void setup(const MotorMCPWMConfig &hw, const MotorBehaviorConfig &beh,
                       const MotorSafetyConfig &sfty, const MotorCaptureConfig &cap)
    {
        (void)sfty;
        (void)cap;
        setup(hw, beh);
    }

    /**
     * @brief Check whether the most recent setup completed successfully.
     * @return true If hardware resources are ready for motor commands.
     * @return false Otherwise.
     */
    [[nodiscard]] virtual bool isSetupComplete() const noexcept { return false; }

    /**
     * @brief Get the result of the most recent setup attempt.
     * @return MotorSetupError Setup result.
     */
    [[nodiscard]] virtual MotorSetupError getLastSetupError() const noexcept
    {
        return MotorSetupError::None;
    }

    /**
     * @brief Set speed and direction.
     * @param speed Duty request in [0, getMaxPwmInput()].
     * @param dir Desired rotation direction.
     */
    virtual void setSpeed(int speed, Dir dir) noexcept = 0;

    /**
     * @brief Set motor speed as a percentage of the maximum input (0..100).
     * @param percent Speed request in percent (values outside 0..100 are clamped).
     * @param dir Desired direction (CW/CCW).
     */
    virtual void setSpeedPercent(float percent, Dir dir) noexcept = 0;

    /**
     * @brief Enter freewheel (coast) according to current FreewheelMode.
     */
    virtual void setFreewheel() noexcept = 0;

    /**
     * @brief Apply a hard electronic brake (A=100%, B=100%).
     */
    virtual void setHardBrake() noexcept = 0;

    /**
     * @brief Set the soft-brake PWM level (0..getMaxPwmInput()).
     * @param pwm Requested soft-brake level.
     */
    virtual void setSoftBrakePWM(uint16_t pwm) noexcept {}

    /**
     * @brief Process deferred safety work in normal task context.
     *
     * Call this regularly when a configured fault input requires it.
     */
    virtual void pollFaults() noexcept {}

    /**
     * @brief Get the maximum accepted logical PWM input.
     * @return int Maximum input value (e.g., 1023).
     */
    [[nodiscard]] virtual int getMaxPwmInput() const noexcept { return 255; }

    /**
     * @brief Set the freewheel mode for subsequent setFreewheel() calls.
     * @param mode Freewheel strategy to use.
     */
    virtual void setFreewheelMode(FreewheelMode mode) noexcept {}

    /**
     * @brief Set the freewheel mode and immediately apply freewheel.
     * @param mode Freewheel strategy to use.
     */
    virtual void applyFreewheel(FreewheelMode mode) noexcept
    {
        setFreewheelMode(mode);
        setFreewheel();
    }

    /**
     * @brief Start MCPWM and reapply the configured freewheel state.
     */
    virtual void start() noexcept {}

    /**
     * @brief Deassert EN, set A/B to 0%, and stop the MCPWM outputs.
     */
    virtual void stop() noexcept {}

    /**
     * @brief Attempt to change the PWM frequency at runtime.
     * @param new_hz New frequency in Hz.
     * @return true If the change was applied.
     * @return false If not supported or failed.
     */
    virtual bool reconfigureFrequency(int new_hz) noexcept { return false; }

    /**
     * @brief Get the latest measured interval between selected capture edges.
     * @return uint32_t Edge interval in microseconds, or zero before a valid measurement.
     */
    [[nodiscard]] virtual uint32_t getLastCapturePeriodUs() const noexcept { return 0; }

    /**
     * @brief Check whether a one-shot fault is latched or a followed level is active.
     * @return true If fault output inhibition is active.
     * @return false Otherwise.
     */
    virtual bool hasFault() const noexcept { return false; }

    /**
     * @brief Check whether a bridge EN pin is configured for library control.
     * @return true If the configured EN pin can be commanded by the library.
     * @return false If no EN pin is configured.
     */
    [[nodiscard]] virtual bool hasEnableControl() const noexcept { return false; }

    /**
     * @brief Clear an inactive latched fault and return to zero-output idle.
     */
    virtual void clearFault() noexcept {}

    /**
     * @brief Force raw outputs (100% on the requested sides).
     * @param a_high True for high on A side.
     * @param b_high True for high on B side.
     */
    virtual void forceOutputs(bool a_high, bool b_high) noexcept {}

    /**
     * @brief Optional fault notification callback (level or latched).
     * @param cb  Callback: (active, ctx).
     * @param ctx Opaque pointer (handed back).
     */
    virtual void setFaultCallback(FaultCallback cb, void *ctx) noexcept
    {
        (void)cb;
        (void)ctx;
    }

    /**
     * @brief Utility to invert direction.
     * @param d Input direction.
     * @return Dir Opposite direction.
     */
    static Dir changeDir(Dir d) noexcept { return (d == Dir::CW) ? Dir::CCW : Dir::CW; }
};
