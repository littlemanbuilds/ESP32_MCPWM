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
    int pwm_freq_hz = 20000;                         ///< PWM frequency (Hz).
    int input_max = 1023;                            ///< Max logical input (e.g., 1023).
    mcpwm_counter_type_t counter = MCPWM_UP_COUNTER; ///< MCPWM counter mode.

    // Dead-time (optional). Safe defaults: off.
    bool use_deadtime = false;                                               ///< Enable MCPWM dead-time insertion.
    mcpwm_deadtime_type_t deadtime_type = MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE; ///< Dead-time mode.
    uint32_t deadtime_red_ns = 500;                                          ///< Rising-edge delay (ns).
    uint32_t deadtime_fed_ns = 500;                                          ///< Falling-edge delay (ns).

    /// @brief Convenience constructor; keeps legacy brace-init working.
    MotorMCPWMConfig() = default;

    /// @brief Convenience constructor with optional frequency and input max.
    MotorMCPWMConfig(int lp, int rp, int en,
                     mcpwm_unit_t u, mcpwm_timer_t t,
                     mcpwm_io_signals_t sl, mcpwm_io_signals_t sr,
                     int freq_hz = 20000, int in_max = 1023)
        : lpwm_pin(lp), rpwm_pin(rp), en_pin(en),
          unit(u), timer(t), sig_l(sl), sig_r(sr),
          pwm_freq_hz(freq_hz), input_max(in_max) {}
};

// ---- Optional Modules (software fallbacks) ----

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
};

/**
 * @brief Edge selection for capture.
 */
enum class CaptureEdge : uint8_t
{
    Rising,  ///< Capture on rising edges.
    Falling, ///< Capture on falling edges.
    Both     ///< Capture on both edges.
};

/// @brief User callback for period measurements (microseconds).
using CaptureCallback = void (*)(uint32_t period_us, void *user);

/**
 * @brief Optional capture configuration (period measurement).
 *
 * If @p cap_gpio >= 0, an ISR measures period via micros() on the selected edge.
 */
struct MotorCaptureConfig
{
    int cap_gpio = -1;                      ///< Capture input pin; -1 to disable.
    CaptureEdge edge = CaptureEdge::Rising; ///< Capture edge selection.
    CaptureCallback on_capture = nullptr;   ///< Optional period callback.
    void *user = nullptr;                   ///< Opaque user pointer passed to callback.
};

// ---- Behavior Config ----

/**
 * @brief How the driver behaves when setFreewheel() is called.
 */
enum class FreewheelMode : uint8_t
{
    HiZ,        ///< Coast with outputs Hi-Z; driver may sleep.
    HiZ_Awake,  ///< Coast with outputs Hi-Z; driver stays awake.
    DitherBrake ///< Pulsed brake/coast for light drag.
};

/**
 * @brief Per-instance behavior (tunable at setup).
 */
struct MotorBehaviorConfig
{
    FreewheelMode freewheel_mode = FreewheelMode::HiZ; ///< Default freewheel strategy.
    int soft_brake_hz = 300;                           ///< Dither frequency for soft-brake.
    uint16_t dither_pwm = 30;                          ///< Tiny duty used in DitherBrake mode.
    uint16_t default_soft_brake_pwm = 50;              ///< Soft-brake duty when speed goes to zero.
    uint16_t min_phase_us = 1500;                      ///< Minimum brake/coast phase (µs). Lower for gentler dither at higher Hz.
    bool dither_coast_hi_z = false;                    ///< If true, DitherBrake "coast" uses Hi-Z (EN LOW) instead of 0/0.

    constexpr MotorBehaviorConfig() = default;

    constexpr MotorBehaviorConfig(FreewheelMode mode, int hz, uint16_t dither, uint16_t def_soft = 50,
                                  uint16_t min_phase = 1500, bool dither_hi_z = false)
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
     * @brief Start the MCPWM outputs (0% duty).
     */
    virtual void start() noexcept {}

    /**
     * @brief Stop the MCPWM outputs.
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
     * @brief Check if a fault has been latched.
     * @return true If a fault is active or latched.
     * @return false Otherwise.
     */
    virtual bool hasFault() const noexcept { return false; }

    /**
     * @brief Clear a latched fault and return to a safe idle state.
     */
    virtual void clearFault() noexcept {}

    /**
     * @brief Force raw outputs (100% on the requested sides).
     * @param a_high True for high on A side.
     * @param b_high True for high on B side.
     */
    virtual void forceOutputs(bool a_high, bool b_high) noexcept {}

    /**
     * @brief Utility to invert direction.
     * @param d Input direction.
     * @return Dir Opposite direction.
     */
    static Dir changeDir(Dir d) noexcept { return (d == Dir::CW) ? Dir::CCW : Dir::CW; }
};