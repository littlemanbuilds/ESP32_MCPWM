/**
 * MIT License
 *
 * @brief Dual-H-bridge motor driver (ESP32 MCPWM) with soft-brake, dead-time,
 *        center-aligned mode, start/stop, runtime retune, and software fallbacks
 *        for fault (E-stop) and capture (period measurement).
 *
 * @file HBridgeMotor.h
 * @author Little Man Builds (Darren Osborne)
 * @date 2025-08-28
 * @copyright Copyright (c) 2025 Little Man Builds
 */

#pragma once

#include <Arduino.h>
#include <cstdint>
#include <driver/mcpwm.h>
#include <esp_timer.h>
#include <freertos/portmacro.h>
#include <IMotorDriver.h>

/**
 * @brief Concrete MCPWM-based dual H-bridge motor driver.
 */
class HBridgeMotor : public IMotorDriver
{
public:
    /**
     * @brief Default constructor.
     */
    HBridgeMotor() = default;

    /**
     * @brief Virtual destructor for safe polymorphic deletion.
     */
    ~HBridgeMotor() noexcept override;

    // ---- Setup ----

    /**
     * @brief Initialize the driver with hardware configuration.
     * @param hw Hardware configuration for MCPWM and pins.
     */
    void setup(const MotorMCPWMConfig &hw) override;

    /**
     * @brief Initialize the driver with hardware and behavior configuration.
     * @param hw Hardware configuration for MCPWM and pins.
     * @param beh Behavior configuration (freewheel, soft-brake).
     */
    void setup(const MotorMCPWMConfig &hw, const MotorBehaviorConfig &beh) override;

    /**
     * @brief Initialize the driver with hardware, behavior, safety, and capture configs.
     * @param hw Hardware configuration for MCPWM and pins.
     * @param beh Behavior configuration (freewheel, soft-brake).
     * @param sfty Optional safety (fault) configuration.
     * @param cap Optional capture configuration.
     */
    void setup(const MotorMCPWMConfig &hw, const MotorBehaviorConfig &beh,
               const MotorSafetyConfig &sfty, const MotorCaptureConfig &cap) override;

    // ---- Core control ----

    /**
     * @brief Set speed and direction.
     * @param speed Duty request in [0, getMaxPwmInput()].
     * @param dir Desired rotation direction.
     */
    void setSpeed(int speed, Dir dir) noexcept override;

    /**
     * @brief Set motor speed as a percentage of the maximum input (0..100).
     * @param percent Speed request in percent (values outside 0..100 are clamped).
     * @param dir Desired direction (CW/CCW).
     */
    void setSpeedPercent(float percent, Dir dir) noexcept override;

    /**
     * @brief Enter freewheel (coast) according to current FreewheelMode.
     */
    void setFreewheel() noexcept override;

    /**
     * @brief Apply a hard electronic brake (A=100%, B=100%).
     */
    void setHardBrake() noexcept override;

    /**
     * @brief Set the soft-brake PWM level (0..getMaxPwmInput()).
     * @param pwm Requested soft-brake level.
     */
    void setSoftBrakePWM(uint16_t pwm) noexcept override;

    /**
     * @brief Convenience to immediately start soft-brake at the given level.
     * @param pwm Requested soft-brake level.
     */
    void softBrakeNow(uint16_t pwm) noexcept;

    /**
     * @brief Get the maximum accepted logical PWM input.
     * @return int Maximum input value (e.g., 1023).
     */
    [[nodiscard]] int getMaxPwmInput() const noexcept override { return input_max_; }

    /**
     * @brief Set the freewheel mode for subsequent setFreewheel() calls.
     * @param mode Freewheel strategy to use.
     */
    void setFreewheelMode(FreewheelMode mode) noexcept override;

    /**
     * @brief Set the freewheel mode and immediately apply freewheel.
     * @param mode Freewheel strategy to use.
     */
    void applyFreewheel(FreewheelMode mode) noexcept override;

    // ---- Lifecycle ----

    /**
     * @brief Start the MCPWM outputs (0% duty).
     */
    void start() noexcept override;

    /**
     * @brief Stop the MCPWM outputs.
     */
    void stop() noexcept override;

    /**
     * @brief Attempt to change the PWM frequency at runtime.
     * @param new_hz New frequency in Hz.
     * @return true If the change was applied.
     * @return false If not supported or failed.
     */
    bool reconfigureFrequency(int new_hz) noexcept override;

    // ---- Safety / raw outputs ----

    /**
     * @brief Check if a fault has been latched.
     * @return true If a fault is active or latched.
     * @return false Otherwise.
     */
    bool hasFault() const noexcept override { return fault_latched_; }

    /**
     * @brief Clear a latched fault and return to a safe idle state.
     */
    void clearFault() noexcept override;

    /**
     * @brief Force raw outputs (100% on the requested sides).
     * @param a_high True for high on A side.
     * @param b_high True for high on B side.
     */
    void forceOutputs(bool a_high, bool b_high) noexcept override;

private:
    // ---- Internal constants ----
    static constexpr double kMicrosPerSec = 1e6; ///< Conversion for Hz→µs.
    static constexpr float kDutyEps = 0.01f;     ///< Duty cache epsilon.

    // ---- Soft-brake phase machine ----
    enum class BrakePhase : uint8_t
    {
        Coast, ///< Coasting phase.
        Brake  ///< Braking phase.
    };

    void softBrakeISR() noexcept;           ///< Timer ISR trampoline for soft-brake toggling.
    void applyPhase(BrakePhase p) noexcept; ///< Apply current soft-brake phase.
    void scheduleNextPhase() noexcept;      ///< Schedule next dither phase tick.
    void startSoftBrake() noexcept;         ///< Begin soft-brake dither.
    void stopSoftBrake() noexcept;          ///< Stop soft-brake dither.

    // ---- GPIO / IO helpers ----
    void setEnable(bool on) noexcept;                        ///< Control optional EN pin.
    void writeAB(float a_percent, float b_percent) noexcept; ///< Write MCPWM A/B duties (0..100).

    // ---- Calculations ----
    void recomputeSoftDurations() noexcept; ///< Recompute dither phase durations.

    // ---- Safety (software fallback) ----
    static void IRAM_ATTR faultISRThunk(void *arg); ///< Static thunk to instance ISR.
    void IRAM_ATTR faultISR() noexcept;             ///< Fault ISR.
    void emergencyBrake() noexcept;                 ///< Immediate full electronic brake.

    // ---- Capture (software fallback) ----
    static void IRAM_ATTR capISRThunk(void *arg); ///< Static thunk to instance ISR.
    void IRAM_ATTR capISR() noexcept;             ///< Capture ISR.

    // ---- State (hardware & behavior) ----
    // Pins & routing.
    int lpwm_pin_{-1};                         ///< LPWM pin.
    int rpwm_pin_{-1};                         ///< RPWM pin.
    int en_pin_{-1};                           ///< Enable pin; -1 if unused.
    mcpwm_unit_t mcpwm_unit_{MCPWM_UNIT_0};    ///< MCPWM unit.
    mcpwm_timer_t mcpwm_timer_{MCPWM_TIMER_0}; ///< MCPWM timer.
    mcpwm_io_signals_t mcpwm_sig_l_{MCPWM0A};  ///< MCPWM signal for LPWM.
    mcpwm_io_signals_t mcpwm_sig_r_{MCPWM0B};  ///< MCPWM signal for RPWM.

    // Config mirrors.
    int pwm_freq_hz_{20000};                              ///< PWM frequency (Hz).
    int input_max_{1023};                                 ///< Max logical input.
    float percent_per_count_{0.0f};                       ///< 100 / input_max.
    uint32_t min_phase_us_{1500};                         ///< Minimum dither phase (µs).
    bool dither_coast_hi_z_{false};                       ///< Dither brake coast uses Hi-Z (EN low) when true.
    mcpwm_counter_type_t counter_mode_{MCPWM_UP_COUNTER}; ///< Counter mode.
    MotorBehaviorConfig beh_{};                           ///< Behavior config.
    MotorSafetyConfig sfty_{};                            ///< Safety config.
    MotorCaptureConfig cap_{};                            ///< Capture config.

    // EN control.
    bool use_en_{false};   ///< True if EN pin is used.
    bool en_state_{false}; ///< Cached EN state.

    // Soft-brake runtime.
    esp_timer_handle_t soft_timer_{nullptr};   ///< esp_timer used for dither.
    bool soft_active_{false};                  ///< True if dither is running.
    BrakePhase soft_phase_{BrakePhase::Coast}; ///< Current dither phase.
    float soft_level_{0.0f};                   ///< 0..1 from soft_brake_pwm_ / input_max_.
    int soft_hz_{300};                         ///< Dither frequency (Hz).
    int64_t soft_us_brake_{0};                 ///< Brake phase duration (µs).
    int64_t soft_us_coast_{0};                 ///< Coast phase duration (µs).
    uint16_t soft_brake_pwm_{0};               ///< Current soft-brake PWM request.

    // Safety (fault).
    volatile bool fault_latched_{false}; ///< Latched fault state.

    // Capture (period measurement).
    volatile uint32_t last_edge_us_{0}; ///< Last edge timestamp (µs).
    volatile uint32_t period_us_{0};    ///< Measured period (µs).

    // Concurrency & caching.
    mutable portMUX_TYPE soft_mux_ = portMUX_INITIALIZER_UNLOCKED;             ///< Tiny critical section.
    inline void lockSoft() const noexcept { portENTER_CRITICAL(&soft_mux_); }  ///< Lock for soft state.
    inline void unlockSoft() const noexcept { portEXIT_CRITICAL(&soft_mux_); } ///< Unlock for soft state.
    float last_a_percent_{-1.0f};                                              ///< Cached last A duty.
    float last_b_percent_{-1.0f};                                              ///< Cached last B duty.

    // Non-copyable.
    HBridgeMotor(const HBridgeMotor &) = delete;            ///< No copy.
    HBridgeMotor &operator=(const HBridgeMotor &) = delete; ///< No assign.
};