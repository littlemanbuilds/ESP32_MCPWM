/**
 * MIT License
 *
 * @brief Dual-H-bridge motor driver (ESP32 MCPWM) with soft-brake, dead-time,
 *        center-aligned mode, start/stop, runtime retune, and software fallbacks
 *        for fault (E-stop) and capture (edge-interval measurement).
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
     *
     * Safe to call again: prior soft-brake timing is stopped and any attached
     * fault/capture GPIO interrupts are detached before the new setup is applied.
     * On success, the configured freewheel behavior has been applied. Check
     * isSetupComplete() before issuing motor commands.
     *
     * @param hw Hardware configuration for MCPWM and pins.
     */
    void setup(const MotorMCPWMConfig &hw) override;

    /**
     * @brief Initialize the driver with hardware and behavior configuration.
     *
     * Safe to call again; previous optional fault/capture interrupts are cleaned up.
     * On success, the configured freewheel behavior has been applied. Check
     * isSetupComplete() before issuing motor commands.
     *
     * @param hw Hardware configuration for MCPWM and pins.
     * @param beh Behavior configuration (freewheel, soft-brake).
     */
    void setup(const MotorMCPWMConfig &hw, const MotorBehaviorConfig &beh) override;

    /**
     * @brief Initialize the driver with hardware, behavior, safety, and capture configs.
     *
     * Safe to call again; previous optional fault/capture interrupts are cleaned up.
     * On success, the configured freewheel behavior has been applied. Check
     * isSetupComplete() before issuing motor commands.
     *
     * @param hw Hardware configuration for MCPWM and pins.
     * @param beh Behavior configuration (freewheel, soft-brake).
     * @param safety Optional safety (fault) configuration.
     * @param cap Optional capture configuration.
     */
    void setup(const MotorMCPWMConfig &hw, const MotorBehaviorConfig &beh,
               const MotorSafetyConfig &safety, const MotorCaptureConfig &cap) override;

    /**
     * @brief Check whether the most recent setup completed successfully.
     * @return true If hardware resources are ready for motor commands.
     * @return false Otherwise.
     */
    [[nodiscard]] bool isSetupComplete() const noexcept override { return setup_done_; }

    /**
     * @brief Get the result of the most recent setup attempt.
     * @return MotorSetupError Setup result.
     */
    [[nodiscard]] MotorSetupError getLastSetupError() const noexcept override
    {
        return setup_error_;
    }

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
     * @brief Process deferred fault actions and notify callback in normal task context.
     */
    void pollFaults() noexcept override;

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
     * @brief Start MCPWM and reapply the configured freewheel state.
     */
    void start() noexcept override;

    /**
     * @brief Deassert EN, set A/B to 0%, and stop the MCPWM outputs.
     */
    void stop() noexcept override;

    /**
     * @brief Attempt to change the PWM frequency at runtime.
     *
     * Normal outputs are first brought to disabled zero output. Active dither
     * restarts after a successful change; failed changes remain inactive.
     * @param new_hz New frequency in Hz.
     * @return true If the change was applied.
     * @return false If not supported or failed.
     */
    bool reconfigureFrequency(int new_hz) noexcept override;

    /**
     * @brief Get the latest measured interval between selected capture edges.
     * @return uint32_t Edge interval in microseconds, or zero before a valid measurement.
     */
    [[nodiscard]] uint32_t getLastCapturePeriodUs() const noexcept override { return period_us_; }

    // ---- Safety / raw outputs ----

    /**
     * @brief Check whether a one-shot fault is latched or a followed level is active.
     * @return true If fault output inhibition is active.
     * @return false Otherwise.
     */
    bool hasFault() const noexcept override { return fault_latched_; }

    /**
     * @brief Check whether a bridge EN pin is configured for library control.
     * @return true If the configured EN pin can be commanded by the library.
     * @return false If no EN pin is configured.
     */
    [[nodiscard]] bool hasEnableControl() const noexcept override { return use_en_; }

    /**
     * @brief Clear an inactive latched fault and return to zero-output idle.
     */
    void clearFault() noexcept override;

    /**
     * @brief Force raw outputs (100% on the requested sides).
     * @param a_high True for high on A side.
     * @param b_high True for high on B side.
     */
    void forceOutputs(bool a_high, bool b_high) noexcept override;

    /**
     * @brief Optional fault notification callback (level or latched).
     * @param cb Callback: (active, ctx).
     * @param ctx Opaque pointer (handed back).
     */
    void setFaultCallback(FaultCallback cb, void *ctx) noexcept override
    {
        fault_cb_ = cb;
        fault_ctx_ = ctx;
    }

private:
    // ---- Internal constants ----
    static constexpr uint32_t kMicrosPerSec = 1000000U; ///< Conversion for Hz→µs.
    static constexpr float kDutyEps = 0.01f;     ///< Duty cache epsilon.
    static constexpr int kPwmHzMin = 1;          ///< Lowest accepted drive PWM frequency.
    static constexpr int kPwmHzMax = 1000000;    ///< Highest accepted drive PWM frequency.
    static constexpr int kSoftHzMax = 10000;     ///< Highest practical esp_timer dither frequency.

    // ---- Soft-brake phase machine ----
    enum class BrakePhase : uint8_t
    {
        Coast, ///< Coasting phase.
        Brake  ///< Braking phase.
    };

    /**
     * @brief Toggle the soft-brake phase from the esp_timer task.
     */
    void softBrakeTimerTask() noexcept;

    /**
     * @brief Apply a non-timed brake or coast phase.
     * @param phase Phase to apply.
     */
    void applyPhase(BrakePhase phase) noexcept;

    /**
     * @brief Apply a timed phase if its dither sequence is still current.
     * @param phase Phase to apply.
     * @param sequence Dither sequence that requested the phase.
     * @return true If the phase was applied and remains current.
     * @return false If a newer command invalidated the phase.
     */
    bool applyDitherPhase(BrakePhase phase, uint32_t sequence) noexcept;

    /**
     * @brief Schedule the next phase if its dither sequence is still current.
     * @param sequence Dither sequence that requested the timer.
     * @return true If the next phase was scheduled.
     * @return false If the sequence is stale or timer scheduling failed.
     */
    bool scheduleNextPhase(uint32_t sequence) noexcept;

    /**
     * @brief Begin soft-brake dither or apply a steady endpoint state.
     */
    void startSoftBrake() noexcept;

    /**
     * @brief Stop soft-brake dither and invalidate pending callbacks.
     */
    void stopSoftBrake() noexcept;

    // ---- GPIO / IO helpers ----

    /**
     * @brief Control the optional bridge EN pin.
     * @param enabled True to assert EN, false to deassert it.
     */
    void setEnable(bool enabled) noexcept;

    /**
     * @brief Write the MCPWM A and B duty values.
     * @param a_percent A-channel duty in percent.
     * @param b_percent B-channel duty in percent.
     */
    void writeAB(float a_percent, float b_percent) noexcept;

    /**
     * @brief Apply one output snapshot in a safe transition order.
     * @param enable Desired EN state.
     * @param a_percent Desired A-channel duty in percent.
     * @param b_percent Desired B-channel duty in percent.
     */
    void writeHardwareOutput(bool enable, float a_percent, float b_percent) noexcept;

    /**
     * @brief Apply the newest output if the supplied snapshot becomes stale.
     * @param sequence Output sequence associated with the snapshot.
     * @param enable Desired EN state.
     * @param a_percent Desired A-channel duty in percent.
     * @param b_percent Desired B-channel duty in percent.
     */
    void writeOutputUntilCurrent(uint32_t sequence, bool enable,
                                 float a_percent, float b_percent) noexcept;

    /**
     * @brief Publish and apply a new output command.
     * @param enable Desired EN state.
     * @param a_percent Desired A-channel duty in percent.
     * @param b_percent Desired B-channel duty in percent.
     */
    void commandOutput(bool enable, float a_percent, float b_percent) noexcept;

    /**
     * @brief Apply the newest timer command if the supplied command becomes stale.
     * @param sequence Timer sequence associated with the command.
     * @param active True to start the timer, false to stop it.
     * @param timeout_us One-shot timeout in microseconds.
     * @return true If the current timer command was applied successfully.
     * @return false If the current timer start failed.
     */
    bool writeTimerUntilCurrent(uint32_t sequence, bool active,
                                int64_t timeout_us) noexcept;

    // ---- Calculations ----

    /**
     * @brief Recompute the brake and coast durations for the current dither level.
     * @return true If valid durations were calculated.
     * @return false If the configured timing cannot form a valid period.
     */
    bool recomputeSoftDurations() noexcept;

    // ---- Safety (software fallback) ----

    /**
     * @brief Forward the static fault interrupt to its driver instance.
     * @param arg Driver instance supplied to attachInterruptArg().
     */
    static void IRAM_ATTR faultISRThunk(void *arg);

    /**
     * @brief Sample the fault input and defer safety work to task context.
     */
    void IRAM_ATTR faultISR() noexcept;

    /**
     * @brief Read whether the configured fault input is active.
     * @return true If the configured active level is present.
     * @return false Otherwise.
     */
    bool IRAM_ATTR faultInputActive() const noexcept;

    /**
     * @brief Apply the configured low-level fault action.
     */
    void applyFaultAction() noexcept;

    /**
     * @brief Apply the full electronic brake and keep MCPWM running.
     */
    void emergencyBrake() noexcept;

    /**
     * @brief Detach the configured fault interrupt if one is attached.
     */
    void detachFaultInterrupt() noexcept;

    // ---- Capture (software fallback) ----

    /**
     * @brief Forward the static capture interrupt to its driver instance.
     * @param arg Driver instance supplied to attachInterruptArg().
     */
    static void IRAM_ATTR capISRThunk(void *arg);

    /**
     * @brief Measure the interval since the previous selected capture edge.
     */
    void IRAM_ATTR capISR() noexcept;

    /**
     * @brief Detach the configured capture interrupt if one is attached.
     */
    void detachCaptureInterrupt() noexcept;

    // ---- Setup / teardown helpers ----

    /**
     * @brief Stop and detach existing resources before a setup attempt.
     */
    void prepareForSetup() noexcept;

    /**
     * @brief Validate user configuration before touching MCPWM resources.
     * @param hw Hardware configuration to validate.
     * @param beh Behavior configuration to validate.
     * @param safety Safety configuration to validate.
     * @param cap Capture configuration to validate.
     * @return MotorSetupError::None If the configuration is valid.
     * @return MotorSetupError A specific validation failure otherwise.
     */
    MotorSetupError validateConfig(const MotorMCPWMConfig &hw,
                                   const MotorBehaviorConfig &beh,
                                   const MotorSafetyConfig &safety,
                                   const MotorCaptureConfig &cap) const noexcept;

    /**
     * @brief Leave a failed setup attempt in an inactive state.
     * @param error Error to expose through getLastSetupError().
     */
    void failSetup(MotorSetupError error) noexcept;

    // ---- State (hardware & behavior) ----
    // Pins & routing.
    int lpwm_pin_{-1};                         ///< LPWM pin.
    int rpwm_pin_{-1};                         ///< RPWM pin.
    int en_pin_{-1};                           ///< Enable pin; -1 if unused.
    mcpwm_unit_t mcpwm_unit_{MCPWM_UNIT_0};    ///< MCPWM unit.
    mcpwm_timer_t mcpwm_timer_{MCPWM_TIMER_0}; ///< MCPWM timer.
    mcpwm_io_signals_t mcpwm_sig_l_{MCPWM0A};  ///< MCPWM signal for LPWM.
    mcpwm_io_signals_t mcpwm_sig_r_{MCPWM0B};  ///< MCPWM signal for RPWM.

    // ---- Config mirrors ----
    int pwm_freq_hz_{20000};                              ///< PWM frequency (Hz).
    int input_max_{1023};                                 ///< Max logical input.
    float percent_per_count_{0.0f};                       ///< 100 / input_max.
    uint32_t min_phase_us_{50};                           ///< Minimum dither phase (µs).
    bool dither_coast_hi_z_{false};                       ///< Dither brake coast uses Hi-Z (EN low) when true.
    mcpwm_counter_type_t counter_mode_{MCPWM_UP_COUNTER}; ///< Counter mode.
    MotorBehaviorConfig beh_{};                           ///< Behavior config.
    MotorSafetyConfig safety_{};                          ///< Safety config.
    MotorCaptureConfig cap_{};                            ///< Capture config.
    bool setup_done_{false};                              ///< True after MCPWM setup has completed.
    bool mcpwm_initialized_{false};                       ///< True after MCPWM timer initialization.
    bool deadtime_enabled_{false};                        ///< True while MCPWM dead-time is configured.
    MotorSetupError setup_error_{MotorSetupError::None};  ///< Most recent setup result.

    // ---- EN control ----
    bool use_en_{false};   ///< True if EN pin is used.
    bool en_state_{false}; ///< Cached EN state.

    // ---- Soft-brake runtime ----
    esp_timer_handle_t soft_timer_{nullptr};   ///< esp_timer used for dither.
    bool soft_active_{false};                  ///< True if dither is running.
    BrakePhase soft_phase_{BrakePhase::Coast}; ///< Current dither phase.
    int soft_hz_{300};                         ///< Dither frequency (Hz).
    int64_t soft_us_brake_{0};                 ///< Brake phase duration (µs).
    int64_t soft_us_coast_{0};                 ///< Coast phase duration (µs).
    uint16_t soft_brake_pwm_{0};               ///< Current soft-brake PWM request.
    uint32_t soft_sequence_{0};                 ///< Invalidates callbacks from older dither cycles.

    // ---- Safety (fault) ----
    volatile bool fault_latched_{false}; ///< Latched fault state.
    volatile bool fault_pending_{false}; ///< ISR posts work for task context.
    FaultCallback fault_cb_{nullptr};    ///< User fault callback.
    void *fault_ctx_{nullptr};           ///< User context pointer.
    int fault_irq_pin_{-1};              ///< Attached fault interrupt pin.

    // ---- Capture (edge-interval measurement) ----
    volatile uint32_t last_edge_us_{0}; ///< Last edge timestamp (µs).
    volatile uint32_t period_us_{0};    ///< Measured selected-edge interval (µs).
    volatile bool capture_edge_seen_{false}; ///< True after the first selected edge.
    int cap_irq_pin_{-1};               ///< Attached capture interrupt pin.

    // ---- Concurrency & caching ----
    mutable portMUX_TYPE soft_mux_ = portMUX_INITIALIZER_UNLOCKED;             ///< Tiny critical section.
    /**
     * @brief Enter the critical section protecting dither and output state.
     */
    inline void lockSoft() const noexcept { portENTER_CRITICAL(&soft_mux_); }

    /**
     * @brief Exit the critical section protecting dither and output state.
     */
    inline void unlockSoft() const noexcept { portEXIT_CRITICAL(&soft_mux_); }
    float last_a_percent_{-1.0f};                                              ///< Cached last A duty.
    float last_b_percent_{-1.0f};                                              ///< Cached last B duty.
    uint32_t output_sequence_{0};                                              ///< Identifies the latest output command.
    bool commanded_enable_{false};                                             ///< Latest requested EN state.
    float commanded_a_percent_{0.0f};                                         ///< Latest requested A duty.
    float commanded_b_percent_{0.0f};                                         ///< Latest requested B duty.
    uint32_t timer_sequence_{0};                                               ///< Identifies the latest timer command.
    bool commanded_timer_active_{false};                                      ///< Latest requested timer state.
    int64_t commanded_timer_us_{0};                                           ///< Latest requested timer delay.

    // ---- Non-copyable ----
    /**
     * @brief Disable copy construction because each instance owns hardware resources.
     * @param other Instance that would otherwise be copied.
     */
    HBridgeMotor(const HBridgeMotor &other) = delete;

    /**
     * @brief Disable copy assignment because each instance owns hardware resources.
     * @param other Instance that would otherwise be copied.
     * @return HBridgeMotor& This instance; declaration is deleted and cannot be called.
     */
    HBridgeMotor &operator=(const HBridgeMotor &other) = delete;
};
