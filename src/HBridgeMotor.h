/**
 * MIT License
 *
 * @brief ESP32 MCPWM H-bridge driver with explicit drive/coast/brake semantics,
 *        lifecycle results, software fault observation, hardware MCPWM faults,
 *        dither braking, and edge-interval capture.
 *
 * @file HBridgeMotor.h
 * @author Little Man Builds (Darren Osborne)
 * @date 2025-08-28
 * @copyright Copyright (c) 2026 Little Man Builds
 *
 */

#pragma once

#include "IMotorDriver.h"

#include <Arduino.h>
#include <driver/mcpwm.h>
#include <esp_timer.h>
#include <freertos/portmacro.h>

#include <cstdint>

#ifndef ESP32_MCPWM_ENABLE_COMMISSIONING_API
#define ESP32_MCPWM_ENABLE_COMMISSIONING_API 0
#endif

/**
 * @brief Concrete legacy-ESP-IDF MCPWM dual H-bridge driver.
 *
 * @note ESP32_MCPWM v2.x targets the current stable Arduino-ESP32 3.x line,
 *       whose ESP-IDF 5.x base still provides driver/mcpwm.h. ESP-IDF 6 removes
 *       the legacy MCPWM driver and requires a future backend migration.
 */
class HBridgeMotor : public IMotorDriver
{
  public:
    /// @brief Construct an unconfigured motor driver.
    HBridgeMotor() = default;

    /// @brief Stop timers, detach interrupts, and make a best-effort output shutdown.
    ~HBridgeMotor() noexcept override;

    // ---- Setup ---- //

    /**
     * @brief Configure hardware with default behavior and no optional observers.
     *
     * Repeated setup first stops dither, detaches optional interrupts, and
     * contains resources from the previous setup. On success, the configured
     * coast state is already applied.
     *
     * @param hw MCPWM pin, timer, signal and range configuration.
     * @return Setup result with detailed failure information.
     */
    MotorSetupResult setup(const MotorMCPWMConfig &hw) override;

    /**
     * @brief Configure hardware and freewheel/dither behavior.
     *
     * Repeated setup cleans up previous timer/interrupt resources before the
     * replacement configuration is applied. On success, @p beh controls the
     * already-applied coast state.
     *
     * @param hw MCPWM pin, timer, signal and range configuration.
     * @param beh Freewheel, dither and soft-brake behavior.
     * @return Setup result with detailed failure information.
     */
    MotorSetupResult setup(const MotorMCPWMConfig &hw, const MotorBehaviorConfig &beh) override;

    /**
     * @brief Configure hardware, behavior, software fault observation and capture.
     *
     * Repeated setup cleans up previous timer/interrupt resources before the
     * replacement configuration is applied.
     *
     * @param hw MCPWM pin, timer, signal and range configuration.
     * @param beh Freewheel, dither and soft-brake behavior.
     * @param safety Optional GPIO/ISR software fault observer.
     * @param cap Optional GPIO edge-interval capture.
     * @return Setup result with detailed failure information.
     */
    MotorSetupResult setup(const MotorMCPWMConfig &hw, const MotorBehaviorConfig &beh, const MotorSafetyConfig &safety,
                           const MotorCaptureConfig &cap) override;

    /**
     * @brief Configure all hardware, behavior, software and MCPWM fault options.
     *
     * Existing resources are contained before new configuration is validated;
     * teardown failure therefore takes precedence over a later validation error.
     *
     * @param hw MCPWM pin, timer, signal and range configuration.
     * @param beh Freewheel, dither and soft-brake behavior.
     * @param safety Optional GPIO/ISR software fault observer.
     * @param cap Optional GPIO edge-interval capture.
     * @param hardware_fault Optional MCPWM peripheral fault input/action.
     * @return Setup result with detailed failure information.
     */
    MotorSetupResult setup(const MotorMCPWMConfig &hw, const MotorBehaviorConfig &beh, const MotorSafetyConfig &safety,
                           const MotorCaptureConfig &cap, const MotorHardwareFaultConfig &hardware_fault) override;

    /// @brief Return whether setup completed successfully.
    ESP32_MCPWM_NODISCARD bool isSetupComplete() const noexcept override;

    /// @brief Return the detailed result from the most recent setup attempt.
    ESP32_MCPWM_NODISCARD MotorSetupError getLastSetupError() const noexcept override;

    // ---- Explicit output control ---- //

    /**
     * @brief Apply a positive drive request.
     *
     * @param speed Positive duty request in [1, getMaxPwmInput()].
     * @param dir Desired direction.
     * @return Structured operation result.
     */
    MotorOperationResult drive(int speed, Dir dir) noexcept override;

    /**
     * @brief Apply a positive percentage drive request.
     *
     * @param percent Positive request in (0,100].
     * @param dir Desired direction.
     * @return Structured operation result.
     */
    MotorOperationResult drivePercent(float percent, Dir dir) noexcept override;

    /// @brief Apply a positive drive request through the v1-compatible name.
    MotorOperationResult setSpeed(int speed, Dir dir) noexcept override;

    /// @brief Apply a positive percentage request through the v1-compatible name.
    MotorOperationResult setSpeedPercent(float percent, Dir dir) noexcept override;

    /// @brief Enter the configured explicit coast/freewheel state.
    MotorOperationResult coast() noexcept override;

    /// @brief Deassert outputs where possible and stop MCPWM generation.
    MotorOperationResult disableOutputs() noexcept override
    {
        return stop();
    }

    /// @brief Enter coast through the v1-compatible freewheel name.
    MotorOperationResult setFreewheel() noexcept override
    {
        return coast();
    }

    /**
     * @brief Apply explicit full electronic braking.
     *
     * @warning Both bridge inputs are driven at full duty. Validate current,
     *          regeneration, and the module truth table before use on hardware.
     */
    MotorOperationResult setHardBrake() noexcept override;

    /// @brief Set the soft-brake PWM level without starting soft brake.
    MotorOperationResult setSoftBrakePWM(uint16_t pwm) noexcept override;

    /**
     * @brief Configure and immediately start explicit soft/dither braking.
     *
     * @warning Dither repeatedly alternates electrical brake and coast states.
     *          Validate bridge current and mechanical response before loaded use.
     *
     * @param pwm Brake strength in the configured logical input range.
     * @return Structured operation result.
     */
    MotorOperationResult softBrakeNow(uint16_t pwm) noexcept;

    // ---- Optional fault handling ---- //

    /// @brief Apply deferred software fault work in task context.
    MotorOperationResult pollFaults() noexcept override;

    /// @brief Return whether software or MCPWM fault state is active or latched.
    ESP32_MCPWM_NODISCARD bool hasFault() const noexcept override;

    /// @brief Clear inactive fault state and recover to quiet outputs.
    MotorOperationResult clearFault() noexcept override;

    /**
     * @brief Register a task-context notification callback for the software fault observer.
     *
     * @param cb Callback invoked by pollFaults().
     * @param ctx Opaque callback context.
     */
    void setFaultCallback(FaultCallback cb, void *ctx) noexcept override;

    // ---- Runtime behavior ---- //

    /// @brief Return the configured maximum logical PWM input.
    ESP32_MCPWM_NODISCARD int getMaxPwmInput() const noexcept override;

    /// @brief Change stored freewheel behavior without applying it unless dither is active.
    MotorOperationResult setFreewheelMode(FreewheelMode mode) noexcept override;

    /// @brief Change freewheel behavior and immediately apply coast().
    MotorOperationResult applyFreewheel(FreewheelMode mode) noexcept override;

    /// @brief Start MCPWM generation and return to the configured quiet state.
    MotorOperationResult start() noexcept override;

    /// @brief Stop MCPWM generation and deassert outputs where possible.
    MotorOperationResult stop() noexcept override;

    /**
     * @brief Change drive PWM frequency after moving outputs quiet where possible.
     *
     * A successful non-dither change leaves outputs in a zero-duty coast state;
     * the caller must issue a new drive request. Active dither is restarted only
     * after the frequency update succeeds.
     *
     * @param new_hz Requested drive PWM frequency in Hz.
     * @return Structured result; an already-current frequency is unchanged success.
     */
    MotorOperationResult reconfigureFrequency(int new_hz) noexcept override;

    // ---- Capture ---- //

    /// @brief Return the last selected-edge interval in microseconds, or zero before one exists.
    ESP32_MCPWM_NODISCARD uint32_t getLastCapturePeriodUs() const noexcept override;

    // ---- Diagnostics / capability ---- //

    /// @brief Return whether an EN pin is configured and controlled by the driver.
    ESP32_MCPWM_NODISCARD bool hasEnableControl() const noexcept override;

    /// @brief Return a coherent software status snapshot.
    ESP32_MCPWM_NODISCARD MotorDriverStatus status() const noexcept override;

    /// @brief Return on-demand readback exposed by the legacy MCPWM API.
    ESP32_MCPWM_NODISCARD MotorHardwareReadback readback() const noexcept override;

    /**
     * @brief Force raw full-scale A/B outputs for commissioning only.
     *
     * The method is inert unless ESP32_MCPWM_ENABLE_COMMISSIONING_API is set to 1
     * before compiling the library.
     */
    MotorOperationResult forceOutputs(bool a_high, bool b_high) noexcept override;

  private:
    static constexpr uint32_t kMicrosPerSec = 1000000U; ///< Hz-to-µs conversion.
    static constexpr float kDutyEps = 0.01f;            ///< Duty cache epsilon.
    static constexpr int kPwmHzMin = 1;                 ///< Lowest accepted drive PWM frequency.
    static constexpr int kPwmHzMax = 1000000;           ///< Highest accepted drive PWM frequency.
    static constexpr int kSoftHzMax = 10000;            ///< Highest practical dither frequency.

    enum class BrakePhase : uint8_t
    {
        Coast, ///< Coasting phase.
        Brake  ///< Braking phase.
    };

    enum class OutputCommitResult : uint8_t
    {
        Committed,     ///< Requested output reached the hardware commit boundary.
        Stale,         ///< A newer output generation superseded this request.
        HardwareFailed ///< A hardware API write failed during this generation.
    };

    enum class TimerCommandResult : uint8_t
    {
        Applied,          ///< The requested timer command was applied.
        StaleOrCancelled, ///< A newer timer or dither generation superseded the request.
        TimerFailed       ///< The current timer command failed in the ESP timer API.
    };

    // ---- Result helpers ---- //

    /// @brief Build the structured setup result from synchronized driver state.
    MotorSetupResult setupResult() const noexcept;

    /**
     * @brief Record and return a structured operation result.
     *
     * @param operation Public operation being completed.
     * @param error Primary operation diagnosis.
     * @param changed True only when public semantic or output state changed.
     * @return Structured operation result and current success sequence.
     */
    MotorOperationResult result(MotorOperation operation, MotorOperationError error, bool changed = false) noexcept;

    /**
     * @brief Record and return an unchanged rejected operation.
     *
     * @param operation Public operation that was rejected.
     * @param error Primary rejection diagnosis.
     * @return Structured unchanged operation result.
     */
    MotorOperationResult reject(MotorOperation operation, MotorOperationError error) noexcept;

    /**
     * @brief Publish the latest operation and diagnosis without advancing success state.
     *
     * @param operation Operation to publish.
     * @param error Primary operation diagnosis.
     */
    void recordOperation(MotorOperation operation, MotorOperationError error) noexcept;

    /**
     * @brief Publish the semantic output mode under the state lock.
     *
     * @param mode Truthful mode established by the completed hardware action.
     */
    void setOutputMode(MotorOutputMode mode) noexcept;

    // ---- Soft-brake state machine ---- //

    /// @brief Advance one dither phase from ESP timer task context.
    void softBrakeTimerTask() noexcept;

    /**
     * @brief Apply one untimed coast or brake endpoint.
     *
     * @param phase Endpoint phase to apply.
     * @return True when the hardware output commit succeeded.
     */
    bool applyPhase(BrakePhase phase) noexcept;

    /**
     * @brief Commit one dither phase only while its generation remains current.
     *
     * @param phase Dither phase to apply.
     * @param sequence Dither generation that owns the phase.
     * @return Commit, stale-generation, or hardware-failure outcome.
     */
    OutputCommitResult applyDitherPhase(BrakePhase phase, uint32_t sequence) noexcept;

    /**
     * @brief Schedule the next timer phase for a current dither generation.
     *
     * A genuine timer failure terminates the generation and attempts truthful
     * quiet-output containment before returning. Normal supersession leaves
     * the newer generation and its diagnostics authoritative.
     *
     * @param sequence Dither generation requesting the timer.
     * @return Applied, stale/cancelled, or genuine timer-failure outcome.
     */
    TimerCommandResult scheduleNextPhase(uint32_t sequence) noexcept;

    /// @brief Start dither braking or apply its steady zero/full endpoint.
    MotorOperationError startSoftBrake() noexcept;

    /// @brief Stop dither timing and invalidate all callbacks from older generations.
    void stopSoftBrake() noexcept;

    /// @brief Terminate failed dither work and publish contained or uncertain output truth.
    void containFailedDither() noexcept;

    /**
     * @brief Contain and publish an authoritative dither timer failure.
     *
     * @param sequence Dither generation that encountered the timer failure.
     * @return Timer failure while authoritative, otherwise stale/cancelled.
     */
    TimerCommandResult containDitherTimerFailure(uint32_t sequence) noexcept;

    /// @brief Recompute brake/coast durations from configured frequency and strength.
    bool recomputeSoftDurations() noexcept;

    // ---- Hardware output helpers ---- //

    /**
     * @brief Commit A/B/EN output state while its generation remains current.
     *
     * The caller holds the recursive state critical section across freshness
     * checks and physical writes so older deferred work cannot overwrite a
     * newer completed command.
     *
     * @param sequence Output generation to commit.
     * @param enable Desired logical bridge-enable state.
     * @param a_percent A-channel duty in percent.
     * @param b_percent B-channel duty in percent.
     * @return Commit, stale-generation, or hardware-failure outcome.
     */
    OutputCommitResult writeHardwareOutput(uint32_t sequence, bool enable, float a_percent, float b_percent) noexcept;

    /**
     * @brief Publish and synchronously commit one hardware output request.
     *
     * @param enable Desired logical bridge-enable state.
     * @param a_percent A-channel duty in percent.
     * @param b_percent B-channel duty in percent.
     * @return True when the request reached the hardware commit boundary.
     */
    bool commandOutput(bool enable, float a_percent, float b_percent) noexcept;

    /**
     * @brief Apply the newest timer command when the supplied generation is stale.
     *
     * @param sequence Timer generation to apply first.
     * @param active True to schedule a one-shot callback; false to stop it.
     * @param timeout_us One-shot interval in microseconds.
     * @return Applied, stale/cancelled, or genuine timer-failure outcome.
     */
    TimerCommandResult writeTimerUntilCurrent(uint32_t sequence, bool active, int64_t timeout_us) noexcept;

    // ---- Software fault observer ---- //

    /**
     * @brief Forward the static GPIO fault interrupt to its motor instance.
     *
     * @param arg HBridgeMotor instance supplied to attachInterruptArg().
     */
    static void IRAM_ATTR faultISRThunk(void *arg);

    /// @brief Sample software-fault GPIO state and defer task-context handling.
    void IRAM_ATTR faultISR() noexcept;

    /// @brief Return whether the configured software-fault GPIO level is active.
    bool IRAM_ATTR faultInputActive() const noexcept;

    /// @brief Return a synchronized snapshot of active or latched software fault state.
    bool softwareFaultActiveSnapshot() const noexcept;

    /// @brief Return a synchronized snapshot of deferred software-fault work.
    bool softwareFaultPendingSnapshot() const noexcept;

    /// @brief Apply the configured scheduler-dependent software-fault action.
    MotorOperationError applyFaultAction() noexcept;

    /// @brief Apply the configured explicit full electronic brake for a fault action.
    MotorOperationError emergencyBrake() noexcept;

    /// @brief Detach the software-fault GPIO interrupt when configured.
    void detachFaultInterrupt() noexcept;

    // ---- MCPWM hardware fault path ---- //

    /**
     * @brief Map the public fault selection to an MCPWM GPIO routing signal.
     *
     * @param input Public hardware-fault input selection.
     * @return Legacy MCPWM GPIO routing signal.
     */
    mcpwm_io_signals_t hardwareFaultIoSignal(HardwareFaultInput input) const noexcept;

    /**
     * @brief Map the public fault selection to an MCPWM fault signal.
     *
     * @param input Public hardware-fault input selection.
     * @return Legacy MCPWM peripheral fault signal.
     */
    mcpwm_fault_signal_t hardwareFaultSignal(HardwareFaultInput input) const noexcept;

    /**
     * @brief Forward the hardware-fault GPIO interrupt to its motor instance.
     *
     * @param arg HBridgeMotor instance supplied to attachInterruptArg().
     */
    static void IRAM_ATTR hardwareFaultISRThunk(void *arg);

    /// @brief Observe peripheral fault input state for coherent diagnostics.
    void IRAM_ATTR hardwareFaultISR() noexcept;

    /// @brief Configure exact MCPWM A/B hardware-fault actions and observation.
    bool configureHardwareFault() noexcept;

    /// @brief Re-arm a one-shot hardware fault after quiet output staging.
    bool rearmHardwareFault() noexcept;

    /// @brief Detach and deinitialize the MCPWM hardware-fault path.
    bool detachHardwareFault() noexcept;

    /// @brief Return whether the configured hardware-fault GPIO level is active.
    bool hardwareFaultInputActive() const noexcept;

    // ---- Capture ---- //

    /**
     * @brief Forward the static capture interrupt to its motor instance.
     *
     * @param arg HBridgeMotor instance supplied to attachInterruptArg().
     */
    static void IRAM_ATTR capISRThunk(void *arg);

    /// @brief Measure one selected-edge interval and invoke the optional ISR callback.
    void IRAM_ATTR capISR() noexcept;

    /// @brief Detach the capture GPIO interrupt when configured.
    void detachCaptureInterrupt() noexcept;

    // ---- Setup / teardown ---- //

    /// @brief Tear down previous resources and contain outputs before a setup attempt.
    MotorSetupError prepareForSetup() noexcept;

    /**
     * @brief Validate all setup configuration before touching hardware resources.
     *
     * @param hw MCPWM routing and electrical-interface configuration.
     * @param beh Coast and dither behavior configuration.
     * @param safety Software-fault observer configuration.
     * @param cap GPIO capture configuration.
     * @param hardware_fault MCPWM peripheral fault configuration.
     * @return Specific validation error or MotorSetupError::None.
     */
    MotorSetupError validateConfig(const MotorMCPWMConfig &hw, const MotorBehaviorConfig &beh,
                                   const MotorSafetyConfig &safety, const MotorCaptureConfig &cap,
                                   const MotorHardwareFaultConfig &hardware_fault) const noexcept;

    /**
     * @brief Roll back a failed setup and preserve containment failure precedence.
     *
     * @param error Primary setup-stage error being handled.
     */
    void failSetup(MotorSetupError error) noexcept;

    // ---- Synchronization ---- //

    /// @brief Enter the task-context recursive state critical section.
    inline void lockState() const noexcept
    {
        portENTER_CRITICAL(&state_mux_);
    }

    /// @brief Leave the task-context recursive state critical section.
    inline void unlockState() const noexcept
    {
        portEXIT_CRITICAL(&state_mux_);
    }

    /// @brief Enter the ISR-context recursive state critical section.
    inline void lockStateISR() const noexcept
    {
        portENTER_CRITICAL_ISR(&state_mux_);
    }

    /// @brief Leave the ISR-context recursive state critical section.
    inline void unlockStateISR() const noexcept
    {
        portEXIT_CRITICAL_ISR(&state_mux_);
    }

    // ---- Hardware / configuration ---- //

    int lpwm_pin_{-1};                         ///< GPIO routed to MCPWM output A/LPWM.
    int rpwm_pin_{-1};                         ///< GPIO routed to MCPWM output B/RPWM.
    int en_pin_{-1};                           ///< Optional bridge-enable GPIO, or -1 when absent.
    mcpwm_unit_t mcpwm_unit_{MCPWM_UNIT_0};    ///< MCPWM peripheral unit owned by this instance.
    mcpwm_timer_t mcpwm_timer_{MCPWM_TIMER_0}; ///< MCPWM timer owned by this instance.
    mcpwm_io_signals_t mcpwm_sig_l_{MCPWM0A};  ///< Peripheral signal routed to LPWM.
    mcpwm_io_signals_t mcpwm_sig_r_{MCPWM0B};  ///< Peripheral signal routed to RPWM.

    int pwm_freq_hz_{20000};                              ///< Last successfully configured drive frequency in Hz.
    int input_max_{1023};                                 ///< Maximum accepted logical drive/brake request.
    float percent_per_count_{0.0f};                       ///< Logical-input to duty-percent conversion factor.
    uint32_t min_phase_us_{50};                           ///< Minimum requested dither phase duration in microseconds.
    bool dither_coast_hi_z_{false};                       ///< True to deassert EN during each dither coast phase.
    mcpwm_counter_type_t counter_mode_{MCPWM_UP_COUNTER}; ///< Configured MCPWM counter mode.
    MotorBehaviorConfig beh_{};                           ///< Accepted coast and dither behavior snapshot.
    MotorSafetyConfig safety_{};                          ///< Accepted software-fault observer snapshot.
    MotorCaptureConfig cap_{};                            ///< Accepted edge-capture configuration snapshot.
    MotorHardwareFaultConfig hardware_fault_{};           ///< Accepted peripheral fault configuration snapshot.

    bool setup_done_{false};              ///< True only after all setup and initial containment succeeds.
    bool mcpwm_initialized_{false};       ///< True while MCPWM resources require teardown.
    bool mcpwm_running_{false};           ///< Cached lifecycle state after successful start/stop calls.
    bool deadtime_enabled_{false};        ///< True while dead-time requires explicit teardown.
    bool hardware_fault_enabled_{false};  ///< True while the MCPWM peripheral fault path is configured.
    bool hardware_fault_active_{false};   ///< Synchronized current peripheral fault GPIO level.
    bool hardware_fault_latched_{false};  ///< One-shot fault observed and awaiting safe re-arm.
    uint32_t hardware_fault_sequence_{0}; ///< Monotonic observed hardware-fault transition count.
    int hardware_fault_irq_pin_{-1};      ///< Attached hardware-fault observer GPIO, or -1.
    MotorSetupError setup_error_{MotorSetupError::None}; ///< Most recent setup or teardown diagnosis.

    // ---- EN ---- //

    bool use_en_{false};   ///< True when this instance owns an EN GPIO.
    bool en_state_{false}; ///< Cached physical EN level last written by the driver.

    // ---- Soft brake ---- //

    esp_timer_handle_t soft_timer_{nullptr};   ///< Owned one-shot ESP timer for dither phase changes.
    bool soft_active_{false};                  ///< True only while a dither generation has future work.
    BrakePhase soft_phase_{BrakePhase::Coast}; ///< Most recently committed/current dither phase.
    int soft_hz_{300};                         ///< Configured dither cycle frequency in Hz.
    int64_t soft_us_brake_{0};                 ///< Calculated brake-phase duration in microseconds.
    int64_t soft_us_coast_{0};                 ///< Calculated coast-phase duration in microseconds.
    uint16_t soft_brake_pwm_{0};               ///< Current dither strength in logical input units.
    uint32_t soft_sequence_{0};                ///< Generation token invalidating stale dither callbacks.

    // ---- Software fault observer ---- //

    bool fault_latched_{false};       ///< Active or one-shot-latched software fault state.
    bool fault_pending_{false};       ///< ISR-posted work awaiting pollFaults().
    uint32_t fault_sequence_{0};      ///< Monotonic software-fault transition count.
    FaultCallback fault_cb_{nullptr}; ///< Optional task-context observer callback.
    void *fault_ctx_{nullptr};        ///< Caller-owned context passed to fault_cb_.
    int fault_irq_pin_{-1};           ///< Attached software-fault GPIO, or -1.

    // ---- Capture ---- //

    uint32_t last_edge_us_{0};      ///< micros() timestamp of the previous selected edge.
    uint32_t period_us_{0};         ///< Most recent adjacent selected-edge interval in microseconds.
    bool capture_edge_seen_{false}; ///< True after the first edge establishes a timestamp.
    uint32_t capture_sequence_{0};  ///< Monotonic completed capture-interval count.
    int cap_irq_pin_{-1};           ///< Attached capture GPIO, or -1.

    // ---- Output / operation state ---- //

    mutable portMUX_TYPE state_mux_ =
        portMUX_INITIALIZER_UNLOCKED;    ///< Recursive ISR/task state and output-commit lock.
    float last_a_percent_{-1.0f};        ///< Last successfully committed A duty percent.
    float last_b_percent_{-1.0f};        ///< Last successfully committed B duty percent.
    uint32_t output_sequence_{0};        ///< Generation token linearizing all physical output commits.
    bool commanded_enable_{false};       ///< Enable intent of the newest published output generation.
    float commanded_a_percent_{0.0f};    ///< A duty intent of the newest output generation.
    float commanded_b_percent_{0.0f};    ///< B duty intent of the newest output generation.
    uint32_t timer_sequence_{0};         ///< Generation token linearizing timer start/stop commands.
    bool commanded_timer_active_{false}; ///< Active state requested by the newest timer generation.
    int64_t commanded_timer_us_{0};      ///< One-shot interval owned by the newest timer generation.

    MotorOutputMode output_mode_{MotorOutputMode::Unconfigured}; ///< Truthful public semantic output state.
    uint32_t operation_sequence_{0};                      ///< Monotonic count of successfully completed operations.
    MotorOperation last_operation_{MotorOperation::None}; ///< Most recently attempted public operation.
    MotorOperationError last_operation_error_{MotorOperationError::None}; ///< Primary diagnosis of last operation.

    /// @brief Prevent copying an instance that owns timers, interrupts, and MCPWM state.
    HBridgeMotor(const HBridgeMotor &other) = delete;

    /// @brief Prevent assignment between instances that own hardware resources.
    HBridgeMotor &operator=(const HBridgeMotor &other) = delete;
};
