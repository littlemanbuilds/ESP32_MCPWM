/**
 * MIT License
 *
 * @brief Shared ESP32 MCPWM motor-driver interfaces and configuration types.
 *
 * @file IMotorDriver.h
 * @author Little Man Builds (Darren Osborne)
 * @date 2025-08-28
 * @copyright Copyright (c) 2026 Little Man Builds
 *
 */

#pragma once

#include <driver/mcpwm.h>

#if defined(__has_include)
#if __has_include(<soc/soc_caps.h>)
#include <soc/soc_caps.h>
#endif
#endif

#include <cstdint>

#if defined(SOC_MCPWM_SUPPORTED) && !SOC_MCPWM_SUPPORTED
#error "ESP32_MCPWM requires an ESP32-family SoC where Arduino-ESP32 exposes the legacy MCPWM driver."
#endif

#if defined(SOC_MCPWM_GROUPS) && (SOC_MCPWM_GROUPS == 0)
#error "ESP32_MCPWM requires at least one MCPWM peripheral group."
#endif

#if __cplusplus >= 201703L
#define ESP32_MCPWM_NODISCARD [[nodiscard]]
#else
#define ESP32_MCPWM_NODISCARD
#endif

// ---- Core hardware configuration ---- //

/**
 * @brief Hardware configuration for an MCPWM-driven H-bridge motor.
 *
 * HBridgeMotor validates the timer/A/B mapping and reserves resources against
 * other HBridgeMotor instances. Direct ESP-IDF users must coordinate separately.
 * Hardware faults require exclusive unit use because legacy deinit clears all operators.
 */
struct MotorMCPWMConfig
{
    int lpwm_pin = -1; ///< GPIO for LPWM (A/IN1).
    int rpwm_pin = -1; ///< GPIO for RPWM (B/IN2).
    int en_pin = -1;   ///< Enable pin; -1 if unused.

    mcpwm_unit_t unit = MCPWM_UNIT_0;    ///< MCPWM unit.
    mcpwm_timer_t timer = MCPWM_TIMER_0; ///< MCPWM timer.
    mcpwm_io_signals_t sig_l = MCPWM0A;  ///< MCPWM signal for LPWM.
    mcpwm_io_signals_t sig_r = MCPWM0B;  ///< MCPWM signal for RPWM.

    int pwm_freq_hz = 20000;                         ///< Requested drive waveform frequency in Hz.
    int input_max = 1023;                            ///< Maximum logical drive input.
    mcpwm_counter_type_t counter = MCPWM_UP_COUNTER; ///< MCPWM counter mode.

    bool use_deadtime = false;                                               ///< Enable MCPWM dead-time insertion.
    mcpwm_deadtime_type_t deadtime_type = MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE; ///< Dead-time mode.
    uint32_t deadtime_red_ns = 500;                                          ///< Legacy RED count 0..65535 passed unchanged; suffix retained for compatibility.
    uint32_t deadtime_fed_ns = 500;                                          ///< Legacy FED count 0..65535 passed unchanged; suffix retained for compatibility.

    /// @brief Construct an unassigned hardware configuration.
    MotorMCPWMConfig() = default;

    /**
     * @brief Construct a hardware configuration with MCPWM routing.
     *
     * @param lp GPIO for LPWM.
     * @param rp GPIO for RPWM.
     * @param en GPIO for EN, or -1 when unused.
     * @param u MCPWM unit.
     * @param t MCPWM timer.
     * @param sl MCPWM signal routed to LPWM.
     * @param sr MCPWM signal routed to RPWM.
     * @param freq_hz Drive PWM frequency in Hz.
     * @param in_max Maximum logical PWM input.
     */
    MotorMCPWMConfig(int lp, int rp, int en, mcpwm_unit_t u, mcpwm_timer_t t, mcpwm_io_signals_t sl,
                     mcpwm_io_signals_t sr, int freq_hz = 20000, int in_max = 1023)
        : lpwm_pin(lp), rpwm_pin(rp), en_pin(en), unit(u), timer(t), sig_l(sl), sig_r(sr), pwm_freq_hz(freq_hz),
          input_max(in_max)
    {
    }
};

// ---- Callbacks ---- //

/**
 * @brief User callback for selected-edge interval measurements.
 *
 * Called from GPIO interrupt context. Keep the callback IRAM-safe, short, and
 * non-blocking.
 *
 * @param interval_us Measured interval in microseconds between selected edges.
 * @param user Opaque pointer supplied during registration.
 */
using CaptureCallback = void (*)(uint32_t interval_us, void *user);

/**
 * @brief Software-fault notification callback.
 *
 * Called from pollFaults() in task context after the GPIO ISR posts an update.
 * This callback is diagnostic/control-plane notification, not a hardware E-stop.
 *
 * @param active True while the software fault state is active or latched.
 * @param ctx Opaque pointer supplied during registration.
 */
using FaultCallback = void (*)(bool active, void *ctx);

// ---- Setup and operation results ---- //

/**
 * @brief Result of the most recent setup attempt.
 */
enum class MotorSetupError : uint8_t
{
    None,                          ///< Setup completed successfully.
    Unsupported,                   ///< Requested optional setup capability is not implemented.
    ContainmentFailed,             ///< Existing/partial hardware could not be safely contained.
    InvalidPwmPin,                 ///< LPWM or RPWM is not a usable output pin.
    DuplicatePwmPin,               ///< LPWM and RPWM use the same pin.
    PinConflict,                   ///< Optional pins conflict with another configured pin.
    InvalidPwmFrequency,           ///< Drive PWM frequency or counter mode is unsupported.
    InvalidInputRange,             ///< Logical PWM input range is invalid.
    InvalidDitherConfig,           ///< Dither timing cannot form a valid period.
    InvalidHardwareFaultConfig,    ///< Hardware fault configuration is inconsistent.
    UnsupportedHardwareFaultLevel, ///< Legacy MCPWM hardware fault path requires active-high input.
    HardwareInitFailed,            ///< MCPWM setup failed.
    HardwareFaultInitFailed,       ///< MCPWM peripheral fault setup failed.
    TimerInitFailed,               ///< Soft-brake timer creation failed.
    InvalidTimerSignals,           ///< Unit/timer bounds or A/B signal routing is invalid.
    ResourceConflict,              ///< Another instance owns overlapping peripheral resources or pins.
    InvalidDeadtime                ///< Legacy RED/FED count exceeds the 16-bit register range.
};

/**
 * @brief Structured setup result returned by setup().
 */
struct MotorSetupResult
{
    MotorSetupError error = MotorSetupError::None; ///< Detailed setup outcome.
    bool software_fault_enabled = false;           ///< GPIO/ISR observer configured.
    bool hardware_fault_enabled = false;           ///< MCPWM peripheral fault configured.

    /// @brief Construct a successful setup result with optional paths disabled.
    constexpr MotorSetupResult() = default;

    /**
     * @brief Construct a complete setup result.
     *
     * @param setup_error Detailed setup diagnosis.
     * @param software_enabled True when the GPIO software observer is configured.
     * @param hardware_enabled True when the MCPWM hardware-fault path is configured.
     */
    constexpr MotorSetupResult(MotorSetupError setup_error, bool software_enabled, bool hardware_enabled)
        : error(setup_error), software_fault_enabled(software_enabled), hardware_fault_enabled(hardware_enabled)
    {
    }

    /// @brief Return true when setup completed successfully.
    ///
    /// @return True when error is None; false when a failure is recorded.
    ESP32_MCPWM_NODISCARD bool ok() const noexcept
    {
        return error == MotorSetupError::None;
    }

    /// @brief Convert the result to true when setup completed successfully.
    ///
    /// @return True when error is None; false otherwise.
    operator bool() const noexcept
    {
        return ok();
    }
};

/**
 * @brief Public motor operations that can produce a structured result.
 */
enum class MotorOperation : uint8_t
{
    None,                 ///< No operation has been recorded yet.
    Drive,                ///< Positive drive request.
    Coast,                ///< Explicit configured coast/freewheel request.
    HardBrake,            ///< Explicit full electronic brake request.
    SoftBrake,            ///< Soft/dither brake configuration or activation.
    Start,                ///< Start MCPWM output generation.
    Stop,                 ///< Disable and stop MCPWM output generation.
    ReconfigureFrequency, ///< Runtime PWM-frequency change.
    PollFaults,           ///< Apply deferred software-fault work.
    ClearFault,           ///< Clear software/hardware fault state where possible.
    ForceOutputs          ///< Commissioning-only raw output request.
};

/**
 * @brief Detailed outcome of a runtime motor operation.
 */
enum class MotorOperationError : uint8_t
{
    None,                     ///< Operation completed successfully.
    Unsupported,              ///< Optional operation is unsupported by this driver or configuration.
    NotSetup,                 ///< Driver setup has not completed.
    FaultActive,              ///< A software or hardware fault currently inhibits the request.
    InvalidCommand,           ///< Request is malformed or invalid for the current state.
    HardwareWriteFailed,      ///< MCPWM duty output write failed.
    HardwareStartFailed,      ///< MCPWM start failed.
    HardwareStopFailed,       ///< MCPWM stop was rejected or did not complete within its bound.
    TimerFailed,              ///< Soft-brake timer operation failed.
    FrequencyChangeFailed,    ///< MCPWM frequency update failed.
    HardwareFaultClearFailed, ///< Hardware one-shot fault could not be re-armed.
    CommissioningDisabled     ///< Raw commissioning output API is disabled at build time.
};

/**
 * @brief Structured result for lifecycle and output-state operations.
 */
struct MotorOperationResult
{
    MotorOperation operation = MotorOperation::None;       ///< Operation that was requested.
    MotorOperationError error = MotorOperationError::None; ///< Detailed result.
    bool changed = false;                                  ///< True when public semantic/output state changed.
    uint32_t sequence = 0;                                 ///< Monotonic successful-command sequence.

    /// @brief Construct an unchanged successful result for no recorded operation.
    constexpr MotorOperationResult() = default;

    /**
     * @brief Construct a complete runtime operation result.
     *
     * @param op Operation that was attempted.
     * @param op_error Primary operation diagnosis.
     * @param did_change True only when public semantic/output state changed.
     * @param op_sequence Current successful-operation sequence.
     */
    constexpr MotorOperationResult(MotorOperation op, MotorOperationError op_error, bool did_change,
                                   uint32_t op_sequence)
        : operation(op), error(op_error), changed(did_change), sequence(op_sequence)
    {
    }

    /// @brief Return true when the requested operation completed without error.
    ///
    /// @return True when error is None; false when a failure is recorded.
    ESP32_MCPWM_NODISCARD bool ok() const noexcept
    {
        return error == MotorOperationError::None;
    }

    /// @brief Convert the result to true when the operation completed without error.
    ///
    /// @return True when error is None; false otherwise.
    operator bool() const noexcept
    {
        return ok();
    }
};

// ---- Fault handling ---- //

/**
 * @brief Low-level bridge action applied by the software-polled fault path.
 *
 * The software path depends on pollFaults() running. It is useful for observation,
 * logging, and controlled software containment, but must not be treated as the
 * only emergency power-removal mechanism.
 */
enum class FaultAction : uint8_t
{
    Coast,          ///< Remove PWM drive and deassert EN when available.
    DisableOutputs, ///< Deassert EN, drive A/B low, and stop MCPWM generation.
    HardBrake       ///< Explicit electronic brake; use only after hardware validation.
};

/**
 * @brief Optional scheduler-dependent GPIO fault observer.
 *
 * The tiny GPIO ISR only records synchronized state. pollFaults() performs the
 * configured bridge action in normal task context.
 */
struct MotorSafetyConfig
{
    int fault_gpio = -1;                                    ///< GPIO observer pin; -1 disables the software fault path.
    bool fault_active_high = true;                          ///< True when HIGH means fault active.
    bool oneshot = true;                                    ///< Latch until clearFault() when true.
    FaultAction fault_action = FaultAction::DisableOutputs; ///< Conservative default containment action.

    /// @brief Construct the default disabled software-fault configuration.
    constexpr MotorSafetyConfig() = default;

    /**
     * @brief Construct a software fault-input configuration.
     *
     * @param gpio GPIO observer pin, or -1 to disable.
     * @param active_high True when HIGH asserts the fault.
     * @param one_shot True to latch until clearFault().
     * @param action Task-context action applied by pollFaults().
     */
    constexpr MotorSafetyConfig(int gpio, bool active_high, bool one_shot,
                                FaultAction action = FaultAction::DisableOutputs)
        : fault_gpio(gpio), fault_active_high(active_high), oneshot(one_shot), fault_action(action)
    {
    }
};

/**
 * @brief MCPWM peripheral fault input selection.
 */
enum class HardwareFaultInput : uint8_t
{
    Fault0, ///< MCPWM fault input F0.
    Fault1, ///< MCPWM fault input F1.
    Fault2  ///< MCPWM fault input F2.
};

/**
 * @brief Hardware fault recovery behavior supported by the legacy MCPWM driver.
 */
enum class HardwareFaultMode : uint8_t
{
    Disabled,     ///< Do not configure an MCPWM peripheral fault input.
    CycleByCycle, ///< Outputs recover automatically when the hardware fault input clears.
    OneShot       ///< Outputs remain faulted until the peripheral fault is explicitly re-armed.
};

/**
 * @brief Exact electrical output action for one MCPWM generator during a fault.
 */
enum class HardwareFaultOutputAction : uint8_t
{
    Hold,     ///< Leave this MCPWM generator unchanged.
    ForceLow, ///< Force this MCPWM generator LOW.
    ForceHigh ///< Force this MCPWM generator HIGH.
};

/**
 * @brief Optional MCPWM peripheral fault configuration.
 *
 * This is separate from MotorSafetyConfig because it is a hardware waveform
 * reaction performed by the MCPWM peripheral, not a GPIO ISR plus task poll.
 * On the legacy ESP-IDF MCPWM API used by this library, active-low triggering is
 * documented as unsupported and is therefore rejected at setup.
 */
struct MotorHardwareFaultConfig
{
    int fault_gpio = -1;                                   ///< GPIO routed directly to the selected MCPWM fault input.
    HardwareFaultInput input = HardwareFaultInput::Fault0; ///< MCPWM F0/F1/F2 input.
    HardwareFaultMode mode = HardwareFaultMode::Disabled;  ///< Peripheral fault behavior.
    bool active_high = true;                               ///< Legacy path currently requires true.
    HardwareFaultOutputAction action_a = HardwareFaultOutputAction::ForceLow; ///< Exact A output action.
    HardwareFaultOutputAction action_b = HardwareFaultOutputAction::ForceLow; ///< Exact B output action.
};

// ---- Capture ---- //

/// @brief Edge selection for GPIO edge-interval capture.
enum class CaptureEdge : uint8_t
{
    Rising,  ///< Capture on rising edges.
    Falling, ///< Capture on falling edges.
    Both     ///< Capture adjacent-edge intervals; often half-cycle for a square wave.
};

/**
 * @brief Optional GPIO edge-interval capture configuration.
 */
struct MotorCaptureConfig
{
    int cap_gpio = -1;                      ///< Capture GPIO; -1 disables capture.
    CaptureEdge edge = CaptureEdge::Rising; ///< Selected edge behavior.
    CaptureCallback on_capture = nullptr;   ///< Optional ISR callback.
    void *user = nullptr;                   ///< Opaque callback context.
};

// ---- Behavior configuration ---- //

/**
 * @brief How explicit coast/freewheel behaves.
 */
enum class FreewheelMode : uint8_t
{
    HiZ,        ///< Coast with EN low when EN is controlled by the library.
    HiZ_Awake,  ///< EN asserted with A/B=0; physical behavior is bridge-dependent.
    DitherBrake ///< Pulsed brake/coast for deliberately configured light drag.
};

/**
 * @brief Per-instance behavior configured during setup.
 */
struct MotorBehaviorConfig
{
    FreewheelMode freewheel_mode = FreewheelMode::HiZ; ///< Explicit freewheel strategy.
    int soft_brake_hz = 300;                           ///< Dither frequency in Hz.
    uint16_t dither_pwm = 30;                          ///< Dither strength used by DitherBrake freewheel mode.
    uint16_t default_soft_brake_pwm = 50;              ///< Initial explicit soft-brake strength.
    uint16_t min_phase_us = 50;                        ///< Practical minimum brake/coast phase in µs.
    bool dither_coast_hi_z = false;                    ///< Deassert EN during dither coast phases when true.

    /// @brief Construct the default motor behavior configuration.
    constexpr MotorBehaviorConfig() = default;

    /**
     * @brief Construct a motor behavior configuration.
     *
     * @param mode Freewheel strategy.
     * @param hz Soft-brake dither frequency in Hz.
     * @param dither Dither strength used by DitherBrake.
     * @param def_soft Initial explicit soft-brake strength.
     * @param min_phase Minimum requested brake/coast phase in µs.
     * @param dither_hi_z True to deassert EN during dither coast phases.
     */
    constexpr MotorBehaviorConfig(FreewheelMode mode, int hz, uint16_t dither, uint16_t def_soft = 50,
                                  uint16_t min_phase = 50, bool dither_hi_z = false)
        : freewheel_mode(mode), soft_brake_hz(hz), dither_pwm(dither), default_soft_brake_pwm(def_soft),
          min_phase_us(min_phase), dither_coast_hi_z(dither_hi_z)
    {
    }
};

// ---- Runtime state ---- //

/// @brief Motor direction.
enum class Dir : uint8_t
{
    CW, ///< Clockwise / forward.
    CCW ///< Counter-clockwise / reverse.
};

/**
 * @brief Semantic output state last successfully requested by the driver.
 */
enum class MotorOutputMode : uint8_t
{
    Unconfigured,     ///< Setup has not completed.
    Disabled,         ///< MCPWM generation stopped and bridge disabled where possible.
    Coast,            ///< Explicit configured freewheel/coast state.
    DriveCW,          ///< Positive clockwise drive.
    DriveCCW,         ///< Positive counter-clockwise drive.
    DitherBrake,      ///< Explicit soft/dither braking active.
    HardBrake,        ///< Explicit full electronic brake.
    FaultContainment, ///< Software fault containment action is active.
    Uncertain,        ///< A hardware failure prevents a truthful output-state claim.
    Commissioning     ///< Raw A/B commissioning command; inspect commanded duties and EN.
};

/**
 * @brief Coherent software status snapshot.
 */
struct MotorDriverStatus
{
    bool setup_complete = false;              ///< Setup succeeded.
    bool mcpwm_running = false;               ///< Cached result of lifecycle calls.
    bool enable_control = false;              ///< EN pin is configured.
    bool enable_asserted = false;             ///< Cached EN level.
    bool software_fault_configured = false;   ///< GPIO/ISR fault observer configured.
    bool software_fault_active = false;       ///< Software fault active or latched.
    bool software_fault_pending = false;      ///< Deferred task-context fault work is pending.
    bool hardware_fault_configured = false;   ///< MCPWM peripheral fault configured.
    bool hardware_fault_input_active = false; ///< Current hardware-fault GPIO level.
    bool hardware_fault_latched = false;      ///< One-shot hardware fault has been observed and not re-armed.
    bool dither_active = false;               ///< Soft/dither brake timer state.
    MotorOutputMode output_mode = MotorOutputMode::Unconfigured; ///< Semantic output mode.
    float commanded_a_percent = 0.0f;                            ///< Latest published A duty request.
    float commanded_b_percent = 0.0f;                            ///< Latest published B duty request.
    int pwm_frequency_hz = 0;                                    ///< Configured drive frequency.
    uint32_t operation_sequence = 0;                             ///< Successful runtime operation sequence.
    uint32_t fault_sequence = 0;                                 ///< Software fault-state transition sequence.
    uint32_t hardware_fault_sequence = 0;                        ///< Hardware fault-state transition sequence.
    uint32_t capture_sequence = 0;                               ///< Successful capture measurement sequence.
    MotorOperation last_operation = MotorOperation::None;        ///< Most recent operation request.
    MotorOperationError last_error = MotorOperationError::None;  ///< Most recent operation result.
    bool recovery_inhibited = false;                            ///< Quiet recovery is pending, in progress or failed.
};

/**
 * @brief On-demand MCPWM/GPIO readback where the legacy driver exposes it.
 */
struct MotorHardwareReadback
{
    bool valid = false;                       ///< Driver is configured sufficiently for readback.
    uint32_t frequency_hz = 0;                ///< Estimated waveform Hz from timer configuration; zero if unavailable.
    float duty_a_percent = 0.0f;              ///< Generator A duty derived from the compare setting.
    float duty_b_percent = 0.0f;              ///< Generator B duty derived from the compare setting.
    bool enable_control = false;              ///< EN GPIO exists.
    bool enable_asserted = false;             ///< Physical EN GPIO level when available.
    bool running_cached = false;              ///< Cached lifecycle state; the timer may be running until stop is confirmed.
    bool hardware_fault_configured = false;   ///< Peripheral fault path is configured.
    bool hardware_fault_input_active = false; ///< Current hardware-fault GPIO level.
};

// ---- Abstract interface ---- //

/**
 * @brief Abstract base class for MCPWM-style drive motors.
 *
 * @note Synchronization makes status coherent across ISR/task contexts; it does
 *       not make one actuator safe for multiple command writers. One application
 *       owner or arbitrator must issue commands for each motor instance.
 */
class IMotorDriver
{
  public:
    /// @brief Destroy the abstract motor driver.
    virtual ~IMotorDriver() noexcept = default;

    /**
     * @brief Configure the driver's mandatory hardware resources.
     *
     * @param hw Hardware configuration understood by the driver.
     * @return Structured setup result.
     */
    virtual MotorSetupResult setup(const MotorMCPWMConfig &hw) = 0;

    /**
     * @brief Configure hardware and optional motor behavior.
     *
     * The default implementation accepts only the default behavior contract.
     * A non-default request is rejected rather than silently discarded.
     *
     * @param hw Hardware configuration understood by the driver.
     * @param beh Requested motor behavior.
     * @return Structured setup result.
     */
    virtual MotorSetupResult setup(const MotorMCPWMConfig &hw, const MotorBehaviorConfig &beh)
    {
        const MotorBehaviorConfig defaults{};
        if (beh.freewheel_mode != defaults.freewheel_mode || beh.soft_brake_hz != defaults.soft_brake_hz ||
            beh.dither_pwm != defaults.dither_pwm || beh.default_soft_brake_pwm != defaults.default_soft_brake_pwm ||
            beh.min_phase_us != defaults.min_phase_us || beh.dither_coast_hi_z != defaults.dither_coast_hi_z)
            return {MotorSetupError::Unsupported, false, false};
        return setup(hw);
    }

    /**
     * @brief Configure hardware plus optional software fault and capture paths.
     *
     * @note The inherited implementation supports only disabled optional paths.
     *
     * @param hw Hardware configuration understood by the driver.
     * @param beh Requested motor behavior.
     * @param sfty Optional software-fault observer configuration.
     * @param cap Optional capture configuration.
     * @return Structured setup result.
     */
    virtual MotorSetupResult setup(const MotorMCPWMConfig &hw, const MotorBehaviorConfig &beh,
                                   const MotorSafetyConfig &sfty, const MotorCaptureConfig &cap)
    {
        if (sfty.fault_gpio >= 0 || cap.cap_gpio >= 0)
            return {MotorSetupError::Unsupported, false, false};
        return setup(hw, beh);
    }

    /**
     * @brief Configure all standard and optional motor-driver paths.
     *
     * @note The inherited implementation supports only a disabled MCPWM fault path.
     *
     * @param hw Hardware configuration understood by the driver.
     * @param beh Requested motor behavior.
     * @param sfty Optional software-fault observer configuration.
     * @param cap Optional capture configuration.
     * @param hardware_fault Optional peripheral hardware-fault configuration.
     * @return Structured setup result.
     */
    virtual MotorSetupResult setup(const MotorMCPWMConfig &hw, const MotorBehaviorConfig &beh,
                                   const MotorSafetyConfig &sfty, const MotorCaptureConfig &cap,
                                   const MotorHardwareFaultConfig &hardware_fault)
    {
        if (hardware_fault.mode != HardwareFaultMode::Disabled || hardware_fault.fault_gpio >= 0)
            return {MotorSetupError::Unsupported, false, false};
        return setup(hw, beh, sfty, cap);
    }

    /// @brief Return whether setup completed successfully.
    ///
    /// @return Driver's retained setup-complete state; a failed replacement teardown may preserve an earlier setup.
    ESP32_MCPWM_NODISCARD virtual bool isSetupComplete() const noexcept = 0;

    /// @brief Return the most recent setup error.
    ///
    /// @return Most recent setup or containment error, or None when no setup error is recorded.
    ESP32_MCPWM_NODISCARD virtual MotorSetupError getLastSetupError() const noexcept = 0;

    /**
     * @brief Apply positive drive demand.
     *
     * @param speed Positive duty request in [1, getMaxPwmInput()].
     * @param dir Desired direction.
     * @return Structured operation result.
     */
    virtual MotorOperationResult drive(int speed, Dir dir) noexcept = 0;

    /**
     * @brief Apply positive drive demand as a percentage.
     *
     * @param percent Positive request in (0,100].
     * @param dir Desired direction.
     * @return Structured operation result.
     */
    virtual MotorOperationResult drivePercent(float percent, Dir dir) noexcept = 0;

    /**
     * @brief Compatibility drive API.
     *
     * v2 rejects zero because a numeric drive request no longer chooses a physical
     * stop mode. Use coast(), stop(), setHardBrake(), or softBrakeNow() explicitly.
     *
     * @param speed Positive logical drive demand; HBridgeMotor clamps values above getMaxPwmInput().
     * @param dir Requested motor direction.
     * @return Drive result; nonpositive demand is rejected as InvalidCommand.
     */
    virtual MotorOperationResult setSpeed(int speed, Dir dir) noexcept
    {
        return (speed <= 0) ? MotorOperationResult{MotorOperation::Drive, MotorOperationError::InvalidCommand, false, 0}
                            : drive(speed, dir);
    }

    /// @brief Compatibility percentage API; zero is rejected in v2.
    ///
    /// @param percent Positive drive percentage; HBridgeMotor clamps finite values above 100.
    /// @param dir Requested motor direction.
    /// @return Drive result; nonpositive or nonfinite demand is rejected by HBridgeMotor.
    virtual MotorOperationResult setSpeedPercent(float percent, Dir dir) noexcept
    {
        return (percent <= 0.0f)
                   ? MotorOperationResult{MotorOperation::Drive, MotorOperationError::InvalidCommand, false, 0}
                   : drivePercent(percent, dir);
    }

    /// @brief Enter the configured explicit coast/freewheel state.
    ///
    /// @return Coast result, including setup, fault, output-write or dither failures.
    virtual MotorOperationResult coast() noexcept = 0;

    /**
     * @brief Deassert the bridge where possible and stop MCPWM generation.
     *
     * @return Structured lifecycle result from stop().
     */
    virtual MotorOperationResult disableOutputs() noexcept
    {
        return stop();
    }

    /// @brief Compatibility alias for coast().
    ///
    /// @return Result of the explicit configured coast request.
    virtual MotorOperationResult setFreewheel() noexcept
    {
        return coast();
    }

    /// @brief Apply an explicit, hardware-validated full electronic brake.
    ///
    /// @return Brake result; failure means the requested brake state was not established.
    virtual MotorOperationResult setHardBrake() noexcept = 0;

    /// @brief Configure the soft-brake PWM level without implicitly starting it.
    ///
    /// @param pwm Logical brake level, clamped by HBridgeMotor to [0, getMaxPwmInput()].
    /// @return Unsupported in the base implementation; concrete drivers may configure braking.
    virtual MotorOperationResult setSoftBrakePWM(uint16_t pwm) noexcept
    {
        (void)pwm;
        return {MotorOperation::SoftBrake, MotorOperationError::Unsupported, false, 0};
    }

    /// @brief Process deferred software-fault state in task context.
    ///
    /// @return Unsupported in the base implementation; concrete drivers may process deferred faults.
    virtual MotorOperationResult pollFaults() noexcept
    {
        return {MotorOperation::PollFaults, MotorOperationError::Unsupported, false, 0};
    }

    /// @brief Return the maximum accepted logical drive input.
    ///
    /// @return Configured positive upper bound for logical drive demand.
    ESP32_MCPWM_NODISCARD virtual int getMaxPwmInput() const noexcept = 0;

    /**
     * @brief Change the stored freewheel behavior when the driver supports it.
     *
     * @param mode Requested explicit coast behavior.
     * @return Unsupported unless the concrete driver implements this option.
     */
    virtual MotorOperationResult setFreewheelMode(FreewheelMode mode) noexcept
    {
        (void)mode;
        return {MotorOperation::Coast, MotorOperationError::Unsupported, false, 0};
    }

    /**
     * @brief Change freewheel behavior and immediately request coast.
     *
     * @param mode Requested explicit coast behavior.
     * @return Configuration failure or the resulting coast operation result.
     */
    virtual MotorOperationResult applyFreewheel(FreewheelMode mode) noexcept
    {
        const MotorOperationResult config = setFreewheelMode(mode);
        return config.ok() ? coast() : config;
    }

    /// @brief Start the driver's output-generation lifecycle.
    ///
    /// @return Lifecycle result; success establishes configured coast/dither, never restores prior drive.
    virtual MotorOperationResult start() noexcept = 0;

    /// @brief Stop and contain the driver's output-generation lifecycle.
    ///
    /// @return Lifecycle result; success means zero duty was written and MCPWM was stopped.
    virtual MotorOperationResult stop() noexcept = 0;

    /**
     * @brief Change drive PWM frequency when the concrete driver supports it.
     *
     * @param new_hz Requested PWM frequency in Hz.
     * @return Unsupported by the inherited optional implementation.
     */
    virtual MotorOperationResult reconfigureFrequency(int new_hz) noexcept
    {
        (void)new_hz;
        return {MotorOperation::ReconfigureFrequency, MotorOperationError::Unsupported, false, 0};
    }

    /// @brief Return the latest captured edge interval, or zero when unsupported/unavailable.
    ///
    /// @return Zero in the base implementation; concrete drivers return the selected-edge interval in microseconds.
    ESP32_MCPWM_NODISCARD virtual uint32_t getLastCapturePeriodUs() const noexcept
    {
        return 0;
    }

    /// @brief Return whether a fault or unresolved recovery inhibits energizing commands.
    ///
    /// @return False in the base implementation; concrete drivers report active or latched faults.
    ESP32_MCPWM_NODISCARD virtual bool hasFault() const noexcept
    {
        return false;
    }

    /// @brief Return whether the concrete driver owns an independent EN control.
    ///
    /// @return False in the base implementation; concrete drivers report EN capability.
    ESP32_MCPWM_NODISCARD virtual bool hasEnableControl() const noexcept
    {
        return false;
    }

    /// @brief Clear inactive fault state when the concrete driver supports recovery.
    ///
    /// @return Unsupported in the base implementation; concrete drivers define fault recovery.
    virtual MotorOperationResult clearFault() noexcept
    {
        return {MotorOperation::ClearFault, MotorOperationError::Unsupported, false, 0};
    }

    /**
     * @brief Commissioning-only raw output request.
     *
     * Concrete drivers should reject this unless their explicit commissioning
     * build option is enabled.
     *
     * @param a_high True requests full A duty; false requests zero A duty.
     * @param b_high True requests full B duty; false requests zero B duty.
     * @return CommissioningDisabled by default; enabled builds return setup, fault or output-write failures.
     */
    virtual MotorOperationResult forceOutputs(bool a_high, bool b_high) noexcept
    {
        (void)a_high;
        (void)b_high;
        return {MotorOperation::ForceOutputs, MotorOperationError::CommissioningDisabled, false, 0};
    }

    /**
     * @brief Register the optional task-context software-fault callback.
     *
     * @param cb Callback to invoke from deferred fault handling.
     * @param ctx Caller-owned callback context.
     */
    virtual void setFaultCallback(FaultCallback cb, void *ctx) noexcept
    {
        (void)cb;
        (void)ctx;
    }

    /// @brief Return a coherent software status snapshot when supported.
    ///
    /// @return Default empty snapshot in the base implementation; concrete drivers supply synchronized state.
    ESP32_MCPWM_NODISCARD virtual MotorDriverStatus status() const noexcept
    {
        return {};
    }

    /// @brief Return available direct hardware/API readback when supported.
    ///
    /// @return Invalid empty readback in the base implementation; concrete drivers supply available readings.
    ESP32_MCPWM_NODISCARD virtual MotorHardwareReadback readback() const noexcept
    {
        return {};
    }

    /**
     * @brief Utility to invert direction.
     *
     * @param d Input direction.
     * @return Opposite direction.
     */
    static Dir changeDir(Dir d) noexcept
    {
        return (d == Dir::CW) ? Dir::CCW : Dir::CW;
    }
};
