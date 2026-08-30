/**
 * MIT License
 *
 * @brief Compile/runtime contract checks for custom IMotorDriver implementations.
 *
 * @file test_interface.cpp
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-08-14
 * @copyright Copyright (c) 2026 Little Man Builds
 */

#include <IMotorDriver.h>
#include <cstdlib>
#include <iostream>
#include <type_traits>

#define CHECK(expr)             \
    do                          \
    {                           \
        if (!(expr))            \
            std::abort();       \
    } while (false)

class MissingLifecycleDriver : public IMotorDriver
{
public:
    MotorSetupResult setup(const MotorMCPWMConfig &) override { return {}; }
    bool isSetupComplete() const noexcept override { return true; }
    MotorSetupError getLastSetupError() const noexcept override { return MotorSetupError::None; }
    MotorOperationResult drive(int, Dir) noexcept override { return {}; }
    MotorOperationResult drivePercent(float, Dir) noexcept override { return {}; }
    MotorOperationResult coast() noexcept override { return {}; }
    MotorOperationResult setHardBrake() noexcept override { return {}; }
    int getMaxPwmInput() const noexcept override { return 255; }
};

static_assert(std::is_abstract<MissingLifecycleDriver>::value,
              "custom drivers must implement start() and stop()");

class MinimalDriver : public IMotorDriver
{
public:
    using IMotorDriver::setup;

    MotorSetupResult setup(const MotorMCPWMConfig &) override
    {
        setup_complete_ = true;
        return {};
    }
    bool isSetupComplete() const noexcept override { return setup_complete_; }
    MotorSetupError getLastSetupError() const noexcept override { return MotorSetupError::None; }
    MotorOperationResult drive(int, Dir) noexcept override
    {
        return {MotorOperation::Drive, MotorOperationError::None, true, 1};
    }
    MotorOperationResult drivePercent(float, Dir) noexcept override
    {
        return {MotorOperation::Drive, MotorOperationError::None, true, 1};
    }
    MotorOperationResult coast() noexcept override
    {
        return {MotorOperation::Coast, MotorOperationError::None, true, 1};
    }
    MotorOperationResult setHardBrake() noexcept override
    {
        return {MotorOperation::HardBrake, MotorOperationError::None, true, 1};
    }
    int getMaxPwmInput() const noexcept override { return 255; }
    MotorOperationResult start() noexcept override
    {
        return {MotorOperation::Start, MotorOperationError::None, true, 1};
    }
    MotorOperationResult stop() noexcept override
    {
        return {MotorOperation::Stop, MotorOperationError::None, true, 1};
    }

private:
    bool setup_complete_{false};
};

int main()
{
    MinimalDriver driver;
    MotorMCPWMConfig hardware{};
    MotorBehaviorConfig behavior{};
    MotorSafetyConfig safety{};
    MotorCaptureConfig capture{};
    MotorHardwareFaultConfig hardware_fault{};

    CHECK(driver.setup(hardware, behavior).ok());
    CHECK(driver.setup(hardware, behavior, safety, capture).ok());
    CHECK(driver.setup(hardware, behavior, safety, capture, hardware_fault).ok());

    behavior.freewheel_mode = FreewheelMode::HiZ_Awake;
    CHECK(driver.setup(hardware, behavior).error == MotorSetupError::Unsupported);
    behavior = MotorBehaviorConfig{};
    safety.fault_gpio = 10;
    CHECK(driver.setup(hardware, behavior, safety, capture).error ==
          MotorSetupError::Unsupported);
    safety = MotorSafetyConfig{};
    capture.cap_gpio = 11;
    CHECK(driver.setup(hardware, behavior, safety, capture).error ==
          MotorSetupError::Unsupported);
    capture = MotorCaptureConfig{};
    hardware_fault.mode = HardwareFaultMode::OneShot;
    hardware_fault.fault_gpio = 12;
    CHECK(driver.setup(hardware, behavior, safety, capture, hardware_fault).error ==
          MotorSetupError::Unsupported);

    CHECK(driver.setSoftBrakePWM(10).error == MotorOperationError::Unsupported);
    CHECK(driver.pollFaults().error == MotorOperationError::Unsupported);
    CHECK(driver.setFreewheelMode(FreewheelMode::HiZ).error ==
          MotorOperationError::Unsupported);
    CHECK(driver.clearFault().error == MotorOperationError::Unsupported);
    CHECK(driver.reconfigureFrequency(1000).error == MotorOperationError::Unsupported);

    std::cout << "[PASS] truthful custom-driver interface defaults\n";
    return 0;
}
