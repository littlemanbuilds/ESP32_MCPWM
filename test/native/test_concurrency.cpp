/**
 * MIT License
 *
 * @brief Host concurrency stress for ISR-to-task shared state.
 *
 * @file test_concurrency.cpp
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-08-07
 * @copyright Copyright (c) 2026 Little Man Builds
 */

#include <HBridgeMotor.h>
#include <mock_hal.h>
#include <atomic>
#include <cstdint>
#include <iostream>
#include <thread>

namespace
{
MotorMCPWMConfig hardware()
{
    return MotorMCPWMConfig{2, 3, 8, MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM0A, MCPWM0B};
}
}

int main()
{
    mock_hal::reset();

    MotorSafetyConfig safety{};
    safety.fault_gpio = 10;
    safety.oneshot = false;
    safety.fault_action = FaultAction::DisableOutputs;

    MotorCaptureConfig capture{};
    capture.cap_gpio = 11;
    capture.edge = CaptureEdge::Both;

    HBridgeMotor motor;
    if (!motor.setup(hardware(), MotorBehaviorConfig{}, safety, capture).ok())
        return 1;

    std::atomic<bool> done{false};
    std::atomic<uint32_t> observations{0};

    std::thread isr_thread([&]() {
        for (uint32_t i = 1; i <= 50000U; ++i)
        {
            mock_hal::pin_levels[10] = (i & 1U) ? HIGH : LOW;
            mock_hal::invokeInterrupt(10);
            mock_hal::micros_value = i * 7U;
            mock_hal::invokeInterrupt(11);
        }
        done.store(true, std::memory_order_release);
    });

    std::thread task_thread([&]() {
        uint32_t last_fault_sequence = 0;
        uint32_t last_capture_sequence = 0;
        while (!done.load(std::memory_order_acquire))
        {
            const MotorDriverStatus status = motor.status();
            if (status.fault_sequence < last_fault_sequence ||
                status.capture_sequence < last_capture_sequence)
                std::abort();
            last_fault_sequence = status.fault_sequence;
            last_capture_sequence = status.capture_sequence;
            (void)motor.getLastCapturePeriodUs();
            ++observations;
        }
    });

    isr_thread.join();
    task_thread.join();

    if (observations.load() == 0U)
        return 2;

    std::cout << "[PASS] synchronized ISR/task status stress: "
              << observations.load() << " observations\n";
    return 0;
}
