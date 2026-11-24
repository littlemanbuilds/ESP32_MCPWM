#pragma once

#include <mock_hal.h>
#include <soc/gpio_sig_map.h>

inline void esp_rom_gpio_connect_out_signal(uint32_t pin, uint32_t signal, bool, bool)
{
    const int gpio = static_cast<int>(pin);
    mock_hal::events.push_back({mock_hal::EventKind::PwmMatrix, gpio, static_cast<int>(signal)});
    if (signal != SIG_GPIO_OUT_IDX)
    {
        const auto unit = signal < PWM1_OUT0A_IDX ? MCPWM_UNIT_0 : MCPWM_UNIT_1;
        const uint32_t base = unit == MCPWM_UNIT_0 ? PWM0_OUT0A_IDX : PWM1_OUT0A_IDX;
        const uint32_t offset = signal - base;
        const auto &pwm = mock_hal::pwm_timers[unit][offset / 2U];
        if (pwm.highTicksForCompare(pwm.active_compare[offset % 2U]) != 0U)
            ++mock_hal::unsafe_routes;
        mock_hal::pwm_routes[gpio] = {unit, static_cast<mcpwm_io_signals_t>(offset)};
        return;
    }
    const auto erased = mock_hal::pwm_routes.erase(gpio);
    if (erased != 0U && mock_hal::observed_enable_pin >= 0 &&
        mock_hal::pin_levels[mock_hal::observed_enable_pin] == HIGH)
        ++mock_hal::remaps_while_enabled;
}
