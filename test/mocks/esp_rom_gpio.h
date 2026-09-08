#pragma once

#include <mock_hal.h>
#include <soc/gpio_sig_map.h>

inline void esp_rom_gpio_connect_out_signal(uint32_t pin, uint32_t signal, bool, bool)
{
    if (signal != SIG_GPIO_OUT_IDX)
        return;
    const auto erased = mock_hal::pwm_routes.erase(static_cast<int>(pin));
    if (erased != 0U && mock_hal::observed_enable_pin >= 0 &&
        mock_hal::pin_levels[mock_hal::observed_enable_pin] == HIGH)
        ++mock_hal::remaps_while_enabled;
}
