/**
 * MIT License
 *
 * @brief Storage for deterministic ESP32_MCPWM host mocks.
 *
 * @file mock_hal.cpp
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-08-07
 * @copyright Copyright (c) 2026 Little Man Builds
 */

#include <mock_hal.h>

namespace mock_hal
{
std::vector<Event> events;
std::unordered_map<int, int> pin_modes;
std::unordered_map<int, int> pin_levels;
std::unordered_map<int, Interrupt> interrupts;
std::array<std::array<std::array<float, 2>, 3>, 2> duties{};
std::array<std::array<bool, 3>, 2> running{};
std::array<std::array<uint32_t, 3>, 2> frequencies{};
std::vector<MockEspTimer *> timers;
MockEspTimer *last_timer = nullptr;
uint32_t micros_value = 0;
int timer_create_count = 0;
esp_err_t gpio_init_result = ESP_OK;
esp_err_t mcpwm_init_result = ESP_OK;
esp_err_t frequency_result = ESP_OK;
esp_err_t duty_result = ESP_OK;
std::vector<esp_err_t> duty_results;
esp_err_t deadtime_disable_result = ESP_OK;
esp_err_t start_result = ESP_OK;
esp_err_t stop_result = ESP_OK;
esp_err_t fault_init_result = ESP_OK;
esp_err_t fault_mode_result = ESP_OK;
esp_err_t fault_deinit_result = ESP_OK;
esp_err_t timer_create_result = ESP_OK;
esp_err_t timer_start_result = ESP_OK;
void (*duty_hook)() = nullptr;
void (*timer_start_hook)() = nullptr;
} // namespace mock_hal
