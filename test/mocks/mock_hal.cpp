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
std::unordered_map<int, std::pair<bool, bool>> pin_pulls;
std::unordered_map<int, int> pin_levels;
std::unordered_map<int, Interrupt> interrupts;
std::array<std::array<std::array<float, 2>, 3>, 2> duties{};
std::array<std::array<bool, 3>, 2> running{};
std::array<std::array<PwmTimer, 3>, 2> pwm_timers{};
std::unordered_map<int, std::pair<mcpwm_unit_t, mcpwm_io_signals_t>> pwm_routes;
unsigned live_period_writes = 0;
unsigned active_reinitializations = 0;
unsigned unsafe_routes = 0;
unsigned remaps_while_enabled = 0;
int observed_enable_pin = -1;
bool suppress_timer_stop = false;
bool suppress_timer_empty = false;
void (*pwm_wait_hook)() = nullptr;
std::vector<MockEspTimer *> timers;
MockEspTimer *last_timer = nullptr;
uint32_t micros_value = 0;
int timer_create_count = 0;
int gpio_release_failure_pin = -1;
esp_err_t gpio_level_result = ESP_OK;
uint32_t deadtime_red = 0;
uint32_t deadtime_fed = 0;
esp_err_t gpio_init_result = ESP_OK;
esp_err_t mcpwm_init_result = ESP_OK;
esp_err_t frequency_result = ESP_OK;
esp_err_t resolution_result = ESP_OK;
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
void (*fault_mode_hook)() = nullptr;
void (*start_hook)() = nullptr;
void (*duty_hook)() = nullptr;
void (*timer_start_hook)() = nullptr;
} // namespace mock_hal
