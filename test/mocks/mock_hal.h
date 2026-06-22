/**
 * MIT License
 *
 * @brief Deterministic host mocks for ESP32_MCPWM regression tests.
 *
 * @file mock_hal.h
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-06-21
 * @copyright Copyright (c) 2026 Little Man Builds
 */

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <unordered_map>
#include <vector>

#define IRAM_ATTR

using esp_err_t = int;
static constexpr esp_err_t ESP_OK = 0;

#define ESP_ERROR_CHECK(expr)       \
    do                              \
    {                               \
        if ((expr) != ESP_OK)       \
            std::abort();           \
    } while (false)

enum mcpwm_unit_t
{
    MCPWM_UNIT_0,
    MCPWM_UNIT_1
};

enum mcpwm_timer_t
{
    MCPWM_TIMER_0,
    MCPWM_TIMER_1,
    MCPWM_TIMER_2
};

enum mcpwm_io_signals_t
{
    MCPWM0A,
    MCPWM0B,
    MCPWM1A,
    MCPWM1B,
    MCPWM2A,
    MCPWM2B
};

enum mcpwm_operator_t
{
    MCPWM_OPR_A,
    MCPWM_OPR_B
};

enum mcpwm_counter_type_t
{
    MCPWM_UP_COUNTER,
    MCPWM_DOWN_COUNTER,
    MCPWM_UP_DOWN_COUNTER
};

enum mcpwm_duty_type_t
{
    MCPWM_DUTY_MODE_0,
    MCPWM_DUTY_MODE_1
};

enum mcpwm_deadtime_type_t
{
    MCPWM_ACTIVE_HIGH_COMPLIMENT_MODE
};

struct mcpwm_config_t
{
    int frequency;
    float cmpr_a;
    float cmpr_b;
    mcpwm_counter_type_t counter_mode;
    mcpwm_duty_type_t duty_mode;
};

struct MockEspTimer;
using esp_timer_handle_t = MockEspTimer *;

enum esp_timer_dispatch_t
{
    ESP_TIMER_TASK
};

struct esp_timer_create_args_t
{
    void (*callback)(void *);
    void *arg;
    esp_timer_dispatch_t dispatch_method;
    const char *name;
};

struct MockEspTimer
{
    void (*callback)(void *){nullptr};
    void *arg{nullptr};
    bool active{false};
    int64_t timeout_us{0};
};

namespace mock_hal
{
enum class EventKind
{
    PinMode,
    DigitalWrite,
    McpwmGpioInit,
    McpwmInit,
    DeadtimeEnable,
    DeadtimeDisable,
    DutyType,
    Duty,
    McpwmStart,
    McpwmStop,
    TimerCreate,
    TimerStart,
    TimerStop,
    TimerDelete,
    AttachInterrupt,
    DetachInterrupt
};

struct Event
{
    EventKind kind;
    int first{0};
    int second{0};
    float value{0.0f};
};

struct Interrupt
{
    void (*handler)(void *){nullptr};
    void *arg{nullptr};
    int mode{0};
};

inline std::vector<Event> events;
inline std::unordered_map<int, int> pin_modes;
inline std::unordered_map<int, int> pin_levels;
inline std::unordered_map<int, Interrupt> interrupts;
inline std::array<std::array<std::array<float, 2>, 3>, 2> duties{};
inline std::array<std::array<bool, 3>, 2> running{};
inline std::vector<MockEspTimer *> timers;
inline MockEspTimer *last_timer{nullptr};
inline uint32_t micros_value{0};
inline int timer_create_count{0};
inline esp_err_t gpio_init_result{ESP_OK};
inline esp_err_t mcpwm_init_result{ESP_OK};
inline esp_err_t frequency_result{ESP_OK};
inline esp_err_t timer_create_result{ESP_OK};
inline esp_err_t timer_start_result{ESP_OK};
inline void (*duty_hook)(){nullptr};

inline void reset()
{
    events.clear();
    pin_modes.clear();
    pin_levels.clear();
    interrupts.clear();
    duties = {};
    running = {};
    timers.clear();
    last_timer = nullptr;
    micros_value = 0;
    timer_create_count = 0;
    gpio_init_result = ESP_OK;
    mcpwm_init_result = ESP_OK;
    frequency_result = ESP_OK;
    timer_create_result = ESP_OK;
    timer_start_result = ESP_OK;
    duty_hook = nullptr;
}

inline void invokeInterrupt(int pin)
{
    const auto it = interrupts.find(pin);
    if (it != interrupts.end() && it->second.handler)
        it->second.handler(it->second.arg);
}

inline void fireTimer(MockEspTimer *timer)
{
    if (!timer || !timer->active || !timer->callback)
        return;
    timer->active = false;
    timer->callback(timer->arg);
}
} // namespace mock_hal

static constexpr int LOW = 0;
static constexpr int HIGH = 1;
static constexpr int INPUT = 0;
static constexpr int OUTPUT = 1;
static constexpr int INPUT_PULLUP = 2;
static constexpr int INPUT_PULLDOWN = 3;
static constexpr int CHANGE = 4;
static constexpr int RISING = 5;
static constexpr int FALLING = 6;

#define GPIO_IS_VALID_GPIO(pin) ((pin) >= 0 && (pin) <= 48)
#define GPIO_IS_VALID_OUTPUT_GPIO(pin) GPIO_IS_VALID_GPIO(pin)

inline void pinMode(int pin, int mode)
{
    mock_hal::pin_modes[pin] = mode;
    mock_hal::events.push_back({mock_hal::EventKind::PinMode, pin, mode});
}

inline void digitalWrite(int pin, int level)
{
    mock_hal::pin_levels[pin] = level;
    mock_hal::events.push_back({mock_hal::EventKind::DigitalWrite, pin, level});
}

inline void attachInterruptArg(int pin, void (*handler)(void *), void *arg, int mode)
{
    mock_hal::interrupts[pin] = {handler, arg, mode};
    mock_hal::events.push_back({mock_hal::EventKind::AttachInterrupt, pin, mode});
}

inline void detachInterrupt(int pin)
{
    mock_hal::interrupts.erase(pin);
    mock_hal::events.push_back({mock_hal::EventKind::DetachInterrupt, pin});
}

inline uint32_t micros() { return mock_hal::micros_value; }

inline esp_err_t mcpwm_gpio_init(mcpwm_unit_t unit, mcpwm_io_signals_t signal, int pin)
{
    mock_hal::events.push_back({mock_hal::EventKind::McpwmGpioInit, unit, signal, static_cast<float>(pin)});
    return mock_hal::gpio_init_result;
}

inline esp_err_t mcpwm_init(mcpwm_unit_t unit, mcpwm_timer_t timer, const mcpwm_config_t *config)
{
    if (mock_hal::mcpwm_init_result != ESP_OK)
        return mock_hal::mcpwm_init_result;
    mock_hal::duties[unit][timer][MCPWM_OPR_A] = config->cmpr_a;
    mock_hal::duties[unit][timer][MCPWM_OPR_B] = config->cmpr_b;
    mock_hal::running[unit][timer] = true;
    mock_hal::events.push_back({mock_hal::EventKind::McpwmInit, unit, timer,
                                static_cast<float>(config->frequency)});
    return ESP_OK;
}

inline esp_err_t mcpwm_deadtime_enable(mcpwm_unit_t unit, mcpwm_timer_t timer,
                                      mcpwm_deadtime_type_t, uint32_t, uint32_t)
{
    mock_hal::events.push_back({mock_hal::EventKind::DeadtimeEnable, unit, timer});
    return ESP_OK;
}

inline esp_err_t mcpwm_deadtime_disable(mcpwm_unit_t unit, mcpwm_timer_t timer)
{
    mock_hal::events.push_back({mock_hal::EventKind::DeadtimeDisable, unit, timer});
    return ESP_OK;
}

inline esp_err_t mcpwm_set_duty_type(mcpwm_unit_t unit, mcpwm_timer_t timer,
                                     mcpwm_operator_t op, mcpwm_duty_type_t)
{
    mock_hal::events.push_back({mock_hal::EventKind::DutyType, unit, timer, static_cast<float>(op)});
    return ESP_OK;
}

inline esp_err_t mcpwm_set_duty(mcpwm_unit_t unit, mcpwm_timer_t timer,
                                mcpwm_operator_t op, float duty)
{
    if (mock_hal::duty_hook)
    {
        void (*hook)() = mock_hal::duty_hook;
        mock_hal::duty_hook = nullptr;
        hook();
    }
    mock_hal::duties[unit][timer][op] = duty;
    mock_hal::events.push_back({mock_hal::EventKind::Duty, op, timer, duty});
    return ESP_OK;
}

inline esp_err_t mcpwm_start(mcpwm_unit_t unit, mcpwm_timer_t timer)
{
    mock_hal::running[unit][timer] = true;
    mock_hal::events.push_back({mock_hal::EventKind::McpwmStart, unit, timer});
    return ESP_OK;
}

inline esp_err_t mcpwm_stop(mcpwm_unit_t unit, mcpwm_timer_t timer)
{
    mock_hal::running[unit][timer] = false;
    mock_hal::events.push_back({mock_hal::EventKind::McpwmStop, unit, timer});
    return ESP_OK;
}

inline esp_err_t mcpwm_set_frequency(mcpwm_unit_t, mcpwm_timer_t, int)
{
    return mock_hal::frequency_result;
}

inline esp_err_t esp_timer_create(const esp_timer_create_args_t *args, esp_timer_handle_t *handle)
{
    if (mock_hal::timer_create_result != ESP_OK)
        return mock_hal::timer_create_result;
    auto *timer = new MockEspTimer{args->callback, args->arg, false, 0};
    *handle = timer;
    mock_hal::timers.push_back(timer);
    mock_hal::last_timer = timer;
    ++mock_hal::timer_create_count;
    mock_hal::events.push_back({mock_hal::EventKind::TimerCreate});
    return ESP_OK;
}

inline esp_err_t esp_timer_start_once(esp_timer_handle_t timer, int64_t timeout_us)
{
    if (mock_hal::timer_start_result != ESP_OK)
        return mock_hal::timer_start_result;
    timer->active = true;
    timer->timeout_us = timeout_us;
    mock_hal::events.push_back({mock_hal::EventKind::TimerStart, 0, 0, static_cast<float>(timeout_us)});
    return ESP_OK;
}

inline esp_err_t esp_timer_stop(esp_timer_handle_t timer)
{
    if (timer)
        timer->active = false;
    mock_hal::events.push_back({mock_hal::EventKind::TimerStop});
    return ESP_OK;
}

inline esp_err_t esp_timer_delete(esp_timer_handle_t timer)
{
    mock_hal::timers.erase(std::remove(mock_hal::timers.begin(), mock_hal::timers.end(), timer),
                           mock_hal::timers.end());
    if (mock_hal::last_timer == timer)
        mock_hal::last_timer = nullptr;
    delete timer;
    mock_hal::events.push_back({mock_hal::EventKind::TimerDelete});
    return ESP_OK;
}

using gpio_num_t = int;
inline int gpio_get_level(gpio_num_t pin)
{
    const auto it = mock_hal::pin_levels.find(pin);
    return (it == mock_hal::pin_levels.end()) ? LOW : it->second;
}

using portMUX_TYPE = int;
#define portMUX_INITIALIZER_UNLOCKED 0
inline void portENTER_CRITICAL(portMUX_TYPE *) {}
inline void portEXIT_CRITICAL(portMUX_TYPE *) {}
