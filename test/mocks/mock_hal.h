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
#include <mutex>
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
    MCPWM2B,
    MCPWM_SYNC_0,
    MCPWM_SYNC_1,
    MCPWM_SYNC_2,
    MCPWM_FAULT_0,
    MCPWM_FAULT_1,
    MCPWM_FAULT_2
};

enum mcpwm_fault_signal_t
{
    MCPWM_SELECT_F0,
    MCPWM_SELECT_F1,
    MCPWM_SELECT_F2
};

enum mcpwm_fault_input_level_t
{
    MCPWM_LOW_LEVEL_TGR,
    MCPWM_HIGH_LEVEL_TGR
};

enum mcpwm_output_action_t
{
    MCPWM_NO_CHANGE_IN_MCPWMXA = 0,
    MCPWM_FORCE_MCPWMXA_LOW = 1,
    MCPWM_FORCE_MCPWMXA_HIGH = 2,
    MCPWM_NO_CHANGE_IN_MCPWMXB = 0,
    MCPWM_FORCE_MCPWMXB_LOW = 1,
    MCPWM_FORCE_MCPWMXB_HIGH = 2
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
    void (*callback)(void *);
    void *arg;
    bool active;
    int64_t timeout_us;

    MockEspTimer(void (*cb)(void *) = nullptr, void *context = nullptr,
                 bool is_active = false, int64_t timeout = 0)
        : callback(cb), arg(context), active(is_active), timeout_us(timeout) {}
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
    DetachInterrupt,
    HardwareFaultInit,
    HardwareFaultMode,
    HardwareFaultDeinit
};

struct Event
{
    EventKind kind;
    int first;
    int second;
    float value;

    Event(EventKind event_kind, int first_value = 0, int second_value = 0,
          float event_value = 0.0f)
        : kind(event_kind), first(first_value), second(second_value), value(event_value) {}
};

struct Interrupt
{
    void (*handler)(void *);
    void *arg;
    int mode;

    Interrupt(void (*fn)(void *) = nullptr, void *context = nullptr, int interrupt_mode = 0)
        : handler(fn), arg(context), mode(interrupt_mode) {}
};

extern std::vector<Event> events;
extern std::unordered_map<int, int> pin_modes;
extern std::unordered_map<int, int> pin_levels;
extern std::unordered_map<int, Interrupt> interrupts;
extern std::array<std::array<std::array<float, 2>, 3>, 2> duties;
extern std::array<std::array<bool, 3>, 2> running;
extern std::array<std::array<uint32_t, 3>, 2> frequencies;
extern std::vector<MockEspTimer *> timers;
extern MockEspTimer *last_timer;
extern uint32_t micros_value;
extern int timer_create_count;
extern esp_err_t gpio_init_result;
extern esp_err_t mcpwm_init_result;
extern esp_err_t frequency_result;
extern esp_err_t duty_result;
extern std::vector<esp_err_t> duty_results;
extern esp_err_t deadtime_disable_result;
extern esp_err_t start_result;
extern esp_err_t stop_result;
extern esp_err_t fault_init_result;
extern esp_err_t fault_mode_result;
extern esp_err_t fault_deinit_result;
extern esp_err_t timer_create_result;
extern esp_err_t timer_start_result;
extern void (*duty_hook)();
extern void (*timer_start_hook)();

inline void reset()
{
    events.clear();
    pin_modes.clear();
    pin_levels.clear();
    interrupts.clear();
    duties = {};
    running = {};
    frequencies = {};
    timers.clear();
    last_timer = nullptr;
    micros_value = 0;
    timer_create_count = 0;
    gpio_init_result = ESP_OK;
    mcpwm_init_result = ESP_OK;
    frequency_result = ESP_OK;
    duty_result = ESP_OK;
    duty_results.clear();
    deadtime_disable_result = ESP_OK;
    start_result = ESP_OK;
    stop_result = ESP_OK;
    fault_init_result = ESP_OK;
    fault_mode_result = ESP_OK;
    fault_deinit_result = ESP_OK;
    timer_create_result = ESP_OK;
    timer_start_result = ESP_OK;
    duty_hook = nullptr;
    timer_start_hook = nullptr;
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
    mock_hal::frequencies[unit][timer] = static_cast<uint32_t>(config->frequency);
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
    return mock_hal::deadtime_disable_result;
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
    esp_err_t result = mock_hal::duty_result;
    if (!mock_hal::duty_results.empty())
    {
        result = mock_hal::duty_results.front();
        mock_hal::duty_results.erase(mock_hal::duty_results.begin());
    }
    if (result != ESP_OK)
        return result;
    mock_hal::duties[unit][timer][op] = duty;
    mock_hal::events.push_back({mock_hal::EventKind::Duty, op, timer, duty});
    if (mock_hal::duty_hook)
    {
        void (*hook)() = mock_hal::duty_hook;
        mock_hal::duty_hook = nullptr;
        hook();
    }
    return ESP_OK;
}

inline esp_err_t mcpwm_start(mcpwm_unit_t unit, mcpwm_timer_t timer)
{
    if (mock_hal::start_result != ESP_OK)
        return mock_hal::start_result;
    mock_hal::running[unit][timer] = true;
    mock_hal::events.push_back({mock_hal::EventKind::McpwmStart, unit, timer});
    return ESP_OK;
}

inline esp_err_t mcpwm_stop(mcpwm_unit_t unit, mcpwm_timer_t timer)
{
    if (mock_hal::stop_result != ESP_OK)
        return mock_hal::stop_result;
    mock_hal::running[unit][timer] = false;
    mock_hal::events.push_back({mock_hal::EventKind::McpwmStop, unit, timer});
    return ESP_OK;
}

inline esp_err_t mcpwm_set_frequency(mcpwm_unit_t unit, mcpwm_timer_t timer, int frequency)
{
    if (mock_hal::frequency_result != ESP_OK)
        return mock_hal::frequency_result;
    mock_hal::frequencies[unit][timer] = static_cast<uint32_t>(frequency);
    return ESP_OK;
}

inline uint32_t mcpwm_get_frequency(mcpwm_unit_t unit, mcpwm_timer_t timer)
{
    return mock_hal::frequencies[unit][timer];
}

inline float mcpwm_get_duty(mcpwm_unit_t unit, mcpwm_timer_t timer, mcpwm_operator_t op)
{
    return mock_hal::duties[unit][timer][op];
}

inline esp_err_t mcpwm_fault_init(mcpwm_unit_t unit, mcpwm_fault_input_level_t level,
                                  mcpwm_fault_signal_t signal)
{
    mock_hal::events.push_back({mock_hal::EventKind::HardwareFaultInit, unit, signal,
                                static_cast<float>(level)});
    return mock_hal::fault_init_result;
}

inline esp_err_t mcpwm_fault_set_oneshot_mode(mcpwm_unit_t unit, mcpwm_timer_t timer,
                                               mcpwm_fault_signal_t signal,
                                               mcpwm_output_action_t action_a,
                                               mcpwm_output_action_t action_b)
{
    (void)signal;
    mock_hal::events.push_back({mock_hal::EventKind::HardwareFaultMode, unit, timer,
                                static_cast<float>(action_a * 10 + action_b)});
    return mock_hal::fault_mode_result;
}

inline esp_err_t mcpwm_fault_set_cyc_mode(mcpwm_unit_t unit, mcpwm_timer_t timer,
                                           mcpwm_fault_signal_t signal,
                                           mcpwm_output_action_t action_a,
                                           mcpwm_output_action_t action_b)
{
    (void)signal;
    mock_hal::events.push_back({mock_hal::EventKind::HardwareFaultMode, unit, timer,
                                static_cast<float>(100 + action_a * 10 + action_b)});
    return mock_hal::fault_mode_result;
}

inline esp_err_t mcpwm_fault_deinit(mcpwm_unit_t unit, mcpwm_fault_signal_t signal)
{
    mock_hal::events.push_back({mock_hal::EventKind::HardwareFaultDeinit, unit, signal});
    return mock_hal::fault_deinit_result;
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
    if (mock_hal::timer_start_hook)
    {
        void (*hook)() = mock_hal::timer_start_hook;
        mock_hal::timer_start_hook = nullptr;
        hook();
    }
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

struct portMUX_TYPE
{
    std::recursive_mutex mutex;
};
#define portMUX_INITIALIZER_UNLOCKED {}
inline void portENTER_CRITICAL(portMUX_TYPE *mux) { mux->mutex.lock(); }
inline void portEXIT_CRITICAL(portMUX_TYPE *mux) { mux->mutex.unlock(); }
inline void portENTER_CRITICAL_ISR(portMUX_TYPE *mux) { mux->mutex.lock(); }
inline void portEXIT_CRITICAL_ISR(portMUX_TYPE *mux) { mux->mutex.unlock(); }
