#include "trace.h"

#include <Arduino.h>
#include <ESP32_MCPWM.h>
#include <driver/gpio.h>
#include <driver/pcnt.h>
#include <esp_ota_ops.h>
#include <esp_timer.h>
#include <soc/gpio_reg.h>
#include <soc/soc.h>

namespace
{
constexpr gpio_num_t outputs[]{GPIO_NUM_15, GPIO_NUM_16, GPIO_NUM_17};
constexpr gpio_num_t inputs[]{GPIO_NUM_4, GPIO_NUM_6, GPIO_NUM_2};
constexpr gpio_num_t fault_pin = GPIO_NUM_7;  // Input with software-controlled internal pulls.
constexpr mcpwm_unit_t capture_unit = MCPWM_UNIT_1;
constexpr unsigned capacity = 96;

struct Edge
{
    uint32_t tick;
    unsigned channel;
    unsigned rising;
};

HBridgeMotor motor;
Edge edges[capacity];
portMUX_TYPE capture_lock = portMUX_INITIALIZER_UNLOCKED;
unsigned used = 0;
bool overflow = false;
bool recording = false;
uint32_t last_rise_us[2]{};
unsigned captured_route = 0;
bool ready = false;
unsigned next_id = 0;
unsigned pending_kind = 0;
bool recovery_pending = false;

bool IRAM_ATTR capture(mcpwm_unit_t, mcpwm_capture_channel_id_t channel, const cap_event_data_t *event, void *)
{
    const auto capture_channel = static_cast<unsigned>(channel);
    const unsigned rising = event->cap_edge == MCPWM_POS_EDGE;
    portENTER_CRITICAL_ISR(&capture_lock);
    if (recording)
    {
        const auto input_channel = capture_channel == 2 ? 2U : captured_route;
        if (used == capacity)
            overflow = true;
        else
            edges[used++] = {event->cap_value, input_channel, rising};
        if (rising && input_channel < 2)
            last_rise_us[input_channel] = static_cast<uint32_t>(esp_timer_get_time());
    }
    portEXIT_CRITICAL_ISR(&capture_lock);
    return false;
}

bool beginWindow()
{
    for (unsigned channel = 0; channel < 3; ++channel)
    {
        if (pcnt_counter_clear(static_cast<pcnt_unit_t>(channel)) != ESP_OK)
        {
            Serial.printf("FAIL counter_clear %u\n", channel);
            return false;
        }
    }
    portENTER_CRITICAL(&capture_lock);
    used = 0;
    overflow = false;
    last_rise_us[0] = last_rise_us[1] = 0;
    recording = true;
    portEXIT_CRITICAL(&capture_lock);
    return true;
}

bool endWindow(const char *name, unsigned id, uint32_t request_us = 0, uint32_t return_us = 0,
               int witness = -1)
{
    portENTER_CRITICAL(&capture_lock);
    recording = false;
    portEXIT_CRITICAL(&capture_lock);
    int16_t counts[3]{};
    int status[3]{};
    unsigned levels = 0;
    for (unsigned channel = 0; channel < 3; ++channel)
    {
        status[channel] = pcnt_get_counter_value(static_cast<pcnt_unit_t>(channel), &counts[channel]);
        levels |= static_cast<unsigned>(gpio_get_level(inputs[channel])) << channel;
    }
    char line[128]{};
    const int header_length = formatWindow(line, sizeof(line), name, id, used, static_cast<unsigned>(overflow),
                                           levels, request_us, return_us, counts, status);
    if (header_length < 0 || static_cast<unsigned>(header_length) >= sizeof(line))
    {
        Serial.println("FAIL window_format");
        return false;
    }
    Serial.println(line);
    for (unsigned i = 0; i < used; ++i)
    {
        const int edge_length = formatEdge(line, sizeof(line), edges[i].tick, edges[i].channel, edges[i].rising);
        if (edge_length < 0 || static_cast<unsigned>(edge_length) >= sizeof(line))
        {
            Serial.println("FAIL edge_format");
            return false;
        }
        Serial.println(line);
    }
    Serial.println("END");
    if (overflow || levels || status[0] || status[1] || status[2])
        return false;
    if (request_us != 0)
        return true;
    if (witness < 0)
        return used == 0 && counts[0] == 0 && counts[1] == 0 && counts[2] == 0;
    return used == 2 && counts[0] == (witness == 0) && counts[1] == (witness == 1) &&
           counts[2] == (witness == 2) && edges[0].channel == static_cast<unsigned>(witness) &&
           edges[1].channel == static_cast<unsigned>(witness) && edges[0].rising == 1 && edges[1].rising == 0;
}

bool operation(const char *name, MotorOperationResult result)
{
    if (result.ok())
        return true;
    Serial.printf("FAIL %s %d\n", name, static_cast<int>(result.error));
    return false;
}

bool setFaultPull(gpio_pull_mode_t mode)
{
    const esp_err_t result = gpio_set_pull_mode(fault_pin, mode);
    if (result != ESP_OK)
        Serial.printf("FAIL fault_pull %d\n", result);
    return result == ESP_OK;
}

bool selectPwmCapture(unsigned route)
{
    // A single both-edge hardware latch can lose the active rise when the
    // next falling edge arrives before the capture ISR services the first.
    // Give the active route two latches. The other input retains its own PCNT
    // witness, so any unexpected pulse still fails the scored window.
    const int levels = gpio_get_level(inputs[0]) | gpio_get_level(inputs[1]);
    const int rising = mcpwm_gpio_init(capture_unit, MCPWM_CAP_0, inputs[route]);
    const int falling = mcpwm_gpio_init(capture_unit, MCPWM_CAP_1, inputs[route]);
    portENTER_CRITICAL(&capture_lock);
    captured_route = route;
    portEXIT_CRITICAL(&capture_lock);
    Serial.printf("ROUTE %u %d %d %d\n", route, rising, falling, levels);
    return rising == ESP_OK && falling == ESP_OK && levels == 0;
}

bool transition(unsigned id, unsigned duty, unsigned kind, unsigned offset, unsigned route)
{
    if (!selectPwmCapture(route) || !beginWindow())
        return false;
    const Dir direction = route == 0 ? Dir::CW : Dir::CCW;
    if (!operation("drive", motor.drivePercent(static_cast<float>(duty), direction)))
    {
        (void)motor.stop();
        return false;
    }
    delayMicroseconds(450);
    portENTER_CRITICAL(&capture_lock);
    const uint32_t epoch = last_rise_us[route];
    portEXIT_CRITICAL(&capture_lock);
    if (epoch == 0)
    {
        (void)motor.stop();
        Serial.println("FAIL missing_active_epoch");
        return false;
    }
    while (static_cast<uint32_t>(micros()) - epoch < 75U + offset)
    {
    }

    const uint32_t request = static_cast<uint32_t>(micros());
    bool requested = false;
    if (kind == 0)
        requested = operation("coast", motor.coast());
    else if (kind == 1)
        requested = operation("stop", motor.stop());
    else
    {
        const bool pulled = setFaultPull(GPIO_PULLUP_ONLY);
        delayMicroseconds(4);
        const bool active = gpio_get_level(fault_pin) != 0 && motor.hasFault();
        if (!active)
            Serial.println("FAIL missing_fault_assertion");
        if (pulled && active)
            requested = operation("fault_poll", motor.pollFaults()) && motor.hasFault();
    }
    const uint32_t returned = static_cast<uint32_t>(micros());
    if (!requested)
    {
        (void)motor.stop();
        return false;
    }
    delayMicroseconds(300);
    if (!endWindow("shutdown", id, request, returned))
    {
        (void)motor.stop();
        Serial.println("FAIL shutdown_window");
        return false;
    }
    return true;
}

bool recover(unsigned id, unsigned kind)
{
    if (!beginWindow())
        return false;
    bool recovered = false;
    if (kind == 0)
        recovered = operation("repeat_coast", motor.coast());
    else if (kind == 1)
        recovered = operation("repeat_stop", motor.stop()) && operation("start", motor.start());
    else
    {
        recovered = setFaultPull(GPIO_PULLDOWN_ONLY);
        delayMicroseconds(4);
        recovered = recovered && operation("clear_fault", motor.clearFault()) && !motor.hasFault();
    }
    delayMicroseconds(300);
    recovered = endWindow("recovery", id) && recovered;
    if (!recovered)
    {
        (void)motor.stop();
        Serial.println("FAIL recovery");
    }
    return recovered;
}

bool setupMotor()
{
    MotorMCPWMConfig config{outputs[0], outputs[1], outputs[2], MCPWM_UNIT_0, MCPWM_TIMER_0,
                            MCPWM0A, MCPWM0B, 20000, 1023};
    config.counter = MCPWM_UP_DOWN_COUNTER;
    MotorBehaviorConfig behavior{};
    behavior.freewheel_mode = FreewheelMode::HiZ;
    MotorSafetyConfig safety{};
    safety.fault_gpio = fault_pin;
    safety.fault_action = FaultAction::Coast;
    const auto result = motor.setup(config, behavior, safety, MotorCaptureConfig{});
    Serial.printf("MOTOR %d %u\n", static_cast<int>(result.error), static_cast<unsigned>(result.software_fault_enabled));
    return result.ok() && result.software_fault_enabled;
}
}

void setup()
{
    Serial.begin(115200);
    delay(350);
    const auto *app = esp_ota_get_app_description();
    Serial.print("IDENTITY ");
    for (const uint8_t byte : app->app_elf_sha256)
        Serial.printf("%02x", static_cast<unsigned>(byte));
    Serial.printf(" %lu\n", static_cast<unsigned long>(getApbFrequency()));
    ready = true;
    for (unsigned channel = 0; channel < 3; ++channel)
    {
        const auto output = outputs[channel];
        const auto input = inputs[channel];
        const int latch = gpio_set_level(output, 0);
        gpio_config_t output_config{};
        output_config.pin_bit_mask = UINT64_C(1) << static_cast<unsigned>(output);
        output_config.mode = GPIO_MODE_OUTPUT;
        const int output_mode = gpio_config(&output_config);
        gpio_config_t input_config{};
        input_config.pin_bit_mask = UINT64_C(1) << static_cast<unsigned>(input);
        input_config.mode = GPIO_MODE_INPUT;
        input_config.pull_down_en = GPIO_PULLDOWN_ENABLE;
        const int input_mode = gpio_config(&input_config);
        pcnt_config_t counter{};
        counter.pulse_gpio_num = input;
        counter.ctrl_gpio_num = PCNT_PIN_NOT_USED;
        counter.channel = PCNT_CHANNEL_0;
        counter.unit = static_cast<pcnt_unit_t>(channel);
        counter.pos_mode = PCNT_COUNT_INC;
        counter.neg_mode = PCNT_COUNT_DIS;
        counter.counter_h_lim = 32767;
        const int configured = pcnt_unit_config(&counter);
        const int resumed = configured == ESP_OK ? pcnt_counter_resume(counter.unit) : ESP_FAIL;
        const uint32_t direction_mask = REG_READ(GPIO_ENABLE_REG);
        const bool output_enabled = (direction_mask & (UINT32_C(1) << static_cast<unsigned>(output))) != 0;
        const bool input_enabled = (direction_mask & (UINT32_C(1) << static_cast<unsigned>(input))) != 0;
        const int level = gpio_get_level(input);
        Serial.printf("INIT %u %d %d %d %d %d %d %d %d %u %u\n",
                      channel, static_cast<int>(output), static_cast<int>(input), latch, output_mode,
                      input_mode, configured, resumed, level,
                      static_cast<unsigned>(output_enabled), static_cast<unsigned>(input_enabled));
        ready = ready && latch == ESP_OK && output_mode == ESP_OK && input_mode == ESP_OK &&
                configured == ESP_OK && resumed == ESP_OK && level == 0 && output_enabled && !input_enabled;
    }
    for (unsigned channel = 0; channel < 3; ++channel)
    {
        const auto id = static_cast<mcpwm_capture_channel_id_t>(channel);
        const auto signal = static_cast<mcpwm_io_signals_t>(MCPWM_CAP_0 + channel);
        const int routed = mcpwm_gpio_init(capture_unit, signal, inputs[channel == 2 ? 2 : 0]);
        mcpwm_capture_config_t config{};
        const unsigned polarity = channel == 0 ? MCPWM_POS_EDGE :
                                  channel == 1 ? MCPWM_NEG_EDGE : MCPWM_POS_EDGE | MCPWM_NEG_EDGE;
        config.cap_edge = static_cast<mcpwm_capture_on_edge_t>(polarity);
        config.cap_prescale = 1;
        config.capture_cb = capture;
        const int enabled = mcpwm_capture_enable_channel(capture_unit, id, &config);
        Serial.printf("CAPTURE %u %u %d %d\n", channel, polarity, routed, enabled);
        ready = ready && routed == ESP_OK && enabled == ESP_OK;
    }
    if (!ready)
    {
        Serial.println("FAIL init");
        return;
    }
    ready = beginWindow();
    if (!ready)
        return;
    delayMicroseconds(350);
    ready = endWindow("idle", 0);
    if (!ready)
    {
        Serial.println("FAIL idle");
        return;
    }
    for (unsigned channel = 0; channel < 3; ++channel)
    {
        ready = (channel == 2 || selectPwmCapture(channel)) && beginWindow();
        if (!ready)
            return;
        const int high = gpio_set_level(outputs[channel], 1);
        delayMicroseconds(40);
        const int low = gpio_set_level(outputs[channel], 0);
        delayMicroseconds(100);
        const bool quiet_after_witness = endWindow("witness", channel, 0, 0, static_cast<int>(channel));
        if (high != ESP_OK || low != ESP_OK || !quiet_after_witness)
        {
            Serial.printf("FAIL witness_gpio %u %d %d\n", channel, high, low);
            ready = false;
            return;
        }
    }
    ready = setupMotor() && beginWindow();
    if (!ready)
        return;
    delayMicroseconds(350);
    ready = endWindow("preflight", 0);
    Serial.println(ready ? "READY" : "FAIL setup");
}

void loop()
{
    if (!ready || !Serial.available())
    {
        delay(1);
        return;
    }
    char line[64]{};
    const auto size = Serial.readBytesUntil('\n', line, sizeof(line) - 1U);
    if (size == 0 || size == sizeof(line) - 1U)
    {
        ready = false;
        Serial.println("FAIL command_length");
        return;
    }
    if (recovery_pending)
    {
        ready = strcmp(line, "recover") == 0 && recover(next_id, pending_kind);
        if (ready)
        {
            recovery_pending = false;
            ++next_id;
            Serial.println("READY");
        }
        else
            Serial.println("FAIL recovery_command");
        return;
    }
    if (strcmp(line, "finish") == 0)
    {
        const bool started_window = beginWindow();
        const bool stopped = operation("final_stop", motor.stop()) && started_window;
        delayMicroseconds(350);
        const bool valid = stopped && endWindow("final", next_id);
        Serial.println(valid ? "DONE" : "FAIL final_stop");
        ready = false;
        return;
    }
    unsigned id = 0, duty = 0, kind = 0, offset = 0, route = 0;
    char trailing = 0;
    const int parsed = sscanf(line, "%u %u %u %u %u %c", &id, &duty, &kind, &offset, &route, &trailing);
    if (parsed != 5 || id != next_id || (duty != 15 && duty != 40) || kind > 2 || offset >= 50 || route > 1)
    {
        ready = false;
        Serial.println("FAIL command");
        return;
    }
    ready = transition(id, duty, kind, offset, route);
    if (ready)
    {
        pending_kind = kind;
        recovery_pending = true;
        Serial.println("RECOVER");
    }
}
