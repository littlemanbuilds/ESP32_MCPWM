#pragma once

#include <soc/mcpwm_reg.h>

inline uint32_t REG_READ(uint32_t address)
{
    const std::size_t unit = address / 0x1000U;
    const uint32_t offset = address % 0x1000U;
    if (offset == MCPWM_INT_RAW_REG(0))
    {
        uint32_t status = 0;
        for (uint32_t timer = 0; timer < 3U; ++timer)
        {
            const uint32_t events = mock_hal::pwm_timers[unit][timer].events;
            if ((events & mock_hal::stopped_event) != 0U)
                status |= MCPWM_TIMER0_STOP_INT_RAW << timer;
            if ((events & mock_hal::empty_event) != 0U)
                status |= MCPWM_TIMER0_TEZ_INT_RAW << timer;
            if ((events & mock_hal::full_event) != 0U)
                status |= MCPWM_TIMER0_TEP_INT_RAW << timer;
        }
        return status;
    }
    const std::size_t timer = (offset - MCPWM_TIMER0_CFG0_REG(0)) / 0x10U;
    return mock_hal::pwm_timers[unit][timer].period_register << MCPWM_TIMER0_PERIOD_S;
}

inline void REG_WRITE(uint32_t address, uint32_t mask)
{
    const std::size_t unit = address / 0x1000U;
    if (address % 0x1000U != MCPWM_INT_CLR_REG(0))
        return;
    for (uint32_t timer = 0; timer < 3U; ++timer)
    {
        auto &events = mock_hal::pwm_timers[unit][timer].events;
        if ((mask & (MCPWM_TIMER0_STOP_INT_RAW << timer)) != 0U)
            events &= ~mock_hal::stopped_event;
        if ((mask & (MCPWM_TIMER0_TEZ_INT_RAW << timer)) != 0U)
            events &= ~mock_hal::empty_event;
        if ((mask & (MCPWM_TIMER0_TEP_INT_RAW << timer)) != 0U)
            events &= ~mock_hal::full_event;
    }
}
