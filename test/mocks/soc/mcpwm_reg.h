#pragma once

#include <mock_hal.h>

#define MCPWM_TIMER0_CFG0_REG(unit) (0x1000U * static_cast<uint32_t>(unit) + 0x04U)
#define MCPWM_TIMER1_CFG0_REG(unit) (MCPWM_TIMER0_CFG0_REG(unit) + 0x10U)
#define MCPWM_TIMER0_PERIOD_M 0x00ffff00U
#define MCPWM_TIMER0_PERIOD_S 8U
#define MCPWM_GEN0_A_REG(unit) (0x1000U * static_cast<uint32_t>(unit) + 0x50U)
#define MCPWM_GEN0_B_REG(unit) (MCPWM_GEN0_A_REG(unit) + 4U)
#define MCPWM_GEN1_A_REG(unit) (MCPWM_GEN0_A_REG(unit) + 0x38U)
#define MCPWM_INT_RAW_REG(unit) (0x1000U * static_cast<uint32_t>(unit) + 0x114U)
#define MCPWM_INT_CLR_REG(unit) (MCPWM_INT_RAW_REG(unit) + 8U)
#define MCPWM_TIMER0_STOP_INT_RAW 1U
#define MCPWM_TIMER0_TEZ_INT_RAW 8U
#define MCPWM_TIMER0_TEP_INT_RAW 64U
