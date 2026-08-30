/**
 * MIT License
 *
 * @brief Public umbrella header for the ESP32_MCPWM motor-driver library.
 *
 * @file ESP32_MCPWM.h
 * @author Little Man Builds (Darren Osborne)
 * @date 2025-08-28
 * @copyright Copyright (c) 2026 Little Man Builds
 *
 */

#pragma once

// ---- Version ---- //

#define ESP32_MCPWM_VERSION "2.0.0"
#define ESP32_MCPWM_VERSION_MAJOR 2
#define ESP32_MCPWM_VERSION_MINOR 0
#define ESP32_MCPWM_VERSION_PATCH 0

#include "HBridgeMotor.h"
#include "IMotorDriver.h"

// ---- Public API ---- //

/// @brief Beginner-friendly alias for the default H-bridge implementation.
using Motor = HBridgeMotor;

/// @brief Deprecated v1.x version spelling retained for source compatibility.
#define ESP32_MCPWM_MOTOR_VERSION ESP32_MCPWM_VERSION
