/**
 * MIT License
 *
 * @brief Minimal Arduino surface for public-example host syntax checks.
 *
 * @file Arduino.h
 * @author Little Man Builds (Darren Osborne)
 * @date 2026-08-07
 * @copyright Copyright (c) 2026 Little Man Builds
 */

#pragma once

#include <mock_hal.h>
#include <cstdint>

class HostSerial
{
public:
    void begin(unsigned long) {}
    void println(const char *) {}
    void println(int) {}
    void println(unsigned int) {}
    void println(unsigned long) {}
    void println(float) {}
    void print(const char *) {}
    void print(int) {}
    void print(unsigned int) {}
    void print(unsigned long) {}
    void print(float) {}
    int available() const { return 0; }
    int read() { return -1; }
};

static HostSerial Serial;
inline void delay(unsigned long) {}
inline void tone(uint8_t, unsigned int) {}
