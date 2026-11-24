#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <inttypes.h>

/// @brief Format one capture window header.
///
/// @param buffer Destination for the UART record.
/// @param size Destination capacity in bytes.
/// @param name Window kind.
/// @param id Scenario identifier.
/// @param count Number of buffered edge records.
/// @param overflow Capture overflow flag.
/// @param levels Final input level mask.
/// @param request_us Shutdown request timestamp in microseconds.
/// @param return_us Shutdown return timestamp in microseconds.
/// @param counts Independent pulse counts.
/// @param status Pulse counter read statuses.
/// @return Required bytes excluding the terminator, or a formatting error.
inline int formatWindow(char *buffer, std::size_t size, const char *name, unsigned id, unsigned count,
                        unsigned overflow, unsigned levels, uint32_t request_us, uint32_t return_us,
                        const int16_t (&counts)[3], const int (&status)[3])
{
    return std::snprintf(buffer, size, "WINDOW %s %u %u %u %u %" PRIu32 " %" PRIu32 " %d %d %d %d %d %d",
                         name, id, count, overflow, levels, request_us, return_us,
                         static_cast<int>(counts[0]), static_cast<int>(counts[1]), static_cast<int>(counts[2]),
                         status[0], status[1], status[2]);
}

/// @brief Format one timestamped edge record.
///
/// @param buffer Destination for the UART record.
/// @param size Destination capacity in bytes.
/// @param tick Timestamp in shared capture-clock ticks.
/// @param channel Logical input channel.
/// @param rising One for a rising edge, zero for a falling edge.
/// @return Required bytes excluding the terminator, or a formatting error.
inline int formatEdge(char *buffer, std::size_t size, uint32_t tick, unsigned channel, unsigned rising)
{
    return std::snprintf(buffer, size, "EDGE %" PRIu32 " %u %u", tick, channel, rising);
}
