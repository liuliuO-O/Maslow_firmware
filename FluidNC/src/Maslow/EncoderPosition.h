#pragma once

#include <cstdint>

namespace MaslowEncoder {

// Resolve a 12-bit absolute-angle change into the shortest signed movement.
// This is reliable only while consecutive samples remain below half a turn;
// the scheduler is responsible for maintaining that sampling rate.
inline int32_t unwrapDelta(uint16_t previous, uint16_t current) {
    int32_t delta = static_cast<int32_t>(current) - static_cast<int32_t>(previous);

    constexpr int32_t countsPerRevolution = 4096;
    constexpr int32_t halfRevolution      = countsPerRevolution / 2;

    if (delta > halfRevolution) {
        delta -= countsPerRevolution;
    } else if (delta < -halfRevolution) {
        delta += countsPerRevolution;
    }

    return delta;
}

}  // namespace MaslowEncoder
