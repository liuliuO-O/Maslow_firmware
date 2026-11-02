#pragma once

#include <cstdint>

namespace MaslowEncoder {

// Resolve a 12-bit absolute-angle change into a signed movement.  When the
// motor is actively driven, expectedDirection disambiguates samples separated
// by more than half a revolution: +1 means increasing encoder counts and -1
// means decreasing counts.  Small motion against the commanded direction is
// retained so backlash and settling are not turned into full revolutions.
inline int32_t unwrapDelta(uint16_t previous, uint16_t current, int8_t expectedDirection) {
    int32_t delta = static_cast<int32_t>(current) - static_cast<int32_t>(previous);

    constexpr int32_t countsPerRevolution = 4096;
    constexpr int32_t directionThreshold  = 1024;
    constexpr int32_t halfRevolution      = countsPerRevolution / 2;

    if (expectedDirection > 0 && delta < -directionThreshold) {
        delta += countsPerRevolution;
    } else if (expectedDirection < 0 && delta > directionThreshold) {
        delta -= countsPerRevolution;
    } else if (expectedDirection == 0) {
        if (delta > halfRevolution) {
            delta -= countsPerRevolution;
        } else if (delta < -halfRevolution) {
            delta += countsPerRevolution;
        }
    }

    return delta;
}

}  // namespace MaslowEncoder
