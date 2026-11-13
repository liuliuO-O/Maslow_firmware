#pragma once
#include <cmath>
#include <cstdint>

namespace MaslowCalibrationStability {
constexpr uint32_t stableDurationMs = 750;
constexpr uint32_t maxObservationGapMs = 100;
constexpr float maxSpanMm = 0.5f;
constexpr float maxPositionErrorMm = 1.0f;
// Settling gate after a jog/cycle.  It must tolerate the machine's own geometry
// residual: on 2026-11-13 the best achievable four-belt residual was 2.26 mm, so
// a 1 mm gate could never be met once the sled moved (TR parked at 2.08 mm and
// the hold aborted 1.5 s after every move).
constexpr float maxSettleErrorMm = 2.5f;
constexpr uint32_t motionSettleTimeoutMs = 4000;
// A missed settle invalidates the belt reference only after this many
// consecutive misses.  The gate above is what defines "missed"; this constant
// only decides how much a single miss costs the operator.  Keeping it at 1
// meant a 0.24 mm overshoot (BR 2.742 mm on 2026-11-13 18:2x) threw away a
// valid reference and forced a full retract/extend.  Repeated misses still
// void the reference, so a machine that cannot settle does not quietly drift.
constexpr uint8_t maxConsecutiveSettleFailures = 3;

struct Diagnostics {
    unsigned spanResetMask = 0;
    unsigned observationGapResets = 0;
};

// A fixed-duration range check catches slow cumulative drift, not just large
// adjacent steps. Sparse observations and invalid/poorly-held positions restart it.
class Window {
public:
    void reset() { active = false; }
    bool update(uint32_t now, const float position[4], bool held, Diagnostics* diagnostics = nullptr) {
        for (int arm = 0; arm < 4; ++arm) {
            if (!std::isfinite(position[arm])) { reset(); return false; }
        }
        if (!held) { reset(); return false; }
        const bool gap = active && static_cast<uint32_t>(now - last) > maxObservationGapMs;
        if (!active || gap) {
            if (gap && diagnostics) ++diagnostics->observationGapResets;
            start(now, position);
            return false;
        }
        last = now;
        unsigned spanMask = 0;
        for (int arm = 0; arm < 4; ++arm) {
            if (position[arm] < minimum[arm]) minimum[arm] = position[arm];
            if (position[arm] > maximum[arm]) maximum[arm] = position[arm];
            if (maximum[arm] - minimum[arm] > maxSpanMm) spanMask |= 1u << arm;
        }
        if (spanMask) {
            if (diagnostics) diagnostics->spanResetMask |= spanMask;
            start(now, position);
            return false;
        }
        return static_cast<uint32_t>(now - since) >= stableDurationMs;
    }
private:
    void start(uint32_t now, const float position[4]) {
        active = true;
        since = last = now;
        for (int arm = 0; arm < 4; ++arm) minimum[arm] = maximum[arm] = position[arm];
    }
    bool active = false;
    uint32_t since = 0, last = 0;
    float minimum[4] = {}, maximum[4] = {};
};
} // namespace MaslowCalibrationStability
