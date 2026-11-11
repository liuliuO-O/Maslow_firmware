#pragma once

#include <cmath>
#include <limits>

namespace MaslowCalibrationSamples {
constexpr int sampleCount = 4;
constexpr int armCount = 4;

// Samples are ordered by time, with columns TL, TR, BL, BR (not I2C ports).
struct Quality {
    float span = 0.0f;
    float adjacent = 0.0f;
    int arm = 0;
    int minimumSample = 0;
    int maximumSample = 0;
};

inline Quality evaluate(const float (&samples)[sampleCount][armCount]) {
    Quality quality;
    for (int arm = 0; arm < armCount; ++arm) {
        int minimum = 0, maximum = 0;
        for (int sample = 0; sample < sampleCount; ++sample) {
            if (!std::isfinite(samples[sample][arm]) || samples[sample][arm] <= 0.0f) {
                quality.span = std::numeric_limits<float>::infinity();
                quality.arm = arm;
                return quality;
            }
            if (samples[sample][arm] < samples[minimum][arm]) minimum = sample;
            if (samples[sample][arm] > samples[maximum][arm]) maximum = sample;
            if (sample > 0) {
                const float adjacent = std::fabs(samples[sample][arm] - samples[sample - 1][arm]);
                if (adjacent > quality.adjacent) quality.adjacent = adjacent;
            }
        }
        const float span = samples[maximum][arm] - samples[minimum][arm];
        if (span > quality.span) {
            quality.span = span;
            quality.arm = arm;
            quality.minimumSample = minimum;
            quality.maximumSample = maximum;
        }
    }
    return quality;
}

// Preserve three complete four-belt snapshots; never select each belt separately.
inline void slide(float (&samples)[sampleCount][armCount]) {
    for (int sample = 0; sample < sampleCount - 1; ++sample) {
        for (int arm = 0; arm < armCount; ++arm) {
            samples[sample][arm] = samples[sample + 1][arm];
        }
    }
}
}  // namespace MaslowCalibrationSamples
