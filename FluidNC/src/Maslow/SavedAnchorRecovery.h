#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace MaslowSavedAnchorRecovery {
constexpr double repeatabilityLimitMm = 1.0;
constexpr double preloadCreepMmPerSecond = 8.0;
constexpr double preloadSpeedMmPerSecond = 20.0;
constexpr uint32_t preloadNoResponseMs = 750;

// Current is the board's existing ADC reading, used as a load proxy.
// Limit inward speed as well as load so one belt cannot take up slack at full speed.
struct PreloadDrive {
    double duty = 0;

    // No measured current or meaningful inward movement while commanded near full duty.
    // An unloaded belt moving inward normally can have a low ADC reading.
    bool unresponsive(uint32_t now, double inwardPwm, double current, double length) {
        constexpr double minInwardPwm = 900.0;
        constexpr double maxInactiveAdc = 5.0;
        constexpr double minRetractionMm = 0.5;
        if (inwardPwm < minInwardPwm || current > maxInactiveAdc) {
            checkingResponse = false;
            return false;
        }
        if (!checkingResponse || responseStartLength - length >= minRetractionMm) {
            checkingResponse = true;
            responseStarted = now;
            responseStartLength = length;
        }
        return now - responseStarted >= preloadNoResponseMs;
    }

    static bool loaded(double current, double speed, double threshold) {
        return current >= 0.82 * threshold && current <= threshold && std::fabs(speed) <= 1.0;
    }

    int update(double seconds, double current, double speed, double threshold, double speedLimit) {
        const double currentMargin = (0.90 * threshold - current) / (0.10 * threshold);
        const double speedMargin = (speedLimit + speed) / 4.0; // Shortening gives negative speed.
        const double rate = 180.0 * std::clamp(std::min(currentMargin, speedMargin), -3.0, 1.0);
        duty = std::clamp(duty + rate * seconds, 0.0, 1023.0);
        return static_cast<int>(std::lround(duty));
    }

private:
    bool checkingResponse = false;
    uint32_t responseStarted = 0;
    double responseStartLength = 0;
};

struct PoseFit {
    bool converged = false;
    double x = 0, y = 0, rms = 0, maxResidual = 0;
    bool acceptable(double limit) const { return converged && maxResidual < limit; }
};

// Fit two position coordinates to four belt measurements. Anchors are never
// parameters. A bounded backtracking search rejects singular or invalid fits.
template <typename Predict, typename Service>
PoseFit fitPosition(const double lengths[4], Predict predict, Service service, double initialX = 0, double initialY = 0) {
    PoseFit result;
    result.x = initialX;
    result.y = initialY;
    for (int arm = 0; arm < 4; ++arm) {
        if (!std::isfinite(lengths[arm]) || lengths[arm] <= 0) return result;
    }
    const auto score = [&](double x, double y, double& ss, double& maxResidual) {
        ss = maxResidual = 0;
        for (int arm = 0; arm < 4; ++arm) {
            const double estimate = predict(arm, x, y);
            if (!std::isfinite(estimate) || estimate <= 0) return false;
            const double residual = estimate - lengths[arm];
            ss += residual * residual;
            maxResidual = std::max(maxResidual, std::fabs(residual));
        }
        return std::isfinite(ss);
    };
    double ss = 0;
    if (!score(result.x, result.y, ss, result.maxResidual)) return result;
    constexpr double h = 0.25; // Central difference in the production position model.
    for (int iteration = 0; iteration < 32; ++iteration) {
        service();
        double a = 0, b = 0, c = 0, u = 0, v = 0;
        for (int arm = 0; arm < 4; ++arm) {
            const double x = result.x, y = result.y;
            const double r = predict(arm, x, y) - lengths[arm];
            const double jx = (predict(arm, x + h, y) - predict(arm, x - h, y)) / (2 * h);
            const double jy = (predict(arm, x, y + h) - predict(arm, x, y - h)) / (2 * h);
            a += jx * jx; b += jx * jy; c += jy * jy; u += jx * r; v += jy * r;
        }
        const double det = a * c - b * b;
        if (!std::isfinite(det) || det <= 1e-9) return result;
        const double dx = (-c * u + b * v) / det, dy = (b * u - a * v) / det;
        if (!std::isfinite(dx) || !std::isfinite(dy)) return result;
        if (std::hypot(dx, dy) < 0.001) {
            result.converged = true;
            result.rms = std::sqrt(ss / 4);
            return result;
        }
        bool improved = false;
        for (double step = 1; step >= 1.0 / 2048; step *= 0.5) {
            const double x = result.x + step * dx, y = result.y + step * dy;
            double next = 0, maxResidual = 0;
            if (score(x, y, next, maxResidual) && next < ss) {
                result.x = x;
                result.y = y;
                result.maxResidual = maxResidual;
                ss = next;
                improved = true;
                break;
            }
        }
        if (!improved) return result;
    }
    return result;
}
} // namespace MaslowSavedAnchorRecovery
