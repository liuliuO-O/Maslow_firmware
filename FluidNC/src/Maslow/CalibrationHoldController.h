#pragma once
#include <cmath>
#include <cstdint>

// Fixed-reference position hold. Motion uses the separate position controller.
namespace MaslowCalibrationHold {
constexpr double proportionalGain = 300.0;
constexpr double velocityDamping = 8.0;          // PWM / (mm/s), derivative on measurement.
constexpr double velocityFilterSeconds = 0.020;
constexpr double maxDrivePWM = 600.0;           // Effective bridge duty, after compensation.
constexpr double maxSlewPWMPerSecond = 2500.0;
constexpr uint32_t maxFeedbackGapMs = 100;

class Controller {
public:
    void reset() { initialized = false; drive = speed = 0; }
    bool active() const { return initialized; }
    double output() const { return drive; }
    double velocity() const { return speed; }
    bool atLimit() const { return std::fabs(drive) >= maxDrivePWM - 1.0; }

    // Fade the old 75-PWM static-friction boost through zero. This is odd and
    // continuous, unlike adding +/-75 for arbitrarily small signed commands.
    static double driveForEffort(double effort, double limit = maxDrivePWM) {
        const double boost = 75.0 * clamp(effort / 150.0, -1.0, 1.0);
        return clamp(effort * (948.0 / 1023.0) + boost, -limit, limit);
    }
    bool update(uint32_t now, double position, double target) {
        if (!std::isfinite(position) || !std::isfinite(target)) { reset(); return false; }
        if (!initialized || target != reference) {
            initialized = true;
            reference = target;
            lastPosition = position;
            lastTime = now;
            drive = speed = 0;
            return true;
        }
        const uint32_t elapsed = now - lastTime;
        if (elapsed == 0) return true;
        if (elapsed > maxFeedbackGapMs) { reset(); return false; }
        const double seconds = elapsed / 1000.0;
        const double measuredSpeed = (position - lastPosition) / seconds;
        const double alpha = seconds / (velocityFilterSeconds + seconds);
        speed += alpha * (measuredSpeed - speed);
        const double effort = proportionalGain * (target - position) - velocityDamping * speed;
        const double desired = driveForEffort(effort);
        const double change = maxSlewPWMPerSecond * seconds;
        drive = clamp(desired, drive - change, drive + change);
        lastPosition = position;
        lastTime = now;
        return true;
    }
private:
    static double clamp(double x, double low, double high) { return x < low ? low : x > high ? high : x; }
    bool initialized = false;
    uint32_t lastTime = 0;
    double lastPosition = 0, reference = 0, drive = 0, speed = 0;
};
} // namespace MaslowCalibrationHold
