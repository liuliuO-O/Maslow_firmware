#pragma once
#include <cmath>
#include <algorithm>
#include <cstdint>

// Fixed-reference position hold. Motion uses the separate position controller.
namespace MaslowCalibrationHold {
constexpr double proportionalGain = 300.0;
constexpr double velocityDamping = 8.0;          // PWM / (mm/s), derivative on measurement.
constexpr double velocityFilterSeconds = 0.020;
// Effective bridge duty, after compensation.  Preload may legally reach 1023 to
// establish the reference tension, so a 600 cap left the hold loop unable to
// maintain what preload had just built: on 2026-11-13 16:2x the four belts hit
// 762 ADC at 640/724/630/452 PWM, and TR (724) plus TL (640) were already over
// 600 before sampling began.  The hold saturated at 599, which is also the
// sampling gate, so every 750 ms window reset and 12 s elapsed with only 3
// snapshots.  Positions never moved (position_error_mask=0, span <0.02 mm), so
// this is an authority mismatch, not a mechanical stall.  800 clears the worst
// measured axis with margin while staying under the 900-923 ADC current seen
// during real tightening.
constexpr double maxDrivePWM = 800.0;
constexpr double maxSlewPWMPerSecond = 2500.0;
constexpr uint32_t maxFeedbackGapMs = 100;

// A pure proportional position hold settles at drive = kp * error, so an axis
// that needs a large standing duty (the belt with the most friction or stretch)
// parks at a permanent offset: measured 0.58/0.61/0.89/1.15 mm against
// 236/245/323/395 PWM on 2026-11-13 wp=1, which is exactly the P curve.  With
// no integral action the loop never closes that offset, and BR crossed the 1 mm
// sampling gate.  The fix is a bounded breakaway assist: while an axis is off
// target and NOT moving, ramp extra duty until it breaks static friction, then
// release it as soon as motion appears so the P+D loop can settle normally.
// The assist is capped, so a genuinely jammed belt saturates the cap and the
// stuck timer reports it instead of silently burning current forever.
constexpr double breakawayErrorMm = 0.25;          // only for a real position error
constexpr double breakawaySpeedMmPerSecond = 0.15;  // "not moving" threshold
constexpr double breakawayRampPwmPerSecond = 400.0;
constexpr double breakawayLimitPwm = 380.0;
constexpr double breakawayDecayPwmPerSecond = 1200.0;
constexpr double breakawayJamErrorMm = 1.0;       // still outside the sampling gate
constexpr uint32_t breakawayStuckLimitMs = 2000;  // at the cap this long = jam

class Controller {
public:
    void reset() { initialized = false; drive = speed = assist = 0; stuckMs_ = 0; }
    bool active() const { return initialized; }
    double output() const { return drive; }
    double velocity() const { return speed; }
    double assistOutput() const { return assist; }
    // Milliseconds spent at the assist cap without moving: a jam, not friction.
    uint32_t stuckMs() const { return stuckMs_; }
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
            drive = speed = assist = 0;
            stuckMs_ = 0;
            return true;
        }
        const uint32_t elapsed = now - lastTime;
        if (elapsed == 0) return true;
        if (elapsed > maxFeedbackGapMs) { reset(); return false; }
        const double seconds = elapsed / 1000.0;
        const double measuredSpeed = (position - lastPosition) / seconds;
        const double alpha = seconds / (velocityFilterSeconds + seconds);
        speed += alpha * (measuredSpeed - speed);
        const double error = target - position;
        // Off target and stationary: ramp the breakaway assist.  Any real motion
        // (or getting back inside the error band) releases it immediately, so it
        // cannot keep pushing after the belt has started to move.
        const bool stuck = std::fabs(error) > breakawayErrorMm
                        && std::fabs(speed) < breakawaySpeedMmPerSecond;
        if (stuck) {
            assist = std::min(assist + breakawayRampPwmPerSecond * seconds, breakawayLimitPwm);
        } else if (std::fabs(error) <= breakawayErrorMm) {
            // Back inside the window: release the extra force so the P+D loop
            // owns the hold again and no standing push remains.
            assist = std::max(assist - breakawayDecayPwmPerSecond * seconds, 0.0);
        }
        // Otherwise the belt is moving but still outside the window: hold the
        // assist.  Decaying it here made the axis creep one sample at a time,
        // because every small step looked like "motion" and cancelled the ramp.
        // Only a belt that is still outside the 1 mm sampling gate counts as
        // jammed: resting at the edge of the assist band is a normal hold.
        const bool jammed = std::fabs(error) > breakawayJamErrorMm
                        && std::fabs(speed) < breakawaySpeedMmPerSecond;
        stuckMs_ = (jammed && assist >= breakawayLimitPwm - 1.0) ? stuckMs_ + elapsed : 0;
        const double direction = error < 0.0 ? -1.0 : 1.0;
        const double effort = proportionalGain * error - velocityDamping * speed + direction * assist;
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
    uint32_t stuckMs_ = 0;
    double lastPosition = 0, reference = 0, drive = 0, speed = 0, assist = 0;
};
} // namespace MaslowCalibrationHold
