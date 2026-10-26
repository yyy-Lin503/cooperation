#ifndef RADAR_MOUNT_HPP
#define RADAR_MOUNT_HPP
#include "radar_connection_config.hpp"
namespace RadarMount {
// Retained defaults are also used by frozen comparator source interfaces.
constexpr float kRadarXOffsetM = 0.0f;
constexpr float kRadarYOffsetM = 0.20f;
constexpr float kAngularCompensationSign = 1.0f;
// Lazily read once so invalid configuration is reported by the application,
// rather than throwing during static initialization before main().
inline float xOffsetM() {
    static const float value = static_cast<float>(RadarEnvironment::number("RADAR_MOUNT_X_M", kRadarXOffsetM, -10, 10));
    return value;
}
inline float yOffsetM() {
    static const float value = static_cast<float>(RadarEnvironment::number("RADAR_MOUNT_Y_M", kRadarYOffsetM, -10, 10));
    return value;
}
inline float yawSign() {
    static const float value = [] {
        const float sign = static_cast<float>(RadarEnvironment::number("RADAR_YAW_SIGN", kAngularCompensationSign, -1, 1));
        if (sign != -1 && sign != 1) throw std::invalid_argument("RADAR_YAW_SIGN must be -1 or 1");
        return sign;
    }();
    return value;
}
}
#endif
