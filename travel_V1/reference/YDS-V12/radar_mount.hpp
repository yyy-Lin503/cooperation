#ifndef RADAR_MOUNT_HPP
#define RADAR_MOUNT_HPP

namespace RadarMount {

// Radar array center offset in the vehicle/body frame.
// x: lateral offset, left-positive by convention used by the compensation code.
// y: forward offset from the vehicle odom/rotation center to the radar array.
constexpr float kRadarXOffsetM = 0.0f;
constexpr float kRadarYOffsetM = 0.20f;

// Flip this to -1.0f if validation shows the yaw-rate sign from odom is opposite
// to the radar coordinate convention.
constexpr float kAngularCompensationSign = 1.0f;

} // namespace RadarMount

#endif // RADAR_MOUNT_HPP
