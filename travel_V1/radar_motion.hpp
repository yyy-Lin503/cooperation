#ifndef RADAR_MOTION_HPP
#define RADAR_MOTION_HPP
#include "radar_mount.hpp"
#include "chassis_velocity.hpp"
#include "signal_processing.hpp"
#include <deque>

namespace RadarMotion {
struct Transform {
    float dx = 0, dy = 0, yaw = 0;
    void position(float& x, float& y) const {
        const float sx = x - dx, sy = y - dy;
        const float c = std::cos(-yaw), s = std::sin(-yaw);
        x = c * sx - s * sy;
        y = s * sx + c * sy;
    }
    void vector(float& x, float& y) const {
        const float ox = x, oy = y;
        const float c = std::cos(-yaw), s = std::sin(-yaw);
        x = c * ox - s * oy;
        y = s * ox + c * oy;
    }
};
inline Transform transform(float vx, float wz_raw, float dt) {
    const float wz = RadarMount::yawSign() * wz_raw;
    return {-wz * RadarMount::yOffsetM() * dt,
            (vx + wz * RadarMount::xOffsetM()) * dt, wz * dt};
}
inline float radialVelocity(float relative, float angle_deg, const ChassisVelocitySample& sample) {
    const auto motion = transform(sample.vx_mps, sample.wz_radps, 1.0f);
    const float angle = angle_deg * static_cast<float>(M_PI) / 180.0f;
    return relative - (motion.dx * std::sin(angle) + motion.dy * std::cos(angle));
}
// Shared by GUI and batch replay. Realtime retains its one-valid-sample
// fallback; historical replay requires both samples, as before.
template<class Point>
void appendCompensated(std::vector<Point>& output, const std::vector<Point>& input,
                       int source_frame, int current_frame, double period,
                       const ChassisVelocitySample* source, const ChassisVelocitySample* current,
                       bool require_both = true) {
    if (source_frame == current_frame || !source || !current ||
        (require_both ? (!source->valid || !current->valid) : (!source->valid && !current->valid))) {
        output.insert(output.end(), input.begin(), input.end());
        return;
    }
    const auto& a = source->valid ? *source : *current;
    const auto& b = current->valid ? *current : a;
    double dt = (current_frame - source_frame) * period;
    if (source->has_timestamp && current->has_timestamp)
        dt = current->timestamp_sec - source->timestamp_sec;
    // Realtime historically treats repeated odom timestamps as zero motion.
    if (dt < 0 || dt > 1 || (require_both && dt <= 1e-6))
        dt = (current_frame - source_frame) * period;
    const auto motion = transform(0.5f * (a.vx_mps + b.vx_mps),
                                  0.5f * (a.wz_radps + b.wz_radps), static_cast<float>(dt));
    for (auto point : input) {
        motion.position(point.x, point.y);
        output.push_back(point);
    }
}
template<class Lookup>
std::vector<PointCloud> aggregate(const std::deque<std::pair<int, std::vector<PointCloud>>>& window,
                                  double period, Lookup lookup) {
    std::vector<PointCloud> result;
    if (window.empty()) return result;
    std::size_t count = 0;
    for (const auto& frame : window) count += frame.second.size();
    result.reserve(count);
    for (const auto& frame : window)
        appendCompensated(result, frame.second, frame.first, window.back().first, period,
                          lookup(frame.first), lookup(window.back().first));
    return result;
}
template<class Tracker>
void advance(Tracker& tracker, const std::vector<PointCloud>& points,
             const ChassisVelocitySample& chassis, int frame) {
    tracker.updateCandidates(points, &chassis, frame);
    tracker.updateLockedTarget();
}
}
#endif
