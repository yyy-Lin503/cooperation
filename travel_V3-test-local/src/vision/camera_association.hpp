#ifndef V3_CAMERA_ASSOCIATION_HPP
#define V3_CAMERA_ASSOCIATION_HPP
#include <opencv2/core.hpp>
#include <cstdint>
#include <string>
#include <vector>

namespace CameraAssist {
struct Person {
    cv::Rect2f box;
    float confidence = 0;
    std::uint64_t track_id = 0;
    bool stable = false;
};
struct VisionFrame {
    std::uint64_t session = 0, sequence = 0;
    std::string device;
    int width = 0, height = 0;
    double captured_monotonic = 0, captured_wall = 0, completed_monotonic = 0;
    std::string time_quality;
    std::vector<Person> people;
};
struct RadarPerson { std::uint64_t uid = 0; int display_id = -1; float x = 0, y = 0; };
struct RadarFrame {
    std::uint64_t session = 0;
    double captured_monotonic = 0;
    std::vector<RadarPerson> people;
};
struct Binding {
    std::uint64_t camera_session = 0, camera_track = 0, radar_session = 0, selection_epoch = 0;
    std::uint64_t radar_uid = 0;
};
// The user identifies the corresponding person; no projection or radar selection.
bool bindPerson(const RadarFrame&, const VisionFrame&, std::uint64_t locked_uid,
                std::uint64_t selection_epoch, std::uint64_t camera_track,
                double now, Binding& result, std::string& reason);
struct MotionDecision { bool enabled = false, permit = true; std::string reason = "disabled"; };
MotionDecision motionDecision(bool enabled, const Binding&, const RadarFrame&,
                              const VisionFrame&, std::uint64_t locked_uid,
                              std::uint64_t selection_epoch, double now);

// Anonymous short-term box continuity only. Ambiguity and missed detections
// retire IDs rather than silently transferring the selected person's identity.
class PersonTracker {
public:
    std::vector<Person> update(const std::vector<Person>& detections, double now);
    void reset();
private:
    struct Track { Person person; int hits = 0; double last_seen = 0; };
    std::vector<Track> tracks_;
    std::uint64_t next_id_ = 1;
    double previous_time_ = 0;
};
}
#endif
