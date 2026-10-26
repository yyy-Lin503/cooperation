#ifndef TRAJECTORY_ASSOCIATION_HPP
#define TRAJECTORY_ASSOCIATION_HPP

#include <cstdint>
#include <string>
#include <vector>

namespace TrajectoryAssociation {

// A deliberately tracker-agnostic view of one persistent trajectory.  The
// target tracker owns the lifecycle and IDs; this module only decides the
// globally consistent track-to-detection assignment for the current frame.
struct TrackObservation {
    int source_index = -1;
    std::uint64_t internal_uid = 0;
    bool locked = false;
    float x = 0.0f;
    float y = 0.0f;
    float vx = 0.0f;
    float vy = 0.0f;
    float var_x = 0.50f;
    float var_y = 0.50f;
    bool has_signature = false;
    int point_count = 0;
    float width_m = 0.0f;
    float depth_m = 0.0f;
    float median_signed_velocity_mps = 0.0f;
    float moving_ratio = 0.0f;
};

struct DetectionObservation {
    int source_index = -1;
    float x = 0.0f;
    float y = 0.0f;
    float range_m = 0.0f;
    int point_count = 0;
    float width_m = 0.0f;
    float depth_m = 0.0f;
    float median_signed_velocity_mps = 0.0f;
    float moving_ratio = 0.0f;
    float score = 0.0f;
    float background_score = 0.0f;
};

struct Config {
    float dt_s = 0.1f;
    float min_gate_m = 0.9f;
    float max_gate_m = 1.2f;
    float range_gate_scale = 0.04f;
    float turn_allowance_m = 0.0f;
    float measurement_variance = 0.18f;
    float gate_chi2 = 9.21f;
    float max_implied_speed_mps = 5.0f;
    float locked_max_implied_speed_mps = 3.5f;

    // Soft signature costs.  The corresponding hard limits are deliberately
    // broad because an IWR6843 person may contain only 3-4 points.
    float velocity_cost_weight = 0.05f;
    float point_count_cost_weight = 0.35f;
    float width_cost_weight = 0.25f;
    float depth_cost_weight = 0.25f;
    float doppler_cost_weight = 0.30f;
    float moving_ratio_cost_weight = 0.10f;
    float score_reward_weight = 0.08f;
    float background_cost_weight = 0.25f;
    float point_count_ratio_min = 0.15f;
    float point_count_ratio_max = 6.0f;
    float max_width_delta_m = 1.2f;
    float max_depth_delta_m = 1.2f;
    float max_signed_doppler_delta_mps = 3.0f;

    float locked_cost_scale = 0.65f;
    bool locked_best_candidate_only = true;
    bool locked_reserve_best_detection = true;
    bool reject_ambiguous_locked_assignment = true;
    float locked_ambiguity_cost_margin = 0.75f;

    // A real-real match must be cheaper than leaving both sides unmatched.
    float unassigned_track_cost = 5.0f;
    float new_detection_cost = 5.0f;
};

struct Diagnostics {
    std::string method = "hungarian";
    int track_count = 0;
    int detection_count = 0;
    int valid_pair_count = 0;
    int matched_pair_count = 0;
    int unassigned_track_count = 0;
    int unassigned_detection_count = 0;
    bool locked_assignment_ambiguous = false;
    std::uint64_t locked_uid = 0;
    int locked_best_detection_index = -1;
    float locked_best_cost = 0.0f;
    float locked_second_cost = 0.0f;
    float total_assignment_cost = 0.0f;
};

struct Result {
    // Values are source_index values, not positions in the input arrays.
    std::vector<int> detection_to_track;
    std::vector<int> track_to_detection;
    Diagnostics diagnostics;
};

class GlobalAssociator {
public:
    Result associate(const std::vector<TrackObservation>& tracks,
                     const std::vector<DetectionObservation>& detections,
                     const Config& config) const;
};

} // namespace TrajectoryAssociation

#endif // TRAJECTORY_ASSOCIATION_HPP
