#include "trajectory_association.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace TrajectoryAssociation {
namespace {

constexpr float kInvalidCost = 1.0e6f;

struct PairEvaluation {
    bool valid = false;
    float cost = kInvalidCost;
};

float squared(float value)
{
    return value * value;
}

PairEvaluation evaluatePair(const TrackObservation& track,
                            const DetectionObservation& detection,
                            const Config& config)
{
    PairEvaluation result;
    const float dt = std::max(0.01f, config.dt_s);
    const float predicted_x = track.x + track.vx * dt;
    const float predicted_y = track.y + track.vy * dt;
    const float ex = detection.x - predicted_x;
    const float ey = detection.y - predicted_y;
    const float distance2 = ex * ex + ey * ey;
    const float gate = std::clamp(
        0.75f + detection.range_m * config.range_gate_scale +
            std::max(0.0f, config.turn_allowance_m),
        config.min_gate_m,
        std::max(config.min_gate_m, config.max_gate_m));
    if (distance2 > gate * gate) return result;

    const float innovation_var_x = std::max(
        0.01f, track.var_x + config.measurement_variance);
    const float innovation_var_y = std::max(
        0.01f, track.var_y + config.measurement_variance);
    const float nis = ex * ex / innovation_var_x +
                      ey * ey / innovation_var_y;
    if (!std::isfinite(nis) || nis > config.gate_chi2) return result;

    const float displacement = std::hypot(detection.x - track.x,
                                          detection.y - track.y);
    const float implied_speed = displacement / dt;
    const float speed_limit = track.locked
        ? config.locked_max_implied_speed_mps
        : config.max_implied_speed_mps;
    if (!std::isfinite(implied_speed) || implied_speed > speed_limit) {
        return result;
    }

    const float measured_vx = (detection.x - track.x) / dt;
    const float measured_vy = (detection.y - track.y) / dt;
    const float velocity_residual2 = squared(measured_vx - track.vx) +
                                     squared(measured_vy - track.vy);

    float signature_cost = 0.0f;
    if (track.has_signature) {
        const float reference_points = static_cast<float>(
            std::max(1, track.point_count));
        const float point_ratio = static_cast<float>(detection.point_count) /
                                  reference_points;
        if (point_ratio < config.point_count_ratio_min ||
            point_ratio > config.point_count_ratio_max) {
            return result;
        }

        const float width_delta = std::abs(detection.width_m - track.width_m);
        const float depth_delta = std::abs(detection.depth_m - track.depth_m);
        const float doppler_delta = std::abs(
            detection.median_signed_velocity_mps -
            track.median_signed_velocity_mps);
        if (width_delta > config.max_width_delta_m ||
            depth_delta > config.max_depth_delta_m ||
            doppler_delta > config.max_signed_doppler_delta_mps) {
            return result;
        }

        const float log_point_ratio = std::abs(std::log(std::max(0.05f,
                                                                  point_ratio)));
        signature_cost += config.point_count_cost_weight *
                          log_point_ratio * log_point_ratio;
        signature_cost += config.width_cost_weight *
            squared(width_delta / std::max(0.1f, config.max_width_delta_m));
        signature_cost += config.depth_cost_weight *
            squared(depth_delta / std::max(0.1f, config.max_depth_delta_m));
        signature_cost += config.doppler_cost_weight *
            squared(doppler_delta /
                    std::max(0.1f, config.max_signed_doppler_delta_mps));
        signature_cost += config.moving_ratio_cost_weight *
            squared(detection.moving_ratio - track.moving_ratio);
    }

    float cost = nis + config.velocity_cost_weight * velocity_residual2 +
                 signature_cost +
                 config.background_cost_weight * detection.background_score -
                 config.score_reward_weight * detection.score;
    if (track.locked) {
        cost *= std::clamp(config.locked_cost_scale, 0.20f, 1.0f);
    }
    result.valid = std::isfinite(cost);
    result.cost = result.valid ? std::max(0.0f, cost) : kInvalidCost;
    return result;
}

// O(n^3) Hungarian solver for a finite square minimization matrix.  The
// trajectory counts in this project are small, so this is comfortably inside
// the frame budget and avoids adding an external dependency.
std::vector<int> hungarian(const std::vector<std::vector<float>>& cost)
{
    const int n = static_cast<int>(cost.size());
    if (n == 0) return {};

    std::vector<float> u(n + 1, 0.0f);
    std::vector<float> v(n + 1, 0.0f);
    std::vector<int> p(n + 1, 0);
    std::vector<int> way(n + 1, 0);

    for (int i = 1; i <= n; ++i) {
        p[0] = i;
        int j0 = 0;
        std::vector<float> minv(n + 1,
                                std::numeric_limits<float>::infinity());
        std::vector<bool> used(n + 1, false);
        do {
            used[j0] = true;
            const int i0 = p[j0];
            float delta = std::numeric_limits<float>::infinity();
            int j1 = 0;
            for (int j = 1; j <= n; ++j) {
                if (used[j]) continue;
                const float current = cost[i0 - 1][j - 1] - u[i0] - v[j];
                if (current < minv[j]) {
                    minv[j] = current;
                    way[j] = j0;
                }
                if (minv[j] < delta) {
                    delta = minv[j];
                    j1 = j;
                }
            }
            for (int j = 0; j <= n; ++j) {
                if (used[j]) {
                    u[p[j]] += delta;
                    v[j] -= delta;
                } else {
                    minv[j] -= delta;
                }
            }
            j0 = j1;
        } while (p[j0] != 0);

        do {
            const int j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0 != 0);
    }

    std::vector<int> row_to_column(n, -1);
    for (int j = 1; j <= n; ++j) {
        if (p[j] > 0) row_to_column[p[j] - 1] = j - 1;
    }
    return row_to_column;
}

} // namespace

Result GlobalAssociator::associate(
    const std::vector<TrackObservation>& tracks,
    const std::vector<DetectionObservation>& detections,
    const Config& config) const
{
    Result result;
    result.detection_to_track.assign(detections.size(), -1);
    result.track_to_detection.assign(tracks.size(), -1);
    result.diagnostics.track_count = static_cast<int>(tracks.size());
    result.diagnostics.detection_count = static_cast<int>(detections.size());

    if (tracks.empty() || detections.empty()) {
        result.diagnostics.unassigned_track_count =
            static_cast<int>(tracks.size());
        result.diagnostics.unassigned_detection_count =
            static_cast<int>(detections.size());
        return result;
    }

    std::vector<std::vector<PairEvaluation>> pairs(
        tracks.size(), std::vector<PairEvaluation>(detections.size()));
    int locked_row = -1;
    for (std::size_t ti = 0; ti < tracks.size(); ++ti) {
        if (tracks[ti].locked) {
            locked_row = static_cast<int>(ti);
            result.diagnostics.locked_uid = tracks[ti].internal_uid;
        }
        for (std::size_t di = 0; di < detections.size(); ++di) {
            pairs[ti][di] = evaluatePair(tracks[ti], detections[di], config);
            if (pairs[ti][di].valid) result.diagnostics.valid_pair_count++;
        }
    }

    if (locked_row >= 0) {
        float best = std::numeric_limits<float>::infinity();
        float second = std::numeric_limits<float>::infinity();
        int best_detection = -1;
        for (std::size_t di = 0; di < detections.size(); ++di) {
            const auto& pair = pairs[locked_row][di];
            if (!pair.valid) continue;
            if (pair.cost < best) {
                second = best;
                best = pair.cost;
                best_detection = static_cast<int>(di);
            } else if (pair.cost < second) {
                second = pair.cost;
            }
        }
        result.diagnostics.locked_best_detection_index = best_detection >= 0
            ? detections[best_detection].source_index : -1;
        result.diagnostics.locked_best_cost =
            std::isfinite(best) ? best : 0.0f;
        result.diagnostics.locked_second_cost =
            std::isfinite(second) ? second : 0.0f;
        result.diagnostics.locked_assignment_ambiguous =
            config.reject_ambiguous_locked_assignment &&
            std::isfinite(second) &&
            second - best < config.locked_ambiguity_cost_margin;

        for (std::size_t di = 0; di < detections.size(); ++di) {
            const bool reject_for_ambiguity =
                result.diagnostics.locked_assignment_ambiguous;
            const bool reject_non_best = config.locked_best_candidate_only &&
                best_detection >= 0 && static_cast<int>(di) != best_detection;
            if (reject_for_ambiguity || reject_non_best) {
                pairs[locked_row][di].valid = false;
                pairs[locked_row][di].cost = kInvalidCost;
            }
        }
        if (!result.diagnostics.locked_assignment_ambiguous &&
            config.locked_reserve_best_detection && best_detection >= 0) {
            // A clear, gated locked-target observation cannot be stolen by a
            // neighbouring non-locked track merely because its local cost is
            // a few decimals smaller. Ambiguous observations are never
            // reserved and remain fail-safe/unassigned.
            for (std::size_t ti = 0; ti < tracks.size(); ++ti) {
                if (static_cast<int>(ti) == locked_row) continue;
                pairs[ti][best_detection].valid = false;
                pairs[ti][best_detection].cost = kInvalidCost;
            }
        }
    }

    // Real tracks + one dummy row per detection, real detections + one dummy
    // column per track.  This lets the global optimum explicitly leave either
    // side unmatched instead of forcing a poor real-real assignment.
    const int track_count = static_cast<int>(tracks.size());
    const int detection_count = static_cast<int>(detections.size());
    const int dimension = track_count + detection_count;
    std::vector<std::vector<float>> matrix(
        dimension, std::vector<float>(dimension, 0.0f));

    for (int ti = 0; ti < track_count; ++ti) {
        for (int di = 0; di < detection_count; ++di) {
            matrix[ti][di] = pairs[ti][di].valid
                ? pairs[ti][di].cost : kInvalidCost;
        }
        for (int dummy = detection_count; dummy < dimension; ++dummy) {
            matrix[ti][dummy] = config.unassigned_track_cost;
        }
    }
    for (int dummy_row = track_count; dummy_row < dimension; ++dummy_row) {
        for (int di = 0; di < detection_count; ++di) {
            matrix[dummy_row][di] = config.new_detection_cost;
        }
        for (int dummy_col = detection_count; dummy_col < dimension;
             ++dummy_col) {
            matrix[dummy_row][dummy_col] = 0.0f;
        }
    }

    const std::vector<int> assignment = hungarian(matrix);
    for (int ti = 0; ti < track_count; ++ti) {
        const int di = assignment[ti];
        if (di < 0 || di >= detection_count || !pairs[ti][di].valid) continue;
        result.track_to_detection[ti] = detections[di].source_index;
        result.detection_to_track[di] = tracks[ti].source_index;
        result.diagnostics.matched_pair_count++;
        result.diagnostics.total_assignment_cost += pairs[ti][di].cost;
    }
    result.diagnostics.unassigned_track_count =
        track_count - result.diagnostics.matched_pair_count;
    result.diagnostics.unassigned_detection_count =
        detection_count - result.diagnostics.matched_pair_count;
    return result;
}

} // namespace TrajectoryAssociation
