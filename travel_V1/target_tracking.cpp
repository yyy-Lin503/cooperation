#include "target_tracking.hpp"
#include "chassis_velocity.hpp"
#include "radar_mount.hpp"
#include "radar_runtime_config.hpp"
#include "radar_motion.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <map>
#include <numeric>
#include <sstream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace TargetTracking {
namespace {

static ChassisVelocityReader g_tracking_chassis_velocity_reader;

float squaredDistance(float ax, float ay, float bx, float by)
{
    const float dx = ax - bx;
    const float dy = ay - by;
    return dx * dx + dy * dy;
}

float clamp01(float v)
{
    if (v < 0.0f) return 0.0f;
    if (v > 1.0f) return 1.0f;
    return v;
}

float motionPriority(const CandidateTarget& c)
{
    return c.moving_ratio * 3.0f + c.average_abs_velocity * 2.0f - c.background_score * 1.5f;
}

bool isActivelyMoving(const CandidateTarget& c, const TrackingConfig& config)
{
    return c.moving_ratio >= config.moving_ratio_select_threshold ||
           c.average_abs_velocity >= config.moving_velocity_select_threshold;
}

float turnGateBonus(const TrackingConfig& config,
                    const ChassisVelocitySample& chassis)
{
    const float yaw_rate = chassis.valid ? std::abs(chassis.wz_radps) : 0.0f;
    const float over_threshold = yaw_rate - config.high_turn_wz_threshold_radps;
    if (over_threshold <= 0.0f) {
        return 0.0f;
    }
    return over_threshold * config.turn_gate_radius_gain;
}

void updateFollowControlAdvice(LockedTarget& locked, const TrackingConfig& config)
{
    if (locked.lost_frames <= 0) {
        locked.control_state = FollowControlState::NormalFollow;
        locked.control_speed_scale = 1.0f;
        locked.control_allow_translation = true;
        locked.control_allow_yaw = true;
        return;
    }

    if (locked.lost_frames <= config.lost_limited_follow_frames) {
        locked.control_state = FollowControlState::LimitedFollow;
        const float remaining = static_cast<float>(config.lost_limited_follow_frames -
                                                    locked.lost_frames + 1);
        locked.control_speed_scale = std::clamp(
            remaining / std::max(1.0f, static_cast<float>(config.lost_limited_follow_frames)),
            0.1f, 1.0f);
        locked.control_allow_translation = true;
        locked.control_allow_yaw = true;
        return;
    }

    locked.control_state = FollowControlState::StopAndReacquire;
    locked.control_speed_scale = 0.0f;
    locked.control_allow_translation = false;
    locked.control_allow_yaw = false;
}

} // namespace

TargetTracker::TargetTracker(const TrackingConfig& config)
    : config_(config)
{
    switch (RadarRuntimeConfig::kTrackIdMode) {
    case 1: config_.track_id_mode = TrackIdMode::ConfirmedOnly; break;
    case 2: config_.track_id_mode = TrackIdMode::RecycledDisplay; break;
    case 0:
    default: config_.track_id_mode = TrackIdMode::Monotonic; break;
    }

    // Optional temporary override. When this variable is absent, the value in
    // radar_runtime_config.hpp above is used and plain ./radar_gui is enough.
    if (const char* mode = std::getenv("RADAR_TRACK_ID_MODE")) {
        std::string value(mode);
        std::transform(value.begin(), value.end(), value.begin(),
                       [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        if (value == "confirmed" || value == "confirmed_only" || value == "1") {
            config_.track_id_mode = TrackIdMode::ConfirmedOnly;
        } else if (value == "recycled" || value == "recycle" || value == "2") {
            config_.track_id_mode = TrackIdMode::RecycledDisplay;
        } else if (value == "monotonic" || value == "0") {
            config_.track_id_mode = TrackIdMode::Monotonic;
        }
    }
}

void TargetTracker::reset()
{
    frame_context_ = RadarFrameContext();
    selection_event_ = TargetSelectionEvent();
    last_measurement_time_sec_ = 0;
    candidates_.clear();
    association_candidates_.clear();
    previous_candidates_.clear();
    candidate_tracks_.clear();
    cluster_debug_.clear();
    background_tracks_.clear();
    background_modeling_active_ = false;
    background_filter_enabled_ = false;
    background_model_frame_count_ = 0;
    locked_ = LockedTarget();
    association_diagnostics_ = AssociationDiagnostics();
    candidate_gate_audit_.clear();
    trajectory_association_diagnostics_ =
        TrajectoryAssociation::Diagnostics();
    ego_motion_diagnostics_ = EgoMotionDiagnostics();
    reacquire_hypotheses_.clear();
    has_last_accepted_candidate_ = false;
    last_accepted_candidate_ = CandidateTarget();
    frames_since_accepted_measurement_ = 0;
    candidate_uid_assignment_ambiguous_ = false;
    locked_uid_continuity_compromised_ = false;
    manual_lock_confirmation_pending_ = false;
    manual_lock_confirmation_misses_ = 0;
    clearOcclusionCoast();
    next_candidate_id_ = 1;
    next_internal_uid_ = 1;
    total_track_births_ = 0;
    confirmed_track_births_ = 0;
    max_display_id_seen_ = 0;
    free_display_ids_.clear();
    kalman_initialized_ = false;
    kalman_x_ = 0.0f;
    kalman_y_ = 0.0f;
    kalman_vx_ = 0.0f;
    kalman_vy_ = 0.0f;
    last_chassis_timestamp_sec_ = 0.0;
    current_frame_dt_s_ = std::max(0.01f, config_.kalman_dt_s);
    current_chassis_sample_ = ChassisVelocitySample();
    for (auto& row : kalman_p_) {
        for (float& value : row) {
            value = 0.0f;
        }
    }
}

void TargetTracker::setConfig(const TrackingConfig& config)
{
    config_ = config;
}

const std::vector<CandidateTarget>& TargetTracker::updateCandidates(
    const std::vector<PointCloud>& points,
    const ChassisVelocitySample* synchronized_chassis,
    int current_frame_number)
{
    current_chassis_sample_ = synchronized_chassis
        ? *synchronized_chassis : g_tracking_chassis_velocity_reader.read();
    // Bring all previous-frame states into the current radar coordinate frame
    // before association. This compensates translation and yaw of the moving cart.
    compensatePreviousStateForEgoMotion(current_chassis_sample_);
    previous_candidates_ = candidates_;
    candidates_ = buildCandidates(points, current_frame_number);
    associateCandidateTracks();
    suppressLockedRoiCompanions();

    if (background_modeling_active_) {
        updateBackgroundModel();
        background_model_frame_count_++;
        if (background_model_frame_count_ >= std::max(1, config_.background_model_frames)) {
            background_modeling_active_ = false;
        }
    }
    if (background_filter_enabled_ || background_modeling_active_) {
        applyBackgroundScores();
    }
    association_candidates_ = candidates_;
    // Confirmed-only mode keeps tentative tracks internally for association,
    // but does not expose them to GUI selection or automatic locking.
    candidates_.erase(
        std::remove_if(candidates_.begin(), candidates_.end(),
                       [](const CandidateTarget& candidate) { return candidate.id < 0; }),
        candidates_.end());
    return candidates_;
}

TrackingOutput TargetTracker::output() const {
    TrackingOutput result;
    result.frame = frame_context_;
    result.selection = selection_event_;
    result.target = locked_;
    result.last_measurement_time_sec = last_measurement_time_sec_;
    result.predicted = !locked_.has_measurement;
    result.position_variance_x = kalman_p_[0][0];
    result.position_variance_y = kalman_p_[1][1];
    result.motion_reason = !locked_.valid ? "unlocked" :
        (manual_lock_confirmation_pending_ || !reacquire_hypotheses_.empty()) ? "confirmation_pending" :
        !locked_.control_allow_translation && !locked_.control_allow_yaw ? "missing_measurement_stop" :
        locked_.lost_frames > 0 ? "limited_prediction" : "accepted_measurement";
    if(result.target.valid&&!result.target.has_measurement&&last_measurement_time_sec_>0&&
       frame_context_.sample_time_sec-last_measurement_time_sec_>config_.max_prediction_motion_age_s) {
        result.target.control_state=FollowControlState::StopAndReacquire;
        result.target.control_speed_scale=0;result.target.control_allow_translation=false;result.target.control_allow_yaw=false;
        result.motion_reason="measurement_age_limit";
    }
    return result;
}

std::string TargetTracker::trackIdModeName() const
{
    switch (config_.track_id_mode) {
    case TrackIdMode::ConfirmedOnly: return "confirmed";
    case TrackIdMode::RecycledDisplay: return "recycled";
    case TrackIdMode::Monotonic:
    default: return "monotonic";
    }
}

int TargetTracker::allocateDisplayId()
{
    if (config_.track_id_mode == TrackIdMode::RecycledDisplay && !free_display_ids_.empty()) {
        const int id = free_display_ids_.front();
        free_display_ids_.erase(free_display_ids_.begin());
        confirmed_track_births_++;
        max_display_id_seen_ = std::max(max_display_id_seen_, id);
        return id;
    }
    confirmed_track_births_++;
    const int id = next_candidate_id_++;
    max_display_id_seen_ = std::max(max_display_id_seen_, id);
    return id;
}

void TargetTracker::releaseDisplayId(int id)
{
    if (config_.track_id_mode != TrackIdMode::RecycledDisplay || id <= 0 ||
        (locked_.valid && id == locked_.display_id)) {
        return;
    }
    if (std::find(free_display_ids_.begin(), free_display_ids_.end(), id) == free_display_ids_.end()) {
        free_display_ids_.push_back(id);
        std::sort(free_display_ids_.begin(), free_display_ids_.end());
    }
}

void TargetTracker::compensatePreviousStateForEgoMotion(
    const ChassisVelocitySample& chassis)
{
    ego_motion_diagnostics_ = EgoMotionDiagnostics();
    ego_motion_diagnostics_.enabled = ego_motion_compensation_enabled_;
    if (!ego_motion_compensation_enabled_) {
        last_chassis_timestamp_sec_ = 0.0;
        current_frame_dt_s_ = std::max(0.01f, config_.kalman_dt_s);
        return;
    }
    current_frame_dt_s_ = std::max(0.01f, config_.kalman_dt_s);
    ego_motion_diagnostics_.odom_valid = chassis.valid && chassis.has_timestamp;
    ego_motion_diagnostics_.chassis_vx_mps = chassis.vx_mps;
    ego_motion_diagnostics_.chassis_wz_radps = chassis.wz_radps;
    if (!chassis.valid || !chassis.has_timestamp) {
        return;
    }

    if (last_chassis_timestamp_sec_ <= 0.0) {
        last_chassis_timestamp_sec_ = chassis.timestamp_sec;
        return;
    }

    const double raw_dt = chassis.timestamp_sec - last_chassis_timestamp_sec_;
    last_chassis_timestamp_sec_ = chassis.timestamp_sec;
    if (raw_dt <= 0.0 || raw_dt > 0.50) {
        return;
    }

    const float dt = static_cast<float>(raw_dt);
    current_frame_dt_s_ = dt;
    const auto motion = RadarMotion::transform(chassis.vx_mps, chassis.wz_radps, dt);
    const float dx = motion.dx, dy = motion.dy, dtheta = motion.yaw;
    ego_motion_diagnostics_.applied = true;
    ego_motion_diagnostics_.dt_s = dt;
    ego_motion_diagnostics_.radar_dx_m = dx;
    ego_motion_diagnostics_.radar_dy_m = dy;
    ego_motion_diagnostics_.radar_dyaw_rad = dtheta;
    const float c = std::cos(dtheta), s = std::sin(dtheta);
    auto transform_position = [&](float& x, float& y) { motion.position(x, y); };
    auto rotate_vector = [&](float& x, float& y) { motion.vector(x, y); };

    for (auto& track : candidate_tracks_) {
        transform_position(track.x, track.y);
        rotate_vector(track.vx, track.vy);
    }
    for (auto& track : background_tracks_) {
        transform_position(track.x, track.y);
    }
    if (locked_.valid) {
        transform_position(locked_.x, locked_.y);
        rotate_vector(locked_.vx, locked_.vy);
        if (has_last_accepted_candidate_) {
            transform_position(last_accepted_candidate_.x, last_accepted_candidate_.y);
            last_accepted_candidate_.range_m = std::hypot(last_accepted_candidate_.x,
                                                           last_accepted_candidate_.y);
            last_accepted_candidate_.angle_deg =
                std::atan2(last_accepted_candidate_.x, last_accepted_candidate_.y) *
                180.0f / static_cast<float>(M_PI);
        }
        for (auto& hypothesis : reacquire_hypotheses_) {
            transform_position(hypothesis.latest.x,
                               hypothesis.latest.y);
            rotate_vector(hypothesis.vx, hypothesis.vy);
            hypothesis.latest.range_m =
                std::hypot(hypothesis.latest.x,
                           hypothesis.latest.y);
            hypothesis.latest.angle_deg =
                std::atan2(hypothesis.latest.x,
                           hypothesis.latest.y) *
                180.0f / static_cast<float>(M_PI);
        }
        locked_.range_m = std::hypot(locked_.x, locked_.y);
        locked_.angle_deg = std::atan2(locked_.x, locked_.y) * 180.0f /
                            static_cast<float>(M_PI);
    }
    if (kalman_initialized_) {
        transform_position(kalman_x_, kalman_y_);
        rotate_vector(kalman_vx_, kalman_vy_);

        float j[4][4] = {
            {c, s, 0.0f, 0.0f}, {-s, c, 0.0f, 0.0f},
            {0.0f, 0.0f, c, s}, {0.0f, 0.0f, -s, c}
        };
        float jp[4][4] = {};
        float rotated_p[4][4] = {};
        for (int r = 0; r < 4; ++r)
            for (int col = 0; col < 4; ++col)
                for (int k = 0; k < 4; ++k)
                    jp[r][col] += j[r][k] * kalman_p_[k][col];
        for (int r = 0; r < 4; ++r)
            for (int col = 0; col < 4; ++col)
                for (int k = 0; k < 4; ++k)
                    rotated_p[r][col] += jp[r][k] * j[col][k];
        for (int r = 0; r < 4; ++r)
            for (int col = 0; col < 4; ++col)
                kalman_p_[r][col] = rotated_p[r][col];
    }
    if (occlusion_coast_active_) {
        transform_position(occlusion_anchor_x_, occlusion_anchor_y_);
        if (occlusion_has_direction_) {
            rotate_vector(occlusion_dir_x_, occlusion_dir_y_);
            const float norm = std::hypot(occlusion_dir_x_, occlusion_dir_y_);
            if (norm > 1e-4f) {
                occlusion_dir_x_ /= norm;
                occlusion_dir_y_ /= norm;
            } else {
                occlusion_has_direction_ = false;
            }
        }
    }
}

void TargetTracker::beginBackgroundModeling()
{
    background_tracks_.clear();
    background_model_frame_count_ = 0;
    background_modeling_active_ = true;
}

void TargetTracker::stopBackgroundModeling()
{
    background_modeling_active_ = false;
}

void TargetTracker::clearBackgroundModel()
{
    background_tracks_.clear();
    background_model_frame_count_ = 0;
    background_modeling_active_ = false;
}

void TargetTracker::setBackgroundFilterEnabled(bool enabled)
{
    background_filter_enabled_ = enabled;
}

int TargetTracker::backgroundModelTrackCount() const
{
    return static_cast<int>(background_tracks_.size());
}

bool TargetTracker::selectTargetById(int candidate_id)
{
    const CandidateTarget* candidate = findCandidateById(candidate_id);
    if (!candidate) return false;
    beginManualLockConfirmation(*candidate);
    return true;
}

bool TargetTracker::selectTargetByUid(std::uint64_t internal_uid)
{
    const CandidateTarget* candidate = findCandidateByUid(internal_uid);
    if (!candidate) return false;
    beginManualLockConfirmation(*candidate);
    return true;
}

bool TargetTracker::selectTargetNear(float x, float y, float max_distance_m)
{
    const CandidateTarget* best = nullptr;
    float best_d2 = max_distance_m * max_distance_m;

    for (const auto& candidate : candidates_) {
        const float d2 = squaredDistance(x, y, candidate.x, candidate.y);
        if (d2 <= best_d2) {
            best_d2 = d2;
            best = &candidate;
        }
    }

    if (!best) return false;
    beginManualLockConfirmation(*best);
    return true;
}

void TargetTracker::unlockTarget(const std::string& reason)
{
    const int previous_display_id = locked_.display_id;
    const std::uint64_t previous_internal_uid = locked_.matched_internal_uid;
    AssociationDiagnostics previous_diagnostics = association_diagnostics_;
    locked_ = LockedTarget();
    clearReacquireHypothesis();
    has_last_accepted_candidate_ = false;
    last_accepted_candidate_ = CandidateTarget();
    frames_since_accepted_measurement_ = 0;
    locked_uid_continuity_compromised_ = false;
    manual_lock_confirmation_pending_ = false;
    manual_lock_confirmation_misses_ = 0;
    clearOcclusionCoast();
    kalman_initialized_ = false;
    kalman_x_ = 0.0f;
    kalman_y_ = 0.0f;
    kalman_vx_ = 0.0f;
    kalman_vy_ = 0.0f;
    for (auto& row : kalman_p_) {
        for (float& value : row) {
            value = 0.0f;
        }
    }
    // Preserve the last association measurements in the audit row while making
    // the release event and the identity that was released explicit.
    association_diagnostics_ = previous_diagnostics;
    association_diagnostics_.source = reason.empty() ? "manual_unlock" : reason;
    association_diagnostics_.candidate_display_id = previous_display_id;
    association_diagnostics_.candidate_internal_uid = previous_internal_uid;
}

bool TargetTracker::autoSelect(AutoSelectRule rule)
{
    if (candidates_.empty()) return false;

    auto best = candidates_.begin();
    if (rule == AutoSelectRule::Nearest) {
        best = std::min_element(candidates_.begin(), candidates_.end(),
                                [](const CandidateTarget& a, const CandidateTarget& b) {
                                    const float effective_range_a = a.range_m + a.background_score * 5.0f;
                                    const float effective_range_b = b.range_m + b.background_score * 5.0f;
                                    return effective_range_a < effective_range_b;
                                });
    } else if (rule == AutoSelectRule::Fastest) {
        best = std::max_element(candidates_.begin(), candidates_.end(),
                                [](const CandidateTarget& a, const CandidateTarget& b) {
                                    const float effective_velocity_a = a.average_abs_velocity - a.background_score * 0.5f;
                                    const float effective_velocity_b = b.average_abs_velocity - b.background_score * 0.5f;
                                    return effective_velocity_a < effective_velocity_b;
                                });
    } else {
        auto moving_begin = std::partition(candidates_.begin(), candidates_.end(),
                                           [this](const CandidateTarget& c) {
                                               return isActivelyMoving(c, config_);
                                           });
        auto search_begin = candidates_.begin();
        auto search_end = moving_begin == candidates_.begin() ? candidates_.end() : moving_begin;

        best = std::max_element(search_begin, search_end,
                                [](const CandidateTarget& a, const CandidateTarget& b) {
                                    const float motion_a = motionPriority(a);
                                    const float motion_b = motionPriority(b);
                                    if (std::abs(motion_a - motion_b) > 0.05f) {
                                        return motion_a < motion_b;
                                    }
                                    if (std::abs(a.score - b.score) > 0.02f) {
                                        return a.score < b.score;
                                    }
                                    return a.range_m > b.range_m;
                                });
    }

    lockFromCandidate(*best, true);
    association_diagnostics_.source = "auto_lock";
    return true;
}

LockedTarget TargetTracker::updateLockedTarget()
{
    candidate_gate_audit_.clear();
    association_diagnostics_ = AssociationDiagnostics();
    association_diagnostics_.considered_candidates = static_cast<int>(candidates_.size());
    if (!locked_.valid) {
        association_diagnostics_.source = "unlocked";
        return locked_;
    }

    if (!kalman_initialized_) {
        initKalmanFromLocked();
    }

    float predicted_x = locked_.x;
    float predicted_y = locked_.y;
    predictKalman(predicted_x, predicted_y);
    association_diagnostics_.predicted_x = predicted_x;
    association_diagnostics_.predicted_y = predicted_y;
    recordCandidateGateAudit(predicted_x, predicted_y);

    MatchResult match = findBestCandidateWithKalmanGate(predicted_x, predicted_y);
    if (!match.candidate) {
        MatchResult roi_match = findBestCandidateInRoi(predicted_x, predicted_y);
        roi_match.considered_candidates = std::max(match.considered_candidates,
                                                   roi_match.considered_candidates);
        if (!roi_match.candidate && roi_match.reject_reason.empty()) {
            roi_match.reject_reason = match.reject_reason;
        }
        // A valid ROI fallback must carry its own finite NIS/kinematic result.
        // Do not contaminate it with the earlier normal-gate rejection flag.
        // This previously produced accepted/reacquired rows with a stale
        // rejection reason and made unsafe hypotheses hard to audit.
        if (!roi_match.candidate) {
            roi_match.rejected = roi_match.rejected || match.rejected;
        }
        match = roi_match;
    }

    if (!match.candidate && candidate_uid_assignment_ambiguous_) {
        match.rejected = true;
        match.reject_reason = "candidate_uid_ambiguous";
    }

    // An accepted pointer and its innovation/NIS must describe the same
    // candidate.  Keep this as a runtime fail-safe because a mixed result can
    // steer the KF away from a target that is still plainly inside the gate.
    if (match.candidate &&
        !isAcceptedMatchConsistent(match, predicted_x, predicted_y)) {
        match.rejected = true;
        match.reject_reason = "internal_match_inconsistent";
        match.candidate = nullptr;
    }

    const bool identity_changed =
        match.candidate &&
        locked_.matched_internal_uid != 0 &&
        match.candidate->internal_uid != locked_.matched_internal_uid;
    const bool suspicious_continuity =
        match.candidate &&
        isSuspiciousContinuityMatch(*match.candidate, match);
    // A continuous original UID still updates immediately.  After a dropout,
    // identity change or suspicious jump, the occlusion path keeps the old
    // direction and will not hand the lock to another person.
    const bool recovering_from_loss = locked_.lost_frames > 0;
    const bool short_gap_continuity = match.candidate && recovering_from_loss &&
        locked_.lost_frames <= config_.short_gap_same_uid_max_lost_frames &&
        !identity_changed && !suspicious_continuity && !locked_uid_continuity_compromised_ &&
        !manual_lock_confirmation_pending_ && reacquire_hypotheses_.empty() &&
        match.candidate->current_frame_point_count >= 2;
    if (short_gap_continuity) {
        match.source = "short_gap_same_uid";
        acceptMeasurement(*match.candidate, match, true);
        return locked_;
    }
    if (manual_lock_confirmation_pending_) {
        const std::vector<MatchResult> matches =
            collectReacquireMatches(predicted_x, predicted_y);
        updateReacquireHypotheses(matches, predicted_x, predicted_y,
                                  match.reject_reason);
        return locked_;
    }

    // Keep the task identity, but a changed UID must pass the existing
    // shape/kinematic gates and win multi-frame confirmation. Previously the
    // original-UID-only branch bypassed this path on every dropout.
    const bool recovering = !match.candidate || identity_changed || recovering_from_loss ||
        suspicious_continuity || locked_uid_continuity_compromised_ || candidate_uid_assignment_ambiguous_;
    if (recovering) {
        ensureOcclusionCoast();
        if (candidate_uid_assignment_ambiguous_) {
            clearReacquireHypothesis();
            handleTrackingMiss(predicted_x,predicted_y,"candidate_uid_ambiguous");
            locked_.control_state=FollowControlState::StopAndReacquire;
            locked_.control_speed_scale=0;locked_.control_allow_translation=false;locked_.control_allow_yaw=false;
            return locked_;
        }
        const auto matches=collectReacquireMatches(predicted_x,predicted_y);
        updateReacquireHypotheses(matches,predicted_x,predicted_y,
                                  match.reject_reason.empty()?"awaiting_identity_confirmation":match.reject_reason);
        if (!locked_.has_measurement && reacquire_hypotheses_.empty())applyOcclusionCoastOutput();
        return locked_;
    }

    if (!reacquire_hypotheses_.empty()) {
        const std::vector<MatchResult> matches =
            collectReacquireMatches(predicted_x, predicted_y);
        updateReacquireHypotheses(matches, predicted_x, predicted_y,
                                  match.reject_reason);
    } else {
        acceptMeasurement(*match.candidate, match, true);
    }
    return locked_;
}

void TargetTracker::clearOcclusionCoast()
{
    occlusion_coast_active_ = false;
    occlusion_has_direction_ = false;
    occlusion_anchor_x_ = 0.0f;
    occlusion_anchor_y_ = 0.0f;
    occlusion_dir_x_ = 0.0f;
    occlusion_dir_y_ = 0.0f;
}

void TargetTracker::ensureOcclusionCoast()
{
    if (occlusion_coast_active_ || manual_lock_confirmation_pending_) return;
    occlusion_coast_active_ = true;
    if (has_last_accepted_candidate_) {
        occlusion_anchor_x_ = last_accepted_candidate_.x;
        occlusion_anchor_y_ = last_accepted_candidate_.y;
    } else {
        occlusion_anchor_x_ = locked_.x;
        occlusion_anchor_y_ = locked_.y;
    }

    const float speed = std::hypot(kalman_vx_, kalman_vy_);
    if (kalman_initialized_ && speed >= config_.occlusion_min_heading_speed_mps) {
        occlusion_dir_x_ = kalman_vx_ / speed;
        occlusion_dir_y_ = kalman_vy_ / speed;
        occlusion_has_direction_ = true;
        return;
    }

    // A well-matched follow has almost no radar-frame velocity. Keep the
    // direction the cart was already facing: from the radar toward the person.
    const float range = std::hypot(occlusion_anchor_x_, occlusion_anchor_y_);
    if (range >= 0.50f) {
        occlusion_dir_x_ = occlusion_anchor_x_ / range;
        occlusion_dir_y_ = occlusion_anchor_y_ / range;
        occlusion_has_direction_ = true;
        return;
    }
    occlusion_has_direction_ = false;
    occlusion_dir_x_ = 0.0f;
    occlusion_dir_y_ = 0.0f;
}

bool TargetTracker::occlusionSearchContains(float x, float y) const
{
    const float dx = x - occlusion_anchor_x_;
    const float dy = y - occlusion_anchor_y_;
    const float distance = std::hypot(dx, dy);
    if (distance <= config_.occlusion_stop_radius_m) return true;
    if (!occlusion_has_direction_) {
        return distance <= std::max(config_.occlusion_stop_radius_m,
                                    config_.occlusion_search_lateral_m);
    }
    const float along = dx * occlusion_dir_x_ + dy * occlusion_dir_y_;
    const float lateral = dx * (-occlusion_dir_y_) + dy * occlusion_dir_x_;
    return along >= -config_.occlusion_search_back_m &&
           along <= config_.occlusion_search_forward_m &&
           std::abs(lateral) <= config_.occlusion_search_lateral_m;
}

CandidateTarget* TargetTracker::findOriginalUidInOcclusionRegion()
{
    if (!occlusion_coast_active_ || locked_.matched_internal_uid == 0) return nullptr;
    CandidateTarget* best = nullptr;
    float best_d2 = std::numeric_limits<float>::infinity();
    for (auto& candidate : association_candidates_) {
        if (candidate.internal_uid != locked_.matched_internal_uid) continue;
        if (!occlusionSearchContains(candidate.x, candidate.y)) continue;
        const float d2 = squaredDistance(candidate.x, candidate.y,
                                         occlusion_anchor_x_, occlusion_anchor_y_);
        if (d2 < best_d2) {
            best_d2 = d2;
            best = &candidate;
        }
    }
    return best;
}

void TargetTracker::applyOcclusionCoastOutput()
{
    if (!occlusion_coast_active_ || !locked_.valid || locked_.has_measurement ||
        manual_lock_confirmation_pending_) {
        return;
    }

    const float dt = std::max(0.05f, current_frame_dt_s_);
    float travel = 0.0f;
    if (occlusion_has_direction_ && locked_.lost_frames > 0) {
        travel = std::min(config_.occlusion_max_forward_m,
                          std::max(0.0f, config_.occlusion_coast_rate_mps) *
                              static_cast<float>(locked_.lost_frames) * dt);
    }
    locked_.x = occlusion_anchor_x_ + occlusion_dir_x_ * travel;
    locked_.y = occlusion_anchor_y_ + occlusion_dir_y_ * travel;
    locked_.vx = occlusion_has_direction_
        ? occlusion_dir_x_ * config_.occlusion_coast_rate_mps : 0.0f;
    locked_.vy = occlusion_has_direction_
        ? occlusion_dir_y_ * config_.occlusion_coast_rate_mps : 0.0f;
    locked_.range_m = std::hypot(locked_.x, locked_.y);
    locked_.angle_deg = std::atan2(locked_.x, locked_.y) *
                        180.0f / static_cast<float>(M_PI);
    locked_.has_measurement = false;

    const double measurementAge=frame_context_.sample_time_sec-last_measurement_time_sec_;
    if (locked_.lost_frames > config_.occlusion_coast_frames ||
        (last_measurement_time_sec_>0&&measurementAge>config_.max_prediction_motion_age_s)) {
        locked_.control_state = FollowControlState::StopAndReacquire;
        locked_.control_speed_scale = 0.0f;
        locked_.control_allow_translation = false;
        locked_.control_allow_yaw = false;
        association_diagnostics_.source = "occlusion_coast_stop";
        return;
    }

    locked_.control_state = FollowControlState::LimitedFollow;
    locked_.control_speed_scale = std::clamp(config_.occlusion_speed_scale, 0.0f, 1.0f);
    locked_.control_allow_translation = true;
    locked_.control_allow_yaw = true;
    association_diagnostics_.source = "occlusion_coast";
}

void TargetTracker::handleTrackingMiss(float predicted_x, float predicted_y,
                                       const std::string& reject_reason)
{
    frames_since_accepted_measurement_++;
    locked_.lost_frames++;
    applyMissingMeasurementVelocityPolicy();
    locked_.stable_frames = 0;
    locked_.has_measurement = false;
    locked_.x = predicted_x;
    locked_.y = predicted_y;
    locked_.vx = kalman_vx_;
    locked_.vy = kalman_vy_;
    locked_.range_m = std::hypot(locked_.x, locked_.y);
    locked_.angle_deg = std::atan2(locked_.x, locked_.y) *
                        180.0f / static_cast<float>(M_PI);
    locked_.confidence = std::max(0.0f,
                                  locked_.confidence - config_.confidence_miss_drop);
    for (auto& hypothesis : reacquire_hypotheses_) {
        hypothesis.window_frames++;
        hypothesis.frames_since_last_hit++;
    }
    pruneAndLimitReacquireHypotheses();
    const bool preserve_reacquire_window = !reacquire_hypotheses_.empty();
    locked_.reacquire_hits = 0;
    if (preserve_reacquire_window) {
        const auto best_it = std::max_element(
            reacquire_hypotheses_.begin(), reacquire_hypotheses_.end(),
            [this](const ReacquireHypothesis& a,
                   const ReacquireHypothesis& b) {
                return reacquireHypothesisScore(a) <
                       reacquireHypothesisScore(b);
            });
        locked_.reacquire_hits = best_it->consecutive_hits;
    }
    association_diagnostics_.source = association_diagnostics_.measurement_rejected
                                          ? "rejected"
                                          : "prediction_only";
    if (!reject_reason.empty()) {
        association_diagnostics_.reject_reason = reject_reason;
    }
    if (manual_lock_confirmation_pending_) {
        manual_lock_confirmation_misses_++;
        association_diagnostics_.source = "manual_lock_pending";
        fillBestReacquireDiagnostics();
        updateFollowControlAdvice(locked_, config_);
        if (locked_.lost_frames <= config_.lost_limited_follow_frames) {
            locked_.control_state = FollowControlState::ReacquireConfirming;
        }
        holdManualConfirmationMotion();
        if (manual_lock_confirmation_misses_ >
            std::max(0, config_.manual_lock_confirm_max_missed_frames)) {
            unlockTarget("manual_lock_confirmation_failed");
        }
        return;
    }

    if (preserve_reacquire_window) {
        association_diagnostics_.source = "reacquire_pending_gap";
        fillBestReacquireDiagnostics();
        updateFollowControlAdvice(locked_, config_);
        if (locked_.lost_frames <= config_.lost_limited_follow_frames) {
            locked_.control_state = FollowControlState::ReacquireConfirming;
        }
        holdManualConfirmationMotion();
        return;
    }

    updateFollowControlAdvice(locked_, config_);
    if (locked_.lost_frames > config_.max_lost_frames) {
        locked_.valid = false;
        kalman_initialized_ = false;
        association_diagnostics_.source = "tracker_timeout_unlock";
    }
}

void TargetTracker::beginOrAdvanceReacquire(const CandidateTarget& candidate,
                                            const MatchResult& match,
                                            float predicted_x,
                                            float predicted_y)
{
    MatchResult candidate_match = match;
    candidate_match.candidate = const_cast<CandidateTarget*>(&candidate);
    updateReacquireHypotheses({candidate_match}, predicted_x, predicted_y,
                              match.reject_reason);
}

std::vector<TargetTracker::MatchResult>
TargetTracker::collectReacquireMatches(float predicted_x, float predicted_y)
{
    std::vector<MatchResult> matches;
    matches.reserve(association_candidates_.size());

    for (auto& candidate : association_candidates_) {
        if (candidate.internal_uid == 0) continue;
        if (manual_lock_confirmation_pending_ &&
            candidate.internal_uid != locked_.matched_internal_uid) {
            continue;
        }

        float ex = 0.0f;
        float ey = 0.0f;
        float nis = 0.0f;
        float speed = 0.0f;
        std::string reason;
        bool roi_fallback = false;
        bool accepted = evaluateCandidateMeasurement(
            candidate, predicted_x, predicted_y, false,
            ex, ey, nis, speed, reason);
        if (!accepted) {
            roi_fallback = true;
            accepted = evaluateCandidateMeasurement(
                candidate, predicted_x, predicted_y, true,
                ex, ey, nis, speed, reason);
        }
        if (!accepted) continue;

        MatchResult match;
        match.candidate = &candidate;
        match.source = roi_fallback ? "reacquire_roi" : "reacquire_kalman";
        match.innovation_x = ex;
        match.innovation_y = ey;
        match.nis = nis;
        match.implied_speed_mps = speed;
        match.considered_candidates =
            static_cast<int>(association_candidates_.size());

        const bool new_uid = locked_.matched_internal_uid != 0 &&
            candidate.internal_uid != locked_.matched_internal_uid;
        if (new_uid || locked_uid_continuity_compromised_) {
            if (!passesNewUidReacquireChecks(
                    candidate, match, predicted_x, predicted_y, reason)) {
                continue;
            }
        }

        match.cost = nis + 0.04f * speed * speed +
                     0.20f * candidate.background_score -
                     0.08f * candidate.score -
                     0.10f * candidate.moving_ratio -
                     0.03f * candidate.average_abs_velocity;
        if (!std::isfinite(match.cost)) continue;

        auto same_uid = std::find_if(
            matches.begin(), matches.end(),
            [&candidate](const MatchResult& existing) {
                return existing.candidate &&
                    existing.candidate->internal_uid == candidate.internal_uid;
            });
        if (same_uid == matches.end()) {
            matches.push_back(match);
        } else if (match.cost < same_uid->cost) {
            *same_uid = match;
        }
    }

    const int gate_count = static_cast<int>(matches.size());
    for (auto& match : matches) {
        match.gate_candidate_count = gate_count;
    }
    std::sort(matches.begin(), matches.end(),
              [](const MatchResult& a, const MatchResult& b) {
                  return a.cost < b.cost;
              });
    return matches;
}

int TargetTracker::reacquireRequiredHits(
    const ReacquireHypothesis& hypothesis) const
{
    if (manual_lock_confirmation_pending_) {
        return std::max(2, config_.manual_lock_confirm_frames);
    }
    return hypothesis.original_uid
        ? std::max(1, config_.reacquire_confirm_frames)
        : std::max(1, config_.new_uid_reacquire_confirm_frames);
}

int TargetTracker::reacquireWindowLimit(
    const ReacquireHypothesis& hypothesis) const
{
    const int required = reacquireRequiredHits(hypothesis);
    if (manual_lock_confirmation_pending_) {
        return std::max(required,
                        required +
                        std::max(0, config_.manual_lock_confirm_max_missed_frames));
    }
    const int configured = hypothesis.original_uid
        ? config_.reacquire_window_frames
        : config_.new_uid_reacquire_window_frames;
    return std::max(required, configured);
}

float TargetTracker::reacquireHypothesisScore(
    const ReacquireHypothesis& hypothesis) const
{
    const float mean_cost = hypothesis.consecutive_hits > 0
        ? hypothesis.cost_sum /
              static_cast<float>(hypothesis.consecutive_hits)
        : std::numeric_limits<float>::max();
    float score = 3.0f * static_cast<float>(hypothesis.consecutive_hits) -
                  0.35f * mean_cost -
                  0.40f * static_cast<float>(hypothesis.frames_since_last_hit);
    if (hypothesis.original_uid) {
        score += config_.reacquire_original_uid_score_bonus;
    }
    return score;
}

void TargetTracker::pruneAndLimitReacquireHypotheses()
{
    reacquire_hypotheses_.erase(
        std::remove_if(
            reacquire_hypotheses_.begin(), reacquire_hypotheses_.end(),
            [this](const ReacquireHypothesis& hypothesis) {
                if (!hypothesis.active || hypothesis.internal_uid == 0) {
                    return true;
                }
                const int required = reacquireRequiredHits(hypothesis);
                const int window_limit = reacquireWindowLimit(hypothesis);
                if (hypothesis.window_frames > window_limit) return true;
                const int remaining = std::max(
                    0, window_limit - hypothesis.window_frames);
                return hypothesis.consecutive_hits + remaining < required;
            }),
        reacquire_hypotheses_.end());

    std::stable_sort(
        reacquire_hypotheses_.begin(), reacquire_hypotheses_.end(),
        [this](const ReacquireHypothesis& a,
               const ReacquireHypothesis& b) {
            return reacquireHypothesisScore(a) >
                   reacquireHypothesisScore(b);
        });
    const std::size_t max_count = static_cast<std::size_t>(
        std::max(1, config_.max_reacquire_hypotheses));
    if (reacquire_hypotheses_.size() > max_count) {
        reacquire_hypotheses_.resize(max_count);
    }
}

void TargetTracker::fillBestReacquireDiagnostics()
{
    association_diagnostics_.reacquire_hypothesis_count =
        static_cast<int>(reacquire_hypotheses_.size());
    association_diagnostics_.pending_internal_uid = 0;
    association_diagnostics_.pending_hits = 0;
    association_diagnostics_.pending_window_frames = 0;
    association_diagnostics_.reacquire_best_score = 0.0f;
    association_diagnostics_.reacquire_second_score = 0.0f;
    association_diagnostics_.reacquire_score_margin = 0.0f;
    if (reacquire_hypotheses_.empty()) return;

    std::vector<const ReacquireHypothesis*> ranked;
    ranked.reserve(reacquire_hypotheses_.size());
    for (const auto& hypothesis : reacquire_hypotheses_) {
        ranked.push_back(&hypothesis);
    }
    std::sort(ranked.begin(), ranked.end(),
              [this](const ReacquireHypothesis* a,
                     const ReacquireHypothesis* b) {
                  return reacquireHypothesisScore(*a) >
                         reacquireHypothesisScore(*b);
              });

    const ReacquireHypothesis& best = *ranked.front();
    const float best_score = reacquireHypothesisScore(best);
    association_diagnostics_.pending_internal_uid = best.internal_uid;
    association_diagnostics_.pending_hits = best.consecutive_hits;
    association_diagnostics_.pending_window_frames = best.window_frames;
    association_diagnostics_.reacquire_best_score = best_score;
    if (ranked.size() > 1) {
        const float second_score = reacquireHypothesisScore(*ranked[1]);
        association_diagnostics_.reacquire_second_score = second_score;
        association_diagnostics_.reacquire_score_margin =
            best_score - second_score;
    } else {
        association_diagnostics_.reacquire_score_margin = 999.0f;
    }
}

void TargetTracker::updateReacquireHypotheses(
    const std::vector<MatchResult>& input_matches,
    float predicted_x,
    float predicted_y,
    const std::string& reject_reason)
{
    std::vector<MatchResult> matches;
    matches.reserve(input_matches.size());
    for (const MatchResult& input : input_matches) {
        if (!input.candidate || input.rejected ||
            !std::isfinite(input.nis) ||
            !std::isfinite(input.implied_speed_mps)) {
            continue;
        }
        if (manual_lock_confirmation_pending_ &&
            input.candidate->internal_uid != locked_.matched_internal_uid) {
            continue;
        }

        MatchResult match = input;
        const bool new_uid = locked_.matched_internal_uid != 0 &&
            input.candidate->internal_uid != locked_.matched_internal_uid;
        if (new_uid || locked_uid_continuity_compromised_) {
            std::string reason;
            if (!passesNewUidReacquireChecks(
                    *input.candidate, match,
                    predicted_x, predicted_y, reason)) {
                continue;
            }
        }
        if (!std::isfinite(match.cost)) {
            match.cost = match.nis +
                         0.04f * match.implied_speed_mps *
                             match.implied_speed_mps;
        }
        matches.push_back(match);
    }

    if (matches.empty()) {
        handleTrackingMiss(predicted_x, predicted_y, reject_reason);
        return;
    }

    // This frame contains plausible observations, but none of them is a main
    // KF measurement until a hypothesis wins. Count one prediction-only frame,
    // never one miss per candidate.
    frames_since_accepted_measurement_++;
    locked_.lost_frames++;
    applyMissingMeasurementVelocityPolicy();

    for (auto& hypothesis : reacquire_hypotheses_) {
        hypothesis.window_frames++;
        hypothesis.frames_since_last_hit++;
    }

    const float dt = std::max(0.05f, current_frame_dt_s_);
    for (const MatchResult& match : matches) {
        const CandidateTarget& candidate = *match.candidate;
        auto hypothesis_it = std::find_if(
            reacquire_hypotheses_.begin(), reacquire_hypotheses_.end(),
            [&candidate](const ReacquireHypothesis& hypothesis) {
                return hypothesis.internal_uid == candidate.internal_uid;
            });

        const bool original_uid =
            candidate.internal_uid == locked_.matched_internal_uid;
        if (hypothesis_it == reacquire_hypotheses_.end()) {
            ReacquireHypothesis hypothesis;
            hypothesis.internal_uid = candidate.internal_uid;
            hypothesis.candidate_display_id = candidate.id;
            hypothesis.consecutive_hits = 1;
            hypothesis.window_frames = 1;
            hypothesis.frames_since_last_hit = 0;
            hypothesis.allow_one_frame_gap = true;
            hypothesis.latest = candidate;
            hypothesis.nis_sum = match.nis;
            hypothesis.cost_sum = match.cost;
            hypothesis.last_cost = match.cost;
            hypothesis.original_uid = original_uid;
            reacquire_hypotheses_.push_back(hypothesis);
            continue;
        }

        ReacquireHypothesis& hypothesis = *hypothesis_it;
        if (!isReacquireSequenceContinuous(candidate, hypothesis)) {
            ReacquireHypothesis restarted;
            restarted.internal_uid = candidate.internal_uid;
            restarted.candidate_display_id = candidate.id;
            restarted.consecutive_hits = 1;
            restarted.window_frames = 1;
            restarted.latest = candidate;
            restarted.nis_sum = match.nis;
            restarted.cost_sum = match.cost;
            restarted.last_cost = match.cost;
            restarted.original_uid = original_uid;
            hypothesis = restarted;
            continue;
        }

        const float hit_dt = dt * static_cast<float>(
            std::max(1, hypothesis.frames_since_last_hit));
        const float measured_vx =
            (candidate.x - hypothesis.latest.x) / hit_dt;
        const float measured_vy =
            (candidate.y - hypothesis.latest.y) / hit_dt;
        if (hypothesis.consecutive_hits <= 1) {
            hypothesis.vx = measured_vx;
            hypothesis.vy = measured_vy;
        } else {
            hypothesis.vx = 0.5f * hypothesis.vx + 0.5f * measured_vx;
            hypothesis.vy = 0.5f * hypothesis.vy + 0.5f * measured_vy;
        }
        hypothesis.latest = candidate;
        hypothesis.candidate_display_id = candidate.id;
        hypothesis.consecutive_hits++;
        hypothesis.frames_since_last_hit = 0;
        hypothesis.nis_sum += match.nis;
        hypothesis.cost_sum += match.cost;
        hypothesis.last_cost = match.cost;
    }

    pruneAndLimitReacquireHypotheses();
    if (reacquire_hypotheses_.empty()) {
        locked_.has_measurement = false;
        locked_.x = predicted_x;
        locked_.y = predicted_y;
        locked_.vx = kalman_vx_;
        locked_.vy = kalman_vy_;
        locked_.range_m = std::hypot(predicted_x, predicted_y);
        locked_.angle_deg = std::atan2(predicted_x, predicted_y) *
                            180.0f / static_cast<float>(M_PI);
        locked_.stable_frames = 0;
        locked_.reacquire_hits = 0;
        locked_.confidence = std::max(
            0.0f, locked_.confidence - config_.confidence_miss_drop);
        association_diagnostics_.source = "reacquire_hypotheses_expired";
        association_diagnostics_.reject_reason = reject_reason;
        updateFollowControlAdvice(locked_, config_);
        holdManualConfirmationMotion();
        if (locked_.lost_frames > config_.max_lost_frames) {
            locked_.valid = false;
            kalman_initialized_ = false;
            association_diagnostics_.source = "tracker_timeout_unlock";
        }
        return;
    }

    const ReacquireHypothesis* best_eligible = nullptr;
    float best_eligible_score = -std::numeric_limits<float>::infinity();
    float best_competitor_score = -std::numeric_limits<float>::infinity();
    for (const auto& hypothesis : reacquire_hypotheses_) {
        const float score = reacquireHypothesisScore(hypothesis);
        if (hypothesis.frames_since_last_hit == 0 &&
            hypothesis.consecutive_hits >=
                reacquireRequiredHits(hypothesis) &&
            score > best_eligible_score) {
            best_eligible = &hypothesis;
            best_eligible_score = score;
        }
    }
    if (best_eligible) {
        for (const auto& hypothesis : reacquire_hypotheses_) {
            if (&hypothesis == best_eligible) continue;
            best_competitor_score = std::max(
                best_competitor_score,
                reacquireHypothesisScore(hypothesis));
        }

        const bool has_competitor =
            std::isfinite(best_competitor_score);
        const float margin = has_competitor
            ? best_eligible_score - best_competitor_score
            : std::numeric_limits<float>::infinity();
        const bool unambiguous =
            best_eligible->original_uid || !has_competitor ||
            margin >= config_.reacquire_min_score_margin;
        if (unambiguous) {
            auto committed_match = std::find_if(
                matches.begin(), matches.end(),
                [best_eligible](const MatchResult& match) {
                    return match.candidate &&
                        match.candidate->internal_uid ==
                            best_eligible->internal_uid;
                });
            if (committed_match != matches.end()) {
                const ReacquireHypothesis confirmed = *best_eligible;
                commitReacquireHypothesis(confirmed, *committed_match);
                return;
            }
        }
    }

    auto best_it = std::max_element(
        reacquire_hypotheses_.begin(), reacquire_hypotheses_.end(),
        [this](const ReacquireHypothesis& a,
               const ReacquireHypothesis& b) {
            return reacquireHypothesisScore(a) <
                   reacquireHypothesisScore(b);
        });
    const MatchResult* diagnostic_match = nullptr;
    for (const auto& match : matches) {
        if (match.candidate &&
            match.candidate->internal_uid == best_it->internal_uid) {
            diagnostic_match = &match;
            break;
        }
    }
    if (diagnostic_match) {
        fillAssociationDiagnostics(*diagnostic_match,
                                   diagnostic_match->candidate);
    }
    association_diagnostics_.source = "reacquire_pending";
    fillBestReacquireDiagnostics();

    // Candidate positions remain diagnostic only.  The controller follows the
    // decelerating KF prediction, not an unconfirmed neighbouring cluster.
    locked_.has_measurement = false;
    locked_.x = predicted_x;
    locked_.y = predicted_y;
    locked_.vx = kalman_vx_;
    locked_.vy = kalman_vy_;
    locked_.range_m = std::hypot(predicted_x, predicted_y);
    locked_.angle_deg = std::atan2(predicted_x, predicted_y) *
                        180.0f / static_cast<float>(M_PI);
    locked_.stable_frames = 0;
    locked_.reacquire_hits = best_it->consecutive_hits;
    locked_.confidence = std::max(
        0.0f, locked_.confidence - config_.confidence_miss_drop);
    if (manual_lock_confirmation_pending_) {
        manual_lock_confirmation_misses_ = 0;
        association_diagnostics_.source = "manual_lock_pending";
    }

    updateFollowControlAdvice(locked_, config_);
    if (locked_.lost_frames <= config_.lost_limited_follow_frames) {
        locked_.control_state = FollowControlState::ReacquireConfirming;
    }
    holdManualConfirmationMotion();
    if (locked_.lost_frames > config_.max_lost_frames) {
        locked_.valid = false;
        kalman_initialized_ = false;
        clearReacquireHypothesis();
        association_diagnostics_.source = "tracker_timeout_unlock";
    }
}

void TargetTracker::commitReacquireHypothesis(
    const ReacquireHypothesis& hypothesis,
    const MatchResult& match)
{
    const bool was_manual_confirmation = manual_lock_confirmation_pending_;
    const CandidateTarget candidate = hypothesis.latest;
    const int confirmed_hits = hypothesis.consecutive_hits;
    const std::uint64_t confirmed_uid = hypothesis.internal_uid;
    float confirmed_vx = hypothesis.vx;
    float confirmed_vy = hypothesis.vy;

    const float speed_limit = std::max(
        0.0f, was_manual_confirmation
                  ? config_.manual_lock_max_initial_speed_mps
                  : config_.reacquire_auto_max_initial_speed_mps);
    const float speed = std::hypot(confirmed_vx, confirmed_vy);
    if (!std::isfinite(speed)) {
        confirmed_vx = 0.0f;
        confirmed_vy = 0.0f;
    } else if (speed_limit > 0.0f && speed > speed_limit) {
        const float scale = speed_limit / speed;
        confirmed_vx *= scale;
        confirmed_vy *= scale;
    }

    locked_.matched_internal_uid = confirmed_uid;
    locked_.matched_candidate_id = candidate.id;
    locked_.has_measurement = true;
    if (frame_context_.sample_time_sec > 0) last_measurement_time_sec_ = frame_context_.sample_time_sec;
    locked_.measured_x = candidate.x;
    locked_.measured_y = candidate.y;
    locked_.measured_range_m = candidate.range_m;
    locked_.measured_angle_deg = candidate.angle_deg;
    locked_.x = candidate.x;
    locked_.y = candidate.y;
    locked_.vx = confirmed_vx;
    locked_.vy = confirmed_vy;
    locked_.range_m = candidate.range_m;
    locked_.angle_deg = candidate.angle_deg;
    locked_.lost_frames = 0;
    locked_.stable_frames = 1;
    locked_.reacquire_hits = 0;
    locked_.confidence = std::clamp(
        locked_.confidence + config_.confidence_hit_base_gain +
            config_.confidence_candidate_score_gain *
                clamp01(candidate.score),
        0.0f, 1.0f);

    initKalmanFromLocked();
    last_accepted_candidate_ = candidate;
    has_last_accepted_candidate_ = true;
    frames_since_accepted_measurement_ = 0;
    fillAssociationDiagnostics(match, &candidate);
    fillBestReacquireDiagnostics();
    association_diagnostics_.source = was_manual_confirmation
        ? "manual_lock_confirmed"
        : "reacquire_confirmed";
    association_diagnostics_.pending_internal_uid = confirmed_uid;
    association_diagnostics_.pending_hits = confirmed_hits;
    association_diagnostics_.pending_window_frames =
        hypothesis.window_frames;
    locked_uid_continuity_compromised_ = false;
    manual_lock_confirmation_pending_ = false;
    manual_lock_confirmation_misses_ = 0;
    clearReacquireHypothesis();
    clearOcclusionCoast();
    updateFollowControlAdvice(locked_, config_);
}
void TargetTracker::acceptMeasurement(const CandidateTarget& candidate,
                                      const MatchResult& match,
                                      bool reset_lost_state)
{
    const float effective_gate = config_.kalman_gate_radius_m +
                                 turnGateBonus(config_, current_chassis_sample_);
    const float match_distance = std::hypot(match.innovation_x, match.innovation_y);
    const float match_quality =
        1.0f - clamp01(match_distance / std::max(0.01f, effective_gate));

    locked_.has_measurement = true;
    if (frame_context_.sample_time_sec > 0) last_measurement_time_sec_ = frame_context_.sample_time_sec;
    locked_.measured_x = candidate.x;
    locked_.measured_y = candidate.y;
    locked_.measured_range_m = candidate.range_m;
    locked_.measured_angle_deg = candidate.angle_deg;
    updateKalman(candidate.x, candidate.y);
    locked_.x = kalman_x_;
    locked_.y = kalman_y_;
    locked_.vx = kalman_vx_;
    locked_.vy = kalman_vy_;
    locked_.range_m = std::hypot(kalman_x_, kalman_y_);
    locked_.angle_deg = std::atan2(kalman_x_, kalman_y_) *
                        180.0f / static_cast<float>(M_PI);
    locked_.matched_internal_uid = candidate.internal_uid;
    locked_.matched_candidate_id = candidate.id;
    if (reset_lost_state) {
        locked_.lost_frames = 0;
        locked_.reacquire_hits = 0;
        locked_.stable_frames++;
    }

    last_accepted_candidate_ = candidate;
    has_last_accepted_candidate_ = true;
    frames_since_accepted_measurement_ = 0;
    clearReacquireHypothesis();
    clearOcclusionCoast();
    fillAssociationDiagnostics(match, &candidate);
    updateFollowControlAdvice(locked_, config_);

    const float confidence_delta = config_.confidence_hit_base_gain +
        config_.confidence_match_gain * match_quality +
        config_.confidence_candidate_score_gain * clamp01(candidate.score) -
        config_.confidence_background_penalty * clamp01(candidate.background_score);
    locked_.confidence = std::clamp(locked_.confidence + confidence_delta,
                                    0.0f, 1.0f);
}

void TargetTracker::clearReacquireHypothesis()
{
    reacquire_hypotheses_.clear();
}

void TargetTracker::holdManualConfirmationMotion()
{
    if (!manual_lock_confirmation_pending_ &&
        !(config_.hold_motion_during_reacquire && !reacquire_hypotheses_.empty())) return;
    locked_.has_measurement = false;
    locked_.control_state = FollowControlState::ReacquireConfirming;
    locked_.control_speed_scale = 0.0f;
    locked_.control_allow_translation = false;
    locked_.control_allow_yaw = false;
}

void TargetTracker::applyMissingMeasurementVelocityPolicy()
{
    if (!kalman_initialized_) return;

    // Optional co-motion prior: decay RADAR-RELATIVE drift instead of assuming
    // the person becomes world-stationary while the cart keeps translating.
    // This is an ablation option, not an identity measurement.
    float origin_vx=0.0f, origin_vy=0.0f;
    if (config_.ego_relative_dropout_decay && ego_motion_compensation_enabled_ && current_chassis_sample_.valid) {
        const float wz=RadarMount::yawSign()*current_chassis_sample_.wz_radps;
        origin_vx=-wz*(RadarMount::yOffsetM()+kalman_y_);
        origin_vy=current_chassis_sample_.vx_mps+wz*(RadarMount::xOffsetM()+kalman_x_);
    }
    float relative_vx=kalman_vx_-origin_vx, relative_vy=kalman_vy_-origin_vy;
    float speed = std::hypot(relative_vx, relative_vy);
    if (!std::isfinite(speed)) {
        kalman_vx_ = 0.0f;
        kalman_vy_ = 0.0f;
        return;
    }

    const float speed_limit = std::max(0.0f, config_.prediction_max_speed_mps);
    if (speed_limit > 0.0f && speed > speed_limit) {
        const float scale = speed_limit / speed;
        relative_vx *= scale;
        relative_vy *= scale;
        speed = speed_limit;
    }

    const int stop_frame = std::max(
        1, config_.prediction_velocity_stop_lost_frames);
    if (locked_.lost_frames >= stop_frame) {
        kalman_vx_ = origin_vx;
        kalman_vy_ = origin_vy;
        return;
    }

    const int decay_start = std::max(
        1, config_.prediction_velocity_decay_start_lost_frames);
    if (locked_.lost_frames >= decay_start) {
        const float decay = std::clamp(
            config_.prediction_velocity_decay_per_frame, 0.0f, 1.0f);
        relative_vx *= decay;
        relative_vy *= decay;
    }
    kalman_vx_=origin_vx+relative_vx;
    kalman_vy_=origin_vy+relative_vy;
}

std::string TargetTracker::candidateSummary() const
{
    std::ostringstream oss;
    oss << "候选目标数量: " << candidates_.size();
    for (const auto& c : candidates_) {
        oss << "\nT" << c.id
            << " uid=" << c.internal_uid
            << " x=" << c.x
            << " y=" << c.y
            << " range=" << c.range_m
            << " angle=" << c.angle_deg
            << " v=" << c.average_abs_velocity
            << " points=" << c.point_count
            << " bg=" << c.background_score
            << " score=" << c.score;
    }
    return oss.str();
}

std::vector<CandidateTarget> TargetTracker::buildCandidates(
    const std::vector<PointCloud>& points,
    int current_frame_number)
{
    cluster_debug_.clear();

    std::vector<PointCloud> filtered_points;
    filtered_points.reserve(points.size());
    for (const auto& point : points) {
        const float range = std::sqrt(point.x * point.x + point.y * point.y);
        if (config_.enable_near_field_point_filter && range < config_.near_field_filter_range_m) {
            continue;
        }
        // Keep the rectangular chassis mask from accidentally deleting a
        // legitimate off-axis target whose radial range is already >= the
        // common 1.8 m boundary (for example x=1.1 m, y=1.6 m).
        if (config_.enable_chassis_mask && range < config_.min_range_m &&
            std::abs(point.x) <= config_.chassis_mask_half_width_m &&
            point.y >= -config_.chassis_mask_backward_m &&
            point.y <= config_.chassis_mask_forward_m) {
            continue;
        }
        filtered_points.push_back(point);
    }

    const int n = static_cast<int>(filtered_points.size());
    if (n == 0) return {};
    bool has_source_frame_metadata = false;
    int newest_available_source_frame = -1;
    for (const auto& point : filtered_points) {
        if (point.source_frame >= 0) {
            has_source_frame_metadata = true;
            newest_available_source_frame = std::max(
                newest_available_source_frame, point.source_frame);
        }
    }
    // The caller knows the radar frame even when it contains zero points.  In
    // that case max(source_frame) belongs to an older window entry and must not
    // be mistaken for a fresh measurement.
    const int current_source_frame = has_source_frame_metadata
        ? (current_frame_number >= 0
               ? current_frame_number
               : newest_available_source_frame)
        : -1;

    std::vector<int> labels(n, -99);

    auto pointRange = [&](int index) {
        return std::sqrt(filtered_points[index].x * filtered_points[index].x +
                         filtered_points[index].y * filtered_points[index].y);
    };

    auto adaptiveEps = [&](int index) {
        const float r = pointRange(index);
        return std::clamp(config_.cluster_eps * 0.65f + 0.035f * r, 0.45f, 1.05f);
    };

    auto adaptiveMinPts = [&](int index) {
        const float r = pointRange(index);
        int adaptive = config_.min_cluster_points;
        if (r < 4.0f) {
            adaptive = config_.min_cluster_points + 1;
        } else if (r < 8.0f) {
            adaptive = config_.min_cluster_points;
        } else if (r < 12.0f) {
            adaptive = config_.min_cluster_points - 1;
        } else {
            adaptive = config_.min_cluster_points - 2;
        }
        return std::max(2, adaptive);
    };

    auto regionQuery = [&](int index) {
        std::vector<int> neighbors;
        neighbors.reserve(16);
        const float eps = adaptiveEps(index);
        const float eps2 = eps * eps;
        for (int i = 0; i < n; ++i) {
            if (squaredDistance(filtered_points[index].x, filtered_points[index].y,
                                filtered_points[i].x, filtered_points[i].y) <= eps2) {
                neighbors.push_back(i);
            }
        }
        return neighbors;
    };

    int cluster_id = 0;
    for (int i = 0; i < n; ++i) {
        if (labels[i] != -99) continue;

        std::vector<int> neighbors = regionQuery(i);
        if (static_cast<int>(neighbors.size()) < adaptiveMinPts(i)) {
            labels[i] = -1;
            continue;
        }

        labels[i] = cluster_id;
        for (size_t k = 0; k < neighbors.size(); ++k) {
            const int j = neighbors[k];
            if (labels[j] == -1) labels[j] = cluster_id;
            if (labels[j] != -99) continue;

            labels[j] = cluster_id;
            std::vector<int> more = regionQuery(j);
            if (static_cast<int>(more.size()) >= adaptiveMinPts(j)) {
                neighbors.insert(neighbors.end(), more.begin(), more.end());
            }
        }
        cluster_id++;
    }

    std::vector<CandidateTarget> result;
    for (int cid = 0; cid < cluster_id; ++cid) {
        struct FrameCentroidAccumulator {
            float sx = 0.0f;
            float sy = 0.0f;
            int count = 0;
        };

        CandidateTarget candidate;
        candidate.cluster_id=cid;
        float sx = 0.0f;
        float sy = 0.0f;
        float sv = 0.0f;
        float snr_sum = 0.0f;
        float doa_confidence_sum = 0.0f;
        std::vector<float> signed_velocities;
        std::map<std::pair<int, int>, int> rd_cell_counts;
        std::map<int, FrameCentroidAccumulator> frame_centroids;
        int moving_count = 0;
        float min_x = std::numeric_limits<float>::max();
        float max_x = std::numeric_limits<float>::lowest();
        float min_y = std::numeric_limits<float>::max();
        float max_y = std::numeric_limits<float>::lowest();
        float current_min_x=min_x, current_max_x=max_x, current_min_y=min_y, current_max_y=max_y;

        for (int i = 0; i < n; ++i) {
            if (labels[i] != cid) continue;
            const PointCloud& p = filtered_points[i];
            sx += p.x;
            sy += p.y;
            sv += std::abs(p.absolute_velocity);
            snr_sum += p.snr;
            doa_confidence_sum += p.doa_peak_confidence;
            signed_velocities.push_back(p.absolute_velocity);
            moving_count += p.is_moving ? 1 : 0;
            min_x = std::min(min_x, p.x);
            max_x = std::max(max_x, p.x);
            min_y = std::min(min_y, p.y);
            max_y = std::max(max_y, p.y);
            if (p.source_frame >= 0) {
                FrameCentroidAccumulator& frame_accumulator =
                    frame_centroids[p.source_frame];
                frame_accumulator.sx += p.x;
                frame_accumulator.sy += p.y;
                frame_accumulator.count++;
                if (candidate.oldest_source_frame < 0) {
                    candidate.oldest_source_frame = p.source_frame;
                } else {
                    candidate.oldest_source_frame = std::min(
                        candidate.oldest_source_frame, p.source_frame);
                }
                candidate.newest_source_frame = std::max(
                    candidate.newest_source_frame, p.source_frame);
                if (p.source_frame == current_source_frame) {
                    candidate.current_frame_point_count++;
                    current_min_x=std::min(current_min_x,p.x); current_max_x=std::max(current_max_x,p.x);
                    current_min_y=std::min(current_min_y,p.y); current_max_y=std::max(current_max_y,p.y);
                }
            }
            if (p.range_bin >= 0 && p.doppler_bin >= 0) {
                rd_cell_counts[{p.range_bin, p.doppler_bin}]++;
            }
            candidate.point_count++;
        }

        if (candidate.point_count <= 0) continue;
        const float window_centroid_x = sx / candidate.point_count;
        const float window_centroid_y = sy / candidate.point_count;
        candidate.x = window_centroid_x;
        candidate.y = window_centroid_y;
        candidate.range_m = std::sqrt(candidate.x * candidate.x + candidate.y * candidate.y);
        candidate.angle_deg = std::atan2(candidate.x, candidate.y) * 180.0f / static_cast<float>(M_PI);
        candidate.average_abs_velocity = sv / candidate.point_count;
        candidate.average_snr = snr_sum / candidate.point_count;
        candidate.average_doa_peak_confidence =
            doa_confidence_sum / candidate.point_count;
        if (!has_source_frame_metadata) {
            // Legacy point files have no provenance; treating their only
            // available set as current preserves backwards-compatible stats.
            candidate.current_frame_point_count = candidate.point_count;
            candidate.has_current_frame_support = true;
            candidate.position_mode = CandidatePositionMode::WindowCentroid;
        } else {
            const auto current_frame_it = frame_centroids.find(current_source_frame);
            candidate.current_frame_point_count =
                current_frame_it == frame_centroids.end()
                    ? 0 : current_frame_it->second.count;
            candidate.has_current_frame_support =
                candidate.current_frame_point_count > 0;

            if (config_.prefer_current_frame_position &&
                candidate.current_frame_point_count >=
                    std::max(1, config_.current_frame_centroid_min_points)) {
                candidate.x = current_frame_it->second.sx /
                              current_frame_it->second.count;
                candidate.y = current_frame_it->second.sy /
                              current_frame_it->second.count;
                candidate.position_mode =
                    CandidatePositionMode::CurrentFrameCentroid;
            } else if (config_.prefer_current_frame_position &&
                       candidate.current_frame_point_count == 1) {
                float weighted_x = 0.0f;
                float weighted_y = 0.0f;
                float weight_sum = 0.0f;
                const auto add_frame_centroid = [&](
                    const FrameCentroidAccumulator& accumulator,
                    float weight) {
                    if (accumulator.count <= 0 || weight <= 0.0f) return;
                    weighted_x += weight * accumulator.sx / accumulator.count;
                    weighted_y += weight * accumulator.sy / accumulator.count;
                    weight_sum += weight;
                };

                add_frame_centroid(
                    current_frame_it->second,
                    config_.current_frame_single_point_weight);
                int history_rank = 0;
                for (auto it = frame_centroids.rbegin();
                     it != frame_centroids.rend() && history_rank < 2; ++it) {
                    if (it->first >= current_source_frame) continue;
                    const float weight = history_rank == 0
                        ? config_.previous_frame_centroid_weight
                        : config_.second_previous_frame_centroid_weight;
                    add_frame_centroid(it->second, weight);
                    history_rank++;
                }
                if (weight_sum > 1e-6f) {
                    candidate.x = weighted_x / weight_sum;
                    candidate.y = weighted_y / weight_sum;
                }
                candidate.position_mode =
                    CandidatePositionMode::TemporalWeightedCentroid;
            } else if (candidate.current_frame_point_count == 0) {
                candidate.position_mode =
                    CandidatePositionMode::NoCurrentFrameSupport;
            }
        }
        candidate.current_frame_point_ratio =
            static_cast<float>(candidate.current_frame_point_count) /
            static_cast<float>(candidate.point_count);
        // Range/angle must describe the selected measurement centroid rather
        // than the old all-window centroid.
        candidate.range_m = std::sqrt(candidate.x * candidate.x +
                                      candidate.y * candidate.y);
        candidate.angle_deg = std::atan2(candidate.x, candidate.y) *
                              180.0f / static_cast<float>(M_PI);
        candidate.unique_range_doppler_cells =
            static_cast<int>(rd_cell_counts.size());
        int dominant_count = 0;
        for (const auto& entry : rd_cell_counts) {
            if (entry.second > dominant_count) {
                dominant_count = entry.second;
                candidate.dominant_range_bin = entry.first.first;
                candidate.dominant_doppler_bin = entry.first.second;
            }
        }
        if (!signed_velocities.empty()) {
            const auto middle = signed_velocities.begin() +
                static_cast<std::ptrdiff_t>(signed_velocities.size() / 2);
            std::nth_element(signed_velocities.begin(), middle,
                             signed_velocities.end());
            candidate.median_signed_velocity = *middle;
        }
        candidate.moving_ratio = static_cast<float>(moving_count) / candidate.point_count;
        candidate.width_m = max_x - min_x;
        candidate.depth_m = max_y - min_y;
        if(config_.use_current_frame_extent && candidate.current_frame_point_count>=2 &&
           candidate.position_mode==CandidatePositionMode::CurrentFrameCentroid) {
            candidate.width_m=current_max_x-current_min_x;
            candidate.depth_m=current_max_y-current_min_y;
        }

        const float compact_score = 1.0f - clamp01(std::max(candidate.width_m, candidate.depth_m) / 2.0f);
        const float motion_score = clamp01(candidate.moving_ratio * 2.0f + candidate.average_abs_velocity / 0.45f);
        const float density_score = clamp01(static_cast<float>(candidate.point_count) / 12.0f);
        const float range_score = 1.0f - clamp01((candidate.range_m - 2.0f) / 16.0f);
        candidate.score = 0.15f * compact_score + 0.70f * motion_score + 0.10f * density_score + 0.05f * range_score;

        ClusterDebugInfo debug;
        debug.cluster_id = cid;
        debug.x = candidate.x;
        debug.y = candidate.y;
        debug.range_m = candidate.range_m;
        debug.angle_deg = candidate.angle_deg;
        debug.average_abs_velocity = candidate.average_abs_velocity;
        debug.median_signed_velocity = candidate.median_signed_velocity;
        debug.moving_ratio = candidate.moving_ratio;
        debug.width_m = candidate.width_m;
        debug.depth_m = candidate.depth_m;
        debug.score = candidate.score;
        debug.background_score = candidate.background_score;
        debug.point_count = candidate.point_count;
        debug.oldest_source_frame = candidate.oldest_source_frame;
        debug.newest_source_frame = candidate.newest_source_frame;
        debug.current_frame_point_count = candidate.current_frame_point_count;
        debug.current_frame_point_ratio = candidate.current_frame_point_ratio;
        debug.position_mode = candidate.position_mode;
        debug.has_current_frame_support = candidate.has_current_frame_support;
        debug.dominant_range_bin = candidate.dominant_range_bin;
        debug.dominant_doppler_bin = candidate.dominant_doppler_bin;
        debug.unique_range_doppler_cells =
            candidate.unique_range_doppler_cells;
        debug.average_snr = candidate.average_snr;
        debug.average_doa_peak_confidence =
            candidate.average_doa_peak_confidence;
        debug.person_like = isPersonLike(candidate) &&
                            (!config_.require_current_frame_support ||
                             candidate.has_current_frame_support);
        for (int i = 0; i < n; ++i) {
            if (labels[i] == cid) {
                debug.points.push_back(filtered_points[i]);
            }
        }
        cluster_debug_.push_back(debug);

        if (isPersonLike(candidate) &&
            (!config_.require_current_frame_support ||
             candidate.has_current_frame_support)) {
            result.push_back(candidate);
        }
    }

    return result;
}

bool TargetTracker::isPersonLike(const CandidateTarget& candidate) const
{
    if (candidate.range_m < config_.min_range_m || candidate.range_m > config_.max_range_m) return false;
    if (candidate.width_m > config_.max_width_m || candidate.depth_m > config_.max_depth_m) return false;
    if (candidate.point_count < config_.min_cluster_points) return false;
    if (isNearFieldBurstCluster(candidate)) return false;
    return true;
}

bool TargetTracker::isNearFieldBurstCluster(const CandidateTarget& candidate) const
{
    if (!config_.reject_near_field_burst_clusters) return false;
    return candidate.range_m <= config_.near_field_burst_max_range_m &&
           candidate.point_count >= config_.near_field_burst_min_cluster_points &&
           candidate.average_abs_velocity >=
               config_.near_field_burst_min_abs_velocity_mps &&
           candidate.moving_ratio >= config_.near_field_burst_min_moving_ratio;
}

TargetTracker::CandidateTrack* TargetTracker::findCandidateTrackById(int track_id)
{
    auto it = std::find_if(candidate_tracks_.begin(), candidate_tracks_.end(),
                           [track_id](const CandidateTrack& track) { return track.id == track_id; });
    return it == candidate_tracks_.end() ? nullptr : &(*it);
}

TargetTracker::CandidateTrack* TargetTracker::findCandidateTrackByUid(std::uint64_t internal_uid)
{
    auto it = std::find_if(candidate_tracks_.begin(), candidate_tracks_.end(),
                           [internal_uid](const CandidateTrack& track) {
                               return track.internal_uid == internal_uid;
                           });
    return it == candidate_tracks_.end() ? nullptr : &(*it);
}

float TargetTracker::candidateTrackGateRadius(const CandidateTrack&, const CandidateTarget& candidate) const
{
    const float range_gate = 0.75f + candidate.range_m * config_.candidate_track_range_gate_scale;
    // Candidate UID association is deliberately independent from the wider
    // main-Kalman/ROI gates.  Otherwise increasing the lock ROI also lets an
    // existing UID jump to a neighbouring cluster before identity checks run.
    const float turn_allowance = std::min(
        0.20f, 0.25f * turnGateBonus(config_, current_chassis_sample_));
    return std::clamp(range_gate + turn_allowance,
                      config_.candidate_track_min_gate_m,
                      std::max(config_.candidate_track_min_gate_m,
                               config_.candidate_track_max_gate_m));
}

void TargetTracker::associateCandidateTracks()
{
    const float dt = std::max(0.01f, current_frame_dt_s_);
    candidate_uid_assignment_ambiguous_ = false;

    if (locked_.valid && locked_.matched_internal_uid != 0 &&
        !findCandidateTrackByUid(locked_.matched_internal_uid)) {
        CandidateTrack locked_track;
        locked_track.id = locked_.display_id;
        locked_track.internal_uid = locked_.matched_internal_uid;
        locked_track.x = locked_.x;
        locked_track.y = locked_.y;
        locked_track.vx = locked_.vx;
        locked_track.vy = locked_.vy;
        locked_track.was_locked = true;
        if (has_last_accepted_candidate_) {
            locked_track.has_signature = true;
            locked_track.point_count = last_accepted_candidate_.point_count;
            locked_track.width_m = last_accepted_candidate_.width_m;
            locked_track.depth_m = last_accepted_candidate_.depth_m;
            locked_track.median_signed_velocity =
                last_accepted_candidate_.median_signed_velocity;
            locked_track.moving_ratio = last_accepted_candidate_.moving_ratio;
        }
        locked_track.hits = std::max(1, locked_.stable_frames);
        locked_track.lost_frames = locked_.lost_frames;
        locked_track.confidence = locked_.confidence;
        candidate_tracks_.push_back(locked_track);
    }

    // Remember that this immutable UID has been the locked target. The GUI may
    // release motion control after 30 missing frames, while the accepted
    // internal-track policy deliberately retains that identity through frame
    // 50 for diagnosis and a possible manual re-selection.
    if (locked_.valid && locked_.matched_internal_uid != 0) {
        if (auto* locked_track =
                findCandidateTrackByUid(locked_.matched_internal_uid)) {
            locked_track->was_locked = true;
        }
    }

    struct Match {
        int candidate_index = -1;
        int track_index = -1;
        float cost = 0.0f;
    };

    std::vector<Match> matches;
    std::vector<bool> candidate_matched(candidates_.size(), false);
    std::vector<bool> track_matched(candidate_tracks_.size(), false);

    int locked_track_index = -1;
    if (locked_.valid && locked_.matched_internal_uid != 0) {
        for (size_t ti = 0; ti < candidate_tracks_.size(); ++ti) {
            if (candidate_tracks_[ti].internal_uid ==
                locked_.matched_internal_uid) {
                locked_track_index = static_cast<int>(ti);
                break;
            }
        }
    }

    if (RadarRuntimeConfig::kUseHungarianTrajectoryAssociation) {
        std::vector<TrajectoryAssociation::TrackObservation> tracks;
        tracks.reserve(candidate_tracks_.size());
        for (size_t ti = 0; ti < candidate_tracks_.size(); ++ti) {
            const auto& track = candidate_tracks_[ti];
            TrajectoryAssociation::TrackObservation observation;
            observation.source_index = static_cast<int>(ti);
            observation.internal_uid = track.internal_uid;
            observation.locked = locked_.valid &&
                track.internal_uid == locked_.matched_internal_uid;
            observation.x = track.x;
            observation.y = track.y;
            observation.vx = track.vx;
            observation.vy = track.vy;
            observation.var_x = track.var_x;
            observation.var_y = track.var_y;
            observation.has_signature = track.has_signature;
            observation.point_count = track.point_count;
            observation.width_m = track.width_m;
            observation.depth_m = track.depth_m;
            observation.median_signed_velocity_mps =
                track.median_signed_velocity;
            observation.moving_ratio = track.moving_ratio;
            tracks.push_back(observation);
        }

        std::vector<TrajectoryAssociation::DetectionObservation> detections;
        detections.reserve(candidates_.size());
        for (size_t ci = 0; ci < candidates_.size(); ++ci) {
            const auto& candidate = candidates_[ci];
            TrajectoryAssociation::DetectionObservation observation;
            observation.source_index = static_cast<int>(ci);
            observation.x = candidate.x;
            observation.y = candidate.y;
            observation.range_m = candidate.range_m;
            observation.point_count = candidate.point_count;
            observation.width_m = candidate.width_m;
            observation.depth_m = candidate.depth_m;
            observation.median_signed_velocity_mps =
                candidate.median_signed_velocity;
            observation.moving_ratio = candidate.moving_ratio;
            observation.score = candidate.score;
            observation.background_score = candidate.background_score;
            detections.push_back(observation);
        }

        TrajectoryAssociation::Config association_config;
        association_config.dt_s = dt;
        association_config.min_gate_m = config_.candidate_track_min_gate_m;
        association_config.max_gate_m = config_.candidate_track_max_gate_m;
        association_config.range_gate_scale =
            config_.candidate_track_range_gate_scale;
        association_config.turn_allowance_m =
            std::min(0.20f, 0.25f *
                turnGateBonus(config_, current_chassis_sample_));
        association_config.measurement_variance =
            config_.candidate_track_measurement_variance;
        association_config.gate_chi2 = config_.candidate_track_gate_chi2;
        association_config.max_implied_speed_mps =
            config_.max_measurement_speed_mps;
        association_config.locked_max_implied_speed_mps =
            config_.suspicious_continuity_speed_mps;
        association_config.point_count_cost_weight =
            config_.trajectory_point_count_cost_weight;
        association_config.width_cost_weight =
            config_.trajectory_size_cost_weight;
        association_config.depth_cost_weight =
            config_.trajectory_size_cost_weight;
        association_config.doppler_cost_weight =
            config_.trajectory_signed_doppler_cost_weight;
        association_config.moving_ratio_cost_weight =
            config_.trajectory_moving_ratio_cost_weight;
        association_config.point_count_ratio_min =
            config_.trajectory_point_count_ratio_min;
        association_config.point_count_ratio_max =
            config_.trajectory_point_count_ratio_max;
        association_config.max_width_delta_m =
            config_.trajectory_max_width_delta_m;
        association_config.max_depth_delta_m =
            config_.trajectory_max_depth_delta_m;
        association_config.max_signed_doppler_delta_mps =
            config_.trajectory_max_signed_doppler_delta_mps;
        association_config.locked_cost_scale =
            config_.candidate_track_locked_cost_scale;
        association_config.reject_ambiguous_locked_assignment =
            config_.reject_ambiguous_locked_uid_assignment;
        association_config.locked_ambiguity_cost_margin =
            config_.candidate_track_locked_ambiguity_cost_margin;
        association_config.unassigned_track_cost =
            config_.trajectory_unassigned_cost;
        association_config.new_detection_cost =
            config_.trajectory_unassigned_cost;

        const auto association_result = trajectory_associator_.associate(
            tracks, detections, association_config);
        trajectory_association_diagnostics_ =
            association_result.diagnostics;
        candidate_uid_assignment_ambiguous_ =
            association_result.diagnostics.locked_assignment_ambiguous;
        for (size_t ci = 0;
             ci < association_result.detection_to_track.size(); ++ci) {
            const int ti = association_result.detection_to_track[ci];
            if (ti >= 0) {
                matches.push_back(Match{static_cast<int>(ci), ti, 0.0f});
            }
        }
    } else {
        trajectory_association_diagnostics_ =
            TrajectoryAssociation::Diagnostics();
        trajectory_association_diagnostics_.method = "greedy";
        trajectory_association_diagnostics_.track_count =
            static_cast<int>(candidate_tracks_.size());
        trajectory_association_diagnostics_.detection_count =
            static_cast<int>(candidates_.size());

        for (size_t ci = 0; ci < candidates_.size(); ++ci) {
            const auto& candidate = candidates_[ci];
            for (size_t ti = 0; ti < candidate_tracks_.size(); ++ti) {
                const auto& track = candidate_tracks_[ti];
                const float predicted_x = track.x + track.vx * dt;
                const float predicted_y = track.y + track.vy * dt;
                const float gate = candidateTrackGateRadius(track, candidate);
                const float d2 = squaredDistance(candidate.x, candidate.y,
                                                 predicted_x, predicted_y);
                if (d2 > gate * gate) continue;

                const float innovation_var_x = std::max(
                    0.01f, track.var_x +
                    config_.candidate_track_measurement_variance);
                const float innovation_var_y = std::max(
                    0.01f, track.var_y +
                    config_.candidate_track_measurement_variance);
                const float ex = candidate.x - predicted_x;
                const float ey = candidate.y - predicted_y;
                const float mahalanobis2 = ex * ex / innovation_var_x +
                    ey * ey / innovation_var_y;
                if (mahalanobis2 > config_.candidate_track_gate_chi2) continue;

                float cost = mahalanobis2;
                if (locked_.valid &&
                    track.internal_uid == locked_.matched_internal_uid) {
                    cost *= std::clamp(config_.candidate_track_locked_cost_scale,
                                       0.20f, 1.0f);
                }
                const float measured_vx = (candidate.x - track.x) / dt;
                const float measured_vy = (candidate.y - track.y) / dt;
                const float dv2 = squaredDistance(measured_vx, measured_vy,
                                                  track.vx, track.vy);
                cost += 0.05f * dv2;
                cost -= 0.08f * candidate.score;
                cost += 0.25f * candidate.background_score;
                matches.push_back(Match{static_cast<int>(ci),
                                        static_cast<int>(ti), cost});
            }
        }

        trajectory_association_diagnostics_.valid_pair_count =
            static_cast<int>(matches.size());
        if (config_.reject_ambiguous_locked_uid_assignment &&
            locked_track_index >= 0) {
            float best = std::numeric_limits<float>::infinity();
            float second = std::numeric_limits<float>::infinity();
            for (const auto& match : matches) {
                if (match.track_index != locked_track_index) continue;
                if (match.cost < best) {
                    second = best;
                    best = match.cost;
                } else if (match.cost < second) {
                    second = match.cost;
                }
            }
            candidate_uid_assignment_ambiguous_ =
                std::isfinite(second) &&
                second - best <
                    config_.candidate_track_locked_ambiguity_cost_margin;
            trajectory_association_diagnostics_.locked_assignment_ambiguous =
                candidate_uid_assignment_ambiguous_;
        }

        std::sort(matches.begin(), matches.end(),
                  [](const Match& a, const Match& b) {
                      return a.cost < b.cost;
                  });
    }

    if (candidate_uid_assignment_ambiguous_) {
        locked_uid_continuity_compromised_ = true;
    }

    for (const auto& match : matches) {
        if (candidate_matched[match.candidate_index] || track_matched[match.track_index]) continue;
        if (candidate_uid_assignment_ambiguous_ &&
            match.track_index == locked_track_index) {
            // Do not let the immutable locked UID silently migrate to one of
            // two nearly equivalent neighbouring clusters. Leave it as a
            // prediction and let the explicit multi-frame reacquirer decide.
            continue;
        }

        auto& candidate = candidates_[match.candidate_index];
        auto& track = candidate_tracks_[match.track_index];
        const float measured_vx = (candidate.x - track.x) / dt;
        const float measured_vy = (candidate.y - track.y) / dt;
        const float smooth = std::clamp(config_.candidate_track_velocity_smoothing, 0.0f, 1.0f);

        track.vx = track.vx * (1.0f - smooth) + measured_vx * smooth;
        track.vy = track.vy * (1.0f - smooth) + measured_vy * smooth;
        track.x = candidate.x;
        track.y = candidate.y;
        const float measurement_var = std::max(0.01f, config_.candidate_track_measurement_variance);
        track.var_x = (track.var_x * measurement_var) / (track.var_x + measurement_var);
        track.var_y = (track.var_y * measurement_var) / (track.var_y + measurement_var);
        const float signature_smooth = 0.30f;
        if (!track.has_signature) {
            track.point_count = candidate.point_count;
            track.width_m = candidate.width_m;
            track.depth_m = candidate.depth_m;
            track.median_signed_velocity = candidate.median_signed_velocity;
            track.moving_ratio = candidate.moving_ratio;
            track.has_signature = true;
        } else {
            track.point_count = std::max(1, static_cast<int>(std::lround(
                (1.0f - signature_smooth) * track.point_count +
                signature_smooth * candidate.point_count)));
            track.width_m = (1.0f - signature_smooth) * track.width_m +
                            signature_smooth * candidate.width_m;
            track.depth_m = (1.0f - signature_smooth) * track.depth_m +
                            signature_smooth * candidate.depth_m;
            track.median_signed_velocity =
                (1.0f - signature_smooth) * track.median_signed_velocity +
                signature_smooth * candidate.median_signed_velocity;
            track.moving_ratio =
                (1.0f - signature_smooth) * track.moving_ratio +
                signature_smooth * candidate.moving_ratio;
        }
        track.hits++;
        if (track.id < 0 && config_.track_id_mode == TrackIdMode::ConfirmedOnly &&
            track.hits >= std::max(1, config_.track_confirm_hits)) {
            track.id = allocateDisplayId();
        }
        candidate.id = track.id;
        candidate.internal_uid = track.internal_uid;
        track.lost_frames = 0;
        track.confidence = std::clamp(track.confidence + 0.15f + 0.10f * candidate.score,
                                      0.0f, 1.0f);

        candidate_matched[match.candidate_index] = true;
        track_matched[match.track_index] = true;
    }

    if (trajectory_association_diagnostics_.method == "greedy") {
        const int matched_count = static_cast<int>(std::count(
            candidate_matched.begin(), candidate_matched.end(), true));
        trajectory_association_diagnostics_.matched_pair_count = matched_count;
        trajectory_association_diagnostics_.unassigned_track_count =
            static_cast<int>(candidate_tracks_.size()) - matched_count;
        trajectory_association_diagnostics_.unassigned_detection_count =
            static_cast<int>(candidates_.size()) - matched_count;
    }

    for (size_t ti = 0; ti < candidate_tracks_.size(); ++ti) {
        if (track_matched[ti]) continue;
        auto& track = candidate_tracks_[ti];
        track.x += track.vx * dt;
        track.y += track.vy * dt;
        track.var_x += std::max(0.001f, config_.candidate_track_process_variance) * dt;
        track.var_y += std::max(0.001f, config_.candidate_track_process_variance) * dt;
        track.lost_frames++;
        if (track.hits < std::max(1, config_.track_confirm_hits)) {
            track.hits = 0;
        }
        track.confidence = std::max(0.0f, track.confidence - config_.candidate_track_confidence_miss_drop);
    }

    for (size_t ci = 0; ci < candidates_.size(); ++ci) {
        if (candidate_matched[ci]) continue;
        auto& candidate = candidates_[ci];
        // A three-frame display/support window may still contain a cluster made
        // entirely from old points. It is useful for visualization, but it must
        // never create a fresh identity when the current radar frame contains
        // no supporting point. Legacy point files are marked as current by the
        // candidate builder, so realtime and offline processing remain aligned.
        if (!candidate.has_current_frame_support ||
            candidate.current_frame_point_count <= 0) {
            continue;
        }
        CandidateTrack track;
        track.internal_uid = next_internal_uid_++;
        track.id = config_.track_id_mode == TrackIdMode::ConfirmedOnly ? -1 : allocateDisplayId();
        track.x = candidate.x;
        track.y = candidate.y;
        track.has_signature = true;
        track.point_count = candidate.point_count;
        track.width_m = candidate.width_m;
        track.depth_m = candidate.depth_m;
        track.median_signed_velocity = candidate.median_signed_velocity;
        track.moving_ratio = candidate.moving_ratio;
        track.hits = 1;
        track.lost_frames = 0;
        track.confidence = std::clamp(0.35f + 0.45f * candidate.score -
                                      0.25f * candidate.background_score,
                                      0.1f, 0.85f);
        candidate.id = track.id;
        candidate.internal_uid = track.internal_uid;
        total_track_births_++;
        candidate_tracks_.push_back(track);
    }

    const auto track_has_expired = [this](const CandidateTrack& track) {
        if (track.was_locked) {
            // Preserve the existing locked-target contract: fifty complete
            // missed frames remain available even if visible control lock was
            // released earlier; expiry happens on the following miss.
            return track.lost_frames > config_.max_lost_frames;
        }
        const bool confirmed =
            track.hits >= std::max(1, config_.track_confirm_hits);
        const int max_lost = !confirmed
            ? config_.candidate_track_unconfirmed_max_lost_frames
            : config_.candidate_track_confirmed_max_lost_frames;
        // Non-locked tracks are removed as soon as the configured miss count
        // is reached: tentative=5, confirmed=12 by default.
        return track.lost_frames >= std::max(1, max_lost);
    };
    for (const auto& track : candidate_tracks_) {
        if (track_has_expired(track)) {
            releaseDisplayId(track.id);
        }
    }
    candidate_tracks_.erase(
        std::remove_if(candidate_tracks_.begin(), candidate_tracks_.end(),
                       [&track_has_expired](const CandidateTrack& track) {
                           return track_has_expired(track);
                       }),
        candidate_tracks_.end());
}

const CandidateTarget* TargetTracker::findCandidateById(int candidate_id) const
{
    auto it = std::find_if(candidates_.begin(), candidates_.end(),
                           [candidate_id](const CandidateTarget& c) { return c.id == candidate_id; });
    return it == candidates_.end() ? nullptr : &(*it);
}

const CandidateTarget* TargetTracker::findCandidateByUid(std::uint64_t internal_uid) const
{
    auto it = std::find_if(candidates_.begin(), candidates_.end(),
                           [internal_uid](const CandidateTarget& c) {
                               return c.internal_uid == internal_uid;
                           });
    return it == candidates_.end() ? nullptr : &(*it);
}

void TargetTracker::recordCandidateGateAudit(float predicted_x, float predicted_y)
{
    for (const auto& candidate : association_candidates_) {
        CandidateGateAudit audit;
        audit.candidate = candidate;
        audit.locked_uid = locked_.matched_internal_uid;
        audit.lost_before = locked_.lost_frames;
        audit.predicted_x = predicted_x; audit.predicted_y = predicted_y;
        audit.last_accepted_x = last_accepted_candidate_.x;
        audit.last_accepted_y = last_accepted_candidate_.y;
        audit.distance_m = std::hypot(candidate.x-predicted_x, candidate.y-predicted_y);
        float ex=0, ey=0, nis=0, speed=0;
        audit.normal_ok = evaluateCandidateMeasurement(candidate, predicted_x, predicted_y,
            false, ex, ey, nis, speed, audit.normal_reason);
        audit.roi_ok = evaluateCandidateMeasurement(candidate, predicted_x, predicted_y,
            true, ex, ey, nis, speed, audit.roi_reason);
        audit.nis=nis; audit.implied_speed_mps=speed;
        // Audit even a hard-distance rejection: record unrestricted NIS and
        // physical displacement separately; do not turn them into an acceptance.
        const float r=std::max(0.001f,config_.kalman_measurement_noise);
        const float s00=kalman_p_[0][0]+r, s11=kalman_p_[1][1]+r;
        const float s01=kalman_p_[0][1], s10=kalman_p_[1][0];
        const float det=s00*s11-s01*s10;
        if(det>1e-6f) audit.nis=(ex*(s11*ex-s01*ey)+ey*(-s10*ex+s00*ey))/det;
        if(has_last_accepted_candidate_) audit.implied_speed_mps=std::hypot(
            candidate.x-last_accepted_candidate_.x,candidate.y-last_accepted_candidate_.y)/
            (std::max(0.05f,current_frame_dt_s_)*std::max(1,frames_since_accepted_measurement_+1));
        MatchResult match; match.nis=nis; match.implied_speed_mps=speed;
        audit.reacquire_ok = audit.normal_ok || audit.roi_ok;
        if (audit.reacquire_ok && (candidate.internal_uid != locked_.matched_internal_uid || locked_uid_continuity_compromised_))
            audit.reacquire_ok = passesNewUidReacquireChecks(candidate, match, predicted_x, predicted_y, audit.reacquire_reason);
        if (manual_lock_confirmation_pending_ && candidate.internal_uid != locked_.matched_internal_uid) {
            audit.reacquire_ok=false; audit.reacquire_reason="manual_different_uid";
        }
        candidate_gate_audit_.push_back(audit);
    }
}

float TargetTracker::dropoutGateAllowance() const
{
    if (!config_.enable_dropout_gate_expansion || manual_lock_confirmation_pending_ ||
        locked_.lost_frames < config_.dropout_gate_start_lost_frames) return 0.0f;
    return std::clamp((locked_.lost_frames-config_.dropout_gate_start_lost_frames+1)*
        config_.dropout_gate_growth_m_per_frame,0.0f,config_.dropout_gate_max_extra_m);
}

bool TargetTracker::evaluateCandidateMeasurement(const CandidateTarget& candidate,
                                                 float predicted_x,
                                                 float predicted_y,
                                                 bool roi_fallback,
                                                 float& innovation_x,
                                                 float& innovation_y,
                                                 float& nis,
                                                 float& implied_speed_mps,
                                                 std::string& reject_reason) const
{
    innovation_x = candidate.x - predicted_x;
    innovation_y = candidate.y - predicted_y;
    nis = std::numeric_limits<float>::infinity();
    implied_speed_mps = 0.0f;
    reject_reason.clear();

    const float hard_radius = (roi_fallback
        ? config_.roi_radius_m + 0.5f *
              turnGateBonus(config_, current_chassis_sample_)
        : config_.kalman_gate_radius_m +
              turnGateBonus(config_, current_chassis_sample_)) + (roi_fallback ? dropoutGateAllowance() : 0.0f);
    if (innovation_x * innovation_x + innovation_y * innovation_y >
        hard_radius * hard_radius) {
        reject_reason = roi_fallback ? "roi_distance" : "kalman_distance";
        return false;
    }

    const float r = std::max(0.001f, config_.kalman_measurement_noise);
    const float s00 = kalman_p_[0][0] + r;
    const float s01 = kalman_p_[0][1];
    const float s10 = kalman_p_[1][0];
    const float s11 = kalman_p_[1][1] + r;
    const float det = s00 * s11 - s01 * s10;
    if (det <= 1e-6f) {
        reject_reason = "singular_innovation_covariance";
        return false;
    }
    const float inv00 = s11 / det;
    const float inv01 = -s01 / det;
    const float inv10 = -s10 / det;
    const float inv11 = s00 / det;
    nis = innovation_x * (inv00 * innovation_x + inv01 * innovation_y) +
          innovation_y * (inv10 * innovation_x + inv11 * innovation_y);
    const float nis_limit = roi_fallback ? config_.roi_gate_chi2
                                         : config_.kalman_gate_chi2;
    if (!std::isfinite(nis) || nis > nis_limit) {
        reject_reason = roi_fallback ? "roi_nis" : "kalman_nis";
        return false;
    }

    if (has_last_accepted_candidate_) {
        const float elapsed = std::max(0.05f, current_frame_dt_s_) *
            static_cast<float>(std::max(1, frames_since_accepted_measurement_ + 1));
        implied_speed_mps = std::sqrt(squaredDistance(
            candidate.x, candidate.y,
            last_accepted_candidate_.x, last_accepted_candidate_.y)) / elapsed;
        if (implied_speed_mps > config_.max_measurement_speed_mps) {
            reject_reason = "physical_speed";
            return false;
        }
    }

    if (roi_fallback && has_last_accepted_candidate_) {
        const float reference_points = static_cast<float>(
            std::max(1, last_accepted_candidate_.point_count));
        const float point_ratio = static_cast<float>(candidate.point_count) /
                                  reference_points;
        if (point_ratio < config_.roi_point_count_ratio_min ||
            point_ratio > config_.roi_point_count_ratio_max) {
            reject_reason = "roi_point_count";
            return false;
        }
        if (std::abs(candidate.width_m - last_accepted_candidate_.width_m) >
            config_.roi_max_width_delta_m) {
            reject_reason = "roi_width";
            return false;
        }
        if (std::abs(candidate.depth_m - last_accepted_candidate_.depth_m) >
            config_.roi_max_depth_delta_m) {
            reject_reason = "roi_depth";
            return false;
        }
        // This deliberately wide Doppler guard still permits a walking person
        // to stop.  It only rejects a grossly inconsistent blob.
        if (std::abs(candidate.average_abs_velocity -
                     last_accepted_candidate_.average_abs_velocity) >
            config_.roi_max_abs_velocity_delta_mps) {
            reject_reason = "roi_doppler";
            return false;
        }
    }
    return true;
}

bool TargetTracker::isSuspiciousContinuityMatch(
    const CandidateTarget& candidate, const MatchResult& match) const
{
    if (!has_last_accepted_candidate_) return false;

    const float innovation_distance = std::hypot(match.innovation_x,
                                                  match.innovation_y);
    if (!std::isfinite(innovation_distance) ||
        innovation_distance > config_.suspicious_continuity_distance_m) {
        return true;
    }
    if (!std::isfinite(match.implied_speed_mps) ||
        match.implied_speed_mps > config_.suspicious_continuity_speed_mps) {
        return true;
    }

    const float reference_points = static_cast<float>(
        std::max(1, last_accepted_candidate_.point_count));
    const float point_ratio = static_cast<float>(candidate.point_count) /
                              reference_points;
    if (point_ratio < config_.reacquire_point_count_ratio_min ||
        point_ratio > config_.reacquire_point_count_ratio_max) {
        return true;
    }
    if (std::abs(candidate.width_m - last_accepted_candidate_.width_m) >
        config_.reacquire_max_width_delta_m) {
        return true;
    }
    if (std::abs(candidate.depth_m - last_accepted_candidate_.depth_m) >
        config_.reacquire_max_depth_delta_m) {
        return true;
    }
    return std::abs(candidate.average_abs_velocity -
                    last_accepted_candidate_.average_abs_velocity) >
           config_.reacquire_max_abs_velocity_delta_mps;
}

bool TargetTracker::passesNewUidReacquireChecks(
    const CandidateTarget& candidate,
    const MatchResult& match,
    float predicted_x,
    float predicted_y,
    std::string& reject_reason) const
{
    reject_reason.clear();
    if (candidate.point_count < config_.new_uid_reacquire_min_points) {
        reject_reason = "reacquire_too_few_points";
        return false;
    }

    const float prediction_distance = std::hypot(candidate.x - predicted_x,
                                                 candidate.y - predicted_y);
    if (!std::isfinite(prediction_distance) ||
        prediction_distance > config_.new_uid_reacquire_max_distance_m + dropoutGateAllowance()) {
        reject_reason = "reacquire_distance";
        return false;
    }
    if (!std::isfinite(match.nis) ||
        match.nis > config_.new_uid_reacquire_max_nis) {
        reject_reason = "reacquire_nis";
        return false;
    }
    if (!std::isfinite(match.implied_speed_mps) ||
        match.implied_speed_mps > config_.new_uid_reacquire_max_speed_mps) {
        reject_reason = "reacquire_speed";
        return false;
    }

    if (!has_last_accepted_candidate_) return true;

    const float reference_points = static_cast<float>(
        std::max(1, last_accepted_candidate_.point_count));
    const float point_ratio = static_cast<float>(candidate.point_count) /
                              reference_points;
    if (point_ratio < config_.reacquire_point_count_ratio_min ||
        point_ratio > config_.reacquire_point_count_ratio_max) {
        reject_reason = "reacquire_point_count";
        return false;
    }
    if (std::abs(candidate.width_m - last_accepted_candidate_.width_m) >
        config_.reacquire_max_width_delta_m) {
        reject_reason = "reacquire_width";
        return false;
    }
    if (std::abs(candidate.depth_m - last_accepted_candidate_.depth_m) >
        config_.reacquire_max_depth_delta_m) {
        reject_reason = "reacquire_depth";
        return false;
    }
    if (std::abs(candidate.average_abs_velocity -
                 last_accepted_candidate_.average_abs_velocity) >
        config_.reacquire_max_abs_velocity_delta_mps) {
        reject_reason = "reacquire_doppler";
        return false;
    }
    return true;
}

bool TargetTracker::isReacquireSequenceContinuous(
    const CandidateTarget& candidate,
    const ReacquireHypothesis& hypothesis) const
{
    if (!hypothesis.active) return true;

    const CandidateTarget& previous = hypothesis.latest;
    const float dt = std::max(0.05f, current_frame_dt_s_);
    const float elapsed_since_hit = dt * static_cast<float>(
        std::max(1, hypothesis.frames_since_last_hit));
    const float max_step = std::min(config_.reacquire_max_consecutive_step_m,
                                    config_.new_uid_reacquire_max_speed_mps *
                                        elapsed_since_hit);
    const float step = std::hypot(candidate.x - previous.x,
                                  candidate.y - previous.y);
    if (!std::isfinite(step) || step > max_step) return false;

    const float previous_points = static_cast<float>(
        std::max(1, previous.point_count));
    const float point_ratio = static_cast<float>(candidate.point_count) /
                              previous_points;
    if (point_ratio < config_.reacquire_point_count_ratio_min ||
        point_ratio > config_.reacquire_point_count_ratio_max) {
        return false;
    }
    if (std::abs(candidate.width_m - previous.width_m) >
        config_.reacquire_max_width_delta_m) {
        return false;
    }
    if (std::abs(candidate.depth_m - previous.depth_m) >
        config_.reacquire_max_depth_delta_m) {
        return false;
    }
    return std::abs(candidate.average_abs_velocity -
                    previous.average_abs_velocity) <=
           config_.reacquire_max_abs_velocity_delta_mps;
}

TargetTracker::MatchResult TargetTracker::findBestCandidateWithKalmanGate(
    float predicted_x, float predicted_y)
{
    MatchResult accepted_result;
    accepted_result.source = "kalman_gate";
    accepted_result.considered_candidates =
        static_cast<int>(association_candidates_.size());
    MatchResult rejected_result;
    rejected_result.source = "kalman_gate";
    rejected_result.considered_candidates =
        accepted_result.considered_candidates;
    float best_cost = std::numeric_limits<float>::max();
    float nearest_rejected_d2 = std::numeric_limits<float>::max();

    for (auto& candidate : association_candidates_) {
        float ex = 0.0f;
        float ey = 0.0f;
        float nis = 0.0f;
        float speed = 0.0f;
        std::string reason;
        if (!evaluateCandidateMeasurement(candidate, predicted_x, predicted_y,
                                          false, ex, ey, nis, speed, reason)) {
            const float d2 = ex * ex + ey * ey;
            if (d2 < nearest_rejected_d2) {
                nearest_rejected_d2 = d2;
                rejected_result.reject_reason = reason;
                rejected_result.rejected = reason != "kalman_distance";
                rejected_result.innovation_x = ex;
                rejected_result.innovation_y = ey;
                rejected_result.nis = nis;
                rejected_result.implied_speed_mps = speed;
            }
            continue;
        }

        accepted_result.gate_candidate_count++;
        float cost = nis + 0.03f * speed * speed;
        if (candidate.internal_uid == locked_.matched_internal_uid) {
            cost *= 0.25f;
        }
        cost -= 0.12f * candidate.moving_ratio;
        cost -= 0.04f * candidate.average_abs_velocity;
        cost += 0.15f * candidate.background_score;
        if (cost < best_cost) {
            best_cost = cost;
            accepted_result.candidate = &candidate;
            accepted_result.innovation_x = ex;
            accepted_result.innovation_y = ey;
            accepted_result.nis = nis;
            accepted_result.implied_speed_mps = speed;
            accepted_result.reject_reason.clear();
            accepted_result.rejected = false;
        }
    }

    // Never let a later rejected distractor overwrite the metrics belonging
    // to an already accepted candidate.  This was the direct cause of rows
    // where candidate UID 1 was correct but innovation/NIS came from UID 9.
    if (accepted_result.candidate) {
        return accepted_result;
    }
    if (rejected_result.reject_reason.empty()) {
        rejected_result.reject_reason = association_candidates_.empty()
            ? "no_candidates" : "no_candidate_in_kalman_gate";
    }
    return rejected_result;
}

TargetTracker::MatchResult TargetTracker::findBestCandidateInRoi(
    float predicted_x, float predicted_y)
{
    MatchResult accepted_result;
    accepted_result.source = "roi_fallback";
    accepted_result.considered_candidates =
        static_cast<int>(association_candidates_.size());
    MatchResult rejected_result;
    rejected_result.source = "roi_fallback";
    rejected_result.considered_candidates =
        accepted_result.considered_candidates;
    float best_cost = std::numeric_limits<float>::max();
    float nearest_rejected_d2 = std::numeric_limits<float>::max();

    for (auto& candidate : association_candidates_) {
        float ex = 0.0f;
        float ey = 0.0f;
        float nis = 0.0f;
        float speed = 0.0f;
        std::string reason;
        if (!evaluateCandidateMeasurement(candidate, predicted_x, predicted_y,
                                          true, ex, ey, nis, speed, reason)) {
            const float d2 = ex * ex + ey * ey;
            if (d2 < nearest_rejected_d2) {
                nearest_rejected_d2 = d2;
                rejected_result.reject_reason = reason;
                rejected_result.rejected = reason != "roi_distance";
                rejected_result.innovation_x = ex;
                rejected_result.innovation_y = ey;
                rejected_result.nis = nis;
                rejected_result.implied_speed_mps = speed;
            }
            continue;
        }

        accepted_result.gate_candidate_count++;
        float cost = nis + 0.05f * speed * speed +
                     0.25f * candidate.background_score -
                     0.08f * candidate.score;
        if (candidate.internal_uid == locked_.matched_internal_uid) {
            cost *= 0.30f;
        }
        if (cost < best_cost) {
            best_cost = cost;
            accepted_result.candidate = &candidate;
            accepted_result.innovation_x = ex;
            accepted_result.innovation_y = ey;
            accepted_result.nis = nis;
            accepted_result.implied_speed_mps = speed;
            accepted_result.reject_reason.clear();
            accepted_result.rejected = false;
        }
    }

    if (accepted_result.candidate) {
        return accepted_result;
    }
    if (rejected_result.reject_reason.empty()) {
        rejected_result.reject_reason = association_candidates_.empty()
            ? "no_candidates" : "no_candidate_in_roi_gate";
    }
    return rejected_result;
}

bool TargetTracker::isAcceptedMatchConsistent(const MatchResult& match,
                                              float predicted_x,
                                              float predicted_y) const
{
    if (!match.candidate) return true;
    if (match.rejected || !match.reject_reason.empty()) return false;
    if (!std::isfinite(match.innovation_x) ||
        !std::isfinite(match.innovation_y) ||
        !std::isfinite(match.nis) ||
        !std::isfinite(match.implied_speed_mps)) {
        return false;
    }

    const float expected_x = match.candidate->x - predicted_x;
    const float expected_y = match.candidate->y - predicted_y;
    const float scale = std::max({1.0f, std::abs(expected_x),
                                  std::abs(expected_y)});
    const float tolerance = 1e-4f * scale;
    return std::abs(match.innovation_x - expected_x) <= tolerance &&
           std::abs(match.innovation_y - expected_y) <= tolerance;
}

void TargetTracker::fillAssociationDiagnostics(const MatchResult& match,
                                               const CandidateTarget* candidate)
{
    association_diagnostics_.source = match.source;
    association_diagnostics_.reject_reason = match.reject_reason;
    association_diagnostics_.measurement_rejected = match.rejected;
    association_diagnostics_.innovation_x = match.innovation_x;
    association_diagnostics_.innovation_y = match.innovation_y;
    association_diagnostics_.nis = match.nis;
    association_diagnostics_.implied_speed_mps = match.implied_speed_mps;
    association_diagnostics_.considered_candidates = match.considered_candidates;
    association_diagnostics_.gate_candidate_count = match.gate_candidate_count;
    if (!candidate) return;
    association_diagnostics_.candidate_display_id = candidate->id;
    association_diagnostics_.candidate_internal_uid = candidate->internal_uid;
    association_diagnostics_.candidate_point_count = candidate->point_count;
    association_diagnostics_.candidate_width_m = candidate->width_m;
    association_diagnostics_.candidate_depth_m = candidate->depth_m;
    association_diagnostics_.candidate_average_abs_velocity =
        candidate->average_abs_velocity;
    association_diagnostics_.candidate_moving_ratio = candidate->moving_ratio;
    association_diagnostics_.candidate_score = candidate->score;
    association_diagnostics_.candidate_background_score =
        candidate->background_score;
    association_diagnostics_.candidate_current_frame_points =
        candidate->current_frame_point_count;
    association_diagnostics_.candidate_current_frame_ratio =
        candidate->current_frame_point_ratio;
    association_diagnostics_.candidate_dominant_range_bin =
        candidate->dominant_range_bin;
    association_diagnostics_.candidate_dominant_doppler_bin =
        candidate->dominant_doppler_bin;
    association_diagnostics_.candidate_unique_range_doppler_cells =
        candidate->unique_range_doppler_cells;
    association_diagnostics_.candidate_average_snr = candidate->average_snr;
    association_diagnostics_.candidate_average_doa_peak_confidence =
        candidate->average_doa_peak_confidence;
}

bool TargetTracker::isStaticBackgroundLike(const CandidateTarget& candidate) const
{
    return candidate.moving_ratio <= config_.background_motion_ratio_threshold &&
           candidate.average_abs_velocity <= config_.background_velocity_threshold;
}

bool TargetTracker::isBackgroundMotionTolerant(const CandidateTarget& candidate) const
{
    return candidate.moving_ratio <= config_.background_motion_tolerant_ratio &&
           candidate.average_abs_velocity <= config_.background_velocity_tolerant_mps;
}

bool TargetTracker::isLockedRoiCompanion(const CandidateTarget& candidate) const
{
    if (!config_.absorb_locked_roi_blobs || !locked_.valid) return false;
    if (!locked_.has_measurement || locked_.lost_frames > 0) return false;
    if (candidate.internal_uid == locked_.matched_internal_uid) return false;
    if (candidate.point_count > config_.locked_roi_absorb_max_points) return false;

    // Do not couple fragment suppression to the much wider tracking ROI.  A
    // user increasing roi_radius_m must not erase neighbouring real people.
    const float absorb_radius = config_.locked_roi_absorb_radius_m;
    const float d2 = squaredDistance(candidate.x, candidate.y, locked_.x, locked_.y);
    if (d2 > absorb_radius * absorb_radius) return false;

    const bool small_blob = candidate.point_count <= config_.locked_roi_absorb_max_points;
    const bool near_locked_measurement =
        locked_.has_measurement &&
        squaredDistance(candidate.x, candidate.y, locked_.measured_x, locked_.measured_y) <=
            absorb_radius * absorb_radius;
    return small_blob && near_locked_measurement;
}

void TargetTracker::updateBackgroundModel()
{
    for (auto& track : background_tracks_) {
        track.missed_frames++;
    }

    for (const auto& candidate : candidates_) {
        if (!isStaticBackgroundLike(candidate)) continue;

        BackgroundTrack* best = nullptr;
        float best_d2 = config_.background_match_radius_m * config_.background_match_radius_m;
        for (auto& track : background_tracks_) {
            const float d2 = squaredDistance(candidate.x, candidate.y, track.x, track.y);
            if (d2 <= best_d2) {
                best_d2 = d2;
                best = &track;
            }
        }

        if (best) {
            // Slow centroid update keeps this model stable but lets it breathe a little.
            best->x = best->x * 0.85f + candidate.x * 0.15f;
            best->y = best->y * 0.85f + candidate.y * 0.15f;
            best->stable_frames++;
            best->missed_frames = 0;
        } else {
            BackgroundTrack track;
            track.x = candidate.x;
            track.y = candidate.y;
            track.stable_frames = 1;
            track.missed_frames = 0;
            background_tracks_.push_back(track);
        }
    }

    background_tracks_.erase(
        std::remove_if(background_tracks_.begin(), background_tracks_.end(),
                       [this](const BackgroundTrack& track) {
                           return track.missed_frames > config_.background_max_missed_frames;
                       }),
        background_tracks_.end());
}

float TargetTracker::backgroundScoreForCandidate(const CandidateTarget& candidate) const
{
    if (!isBackgroundMotionTolerant(candidate)) return 0.0f;

    float best_score = 0.0f;
    const float match2 = config_.background_match_radius_m * config_.background_match_radius_m;
    for (const auto& track : background_tracks_) {
        const float d2 = squaredDistance(candidate.x, candidate.y, track.x, track.y);
        if (d2 > match2) continue;

        const float stability = static_cast<float>(track.stable_frames) /
                                std::max(1, config_.background_stability_frames);
        const float distance_factor = 1.0f - clamp01(std::sqrt(d2) / config_.background_match_radius_m);
        const float motion_factor =
            isStaticBackgroundLike(candidate)
                ? 1.0f
                : (0.45f + 0.35f * (1.0f - clamp01(candidate.average_abs_velocity /
                                                     std::max(0.01f, config_.background_velocity_tolerant_mps))));
        best_score = std::max(best_score,
                              clamp01(stability) * (0.6f + 0.4f * distance_factor) * clamp01(motion_factor));
    }
    return best_score;
}

void TargetTracker::applyBackgroundScores()
{
    for (auto& candidate : candidates_) {
        candidate.background_score = backgroundScoreForCandidate(candidate);
        candidate.score = std::max(0.0f, candidate.score - config_.background_score_penalty * candidate.background_score);
    }

    for (auto& debug : cluster_debug_) {
        debug.background_score = 0.0f;
        debug.score = std::max(0.0f, debug.score);
        for (const auto& candidate : candidates_) {
            const float d2 = squaredDistance(debug.x, debug.y, candidate.x, candidate.y);
            if (d2 <= config_.background_match_radius_m * config_.background_match_radius_m) {
                debug.background_score = candidate.background_score;
                debug.score = candidate.score;
                break;
            }
        }
    }
}

void TargetTracker::suppressLockedRoiCompanions()
{
    if (!config_.absorb_locked_roi_blobs || !locked_.valid || candidates_.empty()) return;

    std::vector<int> suppressed_ids;
    for (auto& candidate : candidates_) {
        if (!isLockedRoiCompanion(candidate)) continue;
        candidate.score = std::max(0.0f, candidate.score - config_.locked_roi_absorb_score_penalty);
        candidate.background_score = std::max(candidate.background_score, 0.35f);
        if (candidate.id >= 0 && candidate.internal_uid != locked_.matched_internal_uid) {
            suppressed_ids.push_back(candidate.id);
        }
    }

    for (auto& debug : cluster_debug_) {
        for (const auto& candidate : candidates_) {
            if (!isLockedRoiCompanion(candidate)) continue;
            const float d2 = squaredDistance(debug.x, debug.y, candidate.x, candidate.y);
            if (d2 <= config_.background_match_radius_m * config_.background_match_radius_m) {
                debug.score = candidate.score;
                debug.background_score = std::max(debug.background_score, candidate.background_score);
                break;
            }
        }
    }

    candidates_.erase(
        std::remove_if(candidates_.begin(), candidates_.end(),
                       [this](const CandidateTarget& candidate) {
                           return isLockedRoiCompanion(candidate);
                       }),
        candidates_.end());

    if (!suppressed_ids.empty()) {
        for (const auto& track : candidate_tracks_) {
            if (std::find(suppressed_ids.begin(), suppressed_ids.end(), track.id) !=
                suppressed_ids.end()) {
                releaseDisplayId(track.id);
            }
        }
        candidate_tracks_.erase(
            std::remove_if(candidate_tracks_.begin(), candidate_tracks_.end(),
                           [&suppressed_ids](const CandidateTrack& track) {
                               return std::find(suppressed_ids.begin(), suppressed_ids.end(), track.id) !=
                                      suppressed_ids.end();
                           }),
            candidate_tracks_.end());
    }
}

void TargetTracker::beginManualLockConfirmation(const CandidateTarget& candidate)
{
    // Seed only the selected identity.  The following frame must observe the
    // same immutable UID before normal tracking is declared.
    lockFromCandidate(candidate, true);
    manual_lock_confirmation_pending_ = true;
    last_measurement_time_sec_ = 0;
    manual_lock_confirmation_misses_ = 0;

    reacquire_hypotheses_.clear();
    ReacquireHypothesis hypothesis;
    hypothesis.internal_uid = candidate.internal_uid;
    hypothesis.candidate_display_id = candidate.id;
    hypothesis.consecutive_hits = 1;
    hypothesis.window_frames = 1;
    hypothesis.frames_since_last_hit = 0;
    hypothesis.allow_one_frame_gap = false;
    hypothesis.latest = candidate;
    hypothesis.original_uid = true;
    reacquire_hypotheses_.push_back(hypothesis);

    locked_.stable_frames = 0;
    locked_.reacquire_hits = 1;
    association_diagnostics_.source = "manual_lock_pending";
    association_diagnostics_.pending_internal_uid = candidate.internal_uid;
    association_diagnostics_.pending_hits = 1;
    fillBestReacquireDiagnostics();
    updateFollowControlAdvice(locked_, config_);
    holdManualConfirmationMotion();
}

void TargetTracker::lockFromCandidate(const CandidateTarget& candidate, bool reset_velocity)
{
    selection_event_.session_id = frame_context_.session_id;
    ++selection_event_.selection_epoch;
    selection_event_.internal_uid = candidate.internal_uid;
    last_measurement_time_sec_ = frame_context_.sample_time_sec;
    const float old_x = locked_.x;
    const float old_y = locked_.y;

    locked_.valid = true;
    locked_.display_id = candidate.id;
    locked_.matched_internal_uid = candidate.internal_uid;
    locked_.matched_candidate_id = candidate.id;
    locked_.has_measurement = true;
    locked_.measured_x = candidate.x;
    locked_.measured_y = candidate.y;
    locked_.measured_range_m = candidate.range_m;
    locked_.measured_angle_deg = candidate.angle_deg;
    locked_.x = candidate.x;
    locked_.y = candidate.y;
    locked_.range_m = candidate.range_m;
    locked_.angle_deg = candidate.angle_deg;
    locked_.lost_frames = 0;
    locked_.stable_frames = 1;
    locked_.reacquire_hits = 0;
    locked_.confidence = std::clamp(0.45f + 0.45f * candidate.score - 0.25f * candidate.background_score,
                                    0.35f, 0.85f);
    last_accepted_candidate_ = candidate;
    has_last_accepted_candidate_ = true;
    frames_since_accepted_measurement_ = 0;
    locked_uid_continuity_compromised_ = false;
    manual_lock_confirmation_pending_ = false;
    manual_lock_confirmation_misses_ = 0;
    clearReacquireHypothesis();
    association_diagnostics_ = AssociationDiagnostics();
    association_diagnostics_.source = "manual_lock";
    association_diagnostics_.candidate_display_id = candidate.id;
    association_diagnostics_.candidate_internal_uid = candidate.internal_uid;
    updateFollowControlAdvice(locked_, config_);

    if (reset_velocity) {
        locked_.vx = 0.0f;
        locked_.vy = 0.0f;
    } else {
        locked_.vx = locked_.x - old_x;
        locked_.vy = locked_.y - old_y;
    }

    initKalmanFromLocked();
}

void TargetTracker::initKalmanFromLocked()
{
    kalman_initialized_ = locked_.valid;
    kalman_x_ = locked_.x;
    kalman_y_ = locked_.y;
    kalman_vx_ = locked_.vx;
    kalman_vy_ = locked_.vy;

    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            kalman_p_[r][c] = 0.0f;
        }
    }
    kalman_p_[0][0] = 0.35f;
    kalman_p_[1][1] = 0.35f;
    kalman_p_[2][2] = 1.0f;
    kalman_p_[3][3] = 1.0f;
}

void TargetTracker::predictKalman(float& predicted_x, float& predicted_y)
{
    if (!kalman_initialized_) {
        initKalmanFromLocked();
    }
    if (!kalman_initialized_) {
        predicted_x = locked_.x;
        predicted_y = locked_.y;
        return;
    }

    const float dt = std::max(0.01f, current_frame_dt_s_);
    float step_x = kalman_vx_ * dt;
    float step_y = kalman_vy_ * dt;
    // The first missing frame still gets a normal CV prediction.  From the
    // second missing frame onward, cap the displacement as an independent
    // guard against an old or spurious velocity estimate and a long frame dt.
    if (locked_.lost_frames > 0) {
        const float max_step = std::max(0.0f, config_.prediction_max_step_m);
        const float step = std::hypot(step_x, step_y);
        if (max_step > 0.0f && step > max_step) {
            const float scale = max_step / step;
            step_x *= scale;
            step_y *= scale;
            // Keep the state transition and the exported velocity consistent.
            kalman_vx_ = step_x / dt;
            kalman_vy_ = step_y / dt;
        }
    }
    kalman_x_ += step_x;
    kalman_y_ += step_y;

    float f[4][4] = {
        {1.0f, 0.0f, dt,   0.0f},
        {0.0f, 1.0f, 0.0f, dt  },
        {0.0f, 0.0f, 1.0f, 0.0f},
        {0.0f, 0.0f, 0.0f, 1.0f}
    };

    float fp[4][4] = {};
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            for (int k = 0; k < 4; ++k) {
                fp[r][c] += f[r][k] * kalman_p_[k][c];
            }
        }
    }

    float predicted_p[4][4] = {};
    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            for (int k = 0; k < 4; ++k) {
                predicted_p[r][c] += fp[r][k] * f[c][k];
            }
        }
    }

    const float q = std::max(0.001f, config_.kalman_process_noise);
    predicted_p[0][0] += q * dt * dt;
    predicted_p[1][1] += q * dt * dt;
    predicted_p[2][2] += q;
    predicted_p[3][3] += q;

    for (int r = 0; r < 4; ++r) {
        for (int c = 0; c < 4; ++c) {
            kalman_p_[r][c] = predicted_p[r][c];
        }
    }

    predicted_x = kalman_x_;
    predicted_y = kalman_y_;
}

void TargetTracker::updateKalman(float measured_x, float measured_y)
{
    if (!kalman_initialized_) {
        initKalmanFromLocked();
    }
    if (!kalman_initialized_) return;

    const float r = std::max(0.001f, config_.kalman_measurement_noise);
    const float innovation_x = measured_x - kalman_x_;
    const float innovation_y = measured_y - kalman_y_;

    const float s00 = kalman_p_[0][0] + r;
    const float s01 = kalman_p_[0][1];
    const float s10 = kalman_p_[1][0];
    const float s11 = kalman_p_[1][1] + r;
    const float det = s00 * s11 - s01 * s10;
    if (std::abs(det) < 1e-6f) return;

    const float inv00 = s11 / det;
    const float inv01 = -s01 / det;
    const float inv10 = -s10 / det;
    const float inv11 = s00 / det;

    float k[4][2] = {};
    for (int row = 0; row < 4; ++row) {
        k[row][0] = kalman_p_[row][0] * inv00 + kalman_p_[row][1] * inv10;
        k[row][1] = kalman_p_[row][0] * inv01 + kalman_p_[row][1] * inv11;
    }

    kalman_x_ += k[0][0] * innovation_x + k[0][1] * innovation_y;
    kalman_y_ += k[1][0] * innovation_x + k[1][1] * innovation_y;
    kalman_vx_ += k[2][0] * innovation_x + k[2][1] * innovation_y;
    kalman_vy_ += k[3][0] * innovation_x + k[3][1] * innovation_y;

    float old_p[4][4] = {};
    for (int r_idx = 0; r_idx < 4; ++r_idx) {
        for (int c_idx = 0; c_idx < 4; ++c_idx) {
            old_p[r_idx][c_idx] = kalman_p_[r_idx][c_idx];
        }
    }

    for (int row = 0; row < 4; ++row) {
        for (int col = 0; col < 4; ++col) {
            kalman_p_[row][col] = old_p[row][col]
                - k[row][0] * old_p[0][col]
                - k[row][1] * old_p[1][col];
        }
    }
}

} // namespace TargetTracking
