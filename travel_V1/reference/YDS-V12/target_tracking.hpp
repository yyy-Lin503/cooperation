#ifndef TARGET_TRACKING_HPP
#define TARGET_TRACKING_HPP

#include <cstdint>
#include <string>
#include <vector>

#include "radar_runtime_config.hpp"
#include "signal_processing.hpp"
#include "trajectory_association.hpp"

namespace TargetTracking {

enum class AutoSelectRule {
    Nearest,
    Fastest,
    MovingFirst
};

// Candidate ID policy. It can also be overridden at runtime with
// RADAR_TRACK_ID_MODE=monotonic|confirmed|recycled.
enum class TrackIdMode {
    Monotonic = 0,       // Every new tentative track immediately consumes a new ID.
    ConfirmedOnly = 1,   // An ID is exposed only after N consecutive associations.
    RecycledDisplay = 2  // Reuse small display IDs; internal UID remains monotonic.
};

struct TrackingConfig {
    TrackIdMode track_id_mode = TrackIdMode::Monotonic;
    int track_confirm_hits = 3;

    // DBSCAN-like clustering parameters in meters.
    float cluster_eps = 0.75f;
    int min_cluster_points = 2;

    // Keep the multi-frame window for sparse DBSCAN support, but do not let
    // old points drag the current position backwards.  Two or more points in
    // the newest frame use their own centroid.  With only one newest-frame
    // point, per-frame centroids are blended using the weights below.  A
    // cluster with no newest-frame support is diagnostic/history only and is
    // not emitted as a measurement candidate.
    bool prefer_current_frame_position = true;
    int current_frame_centroid_min_points = 2;
    float current_frame_single_point_weight = 0.70f;
    float previous_frame_centroid_weight = 0.20f;
    float second_previous_frame_centroid_weight = 0.10f;
    bool require_current_frame_support = true;

    // Human candidate constraints. mmWave 2D point clouds cannot prove a
    // target is a person, so these gates keep only "person-like" compact blobs.
    float min_range_m = RadarRuntimeConfig::kMinimumUsableRangeM;
    float max_range_m = 18.0f;
    float max_width_m = 2.0f;
    float max_depth_m = 2.0f;
    bool enable_near_field_point_filter = true;
    float near_field_filter_range_m = RadarRuntimeConfig::kMinimumUsableRangeM;
    bool enable_chassis_mask = true;
    float chassis_mask_half_width_m = 1.5f;
    float chassis_mask_forward_m = RadarRuntimeConfig::kMinimumUsableRangeM;
    float chassis_mask_backward_m = 0.2f;

    // Dynamic ROI used after a target is locked.
    float roi_radius_m = 2.0f;
    int max_lost_frames = 50;
    // Missing-measurement policy at 10 Hz:
    //   1..30  -> PREDICTED, progressively reduce commands;
    //   31..49 -> STOPPED, retain the identity for reacquisition;
    //   50     -> manual mode releases the visible lock.
    int manual_unlock_lost_frames = 50;
    int lost_limited_follow_frames = 30;
    int lost_yaw_only_frames = 30; // Compatibility only; yaw-only control is disabled.
    int reacquire_confirm_frames = 2;
    int reacquire_window_frames = 4;
    // A manual click selects the identity, but does not immediately authorize
    // motion.  The same immutable UID must remain observable for this many
    // confirmation hits before its position/velocity are committed to the
    // main Kalman state.  The clicked frame is the first confirmation hit.
    int manual_lock_confirm_frames = 2;
    int manual_lock_confirm_max_missed_frames = 5;
    float manual_lock_max_initial_speed_mps = 3.5f;
    float position_smoothing = 0.35f;

    // Single-target Kalman tracking. ROI matching is still kept as fallback.
    float kalman_dt_s = 0.1f;
    float kalman_gate_radius_m = 2.0f;
    float kalman_process_noise = 0.35f;
    float kalman_measurement_noise = 0.18f;
    float kalman_gate_chi2 = 9.21f; // 99% gate for a 2-D innovation.
    // A hard physical guard complements the statistical NIS gate.  The value
    // is deliberately wider than ordinary walking speed because the radar
    // coordinate frame and a moving cart can still leave some residual motion.
    float max_measurement_speed_mps = 5.0f;
    float high_turn_wz_threshold_radps = 0.35f;
    float turn_gate_radius_gain = 1.2f;

    // Missing-measurement prediction policy.  A short dropout keeps the CV
    // model useful, then its velocity is damped so one bad velocity estimate
    // cannot move the search centre many metres away from the real person.
    // Position covariance still grows normally while the mean velocity fades.
    int prediction_velocity_decay_start_lost_frames = 5;
    int prediction_velocity_stop_lost_frames = 15;
    float prediction_velocity_decay_per_frame = 0.75f;
    float prediction_max_speed_mps = 2.0f;
    float prediction_max_step_m = 0.20f;

    // ROI fallback is allowed to be slightly wider than the normal Kalman
    // gate, but it must still pass uncertainty, kinematic and coarse cluster
    // consistency checks.  These limits reject only extreme changes; ordinary
    // pose/point-count variation and a person stopping remain valid.
    float roi_gate_chi2 = 13.82f; // 99.9% gate for a 2-D innovation.
    float roi_point_count_ratio_min = 0.20f;
    float roi_point_count_ratio_max = 5.0f;
    float roi_max_width_delta_m = 1.5f;
    float roi_max_depth_delta_m = 1.5f;
    float roi_max_abs_velocity_delta_mps = 3.0f;

    // Reject only pathological near-field bursts, not ordinary sparse people.
    // The per-frame signal-processing guard is the first line of defence; this
    // cluster-level check also protects offline/imported point clouds.
    bool reject_near_field_burst_clusters = true;
    float near_field_burst_max_range_m = RadarRuntimeConfig::kNearFieldBurstMaxRangeM;
    int near_field_burst_min_cluster_points = 60;
    float near_field_burst_min_abs_velocity_mps =
        RadarRuntimeConfig::kNearFieldBurstMinAbsVelocityMps;
    float near_field_burst_min_moving_ratio = 0.85f;

    // Lightweight track manager for candidate ids. Tentative tracks are cheap
    // hypotheses and must disappear quickly; confirmed non-locked tracks may
    // bridge a longer sparse-point dropout. The locked UID uses max_lost_frames.
    int candidate_track_unconfirmed_max_lost_frames = 5;
    int candidate_track_confirmed_max_lost_frames = 12;
    float candidate_track_min_gate_m = 0.9f;
    float candidate_track_max_gate_m = 2.0f;
    float candidate_track_range_gate_scale = 0.04f;
    // A mild bias helps UID continuity without allowing the locked UID to
    // steal any nearby cluster as the old 0.20 cost multiplier could do.
    float candidate_track_locked_cost_scale = 0.65f;
    bool reject_ambiguous_locked_uid_assignment = true;
    float candidate_track_locked_ambiguity_cost_margin = 0.25f; //0.75f;-------------------------------------------
    float candidate_track_velocity_smoothing = 0.35f;
    float candidate_track_confidence_miss_drop = 0.08f;
    float candidate_track_process_variance = 0.20f;
    float candidate_track_measurement_variance = 0.18f;
    float candidate_track_gate_chi2 = 9.21f;

    // Global trajectory-association signature terms.  Position/velocity remain
    // dominant; these low-weight history terms help distinguish neighbouring
    // sparse clusters without requiring a fragile 5-6 point minimum.
    float trajectory_unassigned_cost = 5.0f;
    float trajectory_point_count_cost_weight = 0.35f;
    float trajectory_size_cost_weight = 0.25f;
    float trajectory_signed_doppler_cost_weight = 0.30f;
    float trajectory_moving_ratio_cost_weight = 0.10f;
    float trajectory_point_count_ratio_min = 0.15f;
    float trajectory_point_count_ratio_max = 6.0f;
    float trajectory_max_width_delta_m = 1.2f;
    float trajectory_max_depth_delta_m = 1.2f;
    float trajectory_max_signed_doppler_delta_mps = 3.0f;

    // Re-association safety. Up to three identities are kept in parallel so a
    // one-frame best candidate cannot erase the previous hypothesis. A
    // suspicious original UID needs 2 hits in 4 frames; a new UID needs
    // 3 hits in 5 frames and must beat the runner-up by a clear score margin.
    bool reject_ambiguous_reacquire = false;
    int max_reacquire_hypotheses = 3;
    float reacquire_min_score_margin = 0.75f;
    float reacquire_original_uid_score_bonus = 1.0f;
    float reacquire_auto_max_initial_speed_mps = 3.5f;
    int new_uid_reacquire_min_points = 3;
    int new_uid_reacquire_confirm_frames = 3;
    int new_uid_reacquire_window_frames = 5;
    float new_uid_reacquire_max_distance_m = 1.8f;
    float new_uid_reacquire_max_nis = 9.21f;
    float new_uid_reacquire_max_speed_mps = 5.0f;
    float reacquire_max_consecutive_step_m = 0.75f;
    float reacquire_point_count_ratio_min = 0.2f;
    float reacquire_point_count_ratio_max = 5.0f;
    float reacquire_max_width_delta_m = 1.5f;
    float reacquire_max_depth_delta_m = 1.5f;
    float reacquire_max_abs_velocity_delta_mps = 2.5f;

    // Even when the upstream candidate manager keeps the same UID, a large
    // discontinuity must be confirmed before it is written into the main KF.
    float suspicious_continuity_distance_m = 1.0f;
    float suspicious_continuity_speed_mps = 5.0f;

    // A candidate is treated as an actively moving person when either condition
    // is met. These thresholds are intentionally low for mmWave walking data.
    float moving_ratio_select_threshold = 0.15f;
    float moving_velocity_select_threshold = 0.08f;
    float motion_switch_margin = 0.35f;
    float turn_reselect_margin_gain = 1.0f;

    // Lightweight static-background model for offline/static-radar debugging.
    // Stable static blobs are only down-weighted during target acquisition;
    // they are not deleted, so a manually locked still person can still be tracked.
    int background_stability_frames = 18;
    int background_max_missed_frames = 30;
    float background_match_radius_m = 0.45f;
    float background_motion_ratio_threshold = 0.08f;
    float background_velocity_threshold = 0.05f;
    float background_motion_tolerant_ratio = 0.45f;
    float background_velocity_tolerant_mps = 0.35f;
    float background_score_penalty = 0.45f;
    int background_model_frames = 120;

    // When a single target is locked, tiny moving blobs around the lock are
    // usually sidelobes/multipath/5-frame smear from the same person. Absorb
    // them into the locked target instead of creating new target ids.
    bool absorb_locked_roi_blobs = true;
    // Only suppress a 2-point fragment immediately beside a currently
    // measured lock.  A 3-4 point blob may be the real sparse person and must
    // remain available for safe re-association.
    float locked_roi_absorb_radius_m = 0.45f;
    int locked_roi_absorb_max_points = 2;
    float locked_roi_absorb_score_penalty = 0.65f;

    // Global re-acquisition is allowed only when the current lock is weak or
    // strongly looks like a stable background object.
    float reacquire_confidence_threshold = 0.55f;
    float hard_reacquire_confidence_threshold = 0.35f;
    float background_reacquire_threshold = 0.85f;
    int reacquire_min_lost_frames = 3;
    int min_locked_frames_before_global_reacquire = 8;

    // Lock quality score update. Confidence is no longer a simple hit counter:
    // it rises faster for close Kalman-gate matches and falls for misses/background.
    float confidence_hit_base_gain = 0.035f;
    float confidence_match_gain = 0.12f;
    float confidence_candidate_score_gain = 0.04f;
    float confidence_background_penalty = 0.12f;
    float confidence_miss_drop = 0.12f;
};

enum class CandidatePositionMode {
    WindowCentroid = 0,
    CurrentFrameCentroid = 1,
    TemporalWeightedCentroid = 2,
    NoCurrentFrameSupport = 3
};

struct CandidateTarget {
    int id = -1;
    std::uint64_t internal_uid = 0;
    float x = 0.0f;
    float y = 0.0f;
    float range_m = 0.0f;
    float angle_deg = 0.0f;
    float average_abs_velocity = 0.0f;
    float median_signed_velocity = 0.0f;
    float moving_ratio = 0.0f;
    float width_m = 0.0f;
    float depth_m = 0.0f;
    float score = 0.0f;
    float background_score = 0.0f;
    int point_count = 0;
    int oldest_source_frame = -1;
    int newest_source_frame = -1;
    int current_frame_point_count = 0;
    float current_frame_point_ratio = 0.0f;
    CandidatePositionMode position_mode = CandidatePositionMode::WindowCentroid;
    bool has_current_frame_support = true;
    int dominant_range_bin = -1;
    int dominant_doppler_bin = -1;
    int unique_range_doppler_cells = 0;
    float average_snr = 0.0f;
    float average_doa_peak_confidence = 0.0f;
};

struct ClusterDebugInfo {
    int cluster_id = -1;
    bool person_like = false;
    float x = 0.0f;
    float y = 0.0f;
    float range_m = 0.0f;
    float angle_deg = 0.0f;
    float average_abs_velocity = 0.0f;
    float median_signed_velocity = 0.0f;
    float moving_ratio = 0.0f;
    float width_m = 0.0f;
    float depth_m = 0.0f;
    float score = 0.0f;
    float background_score = 0.0f;
    int point_count = 0;
    int oldest_source_frame = -1;
    int newest_source_frame = -1;
    int current_frame_point_count = 0;
    float current_frame_point_ratio = 0.0f;
    CandidatePositionMode position_mode = CandidatePositionMode::WindowCentroid;
    bool has_current_frame_support = true;
    int dominant_range_bin = -1;
    int dominant_doppler_bin = -1;
    int unique_range_doppler_cells = 0;
    float average_snr = 0.0f;
    float average_doa_peak_confidence = 0.0f;
    std::vector<PointCloud> points;
};

enum class FollowControlState {
    NormalFollow,
    LimitedFollow,
    YawOnly,
    StopAndReacquire,
    ReacquireConfirming
};

struct LockedTarget {
    bool valid = false;
    // display_id is stable for the user/controller. matched_internal_uid is
    // the immutable identity used by data association.  Keeping them separate
    // makes an internal-track switch observable instead of silently relabeling
    // a new candidate with the old visible ID.
    int display_id = -1;
    std::uint64_t matched_internal_uid = 0;
    int matched_candidate_id = -1;
    bool has_measurement = false;
    float measured_x = 0.0f;
    float measured_y = 0.0f;
    float measured_range_m = 0.0f;
    float measured_angle_deg = 0.0f;
    float x = 0.0f;
    float y = 0.0f;
    float vx = 0.0f;
    float vy = 0.0f;
    float range_m = 0.0f;
    float angle_deg = 0.0f;
    int lost_frames = 0;
    int stable_frames = 0;
    int reacquire_hits = 0;
    float confidence = 0.0f;
    FollowControlState control_state = FollowControlState::NormalFollow;
    float control_speed_scale = 1.0f;
    bool control_allow_translation = true;
    bool control_allow_yaw = true;
};

// One-frame association audit record.  RadarGUI writes this together with the
// complete candidate/cluster snapshot into a separate per-test CSV.
struct AssociationDiagnostics {
    std::string source = "none";
    std::string reject_reason;
    bool measurement_rejected = false;
    float predicted_x = 0.0f;
    float predicted_y = 0.0f;
    float innovation_x = 0.0f;
    float innovation_y = 0.0f;
    float nis = 0.0f;
    float implied_speed_mps = 0.0f;
    int considered_candidates = 0;
    int gate_candidate_count = 0;
    int candidate_display_id = -1;
    std::uint64_t candidate_internal_uid = 0;
    int candidate_point_count = 0;
    float candidate_width_m = 0.0f;
    float candidate_depth_m = 0.0f;
    float candidate_average_abs_velocity = 0.0f;
    float candidate_moving_ratio = 0.0f;
    float candidate_score = 0.0f;
    float candidate_background_score = 0.0f;
    int candidate_current_frame_points = 0;
    float candidate_current_frame_ratio = 0.0f;
    int candidate_dominant_range_bin = -1;
    int candidate_dominant_doppler_bin = -1;
    int candidate_unique_range_doppler_cells = 0;
    float candidate_average_snr = 0.0f;
    float candidate_average_doa_peak_confidence = 0.0f;
    std::uint64_t pending_internal_uid = 0;
    int pending_hits = 0;
    int pending_window_frames = 0;
    int reacquire_hypothesis_count = 0;
    float reacquire_best_score = 0.0f;
    float reacquire_second_score = 0.0f;
    float reacquire_score_margin = 0.0f;
};

// Per-frame transform actually applied to the tracking state.  This is kept
// separate from DSP Doppler compensation: it explains why a world-stationary
// target can move backwards in the radar-relative coordinate display while
// the cart itself moves forwards.
struct EgoMotionDiagnostics {
    bool enabled = true;
    bool odom_valid = false;
    bool applied = false;
    float dt_s = 0.0f;
    float chassis_vx_mps = 0.0f;
    float chassis_wz_radps = 0.0f;
    float radar_dx_m = 0.0f;
    float radar_dy_m = 0.0f;
    float radar_dyaw_rad = 0.0f;
};

class TargetTracker {
public:
    explicit TargetTracker(const TrackingConfig& config = TrackingConfig());

    void reset();
    void setConfig(const TrackingConfig& config);
    const TrackingConfig& config() const { return config_; }
    void setEgoMotionCompensationEnabled(bool enabled) { ego_motion_compensation_enabled_ = enabled; }

    // Build current candidates from a sparse multi-frame support window.  The
    // newest frame number is passed separately so an empty newest frame cannot
    // make old points look fresh. Candidate ids remain stable across frames.
    const std::vector<CandidateTarget>& updateCandidates(
        const std::vector<PointCloud>& points,
        const ChassisVelocitySample* synchronized_chassis = nullptr,
        int current_frame_number = -1);

    // Manual selection for GUI list selection.
    bool selectTargetById(int candidate_id);
    bool selectTargetByUid(std::uint64_t internal_uid);

    // Manual selection for GUI click selection. x/y are radar coordinates in meters.
    bool selectTargetNear(float x, float y, float max_distance_m = 0.8f);

    // Explicitly release the current target. A new manual selection is required
    // before coordinates can be exported again.
    void unlockTarget(const std::string& reason = "manual_unlock");

    // Automatic selection for GUI combo-box rules.
    bool autoSelect(AutoSelectRule rule);

    // Static-radar background modeling. Call beginBackgroundModeling() only
    // when the cart/radar is stationary and the scene is expected to be empty.
    void beginBackgroundModeling();
    void stopBackgroundModeling();
    void clearBackgroundModel();
    void setBackgroundFilterEnabled(bool enabled);
    bool backgroundModelingActive() const { return background_modeling_active_; }
    bool backgroundFilterEnabled() const { return background_filter_enabled_; }
    int backgroundModelFrameCount() const { return background_model_frame_count_; }
    int backgroundModelTrackCount() const;

    // Keep following the selected target using a dynamic ROI around the last
    // locked position. Call this once per frame after updateCandidates().
    LockedTarget updateLockedTarget();

    const std::vector<CandidateTarget>& candidates() const { return candidates_; }
    const std::vector<ClusterDebugInfo>& clusterDebugInfo() const { return cluster_debug_; }
    const LockedTarget& lockedTarget() const { return locked_; }
    const AssociationDiagnostics& associationDiagnostics() const { return association_diagnostics_; }
    const TrajectoryAssociation::Diagnostics& trajectoryAssociationDiagnostics() const {
        return trajectory_association_diagnostics_;
    }
    const EgoMotionDiagnostics& egoMotionDiagnostics() const {
        return ego_motion_diagnostics_;
    }
    bool manualLockConfirmationPending() const {
        return manual_lock_confirmation_pending_;
    }
    int manualLockConfirmationMisses() const {
        return manual_lock_confirmation_misses_;
    }
    bool lockedUidContinuityCompromised() const {
        return locked_uid_continuity_compromised_;
    }
    TrackIdMode trackIdMode() const { return config_.track_id_mode; }
    std::string trackIdModeName() const;
    std::uint64_t totalTrackBirths() const { return total_track_births_; }
    std::uint64_t confirmedTrackBirths() const { return confirmed_track_births_; }
    int maxDisplayIdSeen() const { return max_display_id_seen_; }
    int activeTrackCount() const { return static_cast<int>(candidate_tracks_.size()); }
    std::string candidateSummary() const;

private:
#ifdef TARGET_TRACKING_ENABLE_TEST_ACCESS
    friend struct TargetTrackerTestAccess;
#endif

    TrackingConfig config_;
    std::vector<CandidateTarget> candidates_;
    // Includes tentative id=-1 tracks for the locked-target association path.
    // They stay hidden from the manual selection list, but can accumulate the
    // required multi-frame evidence without an extra display-ID delay.
    std::vector<CandidateTarget> association_candidates_;
    std::vector<CandidateTarget> previous_candidates_;
    std::vector<ClusterDebugInfo> cluster_debug_;
    LockedTarget locked_;
    int next_candidate_id_ = 1;
    std::uint64_t next_internal_uid_ = 1;
    std::uint64_t total_track_births_ = 0;
    std::uint64_t confirmed_track_births_ = 0;
    int max_display_id_seen_ = 0;
    std::vector<int> free_display_ids_;
    bool kalman_initialized_ = false;
    float kalman_x_ = 0.0f;
    float kalman_y_ = 0.0f;
    float kalman_vx_ = 0.0f;
    float kalman_vy_ = 0.0f;
    float kalman_p_[4][4] = {};
    AssociationDiagnostics association_diagnostics_;
    TrajectoryAssociation::GlobalAssociator trajectory_associator_;
    TrajectoryAssociation::Diagnostics trajectory_association_diagnostics_;
    EgoMotionDiagnostics ego_motion_diagnostics_;

    struct ReacquireHypothesis {
        bool active = true;
        std::uint64_t internal_uid = 0;
        int candidate_display_id = -1;
        // Hit count in the current confirmation window. For a new UID this is
        // deliberately not required to be strictly consecutive.
        int consecutive_hits = 0;
        int window_frames = 0;
        int frames_since_last_hit = 0;
        bool allow_one_frame_gap = false;
        CandidateTarget latest;
        float vx = 0.0f;
        float vy = 0.0f;
        float nis_sum = 0.0f;
        float cost_sum = 0.0f;
        float last_cost = 0.0f;
        bool original_uid = false;
    };
    std::vector<ReacquireHypothesis> reacquire_hypotheses_;

    bool has_last_accepted_candidate_ = false;
    CandidateTarget last_accepted_candidate_;
    int frames_since_accepted_measurement_ = 0;
    bool candidate_uid_assignment_ambiguous_ = false;
    bool locked_uid_continuity_compromised_ = false;
    bool manual_lock_confirmation_pending_ = false;
    int manual_lock_confirmation_misses_ = 0;

    struct CandidateTrack {
        int id = -1;
        std::uint64_t internal_uid = 0;
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
        float median_signed_velocity = 0.0f;
        float moving_ratio = 0.0f;
        // Once an internal identity has represented the manually locked
        // target, keep its history for the full locked-track timeout even if
        // the controller releases the visible lock earlier.
        bool was_locked = false;
        int hits = 0;
        int lost_frames = 0;
        float confidence = 0.0f;
    };
    std::vector<CandidateTrack> candidate_tracks_;

    struct BackgroundTrack {
        float x = 0.0f;
        float y = 0.0f;
        int stable_frames = 0;
        int missed_frames = 0;
    };
    std::vector<BackgroundTrack> background_tracks_;
    bool background_modeling_active_ = false;
    bool background_filter_enabled_ = false;
    int background_model_frame_count_ = 0;
    double last_chassis_timestamp_sec_ = 0.0;
    float current_frame_dt_s_ = 0.1f;
    bool ego_motion_compensation_enabled_ = true;
    ChassisVelocitySample current_chassis_sample_;

    std::vector<CandidateTarget> buildCandidates(const std::vector<PointCloud>& points,
                                                 int current_frame_number);
    bool isPersonLike(const CandidateTarget& candidate) const;
    bool isNearFieldBurstCluster(const CandidateTarget& candidate) const;
    void associateCandidateTracks();
    int allocateDisplayId();
    void releaseDisplayId(int id);
    void compensatePreviousStateForEgoMotion(
        const ChassisVelocitySample& chassis);
    CandidateTrack* findCandidateTrackById(int track_id);
    CandidateTrack* findCandidateTrackByUid(std::uint64_t internal_uid);
    float candidateTrackGateRadius(const CandidateTrack& track, const CandidateTarget& candidate) const;
    const CandidateTarget* findCandidateById(int candidate_id) const;
    const CandidateTarget* findCandidateByUid(std::uint64_t internal_uid) const;

    struct MatchResult {
        CandidateTarget* candidate = nullptr;
        std::string source;
        std::string reject_reason;
        bool rejected = false;
        float innovation_x = 0.0f;
        float innovation_y = 0.0f;
        float nis = 0.0f;
        float implied_speed_mps = 0.0f;
        float cost = 0.0f;
        int considered_candidates = 0;
        int gate_candidate_count = 0;
    };

    MatchResult findBestCandidateWithKalmanGate(float predicted_x, float predicted_y);
    MatchResult findBestCandidateInRoi(float predicted_x, float predicted_y);
    bool isAcceptedMatchConsistent(const MatchResult& match,
                                   float predicted_x,
                                   float predicted_y) const;
    bool evaluateCandidateMeasurement(const CandidateTarget& candidate,
                                      float predicted_x,
                                      float predicted_y,
                                      bool roi_fallback,
                                      float& innovation_x,
                                      float& innovation_y,
                                      float& nis,
                                      float& implied_speed_mps,
                                      std::string& reject_reason) const;
    bool isSuspiciousContinuityMatch(const CandidateTarget& candidate,
                                     const MatchResult& match) const;
    bool passesNewUidReacquireChecks(const CandidateTarget& candidate,
                                     const MatchResult& match,
                                     float predicted_x,
                                     float predicted_y,
                                     std::string& reject_reason) const;
    bool isReacquireSequenceContinuous(
        const CandidateTarget& candidate,
        const ReacquireHypothesis& hypothesis) const;
    std::vector<MatchResult> collectReacquireMatches(
        float predicted_x, float predicted_y);
    void updateReacquireHypotheses(
        const std::vector<MatchResult>& matches,
        float predicted_x, float predicted_y,
        const std::string& reject_reason);
    int reacquireRequiredHits(const ReacquireHypothesis& hypothesis) const;
    int reacquireWindowLimit(const ReacquireHypothesis& hypothesis) const;
    float reacquireHypothesisScore(const ReacquireHypothesis& hypothesis) const;
    void pruneAndLimitReacquireHypotheses();
    void fillBestReacquireDiagnostics();
    void fillAssociationDiagnostics(const MatchResult& match,
                                    const CandidateTarget* candidate);
    void handleTrackingMiss(float predicted_x, float predicted_y,
                            const std::string& reject_reason);
    void beginOrAdvanceReacquire(const CandidateTarget& candidate,
                                 const MatchResult& match,
                                 float predicted_x,
                                 float predicted_y);
    void commitReacquireHypothesis(const ReacquireHypothesis& hypothesis,
                                   const MatchResult& match);
    void acceptMeasurement(const CandidateTarget& candidate,
                           const MatchResult& match,
                           bool reset_lost_state);
    void clearReacquireHypothesis();
    void applyMissingMeasurementVelocityPolicy();
    bool isStaticBackgroundLike(const CandidateTarget& candidate) const;
    bool isBackgroundMotionTolerant(const CandidateTarget& candidate) const;
    bool isLockedRoiCompanion(const CandidateTarget& candidate) const;
    void updateBackgroundModel();
    float backgroundScoreForCandidate(const CandidateTarget& candidate) const;
    void applyBackgroundScores();
    void suppressLockedRoiCompanions();
    void beginManualLockConfirmation(const CandidateTarget& candidate);
    void lockFromCandidate(const CandidateTarget& candidate, bool reset_velocity);
    void initKalmanFromLocked();
    void predictKalman(float& predicted_x, float& predicted_y);
    void updateKalman(float measured_x, float measured_y);
};

} // namespace TargetTracking

#endif // TARGET_TRACKING_HPP
