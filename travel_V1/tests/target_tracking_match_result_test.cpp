#include "target_tracking.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

namespace TargetTracking {

struct TargetTrackerTestAccess {
    static void manualPendingHold(TargetTracker& tracker)
    {
        CandidateTarget candidate;
        candidate.id = 1; candidate.internal_uid = 1;
        candidate.x = .2f; candidate.y = 3.0f;
        candidate.range_m = 3.00666f; candidate.score = .9f;
        candidate.point_count = 4;
        candidate.current_frame_point_count = 4;
        tracker.beginManualLockConfirmation(candidate);
        const auto& selected = tracker.lockedTarget();
        if (selected.has_measurement || selected.control_speed_scale != 0.0f ||
            selected.control_allow_translation || selected.control_allow_yaw)
            throw std::runtime_error("Manual selection grants motion before confirmation");
        tracker.handleTrackingMiss(selected.x, selected.y, "manual_pending_test");
        const auto& missed = tracker.lockedTarget();
        if (missed.control_speed_scale != 0.0f || missed.control_allow_translation || missed.control_allow_yaw)
            throw std::runtime_error("Manual confirmation miss grants predicted motion");
        TargetTracker::MatchResult match;
        match.candidate = &candidate;
        match.nis = 1.0f;
        match.implied_speed_mps = 0.0f;
        tracker.beginOrAdvanceReacquire(candidate, match, candidate.x, candidate.y);
        if (tracker.manualLockConfirmationPending())
            tracker.beginOrAdvanceReacquire(candidate, match, candidate.x, candidate.y);
        if (tracker.manualLockConfirmationPending() || !tracker.lockedTarget().has_measurement ||
            !tracker.lockedTarget().control_allow_translation)
            throw std::runtime_error("Confirmed manual target did not release motion hold");
    }
    static void prepare(TargetTracker& tracker,
                        const std::vector<CandidateTarget>& candidates)
    {
        tracker.association_candidates_ = candidates;
        tracker.locked_.valid = true;
        tracker.locked_.matched_internal_uid = 1;
        tracker.kalman_p_[0][0] = 0.50f;
        tracker.kalman_p_[0][1] = 0.0f;
        tracker.kalman_p_[1][0] = 0.0f;
        tracker.kalman_p_[1][1] = 0.50f;
        tracker.has_last_accepted_candidate_ = false;
    }

    static TargetTracker::MatchResult kalman(TargetTracker& tracker)
    {
        return tracker.findBestCandidateWithKalmanGate(0.0f, 0.0f);
    }

    static TargetTracker::MatchResult roi(TargetTracker& tracker)
    {
        return tracker.findBestCandidateInRoi(0.0f, 0.0f);
    }

    static bool consistent(const TargetTracker& tracker,
                           const TargetTracker::MatchResult& result)
    {
        return tracker.isAcceptedMatchConsistent(result, 0.0f, 0.0f);
    }

    static std::vector<CandidateTarget> build(
        TargetTracker& tracker,
        const std::vector<PointCloud>& points,
        int current_frame_number)
    {
        return tracker.buildCandidates(points, current_frame_number);
    }

    static void seedNewUidReacquire(TargetTracker& tracker,
                                    const CandidateTarget& candidate)
    {
        tracker.locked_.valid = true;
        tracker.locked_.display_id = 1;
        tracker.locked_.matched_internal_uid = 1;
        tracker.reacquire_hypotheses_.clear();
        TargetTracker::ReacquireHypothesis hypothesis;
        hypothesis.internal_uid = candidate.internal_uid;
        hypothesis.candidate_display_id = candidate.id;
        hypothesis.consecutive_hits = 1;
        hypothesis.window_frames = 1;
        hypothesis.allow_one_frame_gap = true;
        hypothesis.latest = candidate;
        hypothesis.original_uid =
            candidate.internal_uid == tracker.locked_.matched_internal_uid;
        hypothesis.nis_sum = 1.0f;
        hypothesis.cost_sum = 1.0f;
        hypothesis.last_cost = 1.0f;
        tracker.reacquire_hypotheses_.push_back(hypothesis);
        tracker.current_frame_dt_s_ = 0.1f;
    }

    static void miss(TargetTracker& tracker)
    {
        tracker.handleTrackingMiss(0.0f, 0.0f, "test_gap");
    }

    static void seedLossPolicy(TargetTracker& tracker)
    {
        tracker.locked_ = LockedTarget();
        tracker.locked_.valid = true;
        tracker.locked_.display_id = 1;
        tracker.locked_.matched_internal_uid = 1;
        tracker.locked_.confidence = 1.0f;
    }

    static void reacquireHit(TargetTracker& tracker,
                             CandidateTarget& candidate)
    {
        TargetTracker::MatchResult match;
        match.candidate = &candidate;
        match.source = "test";
        match.nis = 1.0f;
        match.implied_speed_mps = 1.0f;
        match.gate_candidate_count = 1;
        tracker.beginOrAdvanceReacquire(candidate, match, 0.0f, 0.0f);
    }

    static bool pending(const TargetTracker& tracker)
    {
        return !tracker.reacquire_hypotheses_.empty();
    }

    static int pendingHits(const TargetTracker& tracker)
    {
        return tracker.reacquire_hypotheses_.empty()
            ? 0 : tracker.reacquire_hypotheses_.front().consecutive_hits;
    }

    static int pendingWindowFrames(const TargetTracker& tracker)
    {
        return tracker.reacquire_hypotheses_.empty()
            ? 0 : tracker.reacquire_hypotheses_.front().window_frames;
    }

    static std::size_t hypothesisCount(const TargetTracker& tracker)
    {
        return tracker.reacquire_hypotheses_.size();
    }

    static void seedMultiHypothesis(TargetTracker& tracker)
    {
        tracker.locked_ = LockedTarget();
        tracker.locked_.valid = true;
        tracker.locked_.display_id = 1;
        tracker.locked_.matched_internal_uid = 1;
        tracker.locked_.confidence = 1.0f;
        tracker.current_frame_dt_s_ = 0.1f;
        tracker.kalman_initialized_ = true;
        tracker.kalman_p_[0][0] = 0.5f;
        tracker.kalman_p_[1][1] = 0.5f;
    }

    static void multiHypothesisFrame(
        TargetTracker& tracker,
        std::vector<CandidateTarget>& candidates,
        const std::vector<float>& costs)
    {
        std::vector<TargetTracker::MatchResult> matches;
        for (std::size_t i = 0; i < candidates.size(); ++i) {
            TargetTracker::MatchResult match;
            match.candidate = &candidates[i];
            match.source = "test_multi";
            match.nis = 1.0f;
            match.implied_speed_mps = 1.0f;
            match.cost = costs[i];
            match.gate_candidate_count = static_cast<int>(candidates.size());
            matches.push_back(match);
        }
        tracker.updateReacquireHypotheses(matches, 0.0f, 0.0f, "");
    }

    static void seedPredictionDecay(TargetTracker& tracker)
    {
        tracker.locked_ = LockedTarget();
        tracker.locked_.valid = true;
        tracker.locked_.display_id = 1;
        tracker.locked_.matched_internal_uid = 1;
        tracker.locked_.confidence = 1.0f;
        tracker.kalman_initialized_ = true;
        tracker.kalman_x_ = 0.0f;
        tracker.kalman_y_ = 0.0f;
        tracker.kalman_vx_ = 2.5f;
        tracker.kalman_vy_ = 0.0f;
        tracker.current_frame_dt_s_ = 0.1f;
    }

    static void predictionMiss(TargetTracker& tracker)
    {
        float x = 0.0f;
        float y = 0.0f;
        tracker.predictKalman(x, y);
        tracker.handleTrackingMiss(x, y, "test_prediction_miss");
    }

    static float kalmanSpeed(const TargetTracker& tracker)
    {
        return std::hypot(tracker.kalman_vx_, tracker.kalman_vy_);
    }

    static void seedLifecycleTrack(TargetTracker& tracker,
                                   std::uint64_t uid,
                                   int display_id,
                                   bool locked)
    {
        TargetTracker::CandidateTrack track;
        track.internal_uid = uid;
        track.id = display_id;
        track.hits = display_id < 0
            ? 1 : std::max(1, tracker.config_.track_confirm_hits);
        tracker.candidate_tracks_.push_back(track);
        tracker.locked_.valid = locked;
        tracker.locked_.matched_internal_uid = locked ? uid : 0;
    }

    static void missCandidateTracks(TargetTracker& tracker)
    {
        tracker.candidates_.clear();
        tracker.associateCandidateTracks();
    }

    static std::size_t activeTracks(const TargetTracker& tracker)
    {
        return tracker.candidate_tracks_.size();
    }

    static void tryOldOnlyBirth(TargetTracker& tracker)
    {
        CandidateTarget candidate;
        candidate.point_count = 3;
        candidate.has_current_frame_support = false;
        candidate.current_frame_point_count = 0;
        tracker.candidates_ = {candidate};
        tracker.associateCandidateTracks();
    }
};

} // namespace TargetTracking

namespace {

using TargetTracking::CandidateTarget;
using TargetTracking::CandidatePositionMode;
using TargetTracking::TargetTracker;
using TargetTracking::TargetTrackerTestAccess;

CandidateTarget makeCandidate(std::uint64_t uid, float x, float y)
{
    CandidateTarget candidate;
    candidate.id = static_cast<int>(uid);
    candidate.internal_uid = uid;
    candidate.x = x;
    candidate.y = y;
    candidate.point_count = 8;
    candidate.width_m = 0.5f;
    candidate.depth_m = 0.4f;
    candidate.score = 0.8f;
    candidate.moving_ratio = 0.7f;
    candidate.average_abs_velocity = 0.8f;
    return candidate;
}

PointCloud makePoint(int frame, float x, float y = 5.0f)
{
    PointCloud point;
    point.source_frame = frame;
    point.x = x;
    point.y = y;
    point.absolute_velocity = 0.8f;
    point.relative_velocity = 0.8f;
    point.is_moving = true;
    point.snr = 15.0f;
    point.doa_peak_confidence = 0.8f;
    return point;
}

void require(bool condition, const std::string& message)
{
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}

template <typename MatchResult>
void verifyAccepted(const TargetTracker& tracker,
                    const MatchResult& result,
                    const std::string& label)
{
    require(result.candidate != nullptr, label + ": no candidate returned");
    require(result.candidate->internal_uid == 1,
            label + ": distractor was selected");
    require(std::abs(result.innovation_x - 0.08f) < 1e-5f,
            label + ": innovation_x belongs to another candidate");
    require(std::abs(result.innovation_y - 0.04f) < 1e-5f,
            label + ": innovation_y belongs to another candidate");
    require(std::isfinite(result.nis), label + ": NIS is not finite");
    require(!result.rejected, label + ": accepted result is marked rejected");
    require(result.reject_reason.empty(), label + ": accepted result has reject reason");
    require(TargetTrackerTestAccess::consistent(tracker, result),
            label + ": runtime consistency guard failed");
}

void runOrderCase(bool distractor_first)
{
    TargetTracker tracker;
    const CandidateTarget correct = makeCandidate(1, 0.08f, 0.04f);
    const CandidateTarget distractor = makeCandidate(9, 6.00f, -3.00f);
    const std::vector<CandidateTarget> candidates = distractor_first
        ? std::vector<CandidateTarget>{distractor, correct}
        : std::vector<CandidateTarget>{correct, distractor};
    TargetTrackerTestAccess::prepare(tracker, candidates);

    verifyAccepted(tracker, TargetTrackerTestAccess::kalman(tracker),
                   distractor_first ? "kalman far-first" : "kalman far-last");
    verifyAccepted(tracker, TargetTrackerTestAccess::roi(tracker),
                   distractor_first ? "roi far-first" : "roi far-last");
}

void runOriginalUidTwoHitsInFourFramesCase()
{
    TargetTracker tracker;
    CandidateTarget candidate = makeCandidate(1, 0.0f, 0.0f);
    TargetTrackerTestAccess::seedNewUidReacquire(tracker, candidate);

    // Original UID: H, M, H commits inside a four-frame window.
    TargetTrackerTestAccess::miss(tracker);
    require(TargetTrackerTestAccess::pending(tracker),
            "original 2-in-4: one gap cleared the hypothesis");
    candidate.x = 0.10f;
    TargetTrackerTestAccess::reacquireHit(tracker, candidate);
    require(!TargetTrackerTestAccess::pending(tracker),
            "original 2-in-4: confirmed hypothesis was not cleared");
    require(tracker.lockedTarget().matched_internal_uid == 1,
            "original 2-in-4: original identity was not restored");
}

void runNewUidThreeHitsInFiveFramesCase()
{
    TargetTracker tracker;
    CandidateTarget candidate = makeCandidate(2, 0.0f, 0.0f);
    TargetTrackerTestAccess::seedNewUidReacquire(tracker, candidate);

    // New UID: H, M, H, M, H is accepted at frame five.
    TargetTrackerTestAccess::miss(tracker);
    require(TargetTrackerTestAccess::pending(tracker),
            "new 3-in-5: one missed frame cleared the hypothesis");

    candidate.x = 0.05f;
    TargetTrackerTestAccess::reacquireHit(tracker, candidate);
    require(TargetTrackerTestAccess::pending(tracker) &&
                TargetTrackerTestAccess::pendingHits(tracker) == 2 &&
                TargetTrackerTestAccess::pendingWindowFrames(tracker) == 3,
            "new 3-in-5: second hit counters are wrong");
    require(!tracker.lockedTarget().has_measurement,
            "new 3-in-5: pending candidate was exposed as a KF measurement");
    require(tracker.lockedTarget().control_state ==
                TargetTracking::FollowControlState::ReacquireConfirming,
            "new 3-in-5: pending candidate did not export PREDICTED control");
    require(tracker.lockedTarget().matched_internal_uid == 1,
            "new 3-in-5: main identity changed before confirmation");

    TargetTrackerTestAccess::miss(tracker);
    candidate.x = 0.10f;
    TargetTrackerTestAccess::reacquireHit(tracker, candidate);
    require(!TargetTrackerTestAccess::pending(tracker),
            "new 3-in-5: confirmed hypothesis was not cleared");
    require(tracker.lockedTarget().matched_internal_uid == 2,
            "new 3-in-5: third hit did not commit the new UID");

    TargetTracker impossible;
    candidate = makeCandidate(2, 0.0f, 0.0f);
    TargetTrackerTestAccess::seedNewUidReacquire(impossible, candidate);
    TargetTrackerTestAccess::miss(impossible);
    TargetTrackerTestAccess::miss(impossible);
    TargetTrackerTestAccess::miss(impossible);
    require(!TargetTrackerTestAccess::pending(impossible),
            "new 3-in-5: mathematically impossible hypothesis was retained");
    require(impossible.lockedTarget().matched_internal_uid == 1,
            "new 3-in-5: expired hypothesis changed the main identity");
}

void runBoundedMultiHypothesisCase()
{
    TargetTracker tracker;
    TargetTrackerTestAccess::seedMultiHypothesis(tracker);
    std::vector<CandidateTarget> candidates = {
        makeCandidate(2, 0.10f, 0.00f),
        makeCandidate(3, 0.15f, 0.00f)};

    // Both new identities receive three hits with nearly identical scores.
    // Neither may be committed while the evidence remains ambiguous.
    for (int frame = 0; frame < 3; ++frame) {
        TargetTrackerTestAccess::multiHypothesisFrame(
            tracker, candidates, {1.0f, 1.1f});
    }
    require(tracker.lockedTarget().matched_internal_uid == 1,
            "multi-hypothesis: close A/B scores changed the main UID");
    require(TargetTrackerTestAccess::hypothesisCount(tracker) == 2,
            "multi-hypothesis: both plausible UIDs were not retained");
    require(!tracker.lockedTarget().has_measurement,
            "multi-hypothesis: pending cluster was exposed as a measurement");

    // A fourth hit for A while B misses makes the score margin decisive.
    candidates.resize(1);
    TargetTrackerTestAccess::multiHypothesisFrame(tracker, candidates, {1.0f});
    require(tracker.lockedTarget().matched_internal_uid == 2,
            "multi-hypothesis: clear winner was not committed");
    require(TargetTrackerTestAccess::hypothesisCount(tracker) == 0,
            "multi-hypothesis: pool was not cleared after commit");

    TargetTracker bounded;
    TargetTrackerTestAccess::seedMultiHypothesis(bounded);
    std::vector<CandidateTarget> four = {
        makeCandidate(2, 0.10f, 0.00f),
        makeCandidate(3, 0.20f, 0.00f),
        makeCandidate(4, 0.30f, 0.00f),
        makeCandidate(5, 0.40f, 0.00f)};
    TargetTrackerTestAccess::multiHypothesisFrame(
        bounded, four, {1.0f, 1.1f, 1.2f, 1.3f});
    require(TargetTrackerTestAccess::hypothesisCount(bounded) == 3,
            "multi-hypothesis: bounded pool did not enforce max=3");
}

void runPredictionVelocityDecayCase()
{
    TargetTracker tracker;
    TargetTrackerTestAccess::seedPredictionDecay(tracker);

    for (int frame = 1; frame <= 4; ++frame) {
        TargetTrackerTestAccess::predictionMiss(tracker);
    }
    require(std::abs(TargetTrackerTestAccess::kalmanSpeed(tracker) - 2.0f) < 1e-5f,
            "prediction decay: short dropout did not retain capped CV speed");

    TargetTrackerTestAccess::predictionMiss(tracker); // lost frame 5
    require(std::abs(TargetTrackerTestAccess::kalmanSpeed(tracker) - 1.5f) < 1e-5f,
            "prediction decay: damping did not start at lost frame 5");

    for (int frame = 6; frame <= 15; ++frame) {
        TargetTrackerTestAccess::predictionMiss(tracker);
    }
    require(TargetTrackerTestAccess::kalmanSpeed(tracker) < 1e-6f,
            "prediction decay: velocity was not zeroed at lost frame 15");
    require(tracker.lockedTarget().x < 1.8f,
            "prediction decay: search centre still drifted too far");
}

void runLossControlPolicyCase()
{
    TargetTracker tracker;
    TargetTrackerTestAccess::seedLossPolicy(tracker);
    require(tracker.config().lost_limited_follow_frames == 3,
            "loss policy: predicted interval is not 3 frames");
    require(tracker.config().manual_unlock_lost_frames == 50,
            "loss policy: manual unlock threshold is not 50 frames");

    for (int frame = 1; frame <= 3; ++frame) {
        TargetTrackerTestAccess::miss(tracker);
        require(tracker.lockedTarget().valid,
                "loss policy: lock cleared during predicted interval");
        require(tracker.lockedTarget().control_state ==
                    TargetTracking::FollowControlState::LimitedFollow,
                "loss policy: frames 1-3 are not PREDICTED");
    }

    TargetTrackerTestAccess::miss(tracker); // frame 4
    require(tracker.lockedTarget().valid &&
                tracker.lockedTarget().control_state ==
                    TargetTracking::FollowControlState::StopAndReacquire,
            "loss policy: frame 4 is not STOPPED");
    for (int frame = 5; frame <= 50; ++frame) {
        TargetTrackerTestAccess::miss(tracker);
    }
    require(tracker.lockedTarget().valid,
            "loss policy: core tracker cleared before completing frame 50");
    TargetTrackerTestAccess::miss(tracker); // core timeout is strictly > 50
    require(!tracker.lockedTarget().valid,
            "loss policy: core tracker survived beyond frame 50");
}

void runCurrentFramePreferredCentroidCases()
{
    TargetTracker current_centroid_tracker;
    const std::vector<PointCloud> two_current_points = {
        makePoint(1, 0.00f), makePoint(1, 0.10f),
        makePoint(2, 0.30f), makePoint(2, 0.40f),
        makePoint(3, 0.60f), makePoint(3, 0.70f)};
    const auto current_candidates = TargetTrackerTestAccess::build(
        current_centroid_tracker, two_current_points, 3);
    require(current_candidates.size() == 1,
            "current centroid: sparse 3-frame cluster was not retained");
    require(std::abs(current_candidates.front().x - 0.65f) < 1e-5f,
            "current centroid: old frames dragged the measurement backwards");
    require(current_candidates.front().position_mode ==
                CandidatePositionMode::CurrentFrameCentroid,
            "current centroid: wrong position mode");

    TargetTracker weighted_tracker;
    const std::vector<PointCloud> one_current_point = {
        makePoint(1, 0.00f), makePoint(1, 0.10f),
        makePoint(2, 0.30f), makePoint(2, 0.40f),
        makePoint(3, 0.60f)};
    const auto weighted_candidates = TargetTrackerTestAccess::build(
        weighted_tracker, one_current_point, 3);
    require(weighted_candidates.size() == 1,
            "weighted centroid: one current point lost the supported cluster");
    require(std::abs(weighted_candidates.front().x - 0.495f) < 1e-5f,
            "weighted centroid: 0.7/0.2/0.1 frame weights are wrong");
    require(weighted_candidates.front().position_mode ==
                CandidatePositionMode::TemporalWeightedCentroid,
            "weighted centroid: wrong position mode");

    TargetTracker no_current_tracker;
    const std::vector<PointCloud> no_current_points = {
        makePoint(1, 0.00f), makePoint(1, 0.10f),
        makePoint(2, 0.30f), makePoint(2, 0.40f)};
    const auto no_current_candidates = TargetTrackerTestAccess::build(
        no_current_tracker, no_current_points, 3);
    require(no_current_candidates.empty(),
            "no-current support: old points were repeated as a fresh candidate");
    require(!no_current_tracker.clusterDebugInfo().empty() &&
                no_current_tracker.clusterDebugInfo().front().position_mode ==
                    CandidatePositionMode::NoCurrentFrameSupport,
            "no-current support: diagnostic cluster/mode was not preserved");
}

void runCandidateTrackLifecycleCases()
{
    TargetTracker tentative;
    TargetTrackerTestAccess::seedLifecycleTrack(tentative, 10, -1, false);
    for (int i = 0; i < 4; ++i) {
        TargetTrackerTestAccess::missCandidateTracks(tentative);
    }
    require(TargetTrackerTestAccess::activeTracks(tentative) == 1,
            "lifecycle: tentative track expired before five misses");
    TargetTrackerTestAccess::missCandidateTracks(tentative);
    require(TargetTrackerTestAccess::activeTracks(tentative) == 0,
            "lifecycle: tentative track survived five misses");

    TargetTracker confirmed;
    TargetTrackerTestAccess::seedLifecycleTrack(confirmed, 20, 7, false);
    for (int i = 0; i < 11; ++i) {
        TargetTrackerTestAccess::missCandidateTracks(confirmed);
    }
    require(TargetTrackerTestAccess::activeTracks(confirmed) == 1,
            "lifecycle: confirmed track expired before twelve misses");
    TargetTrackerTestAccess::missCandidateTracks(confirmed);
    require(TargetTrackerTestAccess::activeTracks(confirmed) == 0,
            "lifecycle: confirmed track survived twelve misses");

    TargetTracker locked;
    TargetTrackerTestAccess::seedLifecycleTrack(locked, 30, 1, true);
    for (int i = 0; i < 30; ++i) {
        TargetTrackerTestAccess::missCandidateTracks(locked);
    }
    // Manual mode releases motion control at 30 missed measurements. The
    // immutable locked UID must nevertheless remain available through miss 50.
    locked.unlockTarget("test_manual_timeout");
    for (int i = 30; i < 50; ++i) {
        TargetTrackerTestAccess::missCandidateTracks(locked);
    }
    require(TargetTrackerTestAccess::activeTracks(locked) == 1,
            "lifecycle: locked track did not retain fifty misses");
    TargetTrackerTestAccess::missCandidateTracks(locked);
    require(TargetTrackerTestAccess::activeTracks(locked) == 0,
            "lifecycle: locked track survived beyond fifty misses");

    TargetTracker old_only;
    TargetTrackerTestAccess::tryOldOnlyBirth(old_only);
    require(TargetTrackerTestAccess::activeTracks(old_only) == 0,
            "lifecycle: old-only cluster created a fresh UID");
}

} // namespace

int main()
{
    TargetTracker manual_pending;
    TargetTrackerTestAccess::manualPendingHold(manual_pending);
    runOrderCase(false); // Reproduces the original correct-then-rejected bug.
    runOrderCase(true);  // Ensures result is independent of candidate order.
    runOriginalUidTwoHitsInFourFramesCase();
    runNewUidThreeHitsInFiveFramesCase();
    runBoundedMultiHypothesisCase();
    runPredictionVelocityDecayCase();
    runLossControlPolicyCase();
    runCurrentFramePreferredCentroidCases();
    runCandidateTrackLifecycleCases();
    std::cout << "PASS: association, bounded reacquire, loss policy, centroid, track lifecycle\n";
    return 0;
}
