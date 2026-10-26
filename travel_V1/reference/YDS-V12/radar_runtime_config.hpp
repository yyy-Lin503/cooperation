#ifndef RADAR_RUNTIME_CONFIG_HPP
#define RADAR_RUNTIME_CONFIG_HPP

// Central compile-time switches.
// Edit only the values below, rebuild radar_gui, then launch with: ./radar_gui
namespace RadarRuntimeConfig {

// Candidate ID policy:
//   0 = monotonic  : assign immediately, never reuse
//   1 = confirmed  : expose an ID only after 3 consecutive matches
//   2 = recycled   : reuse small display IDs; internal UID remains unique
inline constexpr int kTrackIdMode = 1;

// Three frames provide sparse DBSCAN support and GUI persistence. Tracking
// position is nevertheless current-frame-first; do not set this to 1 merely
// to obtain a current position, because that also removes clustering support.
// Valid range: 1 to 10.
inline constexpr int kPointWindowFrames = 3;

// true  = global Hungarian trajectory-to-cluster association
// false = legacy greedy candidate association (one-line rollback switch)
inline constexpr bool kUseHungarianTrajectoryAssociation = true;

// Minimum physical range that is allowed to enter DOA, visualization,
// clustering and tracking.  Targets at exactly 1.8 m remain valid; only
// ranges strictly below this value are removed.
inline constexpr float kMinimumUsableRangeM = 1.8f;

// Fail-safe for the near-field burst observed during the 2026-07-13 vehicle
// test.  A normal sparse 3-4 point target is never affected.  Only when one
// radar frame contains many fast points in the 1.8-2.2 m band is that band
// quarantined for the frame instead of being allowed to create a false track.
inline constexpr bool kEnableNearFieldBurstGuard = true;
inline constexpr float kNearFieldBurstMaxRangeM = 2.2f;
inline constexpr float kNearFieldBurstMinAbsVelocityMps = 3.0f;
inline constexpr int kNearFieldBurstMinFastPointsPerFrame = 24;

// false = concise operational/statistics logs
// true  = per-stage, per-target and offline cluster debug logs
inline constexpr bool kDebugLogging = false;

} // namespace RadarRuntimeConfig

#endif // RADAR_RUNTIME_CONFIG_HPP
