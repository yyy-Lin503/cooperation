#include "target_tracking.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
namespace TargetTracking {
struct TargetTrackerTestAccess {
    static std::vector<CandidateTarget> build(TargetTracker& t,const std::vector<PointCloud>& p,int f) { return t.buildCandidates(p,f); }
    static void seed(TargetTracker& t, bool new_uid) {
        t.locked_=LockedTarget(); t.locked_.valid=true; t.locked_.display_id=1;
        t.locked_.matched_internal_uid=1; t.locked_.x=0; t.locked_.y=3;
        t.locked_.lost_frames=1; t.locked_.confidence=1;
        t.initKalmanFromLocked();
        CandidateTarget c; c.id=1; c.internal_uid=new_uid?2:1; c.x=.1f; c.y=3;
        c.range_m=std::hypot(c.x,c.y); c.point_count=6; c.current_frame_point_count=3;
        c.width_m=.2f; c.depth_m=.2f; c.has_current_frame_support=true;
        t.has_last_accepted_candidate_=true; t.last_accepted_candidate_=c;
        t.association_candidates_={c}; t.candidates_={c};
    }
    static bool gate(TargetTracker& t,float x) {
        CandidateTarget c;c.x=x;c.y=3;c.point_count=6;c.width_m=.2f;c.depth_m=.2f;
        t.locked_.lost_frames=20;t.manual_lock_confirmation_pending_=false;
        t.has_last_accepted_candidate_=false;
        t.kalman_p_[0][0]=2;t.kalman_p_[1][1]=2;
        float ex,ey,nis,speed;std::string reason;
        return t.evaluateCandidateMeasurement(c,0,3,true,ex,ey,nis,speed,reason);
    }
    static void miss(TargetTracker& t) {t.handleTrackingMiss(t.locked_.x,t.locked_.y,"test_pending_gap");}
    static void placeLost(TargetTracker& t, float x, float y, std::uint64_t uid, int lost) {
        t.locked_=LockedTarget(); t.locked_.valid=true; t.locked_.display_id=1;
        t.locked_.matched_internal_uid=1; t.locked_.x=0; t.locked_.y=3; t.locked_.lost_frames=lost;
        t.initKalmanFromLocked();
        t.occlusion_coast_active_=true; t.occlusion_has_direction_=true;
        t.occlusion_anchor_x_=0; t.occlusion_anchor_y_=3; t.occlusion_dir_x_=0; t.occlusion_dir_y_=1;
        CandidateTarget c; c.id=1; c.internal_uid=uid; c.x=x; c.y=y;
        c.range_m=std::hypot(x,y); c.point_count=6; c.current_frame_point_count=3;
        c.width_m=.2f; c.depth_m=.2f; c.has_current_frame_support=true;
        CandidateTarget last; last.internal_uid=1; last.x=0; last.y=3; last.point_count=6;
        last.width_m=.2f; last.depth_m=.2f;
        t.has_last_accepted_candidate_=true; t.last_accepted_candidate_=last;
        t.association_candidates_={c}; t.candidates_={c};
    }
};
}
static void check(bool condition,const char* message) { if(!condition) throw std::runtime_error(message); }
int main() {
    using namespace TargetTracking;
    std::vector<PointCloud> p;
    for(float x:{-1.2f,-1.0f,-.6f,-.2f,.6f,1.0f,1.2f}) {
        PointCloud q;q.x=x;q.y=3;q.source_frame=1;p.push_back(q);
    }
    for(float x:{0.0f,.1f}) {PointCloud q;q.x=x;q.y=3;q.source_frame=3;p.push_back(q);}
    TrackingConfig old;old.use_current_frame_extent=false;TargetTracker old_tracker(old);
    check(TargetTrackerTestAccess::build(old_tracker,p,3).empty(),"Old multi-frame width did not reproduce rejection");
    TargetTracker fresh;auto result=TargetTrackerTestAccess::build(fresh,p,3);
    check(result.size()==1 && std::abs(result[0].width_m-.1f)<1e-4f,"Current-frame extent does not match current centroid");
    check(TargetTrackerTestAccess::build(fresh,p,4).empty(),"Old-only cluster became a fresh measurement");
    TargetTracker continuity;TargetTrackerTestAccess::seed(continuity,false);
    continuity.updateLockedTarget();
    check(continuity.lockedTarget().has_measurement && continuity.associationDiagnostics().source=="short_gap_same_uid",
          "Short same-UID gap still resets the reacquisition model");
    TargetTracker changed;TargetTrackerTestAccess::seed(changed,true);changed.updateLockedTarget();
    check(!changed.lockedTarget().has_measurement && changed.lockedTarget().matched_internal_uid==1,
          "Nearby other person took the locked identity");
    check(!changed.lockedTarget().control_allow_translation && !changed.lockedTarget().control_allow_yaw &&
          changed.associationDiagnostics().source=="reacquire_pending",
          "Unconfirmed new UID granted motion");
    TargetTracker side;TargetTrackerTestAccess::placeLost(side,1.2f,3.0f,1,5);side.updateLockedTarget();
    check(!side.lockedTarget().has_measurement,"Same UID bypassed speed/statistical gate after a jump");
    TargetTracker ahead;TargetTrackerTestAccess::placeLost(ahead,0.0f,4.4f,1,5);ahead.updateLockedTarget();
    check(!ahead.lockedTarget().has_measurement,"Forward jump bypassed measurement gate");
    TargetTracker stopped;TargetTrackerTestAccess::placeLost(stopped,0.2f,3.1f,1,5);stopped.updateLockedTarget();
    check(!stopped.lockedTarget().has_measurement&&!stopped.lockedTarget().control_allow_translation,"Same UID skipped recovery confirmation");
    stopped.updateLockedTarget();check(stopped.lockedTarget().has_measurement,"Two-hit same UID recovery failed");
    TargetTracker far_person;TargetTrackerTestAccess::placeLost(far_person,0.0f,6.0f,1,5);far_person.updateLockedTarget();
    check(!far_person.lockedTarget().has_measurement,"Out-of-gate person accepted");
    TargetTracker other;TargetTrackerTestAccess::placeLost(other,1.2f,3.0f,2,5);other.updateLockedTarget();
    check(!other.lockedTarget().has_measurement&&other.lockedTarget().matched_internal_uid==1&&!other.lockedTarget().control_allow_translation,"Other UID accepted without confirmation");
    TargetTracker strict;check(!TargetTrackerTestAccess::gate(strict,2.19f),"Default unexpectedly widened the ROI");
    TrackingConfig expanded;expanded.enable_dropout_gate_expansion=true;expanded.dropout_gate_max_extra_m=.2f;
    TargetTracker limited(expanded);
    check(TargetTrackerTestAccess::gate(limited,2.19f) && !TargetTrackerTestAccess::gate(limited,2.3f),"Experimental radius did not remain bounded");
    std::cout << "PASS: fresh extent, old-only suppression, same-UID short gap, occlusion coast, original-UID region resume, other-UID refusal, coast stop, bounded ROI\n";
}
