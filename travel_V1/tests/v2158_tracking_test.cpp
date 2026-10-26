#include "target_tracking.hpp"
#include <stdexcept>
#include <iostream>
namespace TargetTracking {
struct TargetTrackerTestAccess {
 static void seed(TargetTracker&t,bool competitor=false){
  t.frame_context_.session_id=1;t.frame_context_.frame_number=10;t.frame_context_.sample_time_sec=100;
  t.selection_event_.selection_epoch=5;t.last_measurement_time_sec_=100;
  t.locked_.valid=true;t.locked_.display_id=1;t.locked_.matched_internal_uid=1;t.locked_.x=0;t.locked_.y=3;t.locked_.confidence=1;
  t.initKalmanFromLocked();
  CandidateTarget old;old.id=1;old.internal_uid=1;old.x=0;old.y=3;old.range_m=3;old.point_count=6;old.current_frame_point_count=3;old.width_m=.2;old.depth_m=.2;old.has_current_frame_support=true;
  t.has_last_accepted_candidate_=true;t.last_accepted_candidate_=old;
  auto next=old;next.id=2;next.internal_uid=2;next.x=.1;next.range_m=std::hypot(next.x,next.y);
  t.association_candidates_={next};t.candidates_={next};
  if(competitor){next.id=3;next.internal_uid=3;next.x=-.1;t.association_candidates_.push_back(next);t.candidates_.push_back(next);}
 }
 static void step(TargetTracker&t,double dt=.1){t.frame_context_.sample_time_sec+=dt;++t.frame_context_.frame_number;t.current_frame_dt_s_=float(dt);t.updateLockedTarget();}
 static void clear(TargetTracker&t){t.association_candidates_.clear();t.candidates_.clear();}
};}
void check(bool b,const char*m){if(!b)throw std::runtime_error(m);}
int main(){using namespace TargetTracking;
 TargetTracker t;TargetTrackerTestAccess::seed(t);
 for(int i=0;i<2;++i){TargetTrackerTestAccess::step(t);check(!t.lockedTarget().has_measurement&&!t.lockedTarget().control_allow_translation,"new UID moved before confirmation");}
 TargetTrackerTestAccess::step(t);check(t.lockedTarget().has_measurement&&t.lockedTarget().matched_internal_uid==2&&t.lockedTarget().display_id==1&&t.output().selection.selection_epoch==5,"gated three-hit recovery failed or changed task");
 TargetTracker ambiguous;TargetTrackerTestAccess::seed(ambiguous,true);
 for(int i=0;i<6;++i){TargetTrackerTestAccess::step(ambiguous);check(!ambiguous.lockedTarget().has_measurement&&!ambiguous.lockedTarget().control_allow_translation,"ambiguous neighbour acquired");}
 TargetTracker delayed;TargetTrackerTestAccess::seed(delayed);TargetTrackerTestAccess::clear(delayed);TargetTrackerTestAccess::step(delayed,.6);
 check(delayed.lockedTarget().valid&&!delayed.output().target.control_allow_translation&&!delayed.lockedTarget().control_allow_yaw,"elapsed time cap ignored at a low frame rate");
 std::cout<<"PASS: three-hit UID recovery, stable task identity, ambiguous refusal and real-time prediction cap\n";
}
