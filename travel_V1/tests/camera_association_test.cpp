#include "camera_association.hpp"
#include <iostream>
#include <stdexcept>
#include <limits>
using namespace CameraAssist;
static void check(bool ok,const char* text){if(!ok)throw std::runtime_error(text);}
int main(){
    // Radar/image locations deliberately have no geometric correspondence.
    RadarFrame r{11,100,{{42,1,-8,12},{43,2,8,2}}};
    VisionFrame v;v.session=22;v.sequence=1;v.device="camera";v.width=640;v.height=480;
    v.captured_monotonic=100.03;v.completed_monotonic=100.05;v.people={{{270,180,100,180},.9,7,true}};
    Binding binding;std::string reason;
    check(bindPerson(r,v,42,3,7,100.08,binding,reason),"Manual binding requires geometry/calibration");
    check(binding.radar_uid==42&&binding.camera_track==7&&binding.selection_epoch==3,"Binding identity missing");
    check(motionDecision(true,binding,r,v,42,3,100.08).permit,"Valid evidence blocked");
    check(!motionDecision(true,binding,r,v,42,4,100.08).permit,"Old selection binding accepted");
    check(!motionDecision(true,binding,r,v,42,3,100.7).permit,"Stale camera accepted");
    check(!motionDecision(true,binding,r,v,43,3,100.08).permit,"Different existing radar identity accepted");
    check(motionDecision(false,{}, {},VisionFrame(),0,0,100).permit,"Disabled camera changed radar policy");
    check(!motionDecision(true,{},r,v,42,3,100.08).permit,"Unbound guard permitted motion");
    auto attempt=binding;
    check(!bindPerson(r,v,0,3,7,100.08,attempt,reason)&&reason=="no_radar_lock","Click without radar lock accepted");
    check(!bindPerson(r,v,99,3,7,100.08,attempt,reason),"Missing radar target accepted");
    check(!bindPerson(r,v,42,3,9,100.08,attempt,reason),"Missing visual target accepted");
    check(attempt.camera_track==7&&attempt.radar_uid==42,"Failed click replaced existing binding");
    auto changed=v;changed.session=23;
    check(!motionDecision(true,binding,r,changed,42,3,100.08).permit,"Reused camera ID after restart accepted");
    changed=v;changed.captured_monotonic=99.79;changed.completed_monotonic=100.05;
    check(!motionDecision(true,binding,r,changed,42,3,100.08).permit,"Unsynchronized data accepted");
    changed=v;changed.people[0].stable=false;
    check(!bindPerson(r,changed,42,3,7,100.08,attempt,reason),"Unstable visual ID accepted");
    changed=v;changed.people.clear();
    check(!motionDecision(true,binding,r,changed,42,3,100.08).permit,"Missing bound person accepted");
    changed=v;changed.people.push_back({{450,180,100,180},.9,8,true});
    check(motionDecision(true,binding,r,changed,42,3,100.08).permit,"Other people invalidated manual binding");
    check(bindPerson(r,changed,42,3,8,100.08,attempt,reason)&&attempt.camera_track==8,"Explicit rebind failed");
    changed=v;changed.people.push_back(changed.people[0]);
    check(!bindPerson(r,changed,42,3,7,100.08,attempt,reason),"Duplicate visual ID accepted");
    auto other=r;other.session=12;
    check(!motionDecision(true,binding,other,v,42,3,100.08).permit,"Different radar session accepted");
    other=r;other.people.erase(other.people.begin());
    check(!motionDecision(true,binding,other,v,42,3,100.08).permit,"Absent bound radar accepted");
    other=r;other.captured_monotonic=99.7;
    check(!motionDecision(true,binding,other,v,42,3,100.08).permit,"Stale radar accepted");
    changed=v;changed.captured_monotonic=std::numeric_limits<double>::quiet_NaN();
    check(!motionDecision(true,binding,r,changed,42,3,100.08).permit,"NaN time accepted");
    PersonTracker tracker;std::vector<Person> boxes={{{10,20,80,180},.9,0,false}};
    auto a=tracker.update(boxes,1),b=tracker.update(boxes,1.1),d=tracker.update(boxes,1.2);
    check(a[0].track_id==b[0].track_id&&d[0].stable,"Visual continuity failed");
    const auto old=d[0].track_id;tracker.update({},1.3);a=tracker.update(boxes,1.4);
    check(a[0].track_id==old&&a[0].stable,"Brief dropout discarded visual identity");
    tracker.update({},2.1);a=tracker.update(boxes,2.2);
    check(a[0].track_id!=old&&!a[0].stable,"Expired visual identity reused");
    tracker.reset();boxes.push_back({{35,20,80,180},.9,0,false});
    a=tracker.update(boxes,2);b=tracker.update(boxes,2.1);
    check(a[0].track_id!=b[0].track_id&&a[1].track_id!=b[1].track_id&&!b[0].stable,"Ambiguous overlap retained trusted identity");
    std::cout<<"PASS: manual binding without calibration, time/session/UID/selection gates and conservative visual IDs\n";
}
