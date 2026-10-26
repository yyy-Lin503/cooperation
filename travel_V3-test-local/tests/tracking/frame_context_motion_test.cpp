#include "radar_motion.hpp"
#include "target_tracking.hpp"
#include "recording_frame_index.hpp"
#include <fstream>
#include <iostream>
#include <stdexcept>
static void check(bool ok,const char* why){ if(!ok)throw std::runtime_error(why); }
int main() {
    const auto motion=RadarMotion::transform(.5f,.2f,.1f);
    float x=1,y=3; motion.position(x,y);
    float expected_x=std::cos(-.02f)*(1+.004f)-std::sin(-.02f)*(3-.05f);
    check(std::abs(x-expected_x)<1e-6f,"Shared transform changed axes/sign");
    ChassisVelocitySample chassis; chassis.valid=true; chassis.has_timestamp=true;
    chassis.vx_mps=.5f; chassis.wz_radps=.2f; chassis.timestamp_sec=100;
    std::vector<PointCloud> input(1), output;
    input[0].x=1;input[0].y=3;
    RadarMotion::appendCompensated(output,input,1,2,.1,&chassis,&chassis);
    check(std::abs(output[0].x-x)<1e-6f,"Window helper disagrees with transform");
    check(RadarTime::parseTimestamp(RadarTime::timestamp(1791985414.125))==1791985414.125,"Timestamp round trip failed");
    TargetTracking::TargetTracker tracker;
    std::vector<PointCloud> person;
    for(float px:{-.1f,0.f,.1f}){PointCloud p;p.x=px;p.y=3;p.source_frame=1;person.push_back(p);}
    RadarFrameContext context; context.session_id=77;context.sample_time_sec=100;context.frame_number=1;
    tracker.setFrameContext(context);
    for(int i=0;i<3;++i)tracker.updateCandidates(person,&chassis,1);
    check(!tracker.candidates().empty(),"No selectable candidate");
    check(tracker.selectTargetByUid(tracker.candidates().front().internal_uid),"Cannot select");
    check(tracker.output().selection.selection_epoch==1 && tracker.output().selection.session_id==77,"Selection context missing");
    check(tracker.output().last_measurement_time_sec==0,"Pending click fabricated accepted measurement time");
    context.sample_time_sec=100.1;context.frame_number=2;tracker.setFrameContext(context);
    for(auto& p:person)p.source_frame=2;
    tracker.updateCandidates(person,&chassis,2);tracker.updateLockedTarget();
    check(tracker.output().target.has_measurement && tracker.output().last_measurement_time_sec==100.1,"Accepted measurement time missing");
    context.sample_time_sec=100.2;context.frame_number=3;tracker.setFrameContext(context);
    tracker.updateCandidates({},&chassis,3);tracker.updateLockedTarget();
    check(tracker.output().predicted && tracker.output().last_measurement_time_sec==100.1,"Prediction refreshed measurement age");
    check(tracker.output().position_variance_x>0,"Covariance missing");
    tracker.reset();check(tracker.output().last_measurement_time_sec==0 && tracker.output().selection.selection_epoch==0,"Reset leaked session");
    const char* filename="frame-index-test.bin";
    {std::ofstream file(std::string(filename)+".frames.csv");
    file<<"source_frame,session_id,received_time_sec,sample_time_sec,time_quality\n3,7,100,100,receive_estimate\n5,7,100.2,100.2,receive_estimate\n";}
    RecordingFrameIndex index;index.load(filename,2);
    check(index.frame(0).frame_number==3 && index.frame(1).frame_number==5,"Index compressed source frame gaps");
    bool invalid=false;try{index.load(filename,3);}catch(const std::runtime_error&){invalid=true;}
    check(invalid,"Index length mismatch accepted");
    std::remove((std::string(filename)+".frames.csv").c_str());
    std::cout<<"PASS: shared motion, time metadata, confirmed measurement age, reset and recording gaps\n";
}
