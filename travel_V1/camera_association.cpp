#include "camera_association.hpp"
#include <algorithm>
#include <cmath>

namespace CameraAssist {
namespace {
bool validBox(const Person& p) {
    return std::isfinite(p.box.x) && std::isfinite(p.box.y) && std::isfinite(p.box.width) &&
        std::isfinite(p.box.height) && p.box.width > 2 && p.box.height > 4 &&
        std::isfinite(p.confidence) && p.confidence >= .35f;
}
std::string evidenceStatus(const RadarFrame& radar, const VisionFrame& vision,
                           std::uint64_t uid, std::uint64_t camera_track, double now) {
    if (!uid) return "no_radar_lock";
    if (!std::isfinite(now) || !vision.session ||
        !std::isfinite(vision.captured_monotonic) || vision.captured_monotonic <= 0 ||
        now-vision.captured_monotonic < -.02 || now-vision.captured_monotonic > .5 ||
        !std::isfinite(vision.completed_monotonic) ||
        vision.completed_monotonic < vision.captured_monotonic ||
        vision.completed_monotonic > now+.02) return "camera_stale";
    if (!radar.session || !std::isfinite(radar.captured_monotonic) ||
        radar.captured_monotonic <= 0 || now-radar.captured_monotonic < -.02 ||
        now-radar.captured_monotonic > .35) return "radar_stale";
    if (std::abs(radar.captured_monotonic-vision.captured_monotonic) > .2)
        return "unsynchronized";
    const auto radar_count = std::count_if(radar.people.begin(),radar.people.end(),
        [uid](const RadarPerson& p){return p.uid==uid && p.display_id>0;});
    if (radar_count != 1) return "radar_target_missing";
    const Person* selected = nullptr;
    for (const auto& p : vision.people) {
        if (p.track_id != camera_track || !camera_track) continue;
        if (selected) return "ambiguous";
        selected = &p;
    }
    if (!selected) return "person_missing";
    if (!selected->stable || !validBox(*selected)) return "person_unstable";
    return "verified";
}
}

bool bindPerson(const RadarFrame& radar, const VisionFrame& vision,
                std::uint64_t uid, std::uint64_t epoch, std::uint64_t camera_track,
                double now, Binding& result, std::string& reason) {
    reason = evidenceStatus(radar,vision,uid,camera_track,now);
    if (reason != "verified") return false;
    result = {vision.session,camera_track,radar.session,epoch,uid};
    return true;
}

MotionDecision motionDecision(bool enabled, const Binding& binding, const RadarFrame& radar,
                              const VisionFrame& vision, std::uint64_t uid,
                              std::uint64_t epoch, double now) {
    if (!enabled) return {};
    MotionDecision result{true,false,"not_bound"};
    if (!binding.camera_track || !binding.camera_session || !binding.radar_uid) return result;
    if (!uid || binding.radar_uid != uid || binding.radar_session != radar.session ||
        binding.selection_epoch != epoch) {result.reason="selection_changed";return result;}
    if (binding.camera_session != vision.session) {result.reason="camera_session_changed";return result;}
    result.reason=evidenceStatus(radar,vision,uid,binding.camera_track,now);
    result.permit=result.reason=="verified";
    return result;
}
void PersonTracker::reset(){tracks_.clear();previous_time_=0;next_id_=1;}
std::vector<Person> PersonTracker::update(const std::vector<Person>& input,double now) {
    if(!std::isfinite(now)||now<=previous_time_||now-previous_time_>1.0)tracks_.clear();
    tracks_.erase(std::remove_if(tracks_.begin(),tracks_.end(),[now](const Track& t){
        return now-t.last_seen>.6;
    }),tracks_.end());
    previous_time_=now;
    std::vector<Person> detections;for(const auto& p:input)if(validBox(p))detections.push_back(p);
    std::vector<std::vector<int>> edges(detections.size());std::vector<int> degree(tracks_.size());
    for(std::size_t i=0;i<detections.size();++i) for(std::size_t j=0;j<tracks_.size();++j) {
        const auto a=detections[i].box,b=tracks_[j].person.box;
        const float overlap=(a&b).area(),iou=overlap/std::max(1.f,a.area()+b.area()-overlap);
        const auto da=cv::Point2f(a.x+a.width/2,a.y+a.height/2),db=cv::Point2f(b.x+b.width/2,b.y+b.height/2);
        const double distance=cv::norm(da-db)/std::max(1.f,std::min(a.height,b.height));
        if(iou>=.25 || (distance<.25 && a.area()/b.area()>.5 && a.area()/b.area()<2)) {edges[i].push_back(j);++degree[j];}
    }
    std::vector<Track> next;
    for(std::size_t i=0;i<detections.size();++i) {
        Track track;track.person=detections[i];track.hits=1;track.person.track_id=next_id_++;track.last_seen=now;
        if(edges[i].size()==1 && degree[edges[i][0]]==1) {
            const auto& old=tracks_[edges[i][0]];track.person.track_id=old.person.track_id;track.hits=old.hits+1;
        }
        track.person.stable=track.hits>=3;next.push_back(track);detections[i]=track.person;
    }
    // Keep missing hypotheses briefly, but never emit a fabricated detection.
    // Ambiguous overlap retires the involved IDs instead of transferring them.
    for(std::size_t j=0;j<tracks_.size();++j)
        if(degree[j]==0&&now-tracks_[j].last_seen<=.6)next.push_back(tracks_[j]);
    tracks_=std::move(next);return detections;
}
}
