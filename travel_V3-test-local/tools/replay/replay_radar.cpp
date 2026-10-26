#include "signal_processing.hpp"
#include "radar_frame_layout.hpp"
#include "radar_mount.hpp"
#include "radar_motion.hpp"
#include "recording_frame_index.hpp"
#include "radar_runtime_config.hpp"
#include "iwr6843param.hpp"
#include "offline_odom_replay.hpp"
#include "target_tracking.hpp"
#include <deque>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <algorithm>

std::chrono::system_clock::time_point capture_start_time;

static std::vector<std::string> csvFields(const std::string& line)
{
    std::vector<std::string> out;
    std::string value;
    bool quoted = false;
    for (std::size_t i = 0; i < line.size(); ++i) {
        if (line[i] == '"') {
            if (quoted && i + 1 < line.size() && line[i+1] == '"') { value += '"'; ++i; }
            else quoted = !quoted;
        } else if (line[i] == ',' && !quoted) { out.push_back(value); value.clear(); }
        else value += line[i];
    }
    out.push_back(value);
    return out;
}

struct Selection { int after_frame; float x, y; };
static std::map<int,std::vector<PointCloud>> loadCachedPoints(const std::string& path)
{
    std::ifstream input(path);
    if (!input) throw std::runtime_error("Cannot open cached points");
    std::map<int,std::vector<PointCloud>> result;
    std::string line; std::getline(input,line);
    while(std::getline(input,line)) {
        const auto row=csvFields(line);
        if(row.size()!=10) throw std::runtime_error("Bad cached point schema");
        PointCloud point; point.source_frame=std::stoi(row[0]);
        point.x=std::stof(row[1]); point.y=std::stof(row[2]);
        point.relative_velocity=std::stof(row[3]); point.absolute_velocity=std::stof(row[4]);
        point.is_moving=std::stoi(row[5]); point.range_bin=std::stoi(row[6]);
        point.doppler_bin=std::stoi(row[7]); point.snr=std::stof(row[8]);
        point.doa_peak_confidence=std::stof(row[9]);
        result[point.source_frame].push_back(point);
    }
    return result;
}
static std::vector<Selection> loadSelections(const std::string& path)
{
    std::ifstream input(path);
    if (!input) throw std::runtime_error("Cannot open selection events");
    std::string line;
    std::getline(input, line);
    std::vector<Selection> result;
    while (std::getline(input, line)) {
        const auto row = csvFields(line);
        if (row.size() < 3) continue;
        result.push_back({std::stoi(row[0]), std::stof(row[1]), std::stof(row[2])});
    }
    return result;
}

static std::vector<PointCloud> aggregate(
    const std::deque<std::pair<int, std::vector<PointCloud>>>& window,
    const OfflineOdomReplay& motion, float period)
{
    return RadarMotion::aggregate(window, period, [&motion](int frame) -> const ChassisVelocitySample* {
        const auto* sample = motion.frame(frame);
        return sample && sample->has_tracking_sample ? &sample->tracking_sample : nullptr;
    });
}

static const char* controlToken(const TargetTracking::LockedTarget& target)
{
    using S = TargetTracking::FollowControlState;
    if (!target.valid) return "UNLOCKED";
    if (target.control_state == S::NormalFollow) return "TRACKING";
    if (target.control_state == S::LimitedFollow || target.control_state == S::ReacquireConfirming)
        return "PREDICTED";
    return "STOPPED";
}

int main(int argc, char** argv)
{
    if (argc < 6) {
        std::cerr << "usage: replay_radar <BIN> <cfg> <diagnostics.csv> <selections.csv> <output.csv> "
                     "[--legacy-doppler] [--max-frames N] [--points-output file.csv]\n";
        return 2;
    }
    try {
        bool legacy = false;
        int max_frames = std::numeric_limits<int>::max();
        int cached_frame_count = 0;
        std::string points_path;
        std::string cached_path, candidates_path;
        std::string profile="default";
        float gate_extra=-1.0f;
        std::string clusters_path;
        for (int i = 6; i < argc; ++i) {
            const std::string option = argv[i];
            if (option == "--legacy-doppler") legacy = true;
            else if (option == "--max-frames" && i+1 < argc) max_frames = std::stoi(argv[++i]);
            else if (option == "--points-output" && i+1 < argc) points_path = argv[++i];
            else if (option == "--cached-frame-count" && i+1 < argc) cached_frame_count = std::stoi(argv[++i]);
            else if (option == "--cached-points" && i+1 < argc) cached_path = argv[++i];
            else if (option == "--candidates-output" && i+1 < argc) candidates_path = argv[++i];
            else if (option == "--profile" && i+1 < argc) profile = argv[++i];
            else if (option == "--gate-extra" && i+1 < argc) gate_extra = std::stof(argv[++i]);
            else if (option == "--clusters-output" && i+1 < argc) clusters_path = argv[++i];
            else throw std::runtime_error("Unknown replay option: " + option);
        }
        IWR6843AOP_TLV cfg(argv[2]);
        const auto params = cfg.get_params();
        const auto bytes = RadarFrameLayout::frameBytes(RadarFrameLayout::paramsFromMap(params));
        OfflineOdomReplay motion;
        std::string message;
        if (!motion.loadCsv(argv[3], argv[1], &message)) throw std::runtime_error(message);
        const auto selections = loadSelections(argv[4]);
        auto cached = cached_path.empty() ? std::map<int,std::vector<PointCloud>>() : loadCachedPoints(cached_path);
        std::ifstream input(argv[1], std::ios::binary);
        std::uint64_t size = 0;
        if (!input) {
            if (cached_path.empty() || cached_frame_count <= 0) throw std::runtime_error("Cannot open BIN; cached-only replay requires --cached-frame-count");
            size = static_cast<std::uint64_t>(cached_frame_count) * bytes;
        } else {
            input.seekg(0, std::ios::end);
            size = static_cast<std::uint64_t>(input.tellg());
            input.seekg(0);
        }
        if (size % bytes) throw std::runtime_error("Incomplete BIN frame; replay refuses silent truncation");
        std::ofstream output(argv[5]);
        if (!output) throw std::runtime_error("Cannot create replay output");
        std::ofstream points_output;
        std::ofstream candidate_output;
        std::ofstream cluster_output;
        if(!clusters_path.empty()) {
            cluster_output.open(clusters_path);
            if(!cluster_output) throw std::runtime_error("Cannot create cluster audit output");
            cluster_output << "frame,cluster_id,person_like,x,y,range,width,depth,points,current_points,has_current_support\n";
            cluster_output << std::setprecision(9);
        }
        if(!candidates_path.empty()) {
            candidate_output.open(candidates_path);
            if(!candidate_output) throw std::runtime_error("Cannot create candidate audit output");
            candidate_output << "frame,locked_uid,lost_before,candidate_uid,display_id,x,y,points,current_points,predicted_x,predicted_y,last_accepted_x,last_accepted_y,distance_m,nis,implied_speed,normal_ok,roi_ok,reacquire_ok,normal_reason,roi_reason,reacquire_reason\n";
            candidate_output << std::setprecision(9);
        }
        if (!points_path.empty()) {
            points_output.open(points_path);
            if (!points_output) throw std::runtime_error("Cannot create point output");
            points_output << "frame,x,y,relative_velocity,absolute_velocity,is_moving,range_bin,doppler_bin,snr,doa_confidence\n";
            points_output << std::setprecision(9);
        }
        output << "frame,timestamp,points,cfar_targets,dsp_ok,dsp_ms,motion_present,near_field_rejected,"
                  "burst_rejected,velocity_resolution,abs_velocity_median,moving_ratio,candidates,"
                  "active_tracks,lock_valid,measured,control_state,lost_frames,confidence,uid,"
                  "x,y,vx,vy,measured_x,measured_y,association_source,reject_reason,nis,"
                  "hypotheses,score_margin,selection_success,selection_error_m,predicted_x,predicted_y,ego_dx,ego_dy,ego_dyaw,frame_dt,control_speed_scale,allow_translation,allow_yaw,processing_source\n";
        output << std::setprecision(9);
        TargetTracking::TargetTracker tracker;
        tracker.setEgoMotionCompensationEnabled(true);
        auto tracking_cfg = tracker.config();
        if(profile!="default") {
            tracking_cfg.use_current_frame_extent=profile.find("extent")!=std::string::npos;
            tracking_cfg.short_gap_same_uid_max_lost_frames=profile.find("shortgap")!=std::string::npos ? 2 : 0;
            tracking_cfg.enable_dropout_gate_expansion=profile.find("gate")!=std::string::npos;
            tracking_cfg.ego_relative_dropout_decay=profile.find("relative")!=std::string::npos;
        }
        if(gate_extra>=0.0f) tracking_cfg.dropout_gate_max_extra_m=gate_extra;
        tracking_cfg.kalman_dt_s = static_cast<float>(params.at("framePeriodicity") * .001);
        tracker.setConfig(tracking_cfg);
        std::deque<std::pair<int, std::vector<PointCloud>>> window;
        reset_signal_processing_session();
        capture_start_time = std::chrono::system_clock::now();
        std::vector<char> frame_data(bytes);
        RecordingFrameIndex frame_index;
        frame_index.load(argv[1], static_cast<std::size_t>(size / bytes));
        const int total = std::min<int>(size / bytes, max_frames);
        for (int slot = 0; slot < total; ++slot) {
            RadarFrameContext metadata = frame_index.frame(slot);
            const int frame = metadata.frame_number;
            if(cached_path.empty()) {
                input.read(frame_data.data(), static_cast<std::streamsize>(bytes));
                if (input.gcount() != static_cast<std::streamsize>(bytes)) throw std::runtime_error("Short read");
            }
            const auto* sample = motion.frame(frame);
            SignalProcessingFrameContext context;
            context.source_frame_number = frame;
            context.frame = metadata;
            context.persist_outputs = false;
            context.legacy_doppler_offset = legacy;
            context.radar_timestamp = frame_index.loaded() ? RadarTime::timestamp(metadata.sample_time_sec) :
                motion.radarTimestampForFrame(frame, params.at("framePeriodicity"));
            if (!frame_index.loaded()) {
                metadata.sample_time_sec = RadarTime::parseTimestamp(context.radar_timestamp);
                metadata.time_quality = metadata.sample_time_sec > 0 ? "recorded_or_estimated" : "unknown";
                context.frame = metadata;
            }
            bool compensate = false;
            if (sample) {
                if (sample->has_dsp_sample) {
                    context.use_chassis_override = true;
                    context.chassis = sample->dsp_sample;
                    context.has_odom_age = sample->has_dsp_age;
                    context.odom_age_sec = sample->dsp_age_sec;
                    compensate = true;
                }
            }
            ProcessingResult result;
            if(cached_path.empty()) result = run_signal_processing_pipeline(frame_data, params, compensate, &context);
            else { result.frame_number=frame; result.timestamp=context.radar_timestamp; result.point_cloud=cached[frame]; }
            if (!result.diagnostics.processing_ok) throw std::runtime_error(result.diagnostics.error_message);
            for (const auto& point : result.point_cloud) if (points_output) {
                points_output << frame << ',' << point.x << ',' << point.y << ','
                    << point.relative_velocity << ',' << point.absolute_velocity << ',' << point.is_moving
                    << ',' << point.range_bin << ',' << point.doppler_bin << ',' << point.snr
                    << ',' << point.doa_peak_confidence << '\n';
            }
            window.emplace_back(frame, result.point_cloud);
            while (window.size() > static_cast<std::size_t>(RadarRuntimeConfig::kPointWindowFrames)) window.pop_front();
            if (window.size() >= static_cast<std::size_t>(RadarRuntimeConfig::kPointWindowFrames)) {
                auto points = aggregate(window, motion, tracking_cfg.kalman_dt_s);
                ChassisVelocitySample sample_tracking;
                if (sample && sample->has_tracking_sample) sample_tracking = sample->tracking_sample;
                tracker.setFrameContext(metadata);
                RadarMotion::advance(tracker, points, sample_tracking, frame);
                if (tracker.lockedTarget().valid && !tracker.lockedTarget().has_measurement &&
                    tracker.lockedTarget().lost_frames >= tracking_cfg.manual_unlock_lost_frames)
                    tracker.unlockTarget("manual_lost_timeout");
            }
            // Match GUI order: a click between frames affects the following update.
            int selection_success = -1;
            float selection_error = -1.0f;
            for (const auto& event : selections) if (event.after_frame == frame) {
                const TargetTracking::CandidateTarget* best = nullptr;
                float distance = 1.0f;
                for (const auto& candidate : tracker.candidates()) {
                    const float d = std::hypot(candidate.x - event.x, candidate.y - event.y);
                    if (d < distance) { best = &candidate; distance = d; }
                }
                selection_success = best && tracker.selectTargetByUid(best->internal_uid);
                selection_error = best ? distance : -1.0f;
            }
            const auto& target = tracker.lockedTarget();
            const auto& association = tracker.associationDiagnostics();
            const auto& ego = tracker.egoMotionDiagnostics();
            for(const auto& cluster:tracker.clusterDebugInfo()) if(cluster_output) {
                cluster_output << frame << ',' << cluster.cluster_id << ',' << cluster.person_like << ',' << cluster.x
                    << ',' << cluster.y << ',' << cluster.range_m << ',' << cluster.width_m << ',' << cluster.depth_m
                    << ',' << cluster.point_count << ',' << cluster.current_frame_point_count << ',' << cluster.has_current_frame_support << '\n';
            }
            for(const auto& a:tracker.candidateGateAudit()) if(candidate_output) {
                candidate_output << frame << ',' << a.locked_uid << ',' << a.lost_before << ',' << a.candidate.internal_uid
                    << ',' << a.candidate.id << ',' << a.candidate.x << ',' << a.candidate.y << ',' << a.candidate.point_count
                    << ',' << a.candidate.current_frame_point_count << ',' << a.predicted_x << ',' << a.predicted_y
                    << ',' << a.last_accepted_x << ',' << a.last_accepted_y << ',' << a.distance_m << ',' << a.nis
                    << ',' << a.implied_speed_mps << ',' << a.normal_ok << ',' << a.roi_ok << ',' << a.reacquire_ok
                    << ',' << a.normal_reason << ',' << a.roi_reason << ',' << a.reacquire_reason << '\n';
            }
            const auto& dsp = result.diagnostics;
            output << frame << ',' << result.timestamp << ',' << result.point_cloud.size() << ','
                << dsp.cfar_target_count << ',' << dsp.processing_ok << ',' << dsp.processing_time_ms
                << ',' << (sample != nullptr) << ',' << dsp.near_field_range_rejected_count
                << ',' << dsp.near_field_burst_rejected_count << ',' << dsp.velocity_resolution_mps
                << ',' << dsp.all_point_abs_velocity_median_mps << ',' << dsp.moving_ratio
                << ',' << tracker.candidates().size() << ',' << tracker.activeTrackCount()
                << ',' << target.valid << ',' << target.has_measurement << ',' << controlToken(target)
                << ',' << target.lost_frames << ',' << target.confidence << ',' << target.matched_internal_uid
                << ',' << target.x << ',' << target.y << ',' << target.vx << ',' << target.vy
                << ',' << target.measured_x << ',' << target.measured_y << ',' << association.source
                << ',' << association.reject_reason << ',' << association.nis
                << ',' << association.reacquire_hypothesis_count << ',' << association.reacquire_score_margin
                << ',' << selection_success << ',' << selection_error << ',' << association.predicted_x << ',' << association.predicted_y
                << ',' << ego.radar_dx_m << ',' << ego.radar_dy_m << ',' << ego.radar_dyaw_rad << ',' << ego.dt_s
                << ',' << target.control_speed_scale << ',' << target.control_allow_translation << ',' << target.control_allow_yaw
                << ',' << (cached_path.empty() ? "raw_ADC" : "cached_points_ablation") << '\n';
            if (frame % 100 == 0 || frame == total)
                std::cout << "REPLAY " << (legacy ? "v1" : "v2") << ' ' << frame << '/' << total << std::endl;
        }
        reset_signal_processing_session();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "REPLAY FAILED: " << error.what() << '\n';
        return 1;
    }
}
