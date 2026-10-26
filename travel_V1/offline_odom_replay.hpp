#ifndef OFFLINE_ODOM_REPLAY_HPP
#define OFFLINE_ODOM_REPLAY_HPP

#include <cstddef>
#include <map>
#include <string>
#include <vector>

#include "chassis_velocity.hpp"

// Motion samples recovered from a realtime tracking diagnostics CSV.  DSP and
// tracking samples are kept separately because the realtime pipeline records
// both acquisition-time Doppler compensation and GUI/tracker-time ego motion.
struct OfflineMotionFrame {
    int frame = 0;
    std::string radar_timestamp;

    bool has_dsp_sample = false;
    ChassisVelocitySample dsp_sample;
    bool has_dsp_age = false;
    double dsp_age_sec = 0.0;

    bool has_tracking_sample = false;
    ChassisVelocitySample tracking_sample;
    bool has_tracking_age = false;
    double tracking_age_sec = 0.0;
};

class OfflineOdomReplay {
public:
    // Searches the supplied directories for the realtime diagnostic CSV whose
    // input_path matches bin_path. RADAR_OFFLINE_DIAGNOSTIC_CSV can be used to
    // select a file explicitly when several recordings have the same name.
    bool loadForBin(const std::string& bin_path,
                    const std::vector<std::string>& search_directories,
                    std::string* message = nullptr);

    bool loadCsv(const std::string& csv_path,
                 const std::string& expected_bin_path = std::string(),
                 std::string* message = nullptr);

    void clear();
    bool loaded() const { return !frames_.empty(); }
    std::size_t frameCount() const { return frames_.size(); }
    const std::string& sourcePath() const { return source_path_; }
    const std::string& recordedConfigPath() const { return recorded_config_path_; }
    const OfflineMotionFrame* frame(int frame_number) const;
    // Missing boundary frames get an estimated timestamp, never invented odom.
    std::string radarTimestampForFrame(int frame_number, double frame_period_ms) const;

private:
    struct ParsedCsv {
        std::map<int, OfflineMotionFrame> frames;
        std::string input_path;
        std::string source;
        std::string radar_config_file;
    };

    static bool parseCsv(const std::string& csv_path, ParsedCsv& parsed,
                         std::string* error);
    static std::vector<std::string> parseCsvRow(const std::string& line);

    std::map<int, OfflineMotionFrame> frames_;
    std::string source_path_;
    std::string recorded_config_path_;
};

#endif // OFFLINE_ODOM_REPLAY_HPP
