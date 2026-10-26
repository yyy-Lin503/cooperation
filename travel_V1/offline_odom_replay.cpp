#include "offline_odom_replay.hpp"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <experimental/filesystem>
#include <fstream>
#include <set>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <chrono>

namespace fs = std::experimental::filesystem;

namespace {

std::string lower(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}

std::string baseName(const std::string& value)
{
    if (value.empty()) return {};
    try {
        std::string normalized = value;
        std::replace(normalized.begin(), normalized.end(), '\\', '/');
        return lower(fs::path(normalized).filename().string());
    } catch (...) {
        const std::size_t pos = value.find_last_of("/\\");
        return lower(pos == std::string::npos ? value : value.substr(pos + 1));
    }
}

bool parseBool(const std::string& value)
{
    const std::string normalized = lower(value);
    return normalized == "1" || normalized == "true" || normalized == "yes";
}

int parseInt(const std::string& value, int fallback = 0)
{
    try { return std::stoi(value); } catch (...) { return fallback; }
}

std::uint32_t parseUint(const std::string& value)
{
    try { return static_cast<std::uint32_t>(std::stoul(value)); }
    catch (...) { return 0; }
}

float parseFloat(const std::string& value)
{
    try { return std::stof(value); } catch (...) { return 0.0f; }
}

double parseDouble(const std::string& value)
{
    try { return std::stod(value); } catch (...) { return 0.0; }
}

int column(const std::map<std::string, int>& columns, const char* name)
{
    const auto it = columns.find(name);
    return it == columns.end() ? -1 : it->second;
}

std::string field(const std::vector<std::string>& row, int index)
{
    if (index < 0 || index >= static_cast<int>(row.size())) return {};
    return row[static_cast<std::size_t>(index)];
}

void fillSample(const std::vector<std::string>& row,
                const std::map<std::string, int>& columns,
                const std::string& prefix,
                ChassisVelocitySample& sample,
                bool& has_sample,
                bool& has_age,
                double& age_sec)
{
    const int vx_col = column(columns, (prefix + "vx_mps").c_str());
    const int wz_col = column(columns, (prefix + "wz_radps").c_str());
    const int ts_col = column(columns, (prefix + "timestamp_sec").c_str());
    if (vx_col < 0 && wz_col < 0 && ts_col < 0) return;

    has_sample = true;
    sample.vx_mps = parseFloat(field(row, vx_col));
    sample.wz_radps = parseFloat(field(row, wz_col));
    sample.timestamp_sec = parseDouble(field(row, ts_col));
    sample.has_timestamp = sample.timestamp_sec > 0.0;

    const int valid_col = column(columns, (prefix + "valid").c_str());
    const int fresh_col = column(columns, (prefix + "fresh").c_str());
    const int seq_col = column(columns, (prefix + "seq").c_str());
    const int age_col = column(columns, (prefix + "age_sec").c_str());
    sample.valid = valid_col >= 0 ? parseBool(field(row, valid_col))
                                  : sample.has_timestamp;
    sample.fresh = fresh_col >= 0 ? parseBool(field(row, fresh_col))
                                  : sample.valid;
    sample.seq = parseUint(field(row, seq_col));
    if (age_col >= 0) {
        has_age = true;
        age_sec = parseDouble(field(row, age_col));
    }
}

} // namespace

std::vector<std::string> OfflineOdomReplay::parseCsvRow(const std::string& line)
{
    std::vector<std::string> fields;
    std::string current;
    bool quoted = false;
    for (std::size_t i = 0; i < line.size(); ++i) {
        const char c = line[i];
        if (c == '"') {
            if (quoted && i + 1 < line.size() && line[i + 1] == '"') {
                current.push_back('"');
                ++i;
            } else {
                quoted = !quoted;
            }
        } else if (c == ',' && !quoted) {
            fields.push_back(current);
            current.clear();
        } else {
            current.push_back(c);
        }
    }
    fields.push_back(current);
    return fields;
}

bool OfflineOdomReplay::parseCsv(const std::string& csv_path,
                                 ParsedCsv& parsed,
                                 std::string* error)
{
    std::ifstream input(csv_path);
    if (!input) {
        if (error) *error = "cannot open diagnostic CSV: " + csv_path;
        return false;
    }

    std::string line;
    if (!std::getline(input, line)) {
        if (error) *error = "diagnostic CSV is empty: " + csv_path;
        return false;
    }
    if (!line.empty() && line.back() == '\r') line.pop_back();
    const std::vector<std::string> header = parseCsvRow(line);
    std::map<std::string, int> columns;
    for (int i = 0; i < static_cast<int>(header.size()); ++i) {
        columns[header[static_cast<std::size_t>(i)]] = i;
    }
    const int frame_col = column(columns, "frame");
    if (frame_col < 0) {
        if (error) *error = "diagnostic CSV has no frame column: " + csv_path;
        return false;
    }

    const int radar_ts_col = column(columns, "radar_timestamp");
    const int input_col = column(columns, "input_path");
    const int source_col = column(columns, "source");
    const int config_col = column(columns, "radar_config_file");
    while (std::getline(input, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty()) continue;
        const std::vector<std::string> row = parseCsvRow(line);
        const int frame_number = parseInt(field(row, frame_col), -1);
        if (frame_number <= 0) continue;

        OfflineMotionFrame motion;
        motion.frame = frame_number;
        motion.radar_timestamp = field(row, radar_ts_col);
        fillSample(row, columns, "dsp_odom_", motion.dsp_sample,
                   motion.has_dsp_sample, motion.has_dsp_age,
                   motion.dsp_age_sec);
        fillSample(row, columns, "tracking_odom_", motion.tracking_sample,
                   motion.has_tracking_sample, motion.has_tracking_age,
                   motion.tracking_age_sec);
        if (!motion.has_tracking_sample && motion.has_dsp_sample) {
            motion.has_tracking_sample = true;
            motion.tracking_sample = motion.dsp_sample;
            motion.has_tracking_age = motion.has_dsp_age;
            motion.tracking_age_sec = motion.dsp_age_sec;
        }
        parsed.frames[frame_number] = motion;
        if (parsed.input_path.empty()) parsed.input_path = field(row, input_col);
        if (parsed.source.empty()) parsed.source = lower(field(row, source_col));
        if (parsed.radar_config_file.empty()) parsed.radar_config_file = field(row, config_col);
    }

    if (parsed.frames.empty()) {
        if (error) *error = "diagnostic CSV has no usable per-frame rows: " + csv_path;
        return false;
    }
    return true;
}

bool OfflineOdomReplay::loadCsv(const std::string& csv_path,
                                const std::string& expected_bin_path,
                                std::string* message)
{
    clear();
    ParsedCsv parsed;
    std::string error;
    if (!parseCsv(csv_path, parsed, &error)) {
        if (message) *message = error;
        return false;
    }
    if (!expected_bin_path.empty() && !parsed.input_path.empty() &&
        baseName(parsed.input_path) != baseName(expected_bin_path)) {
        if (message) {
            *message = "diagnostic input_path does not match BIN: " +
                       parsed.input_path;
        }
        return false;
    }
    frames_ = std::move(parsed.frames);
    source_path_ = csv_path;
    recorded_config_path_ = parsed.radar_config_file;
    if (message) {
        *message = "loaded " + std::to_string(frames_.size()) +
                   " motion frames from " + csv_path;
    }
    return true;
}

bool OfflineOdomReplay::loadForBin(
    const std::string& bin_path,
    const std::vector<std::string>& search_directories,
    std::string* message)
{
    clear();
    if (const char* explicit_csv = std::getenv("RADAR_OFFLINE_DIAGNOSTIC_CSV")) {
        if (*explicit_csv != '\0') {
            return loadCsv(explicit_csv, bin_path, message);
        }
    }

    std::set<std::string> candidates;
    for (const std::string& directory : search_directories) {
        try {
            if (directory.empty() || !fs::exists(directory) ||
                !fs::is_directory(directory)) {
                continue;
            }
            for (const auto& entry : fs::directory_iterator(directory)) {
                if (!fs::is_regular_file(entry.path())) continue;
                if (lower(entry.path().extension().string()) == ".csv") {
                    candidates.insert(entry.path().string());
                }
            }
        } catch (...) {
            // A missing/unreadable optional search directory must not prevent
            // ordinary offline BIN processing.
        }
    }

    const std::string expected_name = baseName(bin_path);
    int best_score = -1;
    ParsedCsv best;
    std::string best_path;
    for (const std::string& csv_path : candidates) {
        ParsedCsv parsed;
        if (!parseCsv(csv_path, parsed, nullptr)) continue;
        const bool name_matches = !parsed.input_path.empty() &&
                                  baseName(parsed.input_path) == expected_name;
        if (!name_matches) continue;

        int score = static_cast<int>(std::min<std::size_t>(parsed.frames.size(), 999));
        if (parsed.source == "realtime") score += 10000;
        if (parsed.source == "offline") score -= 5000;
        if (score > best_score) {
            best_score = score;
            best = std::move(parsed);
            best_path = csv_path;
        }
    }

    if (best_score < 0) {
        if (message) {
            *message = "no realtime diagnostic CSV matched BIN " + expected_name +
                       "; offline ego-motion replay is disabled";
        }
        return false;
    }
    frames_ = std::move(best.frames);
    source_path_ = best_path;
    recorded_config_path_ = best.radar_config_file;
    if (message) {
        *message = "loaded " + std::to_string(frames_.size()) +
                   " synchronized motion frames from " + best_path;
    }
    return true;
}

void OfflineOdomReplay::clear()
{
    frames_.clear();
    source_path_.clear();
    recorded_config_path_.clear();
}

const OfflineMotionFrame* OfflineOdomReplay::frame(int frame_number) const
{
    const auto it = frames_.find(frame_number);
    return it == frames_.end() ? nullptr : &it->second;
}

std::string OfflineOdomReplay::radarTimestampForFrame(int frame_number, double frame_period_ms) const
{
    const auto* exact = frame(frame_number);
    if (exact && !exact->radar_timestamp.empty()) return exact->radar_timestamp;
    if (frame_number < 1 || !std::isfinite(frame_period_ms) || frame_period_ms <= 0) return {};
    for (const auto& entry : frames_) {
        std::string stamp = entry.second.radar_timestamp;
        if (stamp.size() < 19) continue;
        if (stamp[10] == 'T') stamp[10] = ' ';
        std::tm clock = {};
        clock.tm_isdst = -1;
        std::istringstream input(stamp.substr(0, 19));
        input >> std::get_time(&clock, "%Y-%m-%d %H:%M:%S");
        if (input.fail()) continue;
        int milliseconds = 0;
        if (stamp.size() > 20 && stamp[19] == '.') {
            std::string fraction = stamp.substr(20, 3);
            while (fraction.size() < 3) fraction += '0';
            try { milliseconds = std::stoi(fraction); } catch (...) { continue; }
        }
        const auto epoch = std::mktime(&clock);
        if (epoch == static_cast<std::time_t>(-1)) continue;
        const auto base = std::chrono::system_clock::from_time_t(epoch) + std::chrono::milliseconds(milliseconds);
        const auto estimated = base + std::chrono::milliseconds(static_cast<long long>(std::llround(
            (frame_number - entry.first) * frame_period_ms)));
        const auto whole = std::chrono::system_clock::to_time_t(estimated);
        auto remainder = std::chrono::duration_cast<std::chrono::milliseconds>(estimated.time_since_epoch()).count() % 1000;
        if (remainder < 0) remainder += 1000;
        std::ostringstream output;
        output << std::put_time(std::localtime(&whole), "%Y-%m-%d %H:%M:%S") << '.'
               << std::setw(3) << std::setfill('0') << remainder;
        return output.str();
    }
    return {};
}
