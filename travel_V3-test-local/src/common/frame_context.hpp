#ifndef RADAR_FRAME_CONTEXT_HPP
#define RADAR_FRAME_CONTEXT_HPP
#include <chrono>
#include <cstdint>
#include <string>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <cmath>

namespace RadarTime {
inline double wallSeconds() {
    return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}
inline double monotonicSeconds() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
inline std::string timestamp(double epoch) {
    if (!std::isfinite(epoch) || epoch <= 0) return {};
    const auto seconds = static_cast<std::time_t>(std::floor(epoch));
    std::tm local{};
#ifdef _WIN32
    localtime_s(&local, &seconds);
#else
    localtime_r(&seconds, &local);
#endif
    std::ostringstream out;
    out << std::put_time(&local, "%Y-%m-%d %H:%M:%S") << '.' << std::setfill('0') << std::setw(3)
        << static_cast<long long>(epoch * 1000) % 1000;
    return out.str();
}
inline double parseTimestamp(std::string stamp) {
    if (stamp.size() < 19) return 0;
    if (stamp[10] == 'T') stamp[10] = ' ';
    std::tm local{}; local.tm_isdst = -1;
    std::istringstream input(stamp.substr(0,19));
    input >> std::get_time(&local, "%Y-%m-%d %H:%M:%S");
    if (input.fail()) return 0;
    double fraction = 0;
    if (stamp.size() > 19 && stamp[19] == '.') {
        try { fraction = std::stod("0" + stamp.substr(19)); } catch (...) { return 0; }
    }
    const auto seconds = std::mktime(&local);
    return seconds > 0 ? static_cast<double>(seconds) + fraction : 0;
}
}

// Hardware sample time is unavailable on the DCA UDP interface. Never label
// a receive-time estimate as a hardware timestamp. Replay preserves its source.
struct RadarFrameContext {
    std::uint64_t session_id = 0;
    int frame_number = 0;
    double received_time_sec = 0;
    double received_monotonic_sec = 0;
    double sample_time_sec = 0;
    std::string time_quality = "unknown";
    double processing_started_sec = 0;
    double processing_completed_sec = 0;
    double queue_delay_ms = 0;
};

struct TargetSelectionEvent {
    std::uint64_t session_id = 0;
    std::uint64_t selection_epoch = 0;
    std::uint64_t internal_uid = 0;
};
#endif
