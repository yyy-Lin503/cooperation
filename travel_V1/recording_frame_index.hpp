#ifndef RECORDING_FRAME_INDEX_HPP
#define RECORDING_FRAME_INDEX_HPP
#include "frame_context.hpp"
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <stdexcept>

// BIN contains only complete ADC frames; the companion maps packed file slots
// back to source frames after packet damage or recording starting mid-session.
class RecordingFrameIndex {
public:
    void load(const std::string& binary, std::size_t count) {
        frames_.clear();
        std::ifstream stream(binary + ".frames.csv");
        if (!stream) return; // legacy recording
        std::string line;
        std::getline(stream, line);
        if (line != "source_frame,session_id,received_time_sec,sample_time_sec,time_quality")
            throw std::runtime_error("Invalid recording frame index header");
        while (std::getline(stream, line)) {
            if (line.empty()) continue;
            std::istringstream row(line);
            std::vector<std::string> values;
            std::string value;
            while (std::getline(row, value, ',')) values.push_back(value);
            if (values.size() != 5) throw std::runtime_error("Invalid recording frame index row");
            RadarFrameContext frame;
            frame.frame_number = std::stoi(values[0]);
            frame.session_id = std::stoull(values[1]);
            frame.received_time_sec = std::stod(values[2]);
            frame.sample_time_sec = std::stod(values[3]);
            frame.time_quality = values[4];
            if (frame.frame_number < 1 || !std::isfinite(frame.received_time_sec) ||
                !std::isfinite(frame.sample_time_sec) || frame.received_time_sec <= 0 || frame.sample_time_sec <= 0 ||
                (!frames_.empty() && (frames_.back().frame_number >= frame.frame_number ||
                 frames_.back().session_id != frame.session_id)))
                throw std::runtime_error("Non-monotonic or invalid recording frame index");
            frames_.push_back(frame);
        }
        if (frames_.size() != count) throw std::runtime_error("BIN/frame-index length mismatch");
    }
    RadarFrameContext frame(std::size_t slot) const {
        if (frames_.empty()) {
            RadarFrameContext context;
            context.frame_number = static_cast<int>(slot + 1);
            return context;
        }
        return frames_.at(slot);
    }
    bool loaded() const { return !frames_.empty(); }
private:
    std::vector<RadarFrameContext> frames_;
};
#endif
