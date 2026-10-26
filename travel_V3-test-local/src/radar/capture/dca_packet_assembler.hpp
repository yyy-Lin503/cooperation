#ifndef DCA_PACKET_ASSEMBLER_HPP
#define DCA_PACKET_ASSEMBLER_HPP
#include <cstdint>
#include <vector>
#include <stdexcept>
#include <limits>

// Reassemble only complete, contiguous frames at absolute DCA byte offsets.
// A damaged frame is discarded, never zero-filled into an apparent measurement.
class DcaPacketAssembler {
public:
    struct Frame { int number; std::vector<char> bytes; };
    struct Counters {
        std::uint64_t packets = 0, short_packets = 0, discontinuities = 0;
        std::uint64_t out_of_order = 0, missing_packets = 0, discarded_bytes = 0;
    } counters;
    explicit DcaPacketAssembler(std::size_t frame_bytes) : frame_bytes_(frame_bytes) {
        if (!frame_bytes) throw std::invalid_argument("Zero frame size");
    }
    std::vector<Frame> feed(const char* packet, std::size_t size) {
        counters.packets++;
        if (size <= 10) { counters.short_packets++; return {}; }
        std::uint32_t seq = 0;
        std::uint64_t offset = 0;
        for (int i = 0; i < 4; ++i) seq |= std::uint32_t(static_cast<unsigned char>(packet[i])) << (8*i);
        for (int i = 0; i < 6; ++i) offset |= std::uint64_t(static_cast<unsigned char>(packet[4+i])) << (8*i);
        if (initialized_ && (offset < expected_offset_ || static_cast<std::int32_t>(seq - last_seq_) <= 0)) {
            counters.out_of_order++;
            return {};
        }
        if (initialized_ && (offset != expected_offset_ || seq != last_seq_ + 1)) {
            counters.discontinuities++;
            const auto delta = seq - last_seq_;
            if (delta > 1) counters.missing_packets += delta - 1;
            counters.discarded_bytes += pending_.size();
            pending_.clear();
        }
        initialized_ = true;
        expected_offset_ = offset + size - 10;
        last_seq_ = seq;
        const char* bytes = packet + 10;
        std::size_t length = size - 10;
        if (pending_.empty()) {
            const std::size_t skip = (frame_bytes_ - offset % frame_bytes_) % frame_bytes_;
            const std::size_t consumed = std::min(skip, length);
            counters.discarded_bytes += consumed;
            offset += consumed; bytes += consumed; length -= consumed;
            if (!length) return {};
            pending_offset_ = offset;
        }
        pending_.insert(pending_.end(), bytes, bytes + length);
        std::vector<Frame> frames;
        while (pending_.size() >= frame_bytes_) {
            const auto number = pending_offset_ / frame_bytes_ + 1;
            if (number > static_cast<std::uint64_t>(std::numeric_limits<int>::max()))
                throw std::runtime_error("Capture frame number overflow");
            frames.push_back({static_cast<int>(number), {pending_.begin(), pending_.begin()+frame_bytes_}});
            pending_.erase(pending_.begin(), pending_.begin()+frame_bytes_);
            pending_offset_ += frame_bytes_;
        }
        return frames;
    }
private:
    std::size_t frame_bytes_;
    bool initialized_ = false;
    std::uint32_t last_seq_ = 0;
    std::uint64_t expected_offset_ = 0, pending_offset_ = 0;
    std::vector<char> pending_;
};
#endif
