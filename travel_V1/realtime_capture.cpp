#include "realtime_capture.hpp"
#include "radar_frame_layout.hpp"
#include <algorithm>
#include <cerrno>
#include <cstring>
#include <experimental/filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#ifdef _WIN32
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#endif
namespace {
std::atomic<std::uint64_t> session_sequence{0};
sockaddr_in endpoint(const std::string& ip, int port) {
    sockaddr_in result{};
    result.sin_family = AF_INET;
    if (port < 0 || port > 65535 || inet_pton(AF_INET, ip.c_str(), &result.sin_addr) != 1)
        throw std::invalid_argument("Invalid UDP endpoint: " + ip);
    result.sin_port = htons(static_cast<unsigned short>(port));
    return result;
}
std::string socketError(const char* what) {
#ifdef _WIN32
    return std::string(what) + ": " + std::to_string(WSAGetLastError());
#else
    return std::string(what) + ": " + std::strerror(errno);
#endif
}

}
UdpListener::UdpListener(const std::string&, std::queue<std::vector<char>>&, int frame_length,
    const std::string& address, int port, int buff_size, const std::map<std::string,double>& params,
    const std::string& target_ip, int target_port, const CaptureOptions& options)
    : frame_bytes_(frame_length > 0 ? static_cast<std::size_t>(frame_length) * 2 : 0),
      address_(address), target_ip_(target_ip), port_(port), buff_size_(buff_size), target_port_(target_port),
      radar_params_(params), options_(options) {
    if (!frame_bytes_ || buff_size_ <= 10 || !options_.max_queue_frames || !options_.max_transmission_frames ||
        !std::isfinite(options_.max_frame_age_sec) || options_.max_frame_age_sec <= 0)
        throw std::invalid_argument("Invalid capture dimensions/limits");
    if (!options_.processor && frame_bytes_ != RadarFrameLayout::frameBytes(RadarFrameLayout::paramsFromMap(params)))
        throw std::invalid_argument("Capture and DSP frame sizes differ");
}
UdpListener::~UdpListener() {
    stop_collector();
    join(-1);
    stop_saving();
    closeSockets();
}
void UdpListener::closeSockets() {
    if (data_socket_ != INVALID_SOCKET) { closesocket(data_socket_); data_socket_ = INVALID_SOCKET; }
    if (transmission_socket_ != INVALID_SOCKET) { closesocket(transmission_socket_); transmission_socket_ = INVALID_SOCKET; }
}
void UdpListener::start() {
    if (started_) throw std::logic_error("Capture already started; use a new session after stop");
    try {
        auto bind_addr = endpoint(address_, port_);
        endpoint(target_ip_, target_port_);
        data_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (data_socket_ == INVALID_SOCKET) throw std::runtime_error(socketError("Create data socket"));
        int buffer = 8 * 1024 * 1024;
        setsockopt(data_socket_, SOL_SOCKET, SO_RCVBUF, reinterpret_cast<const char*>(&buffer), sizeof(buffer));
#ifdef _WIN32
        DWORD timeout = 100;
#else
        timeval timeout{0, 100000};
#endif
        if (setsockopt(data_socket_, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&timeout), sizeof(timeout)) < 0)
            throw std::runtime_error(socketError("Set receive timeout"));
        if (bind(data_socket_, reinterpret_cast<sockaddr*>(&bind_addr), sizeof(bind_addr)) < 0)
            throw std::runtime_error(socketError("Bind " ));
        if (port_ == 0) {
#ifdef _WIN32
            int address_size = sizeof(bind_addr);
#else
            socklen_t address_size = sizeof(bind_addr);
#endif
            if (getsockname(data_socket_, reinterpret_cast<sockaddr*>(&bind_addr), &address_size) < 0)
                throw std::runtime_error(socketError("Query data endpoint"));
            port_ = ntohs(bind_addr.sin_port);
        }
        transmission_socket_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (transmission_socket_ == INVALID_SOCKET) throw std::runtime_error(socketError("Create result socket"));
        // Local UDP must not indefinitely block shutdown if a destination is congested.
        if (setsockopt(transmission_socket_, SOL_SOCKET, SO_SNDTIMEO, reinterpret_cast<const char*>(&timeout), sizeof(timeout)) < 0)
            throw std::runtime_error(socketError("Set send timeout"));
        session_id_ = static_cast<std::uint64_t>(RadarTime::wallSeconds() * 1000000) + session_sequence.fetch_add(1);
        running_ = true;
        started_ = true;
        { std::lock_guard<std::mutex> lock(status_mutex_); status_.state = CaptureStatus::State::WaitingForData; }
        auto launch = [this](std::thread& thread, void (UdpListener::*method)()) {
            { std::lock_guard<std::mutex> lock(finished_mutex_); ++workers_; }
            try { thread = std::thread([this, method] {
                try { (this->*method)(); }
                catch (const std::exception& e) { fail(e.what()); }
                catch (...) { fail("Unknown capture worker error"); }
                workerFinished();
            }); } catch (...) { workerFinished(); throw; }
        };
        launch(receiver_, &UdpListener::run);
        launch(processing_, &UdpListener::processing_worker);
        launch(transmission_, &UdpListener::transmission_worker);
    } catch (const std::exception& error) {
        fail(error.what());
        join(-1);
        closeSockets();
        throw;
    }
}
void UdpListener::workerFinished() {
    { std::lock_guard<std::mutex> lock(finished_mutex_); --workers_; }
    finished_cond_.notify_all();
}
bool UdpListener::join(int timeout_ms) {
    std::unique_lock<std::mutex> lock(finished_mutex_);
    if (timeout_ms < 0) finished_cond_.wait(lock, [this] { return workers_ == 0; });
    else if (!finished_cond_.wait_for(lock, std::chrono::milliseconds(timeout_ms), [this] { return workers_ == 0; })) return false;
    lock.unlock();
    for (auto* thread : {&receiver_, &processing_, &transmission_}) if (thread->joinable()) thread->join();
    closeSockets();
    return true;
}
void UdpListener::stop_collector() {
    running_ = false;
    queue_cond_.notify_all(); transmission_cond_.notify_all();
    std::lock_guard<std::mutex> lock(status_mutex_);
    if (status_.state != CaptureStatus::State::Failed) status_.state = CaptureStatus::State::Stopped;
}
void UdpListener::fail(const std::string& error) {
    { std::lock_guard<std::mutex> lock(status_mutex_); status_.state = CaptureStatus::State::Failed; status_.error = error; }
    running_ = false;
    queue_cond_.notify_all(); transmission_cond_.notify_all();
}
CaptureStatus UdpListener::status() const {
    CaptureStatus result;
    { std::lock_guard<std::mutex> lock(status_mutex_); result = status_; }
    { std::lock_guard<std::mutex> lock(queue_mutex_); result.queue_depth = frame_queue_.size(); }
    { std::lock_guard<std::mutex> lock(transmission_mutex_); result.transmission_depth = transmission_queue_.size(); }
    return result;
}
void UdpListener::start_saving() {
    std::lock_guard<std::mutex> lock(file_mutex_);
    if (saving_) return;
    if (!filename_.empty()) throw std::logic_error("Recording already closed; use a new capture session");
    namespace fs = std::experimental::filesystem;
    fs::create_directories(options_.output_directory);
    const auto stem = options_.output_directory + "/capture_" + std::to_string(session_id_ ? session_id_ :
        static_cast<std::uint64_t>(RadarTime::wallSeconds()*1000000));
    filename_ = stem + "_raw.bin";
    int suffix = 0;
    while (fs::exists(filename_)) filename_ = stem + "_" + std::to_string(++suffix) + "_raw.bin";
    file_.open(filename_, std::ios::binary | std::ios::trunc);
    frame_index_.open(filename_ + ".frames.csv", std::ios::trunc);
    if (!file_ || !frame_index_) throw std::runtime_error("Cannot create recording: " + filename_);
    frame_index_ << "source_frame,session_id,received_time_sec,sample_time_sec,time_quality\n";
    saving_ = true;
}
void UdpListener::stop_saving() {
    std::lock_guard<std::mutex> lock(file_mutex_);
    saving_ = false;
    if (file_.is_open()) { file_.flush(); file_.close(); }
    if (frame_index_.is_open()) { frame_index_.flush(); frame_index_.close(); }
}
void UdpListener::run() {
    DcaPacketAssembler assembler(frame_bytes_);
    // Full UDP buffer detects oversized packets instead of silently truncating them.
    std::vector<char> buffer(65536);
    while (running_) {
        const int length = recvfrom(data_socket_, buffer.data(), static_cast<int>(buffer.size()), 0, nullptr, nullptr);
        if (length < 0) {
#ifdef _WIN32
            const int code = WSAGetLastError();
            if (code == WSAETIMEDOUT || code == WSAEINTR) continue;
#else
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) continue;
#endif
            if (running_) fail(socketError("Receive ADC"));
            break;
        }
        if (!running_) break;
        const double received = RadarTime::wallSeconds(), monotonic = RadarTime::monotonicSeconds();
        auto frames = assembler.feed(buffer.data(), static_cast<std::size_t>(length));
        { std::lock_guard<std::mutex> lock(status_mutex_); status_.packets = assembler.counters; }
        for (auto& frame : frames) {
            if (!running_) break;
            QueuedFrame item;
            item.bytes = std::move(frame.bytes);
            item.context.session_id = session_id_;
            item.context.frame_number = frame.number;
            item.context.received_time_sec = received;
            item.context.received_monotonic_sec = monotonic;
            item.context.sample_time_sec = received;
            item.context.time_quality = "receive_estimate";
            {
                std::lock_guard<std::mutex> lock(file_mutex_);
                if (saving_) {
                    file_.write(item.bytes.data(), static_cast<std::streamsize>(item.bytes.size()));
                    frame_index_ << std::setprecision(17) << frame.number << ',' << session_id_ << ','
                        << received << ',' << received << ",receive_estimate\n";
                    file_.flush(); frame_index_.flush();
                    if (!file_ || !frame_index_) throw std::runtime_error("Recording write failed");
                }
            }
            {
                std::lock_guard<std::mutex> lock(queue_mutex_);
                if (frame_queue_.size() >= options_.max_queue_frames) {
                    frame_queue_.pop_front();
                    std::lock_guard<std::mutex> stats(status_mutex_); ++status_.queue_dropped;
                }
                frame_queue_.push_back(std::move(item));
            }
            { std::lock_guard<std::mutex> lock(status_mutex_); ++status_.frames_received;
              if (running_ && status_.state != CaptureStatus::State::Failed) status_.state = CaptureStatus::State::Receiving; }
            queue_cond_.notify_one();
        }
    }
}
void UdpListener::processing_worker() {
    while (running_) {
        QueuedFrame item;
        {
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cond_.wait(lock, [this] { return !running_ || !frame_queue_.empty(); });
            if (!running_) break;
            item = std::move(frame_queue_.front()); frame_queue_.pop_front();
        }
        const double age = RadarTime::monotonicSeconds() - item.context.received_monotonic_sec;
        if (age > options_.max_frame_age_sec) {
            std::lock_guard<std::mutex> lock(status_mutex_); ++status_.stale_dropped; continue;
        }
        item.context.queue_delay_ms = age * 1000;
        item.context.processing_started_sec = RadarTime::wallSeconds();
        SignalProcessingFrameContext context;
        context.source_frame_number = item.context.frame_number;
        context.frame = item.context;
        context.radar_timestamp = RadarTime::timestamp(item.context.sample_time_sec);
        auto result = options_.processor ? options_.processor(item.bytes, radar_params_, context) :
            run_signal_processing_pipeline(item.bytes, radar_params_, true, &context);
        if (!result.diagnostics.processing_ok)
            throw std::runtime_error("DSP failed: " + result.diagnostics.error_message);
        result.frame_number = item.context.frame_number;
        result.frame = item.context;
        result.frame.processing_completed_sec = RadarTime::wallSeconds();
        if (RadarTime::monotonicSeconds() - item.context.received_monotonic_sec > options_.max_frame_age_sec) {
            std::lock_guard<std::mutex> lock(status_mutex_); ++status_.stale_dropped; continue;
        }
        {
            std::lock_guard<std::mutex> lock(transmission_mutex_);
            if (transmission_queue_.size() >= options_.max_transmission_frames) {
                transmission_queue_.pop_front();
                std::lock_guard<std::mutex> stats(status_mutex_); ++status_.send_dropped;
            }
            transmission_queue_.push_back(std::move(result));
        }
        transmission_cond_.notify_one();
    }
}
void UdpListener::transmission_worker() {
    auto destination = endpoint(target_ip_, target_port_);
    while (running_) {
        ProcessingResult result;
        {
            std::unique_lock<std::mutex> lock(transmission_mutex_);
            transmission_cond_.wait(lock, [this] { return !running_ || !transmission_queue_.empty(); });
            if (!running_) break;
            result = std::move(transmission_queue_.front()); transmission_queue_.pop_front();
        }
        if (RadarTime::monotonicSeconds() - result.frame.received_monotonic_sec > options_.max_frame_age_sec) {
            std::lock_guard<std::mutex> lock(status_mutex_); ++status_.stale_dropped; continue;
        }
        // 格式化数据为字符串
        std::stringstream ss;
        const auto& diag = result.diagnostics;
        std::string processing_error = diag.error_message;
        std::replace(processing_error.begin(), processing_error.end(), ',', ';');
        std::replace(processing_error.begin(), processing_error.end(), '\n', ' ');
        std::replace(processing_error.begin(), processing_error.end(), '\r', ' ');
        // A metadata row is always sent, including frames with zero points.
        // Existing numeric point rows are kept unchanged for compatibility.
        ss << "META," << result.frame_number << ","
           << result.timestamp << ","
           << result.processing_time_ms << ","
           << diag.cfar_target_count << ","
           << diag.pre_doa_moving_count << ","
           << diag.pre_doa_static_count << ","
           << diag.doa_input_count << ","
           << diag.point_count << ","
           << (diag.ego_motion_compensation_enabled ? 1 : 0) << ","
           << (diag.odom_valid ? 1 : 0) << ","
           << (diag.odom_fresh ? 1 : 0) << ","
           << diag.odom_vx_mps << ","
           << diag.odom_wz_radps << ","
           // A Unix timestamp needs more than the stream default six digits;
           // otherwise all samples in several seconds collapse to 1.78395e9.
           << std::setprecision(17) << diag.odom_timestamp_sec << ","
           << std::setprecision(9)
           << diag.odom_age_sec << ","
           << diag.all_point_abs_velocity_median_mps << ","
           << diag.velocity_sample_count << ","
           << diag.moving_ratio << ","
           << (diag.compensation_warning ? 1 : 0) << ","
           << diag.velocity_resolution_mps << ","
           << diag.odom_seq << ","
           << diag.near_field_range_rejected_count << ","
           << diag.near_field_burst_rejected_count << ","
           << (diag.processing_ok ? 1 : 0) << ","
           << processing_error;
        for (const auto& point : result.point_cloud) {
            ss << "\n";
            ss << result.frame_number << ", "
               << point.x << ", "
               << point.y << ", "
               << point.relative_velocity << ", "
               << point.absolute_velocity << ", "
               << (point.is_moving ? "1" : "0") << ","
               << result.timestamp << ","
               << point.source_frame << ","
               << point.range_bin << ","
               << point.doppler_bin << ","
               << point.snr << ","
               << point.doa_peak_confidence;
        }
        const auto stats = status();
        const auto& f = result.frame;
        ss << "\nFRAME," << result.frame_number << ',' << f.session_id << ','
           << std::setprecision(17) << f.received_time_sec << ',' << f.sample_time_sec << ','
           << f.time_quality << ',' << f.queue_delay_ms << ',' << f.processing_completed_sec << ','
           << stats.packets.packets << ',' << stats.packets.short_packets << ','
           << stats.packets.discontinuities << ',' << stats.packets.out_of_order << ','
           << stats.packets.missing_packets << ',' << stats.queue_dropped << ','
           << stats.stale_dropped << ',' << stats.send_dropped << ',' << stats.queue_depth << ','
           << f.received_monotonic_sec;
        const std::string message = ss.str();

        if (message.size() > 65507) throw std::runtime_error("Point cloud exceeds UDP datagram limit");
        if (sendto(transmission_socket_, message.data(), static_cast<int>(message.size()), 0,
                   reinterpret_cast<sockaddr*>(&destination), sizeof(destination)) < 0)
            throw std::runtime_error(socketError("Transmit point cloud"));
    }
}
