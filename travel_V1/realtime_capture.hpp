#ifndef REALTIME_CAPTURE_HPP
#define REALTIME_CAPTURE_HPP
#include "signal_processing.hpp"
#include "dca_packet_assembler.hpp"
#include <atomic>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <functional>
#include <map>
#include <mutex>
#include <queue>
#include <thread>
#ifdef _WIN32
#include <winsock2.h>
#else
#include <sys/socket.h>
#include <unistd.h>
#define INVALID_SOCKET -1
#define SOCKET int
#define closesocket close
#endif

struct CaptureOptions {
    std::size_t max_queue_frames = 4;
    std::size_t max_transmission_frames = 4;
    double max_frame_age_sec = 0.5;
    std::string output_directory = "bin";
    // An injectable DSP boundary permits transport/lifecycle tests without hardware.
    std::function<ProcessingResult(const std::vector<char>&,
        const std::map<std::string,double>&, const SignalProcessingFrameContext&)> processor;
};
struct CaptureStatus {
    enum class State { Stopped, WaitingForData, Receiving, Failed } state = State::Stopped;
    std::string error;
    DcaPacketAssembler::Counters packets;
    std::uint64_t frames_received = 0, queue_dropped = 0, stale_dropped = 0, send_dropped = 0;
    std::size_t queue_depth = 0, transmission_depth = 0;
};
class UdpListener {
public:
    // frame_length retains the legacy count of int16 values; bytes = count * 2.
    UdpListener(const std::string&, std::queue<std::vector<char>>&, int frame_length,
                const std::string& address, int port, int buff_size,
                const std::map<std::string, double>& radar_config,
                const std::string& target_ip, int target_port,
                const CaptureOptions& options = CaptureOptions());
    ~UdpListener();
    UdpListener(const UdpListener&) = delete;
    UdpListener& operator=(const UdpListener&) = delete;
    void start(); // synchronous bind; throws on startup failure
    bool join(int timeout_ms); // false retains ownership; never detach a live worker
    void start_saving();
    void stop_saving();
    void stop_collector(); // idempotent stop request; destructor always joins
    const std::string& output_filename() const { return filename_; }
    CaptureStatus status() const;
    std::uint64_t session_id() const { return session_id_; }
    int listening_port() const { return port_; }
private:
    struct QueuedFrame { std::vector<char> bytes; RadarFrameContext context; };
    void run();
    void processing_worker();
    void transmission_worker();
    void fail(const std::string& error);
    void workerFinished();
    void closeSockets();
    std::thread receiver_, processing_, transmission_;
    std::atomic<bool> running_{false};
    bool started_ = false;
    std::size_t frame_bytes_;
    std::string address_, target_ip_;
    int port_, buff_size_, target_port_;
    std::map<std::string,double> radar_params_;
    CaptureOptions options_;
    SOCKET data_socket_ = INVALID_SOCKET, transmission_socket_ = INVALID_SOCKET;
    std::uint64_t session_id_ = 0;
    mutable std::mutex status_mutex_;
    CaptureStatus status_;
    mutable std::mutex queue_mutex_, transmission_mutex_, file_mutex_;
    std::condition_variable queue_cond_, transmission_cond_, finished_cond_;
    std::mutex finished_mutex_;
    int workers_ = 0;
    std::deque<QueuedFrame> frame_queue_;
    std::deque<ProcessingResult> transmission_queue_;
    std::ofstream file_, frame_index_;
    std::string filename_;
    bool saving_ = false;
};
#endif
