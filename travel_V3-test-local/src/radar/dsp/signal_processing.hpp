#ifndef SIGNAL_PROCESSING_HPP
#define SIGNAL_PROCESSING_HPP

#include <vector>
#include <map>
#include <complex>
#include <cstdint>
#include <string>

#include "fft.hpp"
#include "chassis_velocity.hpp"
#include "frame_context.hpp"

struct PointCloud {
    float x = 0.0f;
    float y = 0.0f;
    float relative_velocity = 0.0f;
    float absolute_velocity = 0.0f;
    bool is_moving = false;

    // Source RD cells are preserved through transport, aggregation and clustering.
    int source_frame = -1;
    int range_bin = -1;
    int doppler_bin = -1;
    float snr = 0.0f;
    float doa_peak_confidence = 0.0f; // 0..1 peak-to-competitor margin.
};

// Optional per-frame context used by deterministic offline replay.  Realtime
// callers omit it and continue reading /odom from shared memory.
struct SignalProcessingFrameContext {
    RadarFrameContext frame;
    int source_frame_number = 0;
    bool persist_outputs = true;
    // Offline comparison only; live processing uses the corrected DC bin.
    bool legacy_doppler_offset = false;
    bool use_chassis_override = false;
    ChassisVelocitySample chassis;
    bool has_odom_age = false;
    double odom_age_sec = 0.0;
    std::string radar_timestamp;
};

// Compact per-frame DSP/ego-motion audit data. It is transported to the GUI
// and written into a separate per-test diagnostics CSV.
struct ProcessingDiagnostics {
    bool processing_ok = true;
    std::string error_message;
    float processing_time_ms = 0.0f;
    int cfar_target_count = 0;
    int pre_doa_moving_count = 0;
    int pre_doa_static_count = 0;
    int doa_input_count = 0;
    int point_count = 0;
    int near_field_range_rejected_count = 0;
    int near_field_burst_rejected_count = 0;
    float velocity_resolution_mps = 0.0f;
    bool ego_motion_compensation_enabled = false;
    bool odom_valid = false;
    bool odom_fresh = false;
    float odom_vx_mps = 0.0f;
    float odom_wz_radps = 0.0f;
    double odom_timestamp_sec = 0.0;
    double odom_age_sec = 0.0;
    std::uint32_t odom_seq = 0;
    float all_point_abs_velocity_median_mps = 0.0f;
    int velocity_sample_count = 0;
    float moving_ratio = 0.0f;
    bool compensation_warning = false;
};

struct ProcessingResult {
    RadarFrameContext frame;
    int frame_number = 0;
    std::string timestamp;
    std::vector<PointCloud> point_cloud;
    float processing_time_ms = 0.0f;
    ProcessingDiagnostics diagnostics;
};


/**
 * @brief 运行完整的信号处理流程
 * @param raw_frame_data 从采集线程传来的一帧原始ADC数据
 * @param radar_params 雷达配置参数
 * @return 处理结果，包括帧号、时间戳和点云数据
 */
ProcessingResult run_signal_processing_pipeline(const std::vector<char>& raw_frame_data,
                                                const std::map<std::string, double>& radar_params,
                                                bool enable_ego_motion_compensation = true,
                                                const SignalProcessingFrameContext* frame_context = nullptr);

FourDArray convert_raw_to_4darray(const std::vector<char>& raw_data, const RadarParams& params);

/**
 * @brief 重置点云输出会话（关闭旧文件并将帧计数清零）。
 *        用于在同一进程内“停止-再次开始”采集时，避免继续写入旧文件并延续旧帧号。
 */
void reset_signal_processing_session();

#endif // SIGNAL_PROCESSING_HPP
