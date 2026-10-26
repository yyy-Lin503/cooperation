#include <iostream>
#include <fstream>
#include <stdexcept>
#include <cstdint>
#include <algorithm>
#include <chrono>
#include <iomanip>
#include <cmath>
#include <numeric>  
#include <cstdlib>
#include <experimental/filesystem>

#include "signal_processing.hpp"
#include "chassis_velocity.hpp"
#include "radar_mount.hpp"
#include "radar_runtime_config.hpp"
#include "DoA.hpp"
#include "CFAR.hpp"
#include "radar_frame_layout.hpp"
#include "radar_motion.hpp"
#include <memory>
#include <mutex>
#ifndef RADAR_ENABLE_CUDA
#define RADAR_ENABLE_CUDA 1
#endif
#if RADAR_ENABLE_CUDA
#include "fft_cuda_optimized_v2.hpp"  
#include <cuda_runtime.h>  
#endif


// 全局设置：选择使用CPU还是GPU FFT
static bool USE_CUDA_FFT = RADAR_ENABLE_CUDA != 0;
const bool ENABLE_DEBUG = [] {
    const char* level = std::getenv("RADAR_LOG_LEVEL");
    if (!level) return RadarRuntimeConfig::kDebugLogging;
    return std::string(level) == "debug" || std::string(level) == "trace" ||
           std::string(level) == "2";
}();
const bool ENABLE_CONSOLE_OUTPUT = ENABLE_DEBUG;  // Summary mode suppresses per-stage/per-target logs.

// 全局CUDA FFT对象，减少重复初始化开销
#if RADAR_ENABLE_CUDA
static std::unique_ptr<OptimizedCudaFFTV2> global_range_fft;
static std::unique_ptr<OptimizedCudaFFTV2> global_doppler_fft;
#endif
static bool cuda_checked = false;  // 标记是否已检查CUDA可用性

// —— 点云输出会话状态（文件作用域，支持跨会话复位） ——
namespace {
    static std::recursive_mutex g_pipeline_mutex;
    static std::ofstream g_point_cloud_file;  // 当前会话的点云输出文件
    static int g_frame_counter = 0;           // 帧计数，每次新会话从 1 开始
    static bool g_file_initialized = false;   // 是否已创建文件并写入表头
    static std::string g_session_tag;
    static ChassisVelocityReader g_chassis_velocity_reader;
    static std::vector<float> g_processing_times_ms;
    static std::uint64_t g_processing_deadline_misses = 0;

    constexpr float kFallbackMovingVelocityMps = 0.12f;

    std::string makeSessionTag(std::chrono::system_clock::time_point now)
    {
        const auto in_time_t = std::chrono::system_clock::to_time_t(now);
        const auto milliseconds =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch()) % 1000;
        std::ostringstream out;
        out << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M_%S")
            << "_" << std::setfill('0') << std::setw(3)
            << milliseconds.count();
        return out.str();
    }

    std::string uniqueOutputPath(const std::string& directory,
                                 const std::string& stem,
                                 const std::string& extension)
    {
        namespace fs = std::experimental::filesystem;
        try { fs::create_directories(directory); } catch (...) {}
        std::string path = directory + "/" + stem + extension;
        int suffix = 1;
        while (fs::exists(path)) {
            path = directory + "/" + stem + "_" +
                   std::to_string(suffix++) + extension;
        }
        return path;
    }
    float compensatedRadialVelocity(float relative_velocity,
                                    float azimuth_deg,
                                    const ChassisVelocitySample& chassis)
    {
        if (!chassis.valid) {
            return relative_velocity;
        }

        return RadarMotion::radialVelocity(relative_velocity, azimuth_deg, chassis);
    }
}

// 对外提供：重置点云会话（关闭旧文件并清零计数）
void reset_signal_processing_session() {
    std::lock_guard<std::recursive_mutex> guard(g_pipeline_mutex);
    try {
        if (g_point_cloud_file.is_open()) {
            g_point_cloud_file.flush();
            g_point_cloud_file.close();
        }
    } catch (...) {
        // 忽略关闭时的异常，继续复位
    }
    g_frame_counter = 0;
    g_file_initialized = false;
    g_session_tag.clear();
    g_processing_times_ms.clear();
    g_processing_deadline_misses = 0;
}

bool isCudaAvailable() {
#if RADAR_ENABLE_CUDA
    int deviceCount = 0;
    cudaError_t error = cudaGetDeviceCount(&deviceCount);
    if (error != cudaSuccess || deviceCount == 0) {
        std::cout << "[信号处理] CUDA不可用: " << cudaGetErrorString(error) << " (设备数量: " << deviceCount << ")" << std::endl;
        return false;
    }
    return true;
#else
    return false;
#endif
}

extern std::chrono::system_clock::time_point capture_start_time;

/**
 * @brief 将采集线程传来的原始char向量数据，转换为信号处理所需的FourDArray复数矩阵
 * @param raw_data 原始ADC数据
 * @param params 雷达参数
 * @return FourDArray 格式的雷达数据
 */

FourDArray convert_raw_to_4darray(const std::vector<char>& raw_data, const RadarParams& params) {
    if (raw_data.size() != RadarFrameLayout::frameBytes(params))
        throw std::invalid_argument("ADC frame byte count does not match complex-IQ layout");
    FourDArray frameData(params.num_ADC, params.num_chirp, params.num_RX, params.num_TX);
    std::vector<int16_t> samples(raw_data.size() / sizeof(int16_t));
    std::memcpy(samples.data(), raw_data.data(), raw_data.size());
    const int16_t* adc_data_ptr = samples.data();
    int ptr_idx = 0;

    // 数据格式为 I1 Q1 I2 Q2 ... , TI的文档中说明了如何重新组织
    // 这里的循环顺序是根据硬件输出格式来的
    for (int chirp = 0; chirp < params.num_chirp; chirp++) {
        for (int tx = 0; tx < params.num_TX; tx++) {
            for (int rx = 0; rx < params.num_RX; rx++) {
                for (int adc = 0; adc < params.num_ADC; adc += 2) {
                    // 读取4个连续的int16：A0, A1, A2, A3
                    // A0=I_sampleN, A1=I_sampleN+1, A2=Q_sampleN, A3=Q_sampleN+1
                    int16_t A0 = adc_data_ptr[ptr_idx++];
                    int16_t A1 = adc_data_ptr[ptr_idx++];
                    int16_t A2 = adc_data_ptr[ptr_idx++];
                    int16_t A3 = adc_data_ptr[ptr_idx++];

                    // 组合成复数
                    frameData(adc,   chirp, rx, tx) = std::complex<float>(static_cast<float>(A0), static_cast<float>(A2));
                    frameData(adc+1, chirp, rx, tx) = std::complex<float>(static_cast<float>(A1), static_cast<float>(A3));
                }
            }
        }
    }
    return frameData;
}

/**
 * @brief 运行完整的信号处理流程
 * @param raw_frame_data 从采集线程传来的一帧原始ADC数据
 * @param radar_params_map 从主程序传来的雷达配置
 */
ProcessingResult run_signal_processing_pipeline(const std::vector<char>& raw_frame_data,
                                                const std::map<std::string, double>& radar_params_map,
                                                bool enable_ego_motion_compensation,
                                                const SignalProcessingFrameContext* frame_context) {

    std::lock_guard<std::recursive_mutex> guard(g_pipeline_mutex);
    try {
        // 仅在第一次调用时初始化文件
        const bool persist_outputs = !frame_context || frame_context->persist_outputs;
        if (!g_file_initialized && persist_outputs) {
            auto now = std::chrono::system_clock::now();
            g_session_tag = makeSessionTag(now);
            const std::string point_cloud_path = uniqueOutputPath(
                "txt", "point_cloud_" + g_session_tag, ".txt");
            g_point_cloud_file.open(point_cloud_path,
                                    std::ios::out | std::ios::trunc);
            if (g_point_cloud_file.is_open()) {
                g_point_cloud_file
                    << "Frame,X,Y,RelativeVelocity,AbsoluteVelocity,IsMoving,Timestamp,"
                       "SourceFrame,RangeBin,DopplerBin,SNR,DOAPeakConfidence\n";
                g_file_initialized = true;
            } else {
                std::cerr << "[信号处理] 错误: 无法创建点云文件!" << std::endl;
            }
        }

        g_frame_counter++;
        if (frame_context && frame_context->source_frame_number > 0)
            g_frame_counter = frame_context->source_frame_number;
        auto t_start_total = std::chrono::high_resolution_clock::now();
        auto t0 = t_start_total;
        auto t1 = t_start_total;
        if (ENABLE_CONSOLE_OUTPUT) {
            std::cout << "\n[信号处理] 开始处理新一帧数据..." << std::endl;
        }

        // 1. 设置雷达参数
        const RadarParams params = RadarFrameLayout::paramsFromMap(radar_params_map);
        if (params.num_ADC < 16 || params.num_RX * params.num_TX < 4)
            throw std::invalid_argument("DSP requires at least 16 ADC samples and four virtual antennas");

        if (!cuda_checked) {
            USE_CUDA_FFT = isCudaAvailable();
            cuda_checked = true;
        }

        // 仅在第一次调用时初始化CUDA FFT对象
#if RADAR_ENABLE_CUDA
        if (USE_CUDA_FFT &&
            (!global_range_fft || !global_doppler_fft ||
             global_range_fft->get_fft_size() != params.num_ADC ||
             global_range_fft->get_max_batch_size() != params.num_TX * params.num_chirp * params.num_RX ||
             global_doppler_fft->get_fft_size() != params.num_chirp ||
             global_doppler_fft->get_max_batch_size() != params.num_TX * params.num_RX * params.num_ADC)) {
            global_range_fft.reset(); global_doppler_fft.reset();
            try {
                // 距离FFT初始化
                int total_range_ffts = params.num_TX * params.num_chirp * params.num_RX;
                global_range_fft = std::make_unique<OptimizedCudaFFTV2>(params.num_ADC, total_range_ffts, ENABLE_DEBUG);
                
                // 多普勒FFT初始化
                int total_doppler_ffts = params.num_TX * params.num_RX * params.num_ADC;
                global_doppler_fft = std::make_unique<OptimizedCudaFFTV2>(params.num_chirp, total_doppler_ffts, ENABLE_DEBUG);
            } catch (const std::exception& e) {
                std::cerr << "[信号处理] CUDA初始化失败，回退到CPU FFT: " << e.what() << std::endl;
                USE_CUDA_FFT = false;
                // 清理可能的半初始化对象
                if (global_range_fft) {
                    global_range_fft.reset();
                }
                if (global_doppler_fft) {
                    global_doppler_fft.reset();
                }
            }
        }

#endif
        // 2. 数据重构
        t0 = std::chrono::high_resolution_clock::now();
        FourDArray frame0 = convert_raw_to_4darray(raw_frame_data, params);
        t1 = std::chrono::high_resolution_clock::now();
        if (ENABLE_CONSOLE_OUTPUT) {
            std::cout << "[信号处理]   - 数据重构耗时: " << std::chrono::duration<float, std::milli>(t1 - t0).count() << " ms" << std::endl;
        }

        
        // 物理参数
        const float c = 3.0e8;
        // 从配置参数获取载频与采样率（单位换算：fc 为 GHz，fs 为 ksps）
        const float fc = static_cast<float>(radar_params_map.at("fc") * 1e9);   // Hz
        const float fs = static_cast<float>(radar_params_map.at("fs") * 1e3);   // samples/s
        // 斜率与时序：从 cfg 参数获取（单位换算：slope MHz/us → Hz/s；时间 us → s）
        const float slope_MHz_us = static_cast<float>(radar_params_map.at("slopeConst"));
        const float Slop = slope_MHz_us * 1e12f; // Hz/s
        const float idle_us = static_cast<float>(radar_params_map.at("idleTime"));
        const float ramp_us = static_cast<float>(radar_params_map.at("rampEndTime"));
        const float Tc = (idle_us + ramp_us) * 1e-6f;
        
        const float lambda = c / fc;
        const float Bw = params.num_ADC / fs * Slop;
        const float range_res = c / (2 * Bw);
        const float Tframe = Tc * params.num_TX * params.num_chirp;
        const float velocity_res = lambda / (2.0f * Tframe);
        const ChassisVelocitySample chassis_velocity =
            enable_ego_motion_compensation
                ? ((frame_context && frame_context->use_chassis_override)
                       ? frame_context->chassis
                       : g_chassis_velocity_reader.read())
                : ChassisVelocitySample();
        ProcessingDiagnostics frame_diagnostics;
        frame_diagnostics.velocity_resolution_mps = velocity_res;
        frame_diagnostics.ego_motion_compensation_enabled =
            enable_ego_motion_compensation;
        frame_diagnostics.odom_valid = chassis_velocity.valid;
        frame_diagnostics.odom_fresh = chassis_velocity.fresh;
        frame_diagnostics.odom_vx_mps = chassis_velocity.vx_mps;
        frame_diagnostics.odom_wz_radps = chassis_velocity.wz_radps;
        frame_diagnostics.odom_timestamp_sec = chassis_velocity.timestamp_sec;
        frame_diagnostics.odom_seq = chassis_velocity.seq;
        frame_diagnostics.odom_age_sec =
            frame_context && frame_context->use_chassis_override &&
                    frame_context->has_odom_age
                ? frame_context->odom_age_sec
                : (chassis_velocity.has_timestamp
                       ? std::abs(std::chrono::duration<double>(
                             std::chrono::system_clock::now().time_since_epoch()).count() -
                             chassis_velocity.timestamp_sec)
                       : 0.0);
        if (ENABLE_CONSOLE_OUTPUT && g_frame_counter % 20 == 1) {
            if (chassis_velocity.valid) {
                std::cout << "[运动补偿] 底盘速度 vx=" << chassis_velocity.vx_mps
                          << " m/s, wz=" << chassis_velocity.wz_radps
                          << " rad/s，已用于 Doppler 径向速度补偿" << std::endl;
            } else {
                std::cout << "[运动补偿] 未读取到底盘共享内存，回退到原始 Doppler 速度" << std::endl;
            }
        }

        // 距离FFT
        t0 = std::chrono::high_resolution_clock::now();
        auto win_range = WindowFunctions::kaiser(params.num_ADC, 10.0f);
        
        // 根据全局设置选择FFT实现
#if RADAR_ENABLE_CUDA
        if (USE_CUDA_FFT) {
            if (ENABLE_CONSOLE_OUTPUT) {
                std::cout << "[信号处理] 使用高度优化CUDA FFT V2进行距离FFT..." << std::endl;
            }
            
            // 按距离维打包批量 FFT 输入。
            std::vector<std::vector<std::complex<float>>> batch_data;
            batch_data.reserve(params.num_TX * params.num_chirp * params.num_RX);
            
            // 预分配向量以避免重复分配
            for (int tx = 0; tx < params.num_TX; tx++) {
                for (int chirp = 0; chirp < params.num_chirp; chirp++) {
                    for (int rx = 0; rx < params.num_RX; rx++) {     
                        std::vector<std::complex<float>> adc_data(params.num_ADC);
                        for (int adc = 0; adc < params.num_ADC; adc++) adc_data[adc] = frame0(adc, chirp, rx, tx);
                        batch_data.push_back(std::move(adc_data));
                    }
                }
            }
            
            global_range_fft->forward_batch_with_window_and_fftshift(batch_data, win_range, false);
            
            // 将结果写回frame0
            int batch_idx = 0;
            for (int tx = 0; tx < params.num_TX; tx++) {
                for (int chirp = 0; chirp < params.num_chirp; chirp++) {
                    for (int rx = 0; rx < params.num_RX; rx++) {
                        for (int adc = 0; adc < params.num_ADC; adc++) {
                            frame0(adc, chirp, rx, tx) = batch_data[batch_idx][adc];
                        }
                        batch_idx++;
                    }
                }
            }
        } else
#endif
        {
            if (ENABLE_CONSOLE_OUTPUT) {
                std::cout << "[信号处理] 使用CPU FFT进行距离FFT..." << std::endl;
            }
            OptimizedFFT range_fft(params.num_ADC);
            for (int tx = 0; tx < params.num_TX; tx++) {
                for (int chirp = 0; chirp < params.num_chirp; chirp++) {
                    for (int rx = 0; rx < params.num_RX; rx++) {     
                        std::vector<std::complex<float>> adc_data(params.num_ADC);
                        for (int adc = 0; adc < params.num_ADC; adc++) adc_data[adc] = frame0(adc, chirp, rx, tx);
                        WindowFunctions::apply_window(adc_data, win_range);            
                        range_fft.forward(adc_data);  
                        for (int adc = 0; adc < params.num_ADC; adc++) frame0(adc, chirp, rx, tx) = adc_data[adc];
                    }
                }
            }
        }
        t1 = std::chrono::high_resolution_clock::now();
        if (ENABLE_CONSOLE_OUTPUT) {
            std::cout << "[信号处理]   - 距离FFT耗时: " << std::chrono::duration<float, std::milli>(t1 - t0).count() << " ms" << std::endl;
        }

        // 去除直流分量干扰
        for (int tx = 0; tx < params.num_TX; ++tx) {
            for (int chirp = 0; chirp < params.num_chirp; ++chirp) {
                for (int rx = 0; rx < params.num_RX; ++rx) {
                    for (int adc = 0; adc < 10; ++adc) frame0(adc, chirp, rx, tx) = {0.0f, 0.0f};
                    for (int adc = params.num_ADC - 11; adc < params.num_ADC; ++adc) frame0(adc, chirp, rx, tx) = {0.0f, 0.0f};
                }
            }
        }

        // 多普勒FFT
        t0 = std::chrono::high_resolution_clock::now();
        auto doppler_window = WindowFunctions::kaiser(params.num_chirp, 8.0f);
        
#if RADAR_ENABLE_CUDA
        if (USE_CUDA_FFT) {
            if (ENABLE_CONSOLE_OUTPUT) {
                std::cout << "[信号处理] 使用高度优化CUDA FFT V2进行多普勒FFT..." << std::endl;
            }
            
            // 按多普勒维打包批量 FFT 输入。
            std::vector<std::vector<std::complex<float>>> batch_data;
            batch_data.reserve(params.num_TX * params.num_RX * params.num_ADC);
            
            for (int tx = 0; tx < params.num_TX; tx++) {
                for (int rx = 0; rx < params.num_RX; rx++) {
                    for (int adc = 0; adc < params.num_ADC; adc++) {
                        std::vector<std::complex<float>> chirp_data;
                        chirp_data.reserve(params.num_chirp);
                        for (int chirp = 0; chirp < params.num_chirp; chirp++) {
                            chirp_data.emplace_back(frame0(adc, chirp, rx, tx));
                        }
                        batch_data.emplace_back(std::move(chirp_data));
                    }
                }
            }
            
            // 执行一体化批量窗函数+FFT+FFTShift处理
            global_doppler_fft->forward_batch_with_window_and_fftshift(batch_data, doppler_window, true);
            
            // 将结果写回frame0
            int batch_idx = 0;
            for (int tx = 0; tx < params.num_TX; tx++) {
                for (int rx = 0; rx < params.num_RX; rx++) {
                    for (int adc = 0; adc < params.num_ADC; adc++) {
                        for (int chirp = 0; chirp < params.num_chirp; chirp++) {
                            frame0(adc, chirp, rx, tx) = batch_data[batch_idx][chirp];
                        }
                        batch_idx++;
                    }
                }
            }
        } else
#endif
        {
            if (ENABLE_CONSOLE_OUTPUT) {
                std::cout << "[信号处理] 使用CPU FFT进行多普勒FFT..." << std::endl;
            }
            OptimizedFFT doppler_fft(params.num_chirp);
            for (int tx = 0; tx < params.num_TX; tx++) {
                for (int rx = 0; rx < params.num_RX; rx++) {
                    for (int adc = 0; adc < params.num_ADC; adc++) {
                        std::vector<std::complex<float>> chirp_data(params.num_chirp);
                        for (int chirp = 0; chirp < params.num_chirp; chirp++) chirp_data[chirp] = frame0(adc, chirp, rx, tx);
                        WindowFunctions::apply_window(chirp_data, doppler_window);
                        doppler_fft.forward(chirp_data);
                        doppler_fft.fftshift(chirp_data);
                        for (int chirp = 0; chirp < params.num_chirp; chirp++) frame0(adc, chirp, rx, tx) = chirp_data[chirp];
                    }
                }
            }
        }
        t1 = std::chrono::high_resolution_clock::now();
        if (ENABLE_CONSOLE_OUTPUT) {
            std::cout << "[信号处理]   - 多普勒FFT耗时: " << std::chrono::duration<float, std::milli>(t1 - t0).count() << " ms" << std::endl;
        }

        // RDM生成
        t0 = std::chrono::high_resolution_clock::now();
        std::vector<float> RDM_sum = RadarProcessing::computeRDM(frame0, params.num_ADC, params.num_chirp, params.num_RX, params.num_TX);
        t1 = std::chrono::high_resolution_clock::now();
        if (ENABLE_CONSOLE_OUTPUT) {
            std::cout << "[信号处理]   - RDM生成耗时: " << std::chrono::duration<float, std::milli>(t1 - t0).count() << " ms" << std::endl;
        }
        
        // 仅在第20帧保存RDM_sum数据
        if (g_frame_counter == 20 && persist_outputs) {
            std::cout << "[信号处理] 保存RDM_sum数据到文件 (帧 " << g_frame_counter << ")" << std::endl;
            
            // 保存RDM_sum为文本文件 (矩阵格式：行=ADC样本，列=脉冲)
            const std::string rdm_sum_filename = uniqueOutputPath(
                "txt", "rdm_sum_" + g_session_tag + "_frame_" +
                           std::to_string(g_frame_counter),
                ".txt");
            std::ofstream rdm_sum_file(rdm_sum_filename,
                                       std::ios::out | std::ios::trunc);
            if (rdm_sum_file.is_open()) {
                for (int adc = 0; adc < params.num_ADC; ++adc) {
                    for (int chirp = 0; chirp < params.num_chirp; ++chirp) {
                        int index = adc + chirp * params.num_ADC;
                        rdm_sum_file << RDM_sum[index];
                        if (chirp < params.num_chirp - 1) {
                            rdm_sum_file << ",";
                        }
                    }
                    rdm_sum_file << "\n";
                }
                rdm_sum_file.close();
                if (ENABLE_CONSOLE_OUTPUT) {
                    std::cout << "[信号处理] RDM_sum数据已保存到: " << rdm_sum_filename << " (格式: " << params.num_ADC << "x" << params.num_chirp << " 矩阵)" << std::endl;
                }
            } else {
                std::cerr << "[信号处理] 错误: 无法创建RDM_sum文件!" << std::endl;
            }
        }
        
        // CFAR检测
        t0 = std::chrono::high_resolution_clock::now();
        RadarProcessing::CFARResult cfar_result = RadarProcessing::osca_cfar(RDM_sum, params.num_ADC, params.num_chirp);
        frame_diagnostics.cfar_target_count = cfar_result.num_targets;
        t1 = std::chrono::high_resolution_clock::now();
        if (ENABLE_CONSOLE_OUTPUT) {
            std::cout << "[信号处理]   - CFAR检测耗时: " << std::chrono::duration<float, std::milli>(t1 - t0).count() << " ms" << std::endl;
            std::cout << "[信号处理] CFAR检测目标个数: " << cfar_result.num_targets << std::endl;
        }

        // 目标筛选与DOA估计
        t0 = std::chrono::high_resolution_clock::now();
        const auto rangeMetersFromIndex = [&](int range_idx) {
            return range_res * static_cast<float>(
                params.num_ADC - range_idx - 1);
        };

        // Near-field CFAR cells must not influence either the Doppler-centre
        // estimate or DOA selection.  This is shared by realtime and offline
        // processing because both call this same pipeline.
        std::vector<int> usable_doppler_indices;
        usable_doppler_indices.reserve(cfar_result.num_targets);
        for (int i = 0; i < cfar_result.num_targets; ++i) {
            if (rangeMetersFromIndex(cfar_result.range_indices[i]) >=
                RadarRuntimeConfig::kMinimumUsableRangeM) {
                usable_doppler_indices.push_back(cfar_result.doppler_indices[i]);
            }
        }
        const bool legacy_doppler_offset = frame_context && frame_context->legacy_doppler_offset;
        const float doppler_zero_bin = RadarFrameLayout::dopplerZeroBin(
            params.num_chirp, legacy_doppler_offset);
        int esti_dopp_center_idx = static_cast<int>(doppler_zero_bin);
        if (!usable_doppler_indices.empty()) {
            esti_dopp_center_idx = mode(usable_doppler_indices);
        }

        struct TargetInfo {//检查
            int range_idx;
            int doppler_idx;
            float snr;
            bool is_moving;
        };
        
        std::vector<TargetInfo> moving_targets, static_targets;
        for (int i = 0; i < cfar_result.num_targets; ++i) {
            const float target_range_m =
                rangeMetersFromIndex(cfar_result.range_indices[i]);
            if (target_range_m < RadarRuntimeConfig::kMinimumUsableRangeM) {
                frame_diagnostics.near_field_range_rejected_count++;
                continue;
            }
            bool is_moving = std::abs(cfar_result.doppler_indices[i] - esti_dopp_center_idx) >= 3;
            TargetInfo info{cfar_result.range_indices[i], cfar_result.doppler_indices[i], cfar_result.snrs[i], is_moving};
            if (ENABLE_CONSOLE_OUTPUT) {
                std::cout << "[TARGET] range_idx=" << info.range_idx
                          << ", doppler_idx=" << info.doppler_idx
                          << ", snr=" << info.snr
                          << ", is_moving=" << (info.is_moving ? 1 : 0)
                          << std::endl;
            } 
            if (is_moving) {
                moving_targets.push_back(info);
            } else {
                static_targets.push_back(info);
            }
        }
        frame_diagnostics.pre_doa_moving_count =
            static_cast<int>(moving_targets.size());
        frame_diagnostics.pre_doa_static_count =
            static_cast<int>(static_targets.size());

        std::sort(static_targets.begin(), static_targets.end(), [](const TargetInfo& a, const TargetInfo& b) { return a.snr > b.snr; });

        std::vector<TargetInfo> targets_for_doa;
        for (const auto& target : moving_targets) targets_for_doa.push_back(target);
        
        int remaining_slots = 50 - targets_for_doa.size(); 
        if (remaining_slots > 0) {
            int static_to_add = std::min(remaining_slots, (int)static_targets.size());
            targets_for_doa.insert(targets_for_doa.end(), static_targets.begin(), static_targets.begin() + static_to_add);
        }
        
        if (ENABLE_CONSOLE_OUTPUT) {
            std::cout << "[信号处理] 选择 " << targets_for_doa.size() << " 个目标进行DOA估计" << std::endl;
        }

        // DOA估计
        t0 = std::chrono::high_resolution_clock::now();
        std::vector<PointCloud> current_frame_point_cloud; // 存储当前帧的点云

        for(const auto& target : targets_for_doa){
            std::vector<std::complex<float>> sig;
            for (int tx = 0; tx < params.num_TX; ++tx) {
                for (int rx = 0; rx < params.num_RX; ++rx) {  
                    sig.push_back(frame0(target.range_idx, target.doppler_idx, rx, tx));
                }
            }
            
            DoA::AzimuthOptions doa_options;
            doa_options.num_tx = params.num_TX;
            doa_options.rx_per_tx = params.num_RX;
            doa_options.doppler_bins = params.num_chirp;
            // Physical FFT DC is independent of the old WLW speed-zero convention.
            doa_options.doppler_offset_bins = target.doppler_idx - params.num_chirp/2;
            const auto doa = DoA::estimate_azimuth(sig, doa_options);
            const float doa_peak_confidence = doa.peak_confidence;
            if (doa.valid) {
                float az_doa_out = doa.azimuth_deg;
                float r = rangeMetersFromIndex(target.range_idx); 
                float x = r * std::sin(az_doa_out * M_PI / 180.0f);
                float y = r * std::cos(az_doa_out * M_PI / 180.0f);
                
                // 计算前向和反向的坐标以验证距离映射是否正确

                // 计算相对速度和经过底盘 odom 修正后的径向速度。
                // relative_velocity: 雷达直接测到的相对径向速度。
                // absolute_velocity: 若共享内存可用，减去小车前进速度在该方位角上的径向投影；
                //                    否则保留原始 Doppler 相对速度作为回退值。
                float v_relative_idx = static_cast<float>(target.doppler_idx);
                float relative_velocity = (v_relative_idx - doppler_zero_bin) * velocity_res;
                float absolute_velocity = 0.0f;
                if (chassis_velocity.valid) {
                    absolute_velocity = compensatedRadialVelocity(relative_velocity,
                                                                  az_doa_out,
                                                                  chassis_velocity);
                } else {
                    // Fallback to the original Doppler-center estimate when /odom shared memory is not ready.
                    float v_dog = static_cast<float>(esti_dopp_center_idx) - doppler_zero_bin;
                    float v_real_idx = v_relative_idx - v_dog * std::cos(az_doa_out * M_PI / 180.0f);
                    absolute_velocity = (v_real_idx - doppler_zero_bin) * velocity_res;
                }
                const float moving_velocity_threshold =
                    std::max(kFallbackMovingVelocityMps, 2.0f * velocity_res);
                bool compensated_is_moving = chassis_velocity.valid
                    ? (std::abs(absolute_velocity) >= moving_velocity_threshold)
                    : target.is_moving;

                // 将计算结果存入点云列表
                PointCloud point;
                point.x = x;
                point.y = y;
                point.relative_velocity = relative_velocity;
                point.absolute_velocity = absolute_velocity;
                point.is_moving = compensated_is_moving;
                point.source_frame = g_frame_counter;
                point.range_bin = target.range_idx;
                point.doppler_bin = target.doppler_idx;
                point.snr = target.snr;
                point.doa_peak_confidence = doa_peak_confidence;
                current_frame_point_cloud.push_back(point);
            }
        }

        // A 1.8 m hard cut alone cannot reject the recorded pathological burst
        // centred around 1.94-1.96 m.  If one frame contains many implausibly
        // fast near-field returns, quarantine only the <=2.2 m band for that
        // frame.  Ordinary sparse 3-4 point people never reach this trigger.
        if (RadarRuntimeConfig::kEnableNearFieldBurstGuard) {
            const int fast_near_field_points = static_cast<int>(std::count_if(
                current_frame_point_cloud.begin(),
                current_frame_point_cloud.end(),
                [](const PointCloud& point) {
                    return std::hypot(point.x, point.y) <=
                               RadarRuntimeConfig::kNearFieldBurstMaxRangeM &&
                           std::abs(point.absolute_velocity) >=
                               RadarRuntimeConfig::kNearFieldBurstMinAbsVelocityMps;
                }));
            if (fast_near_field_points >=
                RadarRuntimeConfig::kNearFieldBurstMinFastPointsPerFrame) {
                const auto before = current_frame_point_cloud.size();
                current_frame_point_cloud.erase(
                    std::remove_if(current_frame_point_cloud.begin(),
                                   current_frame_point_cloud.end(),
                                   [](const PointCloud& point) {
                                       return std::hypot(point.x, point.y) <=
                                           RadarRuntimeConfig::kNearFieldBurstMaxRangeM;
                                   }),
                    current_frame_point_cloud.end());
                frame_diagnostics.near_field_burst_rejected_count =
                    static_cast<int>(before - current_frame_point_cloud.size());
                std::cout << "[NEAR-FIELD-GUARD] frame=" << g_frame_counter
                          << " fast_points=" << fast_near_field_points
                          << " rejected_points="
                          << frame_diagnostics.near_field_burst_rejected_count
                          << std::endl;
            }
        }
        t1 = std::chrono::high_resolution_clock::now();
        if (ENABLE_CONSOLE_OUTPUT) {
            std::cout << "[信号处理]   - DOA估计耗时: " << std::chrono::duration<float, std::milli>(t1 - t0).count() << " ms" << std::endl;
        }
        frame_diagnostics.doa_input_count =
            static_cast<int>(targets_for_doa.size());

        // Record the median for every frame.  This is intentionally named
        // all_point_abs_velocity_median in the diagnostics because no true
        // static-point filter is applied here; in a pedestrian-dominated scene
        // it is only a warning signal, not proof that compensation is wrong.
        frame_diagnostics.point_count =
            static_cast<int>(current_frame_point_cloud.size());
        if (!current_frame_point_cloud.empty()) {
            std::vector<float> residuals;
            residuals.reserve(current_frame_point_cloud.size());
            std::size_t moving_count = 0;
            for (const auto& point : current_frame_point_cloud) {
                residuals.push_back(std::abs(point.absolute_velocity));
                if (point.is_moving) moving_count++;
            }
            const size_t mid = residuals.size() / 2;
            std::nth_element(residuals.begin(), residuals.begin() + mid, residuals.end());
            const float median_residual = residuals[mid];
            const float warning_threshold = std::max(0.35f, 2.5f * velocity_res);
            frame_diagnostics.all_point_abs_velocity_median_mps = median_residual;
            frame_diagnostics.velocity_sample_count =
                static_cast<int>(residuals.size());
            frame_diagnostics.moving_ratio =
                static_cast<float>(moving_count) /
                static_cast<float>(residuals.size());
            frame_diagnostics.compensation_warning =
                chassis_velocity.valid && median_residual > warning_threshold;

            if (chassis_velocity.valid && g_frame_counter % 10 == 0 &&
                current_frame_point_cloud.size() >= 5) {
                std::cout << "[运动补偿诊断] frame=" << g_frame_counter
                          << ", static_doppler_median=" << median_residual << " m/s"
                          << ", samples=" << residuals.size()
                          << ", moving_ratio=" << frame_diagnostics.moving_ratio
                          << ", odom_vx=" << chassis_velocity.vx_mps << " m/s"
                          << ", odom_wz=" << chassis_velocity.wz_radps << " rad/s"
                          << ", odom_age=" << frame_diagnostics.odom_age_sec << " s"
                          << ", warn_threshold=" << warning_threshold << " m/s"
                          << (frame_diagnostics.compensation_warning
                                  ? " [WARN: check timestamp/sign/extrinsics]"
                                  : "")
                          << std::endl;
            }
        }
        
        // 将当前帧的点云数据写入文件
    if (g_point_cloud_file.is_open()) {
            // 计算帧时间戳（基于捕获开始时间和帧周期）
            double frame_period_ms = radar_params_map.at("framePeriodicity");
            std::chrono::milliseconds frame_offset(static_cast<long long>((g_frame_counter - 1) * frame_period_ms));
            auto frame_time = capture_start_time + frame_offset;
            auto in_time_t = std::chrono::system_clock::to_time_t(frame_time);
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(frame_time.time_since_epoch()) % 1000;
            std::stringstream ss;
            ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %H:%M:%S") << "." << std::setfill('0') << std::setw(3) << ms.count();
            std::string timestamp =
                frame_context && !frame_context->radar_timestamp.empty()
                    ? frame_context->radar_timestamp
                    : ss.str();
            
            // 在终端输出帧头信息
            if (ENABLE_CONSOLE_OUTPUT) {
                std::cout << "[帧头] 帧号: " << g_frame_counter << ", 时间戳: " << timestamp << std::endl;
            }
            
            bool is_first_point = true;
            for (const auto& point : current_frame_point_cloud) {
                std::string timestamp_str = is_first_point ? timestamp : "";
                g_point_cloud_file << g_frame_counter << ", "
                                   << point.x << ", "
                                   << point.y << ", "
                                   << point.relative_velocity << ", "
                                   << point.absolute_velocity << ", "
                                   << (point.is_moving ? "1" : "0") << ","
                                   << timestamp_str << ","
                                   << point.source_frame << ","
                                   << point.range_bin << ","
                                   << point.doppler_bin << ","
                                   << point.snr << ","
                                   << point.doa_peak_confidence << "\n";
                is_first_point = false;
            }
        }


        auto t_end_total = std::chrono::high_resolution_clock::now();
        float elapsed = std::chrono::duration<float, std::milli>(t_end_total - t_start_total).count();
        if (ENABLE_CONSOLE_OUTPUT) {
            std::cout << "[信号处理] 本帧处理完成，总耗时: " << elapsed << " ms" << std::endl;
        }

        const float processing_deadline_ms = static_cast<float>(radar_params_map.at("framePeriodicity"));
        g_processing_times_ms.push_back(elapsed);
        if (elapsed > processing_deadline_ms) g_processing_deadline_misses++;
        if (g_frame_counter % 10 == 0) {
            std::cout << std::fixed << std::setprecision(3)
                      << "[FRAME] frame=" << g_frame_counter
                      << " backend=" << (USE_CUDA_FFT ? "CUDA" : "CPU")
                      << " cfar=" << cfar_result.num_targets
                      << " doa_points=" << current_frame_point_cloud.size()
                      << " odom=" << (chassis_velocity.valid ? "OK" : "MISSING")
                      << " processing_ms=" << elapsed
                      << std::endl;
        }
        if (g_processing_times_ms.size() >= 100) {
            std::vector<float> sorted = g_processing_times_ms;
            std::sort(sorted.begin(), sorted.end());
            const float mean = std::accumulate(sorted.begin(), sorted.end(), 0.0f) /
                               static_cast<float>(sorted.size());
            const std::size_t p95_index = std::min(sorted.size() - 1,
                static_cast<std::size_t>(std::ceil(0.95 * sorted.size())) - 1);
            std::cout << std::fixed << std::setprecision(3)
                      << "[PERF] frames=" << g_frame_counter
                      << " window_samples=" << sorted.size()
                      << " processing_ms_mean=" << mean
                      << " processing_ms_p50=" << sorted[sorted.size() / 2]
                      << " processing_ms_p95=" << sorted[p95_index]
                      << " processing_ms_max=" << sorted.back()
                      << " deadline_ms=" << processing_deadline_ms
                      << " deadline_misses_total=" << g_processing_deadline_misses
                      << std::endl;
            g_processing_times_ms.clear();
        }

    ProcessingResult result;
    result.frame_number = g_frame_counter;
    if (frame_context) result.frame = frame_context->frame;
    result.frame.frame_number = g_frame_counter;
    result.frame.processing_completed_sec = RadarTime::wallSeconds();
        double frame_period_ms = radar_params_map.at("framePeriodicity");
    std::chrono::milliseconds frame_offset(static_cast<long long>((g_frame_counter - 1) * frame_period_ms));
        auto frame_time = capture_start_time + frame_offset;
        auto in_time_t = std::chrono::system_clock::to_time_t(frame_time);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(frame_time.time_since_epoch()) % 1000;
        std::stringstream ss;
        ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %H:%M:%S") << "." << std::setfill('0') << std::setw(3) << ms.count();
        result.timestamp =
            frame_context && !frame_context->radar_timestamp.empty()
                ? frame_context->radar_timestamp
                : ss.str();
        result.point_cloud = current_frame_point_cloud;
        result.processing_time_ms = elapsed;
        frame_diagnostics.processing_time_ms = elapsed;
        result.diagnostics = frame_diagnostics;

        return result;

    } catch (const std::exception& e) {
        std::cerr << "[信号处理] 错误: " << e.what() << std::endl;
        ProcessingResult failed_result;
        failed_result.frame_number = g_frame_counter;
        if (frame_context) failed_result.frame = frame_context->frame;
        failed_result.diagnostics.processing_ok = false;
        failed_result.diagnostics.error_message = e.what();
        return failed_result;
    }
}

int mode(const std::vector<int>& data) {
    if (data.empty()) return 0;
    std::map<int, int> count;
    for (int val : data) count[val]++;
    int max_count = 0, mode_val = data[0];
    for (const auto& pair : count) {
        if (pair.second > max_count) {
            max_count = pair.second;
            mode_val = pair.first;
        }
    }
    return mode_val;
}


