#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <map>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <cstdint>
#include <cstring>

#include "signal_processing.hpp"      
#include "iwr6843param.hpp"           
#include "radar_frame_layout.hpp"
#include "offline_odom_replay.hpp"
#include "recording_frame_index.hpp"

// 提供给 signal_processing.cpp 使用的全局变量（其内部用 extern 声明）
std::chrono::system_clock::time_point capture_start_time;

static void print_usage(const char* prog) {
    std::cout << "Usage:\n"
              << "  " << prog << " <input.bin> <config.cfg> [--output out.txt] [--max-frames N] [--offset-bytes B] [--diagnostic-csv file.csv]\n\n"
              << "Args:\n"
              << "  <input.bin>     DCA1000 采集的原始二进制文件\n"
              << "  <config.cfg>    雷达配置文件（与 GUI 相同格式）\n\n"
              << "Options:\n"
              << "  --output out.txt   输出点云CSV（默认: input 同目录下自动命名）\n"
              << "  --max-frames N     最多处理 N 帧（默认: 全部）\n"
              << "  --offset-bytes B   从文件偏移 B 字节处开始（默认: 0）\n\n"
              << "  --diagnostic-csv C Replay per-frame vx/wz/timestamp from diagnostics CSV\n\n"
              << "Output CSV: Frame,X,Y,RelativeVelocity,AbsoluteVelocity,IsMoving,Timestamp,"
                 "SourceFrame,RangeBin,DopplerBin,SNR,DOAPeakConfidence\n";
}

int main(int argc, char** argv) {
    if (argc < 3) {
        print_usage(argv[0]);
        return 1;
    }

    std::string input_path = argv[1];
    std::string cfg_path   = argv[2];
    std::string out_path;
    long long max_frames = -1; // all
    long long offset_bytes = 0;
    std::string diagnostic_csv_path;

    for (int i = 3; i < argc; ++i) {
        if (std::strcmp(argv[i], "--output") == 0 && i + 1 < argc) {
            out_path = argv[++i];
        } else if (std::strcmp(argv[i], "--max-frames") == 0 && i + 1 < argc) {
            max_frames = std::stoll(argv[++i]);
        } else if (std::strcmp(argv[i], "--offset-bytes") == 0 && i + 1 < argc) {
            offset_bytes = std::stoll(argv[++i]);
        } else if (std::strcmp(argv[i], "--diagnostic-csv") == 0 && i + 1 < argc) {
            diagnostic_csv_path = argv[++i];
        } else {
            std::cerr << "未知参数: " << argv[i] << "\n";
            print_usage(argv[0]);
            return 1;
        }
    }

    // 解析配置，得到雷达参数
    std::map<std::string, double> radar_params;
    try {
        IWR6843AOP_TLV tlv(cfg_path);
        radar_params = tlv.get_params();
    } catch (const std::exception& e) {
        std::cerr << "读取配置失败: " << e.what() << "\n";
        return 1;
    }

    // 计算每帧字节数（对齐 GUI 的计算方式）
    // I and Q are each int16: every complex sample occupies four bytes.
    const long long bytes_per_frame = static_cast<long long>(
        RadarFrameLayout::frameBytes(RadarFrameLayout::paramsFromMap(radar_params)));
    if (offset_bytes % bytes_per_frame != 0) {
        std::cerr << "--offset-bytes must be aligned to a complete radar frame\n";
        return 1;
    }

    // 输出文件默认名
    if (out_path.empty()) {
        // input.bin -> input_point_cloud.txt
        size_t pos = input_path.find_last_of("/\\");
        std::string base = (pos == std::string::npos) ? input_path : input_path.substr(pos + 1);
        if (base.size() > 4 && (base.substr(base.size()-4) == ".bin" || base.substr(base.size()-4) == ".BIN")) {
            base = base.substr(0, base.size()-4);
        }
        out_path = base + "_point_cloud.txt";
    }

    std::ifstream fin(input_path, std::ios::binary);
    if (!fin) {
        std::cerr << "无法打开输入文件: " << input_path << "\n";
        return 1;
    }
    fin.seekg(0, std::ios::end);
    const long long file_size = static_cast<long long>(fin.tellg());
    if (offset_bytes < 0 || offset_bytes >= file_size) {
        std::cerr << "偏移越界: " << offset_bytes << ", 文件大小: " << file_size << "\n";
        return 1;
    }
    fin.seekg(offset_bytes, std::ios::beg);

    long long remain = file_size - offset_bytes;
    if (bytes_per_frame <= 0) {
        std::cerr << "无效的每帧字节数: " << bytes_per_frame << "\n";
        return 1;
    }
    RecordingFrameIndex frame_index;
    try { frame_index.load(input_path, static_cast<std::size_t>(file_size / bytes_per_frame)); }
    catch (const std::exception& error) { std::cerr << error.what() << '\n'; return 1; }
    long long total_frames = remain / bytes_per_frame;
    if (max_frames > 0 && max_frames < total_frames) total_frames = max_frames;

    if (total_frames <= 0) {
        std::cerr << "可用整帧数为0。请检查配置与 bin 文件是否匹配（每帧字节=" << bytes_per_frame << ")。\n";
        return 1;
    }

    std::ofstream fout(out_path);
    if (!fout) {
        std::cerr << "无法创建输出文件: " << out_path << "\n";
        return 1;
    }
    fout << "Frame,X,Y,RelativeVelocity,AbsoluteVelocity,IsMoving,Timestamp,"
            "SourceFrame,RangeBin,DopplerBin,SNR,DOAPeakConfidence\n";

    // 设定处理起始时间（供 timestamp 计算）
    capture_start_time = std::chrono::system_clock::now();
    OfflineOdomReplay odom_replay;
    if (!diagnostic_csv_path.empty()) {
        std::string replay_message;
        if (!odom_replay.loadCsv(diagnostic_csv_path, input_path,
                                 &replay_message)) {
            std::cerr << "Failed to load motion replay CSV: "
                      << replay_message << "\n";
            return 1;
        }
        std::cout << replay_message << "\n";
    }

    std::vector<char> frame_buf;
    frame_buf.resize(static_cast<size_t>(bytes_per_frame));

    std::cout << "开始转换: 共 " << total_frames << " 帧。每帧 " << bytes_per_frame << " 字节。\n";

    for (long long i = 0; i < total_frames; ++i) {
        fin.read(frame_buf.data(), frame_buf.size());
        if (fin.gcount() != static_cast<std::streamsize>(frame_buf.size())) {
            std::cerr << "读取第 " << (i+1) << " 帧失败/不足，提前结束。\n";
            break;
        }

        const auto metadata = frame_index.frame(static_cast<std::size_t>(offset_bytes / bytes_per_frame + i));
        const int replay_frame = metadata.frame_number;
        const OfflineMotionFrame* motion = odom_replay.frame(replay_frame);
        SignalProcessingFrameContext context;
        context.source_frame_number = replay_frame;
        context.frame = metadata;
        context.persist_outputs = false;
        context.radar_timestamp = frame_index.loaded() ? RadarTime::timestamp(metadata.sample_time_sec) :
            odom_replay.radarTimestampForFrame(replay_frame, radar_params.at("framePeriodicity"));
        const SignalProcessingFrameContext* context_ptr = &context;
        bool enable_ego_compensation = false;
        if (motion) {
            if (motion->has_dsp_sample) {
                context.use_chassis_override = true;
                context.chassis = motion->dsp_sample;
                context.has_odom_age = motion->has_dsp_age;
                context.odom_age_sec = motion->dsp_age_sec;
                enable_ego_compensation = true;
            }
            context_ptr = &context;
        }
        ProcessingResult res = run_signal_processing_pipeline(
            frame_buf, radar_params, enable_ego_compensation, context_ptr);
        if (!res.diagnostics.processing_ok) {
            std::cerr << "Frame " << replay_frame << " failed: "
                      << res.diagnostics.error_message << '\n';
            return 1;
        }

        // 写入本帧点云
        bool first_point = true;
        for (const auto& p : res.point_cloud) {
            fout << res.frame_number << ","
                 << p.x << "," << p.y << ","
                 << p.relative_velocity << "," << p.absolute_velocity << ","
                 << (p.is_moving ? 1 : 0) << ",";
            if (first_point) {
                fout << res.timestamp;
                first_point = false;
            }
            fout << "," << p.source_frame
                 << "," << p.range_bin
                 << "," << p.doppler_bin
                 << "," << p.snr
                 << "," << p.doa_peak_confidence
                 << "\n";
        }

        if (((i+1) % 10) == 0 || i+1 == total_frames) {
            std::cout << "已处理帧: " << (i+1) << "/" << total_frames << "\r" << std::flush;
        }
    }

    std::cout << "\n完成。输出: " << out_path << "\n";

    std::cout << "提示: 已禁用管线重复落盘，结果仅写入指定输出文件。\n";

    return 0;
}
