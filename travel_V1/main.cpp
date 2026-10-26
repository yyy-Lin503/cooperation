#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <queue>
#include <thread>
#include <chrono>
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#include <iomanip>
#include <ctime>
#include <sstream>
#include <limits>
#include <cuda_runtime.h>
#include "radar_config.hpp"
#include "realtime_capture.hpp"
#include "iwr6843param.hpp"

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <poll.h>
#include <dirent.h>
#include <cstring>
#define closesocket close
#define INVALID_SOCKET -1
#define SOCKET int
#endif

AWR2944SerialConfig* radar = nullptr;
DCA1000Config* dca1000_cfg = nullptr;
std::string config_file;
std::map<std::string, double> radar_config;
UdpListener* collector = nullptr;
bool is_capturing = false;
std::queue<std::vector<char>> bin_data; // Not used, but kept for compatibility
std::chrono::system_clock::time_point capture_start_time;

std::vector<std::string> listSerialPorts() {
    std::vector<std::string> ports;

#ifdef _WIN32
    // Windows: Query registry for serial ports
    HKEY hKey;
    LONG result = RegOpenKeyExA(HKEY_LOCAL_MACHINE, "HARDWARE\\DEVICEMAP\\SERIALCOMM", 0, KEY_READ, &hKey);
    if (result != ERROR_SUCCESS) {
        throw std::runtime_error("无法打开串口注册表项");
    }

    DWORD index = 0;
    char valueName[256]; // Use char for ANSI
    DWORD valueNameSize = sizeof(valueName); // Size in bytes
    char data[256]; // ANSI for serial port names
    DWORD dataSize = sizeof(data); // Size in bytes
    DWORD type;
    while (RegEnumValueA(hKey, index, valueName, &valueNameSize, NULL, &type, (LPBYTE)data, &dataSize) == ERROR_SUCCESS) {
        if (type == REG_SZ) { // Ensure the value is a string
            ports.push_back(std::string(data)); // Directly use ANSI string
        }
        valueNameSize = sizeof(valueName);
        dataSize = sizeof(data);
        index++;
    }

    RegCloseKey(hKey);
#else
    // Linux: Scan /dev directory
    DIR* dir = opendir("/dev");
    if (!dir) {
        throw std::runtime_error("无法打开 /dev 目录：" + std::string(strerror(errno)));
    }

    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        std::string name = entry->d_name;
        if (name.find("ttyS") == 0 || name.find("ttyUSB") == 0 || name.find("ttyACM") == 0) {
            std::string fullPath = "/dev/" + name;
            int fd = open(fullPath.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
            if (fd >= 0) {
                ports.push_back(fullPath);
                close(fd);
                std::cout << "[DEBUG] Found accessible port: " << fullPath << std::endl;
            } else {
                std::cout << "[DEBUG] Failed to open port: " << fullPath << " (" << strerror(errno) << ")" << std::endl;
            }
        }
    }

    closedir(dir);
#endif

    return ports;
}

std::string selectSerialPort() {
    auto start_time = std::chrono::high_resolution_clock::now();
    try {
        std::vector<std::string> ports = listSerialPorts();
        if (ports.empty()) {
            std::cout << "没有找到串口。" << std::endl;
            return "";
        }

        for (const auto& port : ports) {
            try {
                AWR2944SerialConfig test_radar(port, 115200);
                // 发送测试命令 "version\n" 并检查响应（首次启动时可靠）
                std::string test_cmd = "version\n";
#ifdef _WIN32
                DWORD bytes_written;
                WriteFile(test_radar.get_cli_port(), test_cmd.c_str(), test_cmd.size(), &bytes_written, NULL);  // 使用 getter
#else
                ssize_t bytes_written = write(test_radar.get_fd(), test_cmd.c_str(), test_cmd.size());  // 使用 getter
                if (bytes_written < 0) {
                    continue; // 写入失败，跳过
                }
#endif
                // 读取响应，带超时
                std::string response;
                auto start = std::chrono::steady_clock::now();
                while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(100)) {
                    char buf[1024];
#ifdef _WIN32
                    DWORD bytes_read;
                    ReadFile(test_radar.get_cli_port(), buf, sizeof(buf), &bytes_read, NULL);  // 使用 getter
                    if (bytes_read > 0) response.append(buf, bytes_read);
#else
                    ssize_t bytes_read = read(test_radar.get_fd(), buf, sizeof(buf));  // 使用 getter
                    if (bytes_read > 0) response.append(buf, bytes_read);
#endif
                    std::this_thread::sleep_for(std::chrono::milliseconds(2));
                }
                // 检查响应是否非空（可靠检测）
                std::cout << "[DEBUG] Port " << port << " response: '" << response << "'" << std::endl;
                // 检查响应是否包含雷达版本信息（更严格的验证）
                if (!response.empty() && response.find("version") != std::string::npos) {
                    std::cout << "检测到雷达串口：" << port << " (响应: " << response << ")" << std::endl;
                    auto end_time = std::chrono::high_resolution_clock::now();
                    std::cout << "selectSerialPort()耗时: " << std::chrono::duration<float, std::milli>(end_time-start_time).count() << " ms" << std::endl;
                    return port; // 找到有效响应，返回此串口
                } else if (!response.empty()) {
                    std::cout << "[DEBUG] Port " << port << " 有响应但不是雷达: '" << response << "'" << std::endl;
                }
                test_radar.close(); // 关闭测试实例
            } catch (const std::exception& e) {
                // 忽略异常，继续下一个
                continue;
            }
        }
        std::cout << "未检测到雷达串口。" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return "";
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    std::cout << "selectSerialPort()耗时: " << std::chrono::duration<float, std::milli>(end_time-start_time).count() << " ms" << std::endl;
    return "";
}


void connect_radar() {
    auto start_time = std::chrono::high_resolution_clock::now();

    if (radar != nullptr && radar->is_open()) {
        std::cout << "已连接到雷达，无需重复连接。" << std::endl;
        return;
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    std::cout << "connect_radar()耗时: " << std::chrono::duration<float, std::milli>(end_time-start_time).count() << " ms" << std::endl;

    std::string selected_port = selectSerialPort();
    if (!selected_port.empty()) {
        try {
            radar = new AWR2944SerialConfig(selected_port, 115200);
            std::cout << "已连接到 " << selected_port << std::endl;
        } catch (const std::exception& e) {
            std::cout << "连接失败：" << e.what() << std::endl;
        }
    }

    }

std::vector<std::string> list_config_files() {
    auto start_time = std::chrono::high_resolution_clock::now();
    std::vector<std::string> files;
    std::string config_dir = "./config";
    if (!fs::exists(config_dir)) {
        std::cout << "配置文件目录不存在。" << std::endl;
    } else {
        for (const auto& entry : fs::directory_iterator(config_dir)) {
            if (fs::is_regular_file(entry.status())) {
                files.push_back(entry.path().filename().string());
            }
        }
        if (files.empty()) {
            std::cout << "没有找到配置文件。" << std::endl;
        } else {
            std::cout << "可用配置文件列表：" << std::endl;
            for (size_t i = 0; i < files.size(); ++i) {
                std::cout << (i + 1) << ". " << files[i] << std::endl;
            }
        }
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    std::cout << "list_config_files()耗时: " << std::chrono::duration<float, std::milli>(end_time-start_time).count() << " ms" << std::endl;
    return files;
}
void configure_radar() {
    auto start_time = std::chrono::high_resolution_clock::now();
    if (radar == nullptr || !radar->is_open()) {
        std::cout << "雷达未连接。" << std::endl;
        return;
    }
    if (config_file.empty()) {
        std::cout << "未选择配置文件。" << std::endl;
        return;
    }
    if (!fs::exists(config_file)) {
        std::cout << "所选配置文件不存在。" << std::endl;
        return;
    }
    try {
        radar->SendConfig(config_file);
        std::cout << "已配置雷达 " << config_file << std::endl;
        IWR6843AOP_TLV tlv(config_file);
        radar_config = tlv.get_params();
        std::cout << "雷达参数：" << std::endl;
        for (const auto& kv : radar_config) {
            std::cout << kv.first << ": " << kv.second << std::endl;
        }
    } catch (const std::exception& e) {
        std::cout << "配置雷达失败：" << e.what() << std::endl;
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    std::cout << "configure_radar()耗时: " << std::chrono::duration<float, std::milli>(end_time-start_time).count() << " ms" << std::endl;
}

void start_capture() {
    auto start_time = std::chrono::high_resolution_clock::now();
    if (radar_config.empty()) {
        std::cout << "雷达未配置。" << std::endl;
        return;
    }
    try {
        int frame_length = static_cast<int>(radar_config["numADC"] * radar_config["numChirp"] *
                                           radar_config["numTX"] * radar_config["numRX"] * 2);
        std::string address = "192.168.33.30";
        int port = 4098;
        int buff_size = 2048; // Can increase if needed

        collector = new UdpListener("Listener", bin_data, frame_length, address, port, buff_size, radar_config, "127.0.0.1", 12345);
        collector->start();
        collector->start_saving();
        dca1000_cfg->DCA1000_start();
        capture_start_time = std::chrono::system_clock::now();
        auto time_stamp = std::chrono::system_clock::to_time_t(capture_start_time);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      capture_start_time.time_since_epoch()) % 1000;
        std::cout << "数据捕获开始时间: " << std::put_time(std::localtime(&time_stamp), "%Y-%m-%d %H:%M:%S") << "." << std::setfill('0') << std::setw(3) << ms.count() << std::endl;
        is_capturing = true;
        std::cout << "已开始捕获数据。" << std::endl;
    } catch (const std::exception& e) {
        std::cout << "开始捕获失败：" << e.what() << std::endl;
        if (collector) {
            collector->stop_saving();
            collector->stop_collector();
            delete collector;
            collector = nullptr;
        }
        is_capturing = false;
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    std::cout << "start_capture()耗时: " << std::chrono::duration<float, std::milli>(end_time-start_time).count() << " ms" << std::endl;
}

void select_config_file() {
    //     config_file = "./config/" + files[choice];
    // } else {
    auto files = list_config_files();    //固定唯一参数文件
    if (files.empty()) return;
    config_file = "./config/" + files[0];
    std::cout << "已选择配置文件：" << config_file << std::endl;
    configure_radar();  //连接雷达
}


void stop_capture() {
    auto start_time = std::chrono::high_resolution_clock::now();
    if (!is_capturing) {
        std::cout << "当前未捕获数据。" << std::endl;
        return;
    }
    try {
        if (dca1000_cfg) dca1000_cfg->DCA1000_close();
        if (collector) {
            collector->stop_saving();
            collector->join(1000);
            collector->stop_collector();
            delete collector;
            collector = nullptr;
        }
        is_capturing = false;
        std::cout << "已停止捕获数据。" << std::endl;
    } catch (const std::exception& e) {
        std::cout << "停止捕获失败：" << e.what() << std::endl;
        collector = nullptr;
        is_capturing = false;
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    std::cout << "stop_capture()耗时: " << std::chrono::duration<float, std::milli>(end_time-start_time).count() << " ms" << std::endl;
}

void exit_program() {
    auto start_time = std::chrono::high_resolution_clock::now();
    std::cout << "正在退出程序..." << std::endl;
    if (is_capturing) stop_capture();
    if (collector) {
        collector->stop_collector();
        collector->join(1000);
        delete collector;
        collector = nullptr;
        std::cout << "UdpListener 线程已停止" << std::endl;
    }
    if (radar && radar->is_open()) {
        radar->close();
        delete radar;
        radar = nullptr;
        std::cout << "雷达串口已关闭" << std::endl;
    }
    if (dca1000_cfg) {
        dca1000_cfg->close();
        delete dca1000_cfg;
        dca1000_cfg = nullptr;
        std::cout << "DCA1000 socket 已关闭" << std::endl;
    }
#ifdef _WIN32
    WSACleanup();
#endif
    std::cout << "程序已退出。" << std::endl;
    exit(0);
    auto end_time = std::chrono::high_resolution_clock::now();
    std::cout << "exit_program()耗时: " << std::chrono::duration<float, std::milli>(end_time-start_time).count() << " ms" << std::endl;
}


int main() {
#ifdef _WIN32
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif

    auto start_time = std::chrono::high_resolution_clock::now();

    int deviceCount = 0;
    cudaError_t cudaError = cudaGetDeviceCount(&deviceCount);
    if (cudaError != cudaSuccess || deviceCount == 0) {
        std::cout << "[主程序] CUDA不可用: " << cudaGetErrorString(cudaError) << " (设备数量: " << deviceCount << ")" << std::endl;
        std::cout << "[主程序] 将使用CPU进行FFT计算" << std::endl;
    } else {
        std::cout << "[主程序] 检测到 " << deviceCount << " 个CUDA设备，将使用GPU加速FFT" << std::endl;
    }

    std::string config_ip = "192.168.33.30";
    int config_port = 4096;
    std::string fpga_ip = "192.168.33.180";
    int fpga_port = 4096;
    dca1000_cfg = new DCA1000Config("DCA1000Config", config_ip, config_port, fpga_ip, fpga_port);

    auto end_time = std::chrono::high_resolution_clock::now();
    std::cout << "DCA1000初始化耗时: " << std::chrono::duration<float, std::milli>(end_time-start_time).count() << " ms" << std::endl;

    while (true) {
        std::cout << "\nTI_IWR6843_Radar CLI" << std::endl;
        std::cout << "1. 连接雷达" << std::endl;
        std::cout << "2. 配置雷达" << std::endl;
        std::cout << "3. 开始捕获数据" << std::endl;
        std::cout << "4. 停止捕获数据" << std::endl;
        std::cout << "5. 退出" << std::endl;
        std::cout << "请输入您的选择：" << std::endl;
        int choice;
        std::cin >> choice;
        switch (choice) {
            case 1: connect_radar(); break;
            case 2: select_config_file(); break;  // 包含参数文件选择、配置雷达和开始采集
            case 3: start_capture(); break;
            case 4: stop_capture(); break;
            case 5: exit_program(); break;
            default: std::cout << "无效的选择，请重试。" << std::endl;
        }
    }

    return 0;
}