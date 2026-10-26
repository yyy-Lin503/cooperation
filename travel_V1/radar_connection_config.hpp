#ifndef RADAR_CONNECTION_CONFIG_HPP
#define RADAR_CONNECTION_CONFIG_HPP
#include <cstdlib>
#include <cmath>
#include <stdexcept>
#include <string>

namespace RadarEnvironment {
inline std::string value(const char* name, const char* fallback) {
    const char* text = std::getenv(name);
    return text && *text ? text : fallback;
}
inline double number(const char* name, double fallback, double low, double high) {
    const char* text = std::getenv(name);
    if (!text || !*text) return fallback;
    std::size_t end = 0;
    const double n = std::stod(text, &end);
    if (end != std::string(text).size() || !std::isfinite(n) || n < low || n > high)
        throw std::invalid_argument(std::string("Invalid ") + name);
    return n;
}
inline int port(const char* name, int fallback) {
    double n = number(name, fallback, 1, 65535);
    if (n != std::floor(n)) throw std::invalid_argument(std::string("Invalid ") + name);
    return static_cast<int>(n);
}
}

struct RadarConnectionConfig {
    std::string host_ip = RadarEnvironment::value("RADAR_HOST_IP", "192.168.33.30");
    std::string fpga_ip = RadarEnvironment::value("RADAR_FPGA_IP", "192.168.33.180");
    int config_port = RadarEnvironment::port("RADAR_CONFIG_PORT", 4096);
    int fpga_port = RadarEnvironment::port("RADAR_FPGA_PORT", 4096);
    int data_port = RadarEnvironment::port("RADAR_DATA_PORT", 4098);
    std::string result_ip = RadarEnvironment::value("RADAR_RESULT_IP", "127.0.0.1");
    int result_port = RadarEnvironment::port("RADAR_RESULT_PORT", 12345);
};
#endif
