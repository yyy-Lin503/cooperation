#include "radar_config.hpp"
#include <fstream>
#include <iostream>
#include <chrono>
#include <thread>
#include <sstream>
#include <cerrno>
#include <cstring>
#include <iomanip>
#ifdef _WIN32
#include <ws2tcpip.h>  
#else
#include <string.h>  
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/select.h>
#endif

struct sockaddr_in DCA1000Config::response_addr;
bool DCA1000Config::response_addr_set = false;

AWR2944SerialConfig::AWR2944SerialConfig(const std::string& port, int baudrate) : opened(false) {
#ifdef _WIN32
    std::string port_name = "\\\\.\\" + port;
    cli_port = CreateFileA(port_name.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (cli_port == INVALID_HANDLE_VALUE) {
        throw std::runtime_error("Failed to open serial port");
    }

    DCB dcbSerialParams = {0};
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    GetCommState(cli_port, &dcbSerialParams);
    dcbSerialParams.BaudRate = baudrate;
    dcbSerialParams.ByteSize = 8;
    dcbSerialParams.StopBits = ONESTOPBIT;
    dcbSerialParams.Parity = NOPARITY;
    SetCommState(cli_port, &dcbSerialParams);

    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = 100; //读取最多等待0.1s
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 100; //写入最多等待0.1s
    SetCommTimeouts(cli_port, &timeouts);
#else
    fd = open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        throw std::runtime_error(std::string("Failed to open serial port: ") + strerror(errno));
    }

    struct termios tty;
    tcgetattr(fd, &old_tio);
    tcgetattr(fd, &tty);

    cfsetispeed(&tty, baudrate);
    cfsetospeed(&tty, baudrate);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS; // No flow control

    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    tty.c_oflag &= ~OPOST;

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 1; // 超时时间为0.1s

    tcsetattr(fd, TCSANOW, &tty);
#endif
    opened = true;
    std::cout << "[radar_config] Serial port opened successfully" << std::endl;
}

AWR2944SerialConfig::~AWR2944SerialConfig() {
    close();
}

void AWR2944SerialConfig::SendConfig(const std::string& config_file) {
    std::ifstream file(config_file);
    if (!file) {
        throw std::runtime_error("Failed to open config file");
    }
    std::string line;
    while (std::getline(file, line)) {
        line += "\n";
#ifdef _WIN32
        DWORD bytes_written;
        WriteFile(cli_port, line.c_str(), line.size(), &bytes_written, NULL);
#else
        ssize_t bytes_written = write(fd, line.c_str(), line.size());
        if (bytes_written < 0) {
            throw std::runtime_error("Write failed");
        }
#endif
        std::cout << "Sent: " << line;

        // 读取串口返回，带超时
        std::string response;
        auto start = std::chrono::steady_clock::now();
        while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(100)) {
            char buf[1024];
#ifdef _WIN32
            DWORD bytes_read;
            ReadFile(cli_port, buf, sizeof(buf), &bytes_read, NULL);
            if (bytes_read > 0) response.append(buf, bytes_read);
#else
            ssize_t bytes_read = read(fd, buf, sizeof(buf));
            if (bytes_read > 0) response.append(buf, bytes_read);
#endif
            std::this_thread::sleep_for(std::chrono::milliseconds(2));  
        }
        std::cout << "Received: " << response << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));  
    }
}

void AWR2944SerialConfig::close() {
    if (opened) {
#ifdef _WIN32
        CloseHandle(cli_port);
#else
        tcsetattr(fd, TCSANOW, &old_tio);
        ::close(fd);
#endif
        opened = false;
    }
}

DCA1000Config::DCA1000Config(const std::string& name, const std::string& config_ip, int config_port,
                             const std::string& fpga_ip, int fpga_port) {
    sock_config = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_config == INVALID_SOCKET) {
        throw std::runtime_error("Socket creation failed"); // 创建UDP socket失败
    }

    config_addr.sin_family = AF_INET;
    config_addr.sin_port = htons(config_port);
    config_addr.sin_addr.s_addr = inet_addr(config_ip.c_str()); // 绑定的本地IP地址

    // 绑定本地UDP端口
    if (bind(sock_config, (struct sockaddr*)&config_addr, sizeof(config_addr)) < 0) {
        #ifdef _WIN32
        std::stringstream ss;
        ss << "Bind failed: " << WSAGetLastError();
        close();
        throw std::runtime_error(ss.str()); // 绑定失败，抛出异常
        #else
        const std::string reason = std::string(strerror(errno));
        close();
        throw std::runtime_error("Bind failed: " + reason);
        #endif
    }

    // ————可选：增大接收缓冲区，防止丢包————

    fpga_addr_cfg.sin_family = AF_INET;
    fpga_addr_cfg.sin_port = htons(fpga_port);
    fpga_addr_cfg.sin_addr.s_addr = inet_addr(fpga_ip.c_str()); // FPGA目标地址

#ifdef _WIN32
    DWORD timeout = 100; // ms
    setsockopt(sock_config, SOL_SOCKET, SO_RCVTIMEO, (const char*)&timeout, sizeof(timeout)); // 设置接收超时
#else
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = 100000;
    setsockopt(sock_config, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
#endif

    try {
        for (const auto& cmd : {"9", "3", "B", "E"}) {
            std::string error;
            if (!transact(cmd, &error)) throw std::runtime_error(error);
        }
    } catch (...) { close(); throw; }
}

DCA1000Config::~DCA1000Config() {
    close(); // 析构时关闭socket
}

bool DCA1000Config::transact(const std::string& code, std::string* error) {
    auto reject = [&](const std::string& why) {
        if (error) *error = "DCA command " + code + ": " + why;
        return false;
    };
    const auto command = send_cmd(code);
    // Discard late acknowledgements but retain asynchronous device status.
    pending_async_status_ = pollAsyncStatus();
    if (sendto(sock_config, command.data(), static_cast<int>(command.size()), 0,
               reinterpret_cast<sockaddr*>(&fpga_addr_cfg), sizeof(fpga_addr_cfg)) != static_cast<int>(command.size()))
        return reject("send failed");
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(100);
    for (;;) {
    const auto remaining = std::chrono::duration_cast<std::chrono::microseconds>(
        deadline - std::chrono::steady_clock::now()).count();
    if (remaining <= 0) return reject("response timeout or receive failure");
    timeval timeout{};
    timeout.tv_sec = static_cast<long>(remaining / 1000000);
    timeout.tv_usec = static_cast<long>(remaining % 1000000);
    fd_set readable; FD_ZERO(&readable); FD_SET(sock_config, &readable);
    const int ready = select(static_cast<int>(sock_config) + 1, &readable, nullptr, nullptr, &timeout);
#ifndef _WIN32
    if (ready < 0 && errno == EINTR) continue;
#endif
    if (ready <= 0) return reject("response timeout or receive failure");
    unsigned char buffer[2048];
    sockaddr_in sender{};
#ifdef _WIN32
    int size = sizeof(sender);
#else
    socklen_t size = sizeof(sender);
#endif
    const int length = recvfrom(sock_config, reinterpret_cast<char*>(buffer), sizeof(buffer), 0,
                                reinterpret_cast<sockaddr*>(&sender), &size);
    if (length < 0) return reject("response timeout or receive failure");
    // The board's command destination port is not necessarily its reply source
    // port (the connected DCA1000 replies from 1024 to commands sent to 4096).
    // Validate the device IP and the acknowledgement below, rather than assuming
    // that both UDP ports must match. Keep sending commands to fpga_addr_cfg.
    if (sender.sin_addr.s_addr != fpga_addr_cfg.sin_addr.s_addr) {
        const std::string actual_ip = inet_ntoa(sender.sin_addr);
        const std::string expected_ip = inet_ntoa(fpga_addr_cfg.sin_addr);
        return reject("unexpected response endpoint " + actual_ip + ":" +
                      std::to_string(ntohs(sender.sin_port)) + "; expected device IP " + expected_ip);
    }
    if (length != 8 || buffer[0] != 0x5a || buffer[1] != 0xa5 ||
        buffer[6] != 0xaa || buffer[7] != 0xee) return reject("malformed or mismatched acknowledgement");
    if (buffer[2] == 0x0a && buffer[3] == 0) {
        if (pending_async_status_.size() < 64)
            pending_async_status_.push_back(static_cast<std::uint16_t>(buffer[4] | (buffer[5] << 8)));
        continue;
    }
    if (buffer[2] != static_cast<unsigned char>(command[2]) || buffer[3] != static_cast<unsigned char>(command[3]))
        return reject("malformed or mismatched acknowledgement");
    if (code != "E" && (buffer[4] || buffer[5])) return reject("device returned a failure status");
    return true;
    }
}

std::vector<std::uint16_t> DCA1000Config::pollAsyncStatus() {
    std::vector<std::uint16_t> result;
    result.swap(pending_async_status_);
    if (sock_config == INVALID_SOCKET) return result;
#ifdef _WIN32
    u_long nonblocking = 1;
    ioctlsocket(sock_config, FIONBIO, &nonblocking);
#endif
    // Bound work per GUI tick, even if unrelated datagrams flood this port.
    for (int i = 0; i < 32; ++i) {
        unsigned char buffer[2048]; sockaddr_in sender{};
#ifdef _WIN32
        int size = sizeof(sender);
        const int length = recvfrom(sock_config, reinterpret_cast<char*>(buffer), sizeof(buffer), 0,
                                   reinterpret_cast<sockaddr*>(&sender), &size);
#else
        socklen_t size = sizeof(sender);
        const int length = recvfrom(sock_config, buffer, sizeof(buffer), MSG_DONTWAIT,
                                   reinterpret_cast<sockaddr*>(&sender), &size);
#endif
        if (length < 0) break;
        if (sender.sin_addr.s_addr == fpga_addr_cfg.sin_addr.s_addr && length == 8 &&
            buffer[0] == 0x5a && buffer[1] == 0xa5 && buffer[2] == 0x0a && buffer[3] == 0 &&
            buffer[6] == 0xaa && buffer[7] == 0xee)
            result.push_back(static_cast<std::uint16_t>(buffer[4] | (buffer[5] << 8)));
    }
#ifdef _WIN32
    nonblocking = 0;
    ioctlsocket(sock_config, FIONBIO, &nonblocking);
#endif
    return result;
}

std::string DCA1000Config::describeAsyncStatus(std::uint16_t status) {
    // TI DCA1000 CLI Developer Guide, section 2.2.6: system status bit positions.
    static const char* names[] = {"STS_NO_LVDS_DATA", "STS_NO_HEADER", "STS_EEPROM_FAILURE",
        "STS_SD_CARD_DETECTED", "STS_SD_CARD_REMOVED", "STS_SD_CARD_FULL", "STS_MODE_CONFIG_FAILURE",
        "STS_DDR_FULL", "STS_REC_COMPLETED", "STS_LVDS_BUFFER_FULL", "STS_PLAYBACK_COMPLETED",
        "STS_PLAYBACK_OUT_OF_SEQ"};
    std::string result;
    for (unsigned bit = 0; bit < 12; ++bit) if (status & (1u << bit)) {
        if (!result.empty()) result += ", ";
        result += names[bit];
    }
    if (result.empty() || (status & 0xf000)) result += result.empty() ? "UNKNOWN_STATUS" : ", UNKNOWN_STATUS";
    return result;
}
bool DCA1000Config::DCA1000_start(std::string* error) { return transact("5", error); }
bool DCA1000Config::DCA1000_close(std::string* error) { return transact("6", error); }

void DCA1000Config::close() {
    if (sock_config != INVALID_SOCKET) {
        #ifdef _WIN32
        closesocket(sock_config);
        #else
        ::close(sock_config);
        #endif
        sock_config = INVALID_SOCKET; // 防止重复关闭
    }
}

std::vector<char> DCA1000Config::send_cmd(const std::string& code) {
    std::vector<char> re;
    auto add_uint16_le = [&re](uint16_t val) {
        re.push_back(static_cast<char>(val & 0xFF));
        re.push_back(static_cast<char>((val >> 8) & 0xFF));
    };

    uint16_t header = 0xA55A;
    uint16_t footer = 0xEEAA;
    uint16_t data_size = 0x00;
    std::vector<char> data;

    uint16_t cmd_code = 0;
    if (code == "9") cmd_code = 0x09;
    else if (code == "E") cmd_code = 0x0E;
    else if (code == "3") {
        cmd_code = 0x03;
        data_size = 0x06;
        data = {0x01, 0x02, 0x01, 0x02, 0x03, 0x1E}; // big endian
    } else if (code == "B") {
        cmd_code = 0x0B;
        data_size = 6;
        data = { (char)0xBE, 0x05, 0x35, 0x0C, 0x00, 0x00 }; // adjust if needed
    } else if (code == "5") cmd_code = 0x05;
    else if (code == "6") cmd_code = 0x06;

    add_uint16_le(header);
    add_uint16_le(cmd_code);
    add_uint16_le(data_size);
    re.insert(re.end(), data.begin(), data.end());
    add_uint16_le(footer);

    return re;
}
