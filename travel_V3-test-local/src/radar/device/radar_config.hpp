#ifndef RADAR_CONFIG_H
#define RADAR_CONFIG_H

#include <string>
#include <vector>
#include <cstdint>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#include <windows.h>
#else
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netinet/in.h>  
#include <arpa/inet.h>   
#define SOCKET int
#define INVALID_SOCKET -1
#define closesocket close
#endif

class AWR2944SerialConfig {
private:
#ifdef _WIN32
    HANDLE cli_port;
#else
    int fd;
    struct termios old_tio;
#endif
    bool opened;

public:
    AWR2944SerialConfig(const std::string& port, int baudrate);
    ~AWR2944SerialConfig();
    void SendConfig(const std::string& config_file);
    void close();
    bool is_open() const { return opened; }

#ifdef _WIN32
    HANDLE get_cli_port() const { return cli_port; }
#else
    int get_fd() const { return fd; }
#endif
};

class DCA1000Config {
private:
    SOCKET sock_config = INVALID_SOCKET;
    bool transact(const std::string& code, std::string* error = nullptr);
    std::vector<std::uint16_t> pending_async_status_;
    struct sockaddr_in config_addr;
    struct sockaddr_in fpga_addr_cfg;
    static struct sockaddr_in response_addr; // Shared response address
    static bool response_addr_set; // Flag to track if response_addr is set

public:
    DCA1000Config(const std::string& name, const std::string& config_ip, int config_port,
                  const std::string& fpga_ip, int fpga_port);
    ~DCA1000Config();
    bool DCA1000_start(std::string* error = nullptr);
    bool DCA1000_close(std::string* error = nullptr);
    // Called by the GUI thread; does not send commands or block for data.
    std::vector<std::uint16_t> pollAsyncStatus();
    static std::string describeAsyncStatus(std::uint16_t status);
    void close();
    std::vector<char> send_cmd(const std::string& code);
};

#endif // RADAR_CONFIG_H
