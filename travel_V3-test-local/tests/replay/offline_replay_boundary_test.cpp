#include "offline_odom_replay.hpp"
#include <iostream>
#include <stdexcept>

int main(int argc, char** argv)
{
    if (argc != 2) return 2;
    OfflineOdomReplay replay;
    std::string message;
    if (!replay.loadCsv(argv[1], "boundary.bin", &message)) throw std::runtime_error(message);
    if (replay.frame(1) || replay.frame(2)) throw std::runtime_error("Boundary odom must not be fabricated");
    if (replay.radarTimestampForFrame(1, 100) != "2026-08-14 16:22:52.103")
        throw std::runtime_error("First timestamp estimation wrong");
    if (replay.radarTimestampForFrame(4, 100) != "2026-08-14 16:22:52.403")
        throw std::runtime_error("Last timestamp estimation wrong");
    if (replay.recordedConfigPath() != "./config/recorded.cfg")
        throw std::runtime_error("Recorded cfg metadata missing");
    if (!replay.loadCsv(argv[1], "C:\\moved\\boundary.bin", &message))
        throw std::runtime_error("Windows path basename did not match recording");
    if (replay.loadCsv(argv[1], "other.bin", &message) || replay.loaded())
        throw std::runtime_error("Failed load retained stale motion/config state");
    std::cout << "PASS: boundary timestamp estimates, no invented odom, cfg metadata and failed-load reset\n";
}
