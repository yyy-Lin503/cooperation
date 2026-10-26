#include "offline_odom_replay.hpp"

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

int main(int argc, char** argv)
{
    if (argc < 3) {
        std::cerr << "usage: offline_odom_replay_test <bin> <diagnostic_dir>...\n";
        return 2;
    }
    std::vector<std::string> directories;
    for (int i = 2; i < argc; ++i) directories.emplace_back(argv[i]);

    OfflineOdomReplay replay;
    std::string message;
    if (!replay.loadForBin(argv[1], directories, &message)) {
        std::cerr << "FAIL: " << message << '\n';
        return 1;
    }
    const OfflineMotionFrame* first = nullptr;
    for (int frame = 1; frame < 100000; ++frame) {
        first = replay.frame(frame);
        if (first) break;
    }
    if (!first || (!first->has_dsp_sample && !first->has_tracking_sample)) {
        std::cerr << "FAIL: no synchronized motion sample loaded\n";
        return 1;
    }
    std::cout << "PASS: " << message
              << " first_frame=" << first->frame
              << " dsp_vx=" << first->dsp_sample.vx_mps
              << " dsp_wz=" << first->dsp_sample.wz_radps
              << " tracking_vx=" << first->tracking_sample.vx_mps
              << " tracking_wz=" << first->tracking_sample.wz_radps
              << '\n';
    return 0;
}
