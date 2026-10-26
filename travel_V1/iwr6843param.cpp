#include "iwr6843param.hpp"
#include <fstream>
#include <sstream>
#include <vector>
#include <bitset>
#include <cmath>
#include <stdexcept>
#include "radar_frame_layout.hpp"

IWR6843AOP_TLV::IWR6843AOP_TLV(const std::string& config_file) {
    std::ifstream file(config_file);
    if (!file) {
        throw std::runtime_error("Failed to open config file for parsing");
    }

    double start_freq = 0, idle_time = 0;
    int num_adc_samples = 0, dig_out_sample_rate = 0;
    bool have_profile = false, have_frame = false, have_channel = false;
    float ramp_end_time = 0.0f;
    double freq_slope_const = 0;
    int chirp_start_idx = 0, chirp_end_idx = 0, num_loops = 0, num_frames = 0;
    float frame_periodicity = 0.0f;
    int num_rx_ant = 4, num_tx_ant = 3;

    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::vector<std::string> split_words;
        std::string word;
        while (iss >> word) {
            split_words.push_back(word);
        }
        if (split_words.empty()) continue;

        if (split_words[0] == "profileCfg") {
            if (split_words.size() < 12) throw std::invalid_argument("Incomplete profileCfg");
            have_profile = true;
            start_freq = std::stod(split_words[2]);
            idle_time = std::stod(split_words[3]);
            ramp_end_time = std::stof(split_words[5]);
            freq_slope_const = std::stod(split_words[8]);
            num_adc_samples = std::stoi(split_words[10]);
            dig_out_sample_rate = std::stoi(split_words[11]);
        } else if (split_words[0] == "frameCfg") {
            if (split_words.size() < 6) throw std::invalid_argument("Incomplete frameCfg");
            have_frame = true;
            chirp_start_idx = std::stoi(split_words[1]);
            chirp_end_idx = std::stoi(split_words[2]);
            num_loops = std::stoi(split_words[3]);
            num_frames = std::stoi(split_words[4]);
            frame_periodicity = std::stof(split_words[5]);
        } else if (split_words[0] == "channelCfg") {
            if (split_words.size() < 3) throw std::invalid_argument("Incomplete channelCfg");
            have_channel = true;
            int rx = std::stoi(split_words[1]);
            int tx = std::stoi(split_words[2]);
            if (rx < 1 || rx > 15 || tx < 1 || tx > 7) throw std::invalid_argument("Invalid channel mask");
            num_rx_ant = std::bitset<4>(rx).count();
            num_tx_ant = std::bitset<4>(tx).count();
        }
    }

    if (!have_profile || !have_frame || !have_channel || !std::isfinite(start_freq) || start_freq <= 0 ||
        !std::isfinite(freq_slope_const) || freq_slope_const <= 0 || !std::isfinite(idle_time) || idle_time < 0 ||
        !std::isfinite(ramp_end_time) || ramp_end_time <= 0 || num_adc_samples < 16 || dig_out_sample_rate <= 0 ||
        num_rx_ant <= 0 || num_tx_ant <= 0 || num_loops <= 0 || num_frames < 0 ||
        !std::isfinite(frame_periodicity) || frame_periodicity <= 0 || chirp_start_idx < 0 ||
        chirp_end_idx < chirp_start_idx || static_cast<long long>(chirp_end_idx) - chirp_start_idx + 1 != num_tx_ant)
        throw std::invalid_argument("Invalid or unsupported radar cfg dimensions/timing");
    RadarParams dimensions;
    dimensions.num_ADC = num_adc_samples; dimensions.num_chirp = num_loops;
    dimensions.num_RX = num_rx_ant; dimensions.num_TX = num_tx_ant;
    RadarFrameLayout::frameBytes(dimensions);
    int num_chirps_per_frame = (chirp_end_idx - chirp_start_idx + 1) * num_loops;
    config_params["numDopplerBins"] = num_chirps_per_frame / num_tx_ant;

    int num_adc_samples_round_to2 = 1;
    while (num_adc_samples > num_adc_samples_round_to2) {
        num_adc_samples_round_to2 *= 2;
    }
    config_params["numRangeBins"] = num_adc_samples_round_to2;

    config_params["rangeResolutionMeters"] = std::round((3e8 * dig_out_sample_rate * 1e3) / (2 * freq_slope_const * 1e12 * num_adc_samples) * 100) / 100;

    config_params["rangeIdxToMeters"] = (3e8 * dig_out_sample_rate * 1e3) / (2 * freq_slope_const * 1e12 * config_params["numRangeBins"]);

    config_params["dopplerResolutionMps"] = std::round(3e8 / (2 * start_freq * 1e9 * (idle_time + ramp_end_time) * 1e-6 * config_params["numDopplerBins"] * num_tx_ant) * 100) / 100;

    config_params["maxRange(m)"] = std::round((300 * 0.9 * dig_out_sample_rate) / (2 * freq_slope_const * 1e3) * 100) / 100;

    config_params["maxVelocity(m/s)"] = std::round(3e8 / (4 * start_freq * 1e9 * (idle_time + ramp_end_time) * 1e-6 * num_tx_ant) * 100) / 100;

    config_params["numADC"] = num_adc_samples;
    config_params["numChirp"] = num_loops;
    config_params["numFrames"] = num_frames;
    config_params["numRX"] = num_rx_ant;
    config_params["numTX"] = num_tx_ant;
    config_params["framePeriodicity"] = frame_periodicity;
    config_params["fc"] = start_freq;                  // GHz
    config_params["fs"] = dig_out_sample_rate;         // ksps
    config_params["idleTime"] = idle_time;             // us
    config_params["rampEndTime"] = ramp_end_time;      // us
    config_params["slopeConst"] = freq_slope_const;    // MHz/us
}