#ifndef DOA_HPP
#define DOA_HPP

#include <complex>
#include <vector>
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <numeric>
#include <map>
#include <cmath>
#include <iostream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


namespace DoA {
    // IWR6843ISK: TX mask 5 (zero-based TX0/TX2), four RX -> eight azimuth channels.
    // Half-wavelength ULA model, NOT a per-board phase or mounting calibration.
    // Input is raw TX-major virtual antennas, never concatenated subarrays.
    struct AzimuthOptions {
        int num_tx = 1;
        int rx_per_tx = 0;
        int doppler_bins = 0;
        double doppler_offset_bins = 0;
        bool compensate_tdm = true;
        int azimuth_sign = -1; // exp(-j*pi*n*sin(theta)) -> +theta.
    };
    struct AzimuthEstimate {
        bool valid = false;
        float azimuth_deg = 0;
        float peak_confidence = 0;
        float coherent_power_ratio = 0;
    };
    AzimuthEstimate estimate_azimuth(const std::vector<std::complex<float>>& signal,
                                    const AzimuthOptions& options = {});

    // Diagnostic confidence for the strongest spatial-spectrum peak.  The
    // value is normalized to 0..1 from the peak-to-best-competing-lobe margin;
    // it is logged for ambiguity analysis and is not currently a hard gate.
    float peak_confidence(const std::vector<std::complex<float>>& sig);

    // FFT-based DoA estimation
    std::vector<float> fft_doa(const std::vector<std::complex<float>>& sig);
}

#endif
