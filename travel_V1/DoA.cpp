#include "DoA.hpp"
#include "fft.hpp"
#include <array>
#include <limits>

namespace DoA {
namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr int kMaxAntennas = 8;
constexpr double kAngleStepDeg = 0.25;
constexpr int kScanCount = 721; // -90..90 degrees, not a linear FFT-bin-to-degree map.
using Sample = std::complex<double>;
using Weights = std::array<std::array<Sample, kMaxAntennas>, kScanCount>;

const Weights& weights() {
    // Retain the old FFT convention: exp(-j*pi*n*sin(theta)) -> +theta.
    // Physical board ordering/left-right polarity must still be calibrated.
    static const Weights table = [] {
        Weights value{};
        for (int a = 0; a < kScanCount; ++a) {
            const double theta = (-90.0 + a * kAngleStepDeg) * kPi / 180.0;
            for (int n = 0; n < kMaxAntennas; ++n)
                value[a][n] = std::polar(1.0, kPi * std::sin(theta) * n);
        }
        return value;
    }();
    return table;
}

bool prepare(const std::vector<std::complex<float>>& input,
             const AzimuthOptions& options, std::vector<Sample>& signal) {
    if (input.size() < 3 || input.size() > kMaxAntennas || options.num_tx < 1 ||
        (options.azimuth_sign != -1 && options.azimuth_sign != 1)) return false;
    const int rx = options.rx_per_tx > 0 ? options.rx_per_tx : int(input.size());
    if (rx < 1 || rx > int(input.size()) || options.num_tx > int(input.size()) ||
        rx * options.num_tx != int(input.size()) ||
        !std::isfinite(options.doppler_offset_bins)) return false;
    if (options.compensate_tdm && options.num_tx > 1 && options.doppler_bins < 1) return false;
    double scale = 0;
    for (auto x : input) {
        if (!std::isfinite(x.real()) || !std::isfinite(x.imag())) return false;
        scale = std::max(scale, std::abs(Sample(x.real(), x.imag())));
    }
    if (!(scale > 0)) return false;
    signal.reserve(input.size());
    for (int i = 0; i < int(input.size()); ++i) {
        Sample value(input[i].real() / scale, input[i].imag() / scale);
        if (options.compensate_tdm && options.num_tx > 1) {
            // TX-major TDM input: TX t is sampled t chirp periods later.
            const int tx = i / rx;
            const double phase = -2.0 * kPi * options.doppler_offset_bins * tx /
                                 (options.doppler_bins * options.num_tx);
            value *= std::polar(1.0, phase);
        }
        signal.push_back(value);
    }
    return true;
}
}

AzimuthEstimate estimate_azimuth(const std::vector<std::complex<float>>& input,
                                 const AzimuthOptions& options) {
    AzimuthEstimate result;
    std::vector<Sample> signal;
    if (!prepare(input, options, signal)) return result;
    const auto& table = weights();
    std::array<double, kScanCount> spectrum{};
    double energy = 0;
    for (auto x : signal) energy += std::norm(x);
    int best = 0;
    for (int a = 0; a < kScanCount; ++a) {
        Sample sum = 0;
        for (int n = 0; n < int(signal.size()); ++n) sum += signal[n] * table[a][n];
        spectrum[a] = std::norm(sum);
        if (spectrum[a] > spectrum[best]) best = a;
    }
    if (!(spectrum[best] > 0) || !std::isfinite(spectrum[best])) return result;
    double fraction = 0;
    if (best > 0 && best + 1 < kScanCount) {
        const double left = spectrum[best-1], center = spectrum[best], right = spectrum[best+1];
        const double curvature = left - 2*center + right;
        if (curvature < -1e-12 * center)
            fraction = std::clamp(0.5*(left-right)/curvature, -0.5, 0.5);
    }
    double competitor = 0;
    constexpr int exclusion = int(12.0/kAngleStepDeg);
    for (int a = 0; a < kScanCount; ++a)
        if (std::abs(a-best) > exclusion) competitor = std::max(competitor, spectrum[a]);
    const double margin_db = 10.0 * std::log10((spectrum[best]+1e-12)/(competitor+1e-12));
    const double angle = -90.0 + (best + fraction) * kAngleStepDeg;
    result.valid = true;
    result.azimuth_deg = float(-options.azimuth_sign * angle);
    result.peak_confidence = float(std::clamp(margin_db/12.0, 0.0, 1.0));
    result.coherent_power_ratio = float(std::clamp(spectrum[best]/(signal.size()*energy), 0.0, 1.0));
    return result;
}

float peak_confidence(const std::vector<std::complex<float>>& signal) {
    return estimate_azimuth(signal).peak_confidence;
}

std::vector<float> fft_doa(const std::vector<std::complex<float>>& input) {
    // Secondary cross-check; the pipeline uses estimate_azimuth, not this FFT.
    std::vector<Sample> prepared;
    if (!prepare(input, AzimuthOptions{}, prepared)) return {};
    constexpr int count = 1024;
    std::vector<std::complex<float>> padded(count, {0,0});
    for (int i=0; i<int(prepared.size()); ++i)
        padded[i] = {float(prepared[i].real()),float(prepared[i].imag())};
    OptimizedFFT fft(count);
    fft.forward(padded);
    fft.fftshift(padded);
    const auto best=std::max_element(padded.begin(),padded.end(),
        [](auto a,auto b){return std::norm(a)<std::norm(b);});
    const int bin = int(best-padded.begin())-count/2;
    const double sine = std::clamp(-2.0*bin/count,-1.0,1.0);
    return {float(std::asin(sine)*180.0/kPi)};
}
}
