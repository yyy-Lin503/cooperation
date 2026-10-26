#ifndef RADAR_FRAME_LAYOUT_HPP
#define RADAR_FRAME_LAYOUT_HPP

#include "fft.hpp"
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>

namespace RadarFrameLayout {
inline std::size_t frameBytes(const RadarParams& params)
{
    if (params.num_ADC <= 0 || params.num_chirp <= 0 || params.num_RX <= 0 || params.num_TX <= 0 ||
        params.num_ADC % 2 != 0 || (params.num_ADC & (params.num_ADC - 1)) != 0 ||
        (params.num_chirp & (params.num_chirp - 1)) != 0)
        throw std::invalid_argument("Invalid ADC/chirp/RX/TX dimensions for complex-IQ FFT");
    std::size_t samples = 1;
    for (int dimension : {params.num_ADC, params.num_chirp, params.num_RX, params.num_TX}) {
        if (samples > static_cast<std::size_t>(std::numeric_limits<int>::max()) / dimension)
            throw std::invalid_argument("Radar tensor dimensions overflow");
        samples *= static_cast<std::size_t>(dimension);
    }
    const std::size_t bytes_per_sample = 2 * sizeof(std::int16_t);
    if (samples > static_cast<std::size_t>(std::numeric_limits<int>::max()) / bytes_per_sample)
        throw std::invalid_argument("Radar frame byte count exceeds supported int size");
    return samples * bytes_per_sample;
}
inline RadarParams paramsFromMap(const std::map<std::string, double>& values)
{
    RadarParams result;
    auto integer = [&values](const char* name) {
        const double value = values.at(name);
        if (!std::isfinite(value) || value < 1 || value > std::numeric_limits<int>::max() ||
            value != std::floor(value)) throw std::invalid_argument(std::string("Invalid ") + name);
        return static_cast<int>(value);
    };
    result.num_ADC = integer("numADC");
    result.num_chirp = integer("numChirp");
    result.num_RX = integer("numRX");
    result.num_TX = integer("numTX");
    frameBytes(result);
    return result;
}
inline float dopplerZeroBin(int count, bool legacy_offset = false)
{
    // fftshift puts DC at the zero-based index N/2, not N/2+1.
    return static_cast<float>(count / 2 + (legacy_offset ? 1 : 0));
}
inline float radialVelocity(int bin, int count, float resolution, bool legacy_offset = false)
{
    return (static_cast<float>(bin) - dopplerZeroBin(count, legacy_offset)) * resolution;
}
} // namespace RadarFrameLayout
#endif
