#include "radar_frame_layout.hpp"
#include "signal_processing.hpp"
#include <chrono>
#include <iostream>

std::chrono::system_clock::time_point capture_start_time;

static void require(bool condition, const char* text)
{
    if (!condition) throw std::runtime_error(text);
}

int main()
{
    RadarParams params;
    params.num_ADC = 256; params.num_chirp = 64; params.num_RX = 4; params.num_TX = 2;
    require(RadarFrameLayout::frameBytes(params) == 524288, "Wrong complex-IQ frame size");
    bool rejected = false;
    try { convert_raw_to_4darray(std::vector<char>(262144), params); }
    catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, "Short frame did not reject before sample reads");
    params.num_ADC = 2; params.num_chirp = 2; params.num_RX = 1; params.num_TX = 1;
    const int16_t samples[] = {10, 20, 30, 40, 50, 60, 70, 80};
    std::vector<char> bytes(sizeof(samples));
    std::memcpy(bytes.data(), samples, sizeof(samples));
    const auto tensor = convert_raw_to_4darray(bytes, params);
    require(tensor(0, 0, 0, 0) == std::complex<float>(10, 30), "IQ pair 0 wrong");
    require(tensor(1, 0, 0, 0) == std::complex<float>(20, 40), "IQ pair 1 wrong");
    require(tensor(0, 1, 0, 0) == std::complex<float>(50, 70), "Chirp layout wrong");
    params.num_ADC = 3;
    rejected = false;
    try { RadarFrameLayout::frameBytes(params); }
    catch (const std::invalid_argument&) { rejected = true; }
    require(rejected, "Odd ADC sample count accepted");
    OptimizedFFT fft(64);
    std::vector<std::complex<float>> signal(64, {1.0f, 0.0f});
    fft.forward(signal); fft.fftshift(signal);
    const int dc = static_cast<int>(std::max_element(signal.begin(), signal.end(),
        [](auto a, auto b) { return std::abs(a) < std::abs(b); }) - signal.begin());
    require(dc == 32, "FFT DC bin assumption wrong");
    require(RadarFrameLayout::radialVelocity(dc, 64, 0.279018f) == 0.0f, "DC has nonzero speed");
    require(std::abs(RadarFrameLayout::radialVelocity(dc, 64, 0.279018f, true) + 0.279018f) < 1e-6f,
            "Legacy one-bin bias not reproduced");
    for (int offset : {-3, 3}) {
        for (int i = 0; i < 64; ++i) signal[i] = std::polar(1.0f, 2.0f * float(M_PI) * offset * i / 64);
        fft.forward(signal); fft.fftshift(signal);
        const int peak = static_cast<int>(std::max_element(signal.begin(), signal.end(),
            [](auto a, auto b) { return std::abs(a) < std::abs(b); }) - signal.begin());
        require(peak == 32 + offset, "Shifted moving signal bin wrong");
        require(std::abs(RadarFrameLayout::radialVelocity(peak, 64, 0.279018f) - offset * .279018f) < 1e-6f,
                "Moving signal speed wrong");
    }
    std::cout << "PASS: IQ frame size, bounds rejection, decode, FFT DC and +/- Doppler bins\n";
}
