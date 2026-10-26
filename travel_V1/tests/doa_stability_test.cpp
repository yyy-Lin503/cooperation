#include "DoA.hpp"
#include <limits>
#include <random>

static void check(bool ok, const char* msg) { if(!ok) throw std::runtime_error(msg); }
static std::vector<std::complex<float>> source(double angle, double amplitude=1, double phase0=0,
                                             double doppler=0, int txs=1, int rxs=8) {
    std::vector<std::complex<float>> s;
    for(int i=0;i<txs*rxs;++i) {
        double phase=-M_PI*std::sin(angle*M_PI/180)*i+phase0+2*M_PI*doppler*(i/rxs)/(64*txs);
        auto z=std::polar(amplitude,phase);
        s.emplace_back(float(z.real()),float(z.imag()));
    }
    return s;
}
int main() {
    check(!DoA::estimate_azimuth({}).valid,"Empty input accepted");
    check(!DoA::estimate_azimuth(std::vector<std::complex<float>>(8)).valid,"Zero energy accepted");
    auto bad=source(0);bad[0]={std::numeric_limits<float>::quiet_NaN(),0};
    check(!DoA::estimate_azimuth(bad).valid,"NaN accepted");
    bad=source(0);bad[0]={std::numeric_limits<float>::infinity(),0};
    check(!DoA::estimate_azimuth(bad).valid,"Inf accepted");
    DoA::AzimuthOptions wrong;wrong.num_tx=2;
    check(!DoA::estimate_azimuth(source(0),wrong).valid,"Invalid TX layout accepted");
    double max_error=0,max_jitter=0;
    std::mt19937 rng(20261020);std::normal_distribution<float> noise(0,1);
    for(double angle:{-80.,-60.,-45.,-30.,-15.,0.,15.,30.,45.,60.,80.}) {
        for(double scale:{1e-30,1e-6,1.,1e6,1e30}) {
            auto estimate=DoA::estimate_azimuth(source(angle,scale,.37));
            check(estimate.valid,"Ideal source rejected");
            max_error=std::max(max_error,std::abs(estimate.azimuth_deg-angle));
            check(std::abs(estimate.azimuth_deg-angle)<.02,"Ideal source angle incorrect");
        }
        const auto signal=source(angle);
        const auto baseline=DoA::estimate_azimuth(signal);
        for(int trial=0;trial<100;++trial) {
            auto s=signal;
            for(auto&z:s)z+=1e-6f*std::complex<float>(noise(rng),noise(rng));
            auto estimate=DoA::estimate_azimuth(s);
            check(estimate.valid,"Micro-perturbed source rejected");
            max_jitter=std::max(max_jitter,double(std::abs(estimate.azimuth_deg-baseline.azimuth_deg)));
            check(std::abs(estimate.azimuth_deg-baseline.azimuth_deg)<.01,"Angle unstable under micro perturbation");
        }
        const auto fft=DoA::fft_doa(signal);
        check(!fft.empty()&&std::abs(fft[0]-angle)<.6,"FFT sine mapping incorrect");
        for(double doppler:{-25.,-6.,0.,6.,25.}) {
            DoA::AzimuthOptions options;options.num_tx=2;options.rx_per_tx=4;
            options.doppler_bins=64;options.doppler_offset_bins=doppler;
            auto estimate=DoA::estimate_azimuth(source(angle,1,.13,doppler,2,4),options);
            check(estimate.valid&&std::abs(estimate.azimuth_deg-angle)<.02,"TDM phase compensation incorrect");
        }
    }
    auto mixed=source(-20);auto second=source(35,.4);
    for(int i=0;i<8;++i)mixed[i]+=second[i];
    auto estimate=DoA::estimate_azimuth(mixed);
    check(estimate.valid&&std::isfinite(estimate.azimuth_deg),"Two-source input produced invalid output");
    std::cout<<"PASS: DOA invalid inputs, scale/phase invariance, 11 angles, 1100 perturbations, FFT sine mapping and 55 TDM cases\n"
             <<"max_ideal_error_deg="<<max_error<<" max_perturbation_delta_deg="<<max_jitter<<'\n';
}
