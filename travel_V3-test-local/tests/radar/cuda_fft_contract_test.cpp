#include "fft_cuda_optimized_v2.hpp"
#include "signal_processing.hpp"
#include "radar_frame_layout.hpp"
#include <iostream>
std::chrono::system_clock::time_point capture_start_time;
static void check(bool ok,const char* why) { if(!ok)throw std::runtime_error(why); }
int main() {
    int count=0;
    if(cudaGetDeviceCount(&count)!=cudaSuccess || !count) return 77;
    OptimizedCudaFFTV2 gpu(16,2);
    std::vector<std::vector<std::complex<float>>> batch(2,std::vector<std::complex<float>>(16));
    for(int b=0;b<2;++b)for(int i=0;i<16;++i)batch[b][i]={std::sin(i*.3f+b),std::cos(i*.2f)};
    const auto window=WindowFunctions::kaiser(16,8);
    auto expected=batch;
    OptimizedFFT cpu(16);
    for(auto& values:expected){WindowFunctions::apply_window(values,window);cpu.forward(values);cpu.fftshift(values);}
    gpu.forward_batch_with_window_and_fftshift(batch,window,true);
    for(int b=0;b<2;++b)for(int i=0;i<16;++i)
        check(std::abs(batch[b][i]-expected[b][i])<1e-4f,"CPU/CUDA FFT mismatch");
    batch.resize(3,std::vector<std::complex<float>>(16));bool rejected=false;
    try{gpu.forward_batch_with_window_and_fftshift(batch,window,true);}catch(const std::invalid_argument&){rejected=true;}
    check(rejected,"Oversized CUDA batch accepted");
    batch.resize(2);batch[0].resize(1);rejected=false;
    try{gpu.forward_batch_with_window_and_fftshift(batch,window);}catch(const std::invalid_argument&){rejected=true;}
    check(rejected,"Short CUDA input accepted");
    SignalProcessingFrameContext context;context.persist_outputs=false;
    std::map<std::string,double> parameters={{"fc",60},{"fs",6667},{"slopeConst",60},{"idleTime",10},
        {"rampEndTime",60},{"framePeriodicity",100},{"numRX",4},{"numTX",2}};
    for(int adc:{32,16,64}){
        parameters["numADC"]=adc;parameters["numChirp"]=8;
        auto params=RadarFrameLayout::paramsFromMap(parameters);
        auto result=run_signal_processing_pipeline(std::vector<char>(RadarFrameLayout::frameBytes(params)),parameters,false,&context);
        check(result.diagnostics.processing_ok,"DSP failed after changing CUDA tensor dimensions");
    }
    reset_signal_processing_session();
    std::cout<<"PASS: CPU/CUDA equivalence, batch bounds and DSP dimension changes\n";
}
