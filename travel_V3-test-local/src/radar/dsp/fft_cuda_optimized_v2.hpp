#ifndef FFT_CUDA_OPTIMIZED_V2_HPP
#define FFT_CUDA_OPTIMIZED_V2_HPP

#include <vector>
#include <complex>
#include <cuda_runtime.h>
#include <cufft.h>

// CUDA错误检查宏
#define CUDA_CHECK(call) \
    do { \
        cudaError_t error = call; \
        if (error != cudaSuccess) { \
            throw std::runtime_error("CUDA error: " + std::string(cudaGetErrorString(error))); \
        } \
    } while(0)

#define CUFFT_CHECK(call) \
    do { \
        cufftResult error = call; \
        if (error != CUFFT_SUCCESS) { \
            throw std::runtime_error("cuFFT error: " + std::to_string(error)); \
        } \
    } while(0)

class OptimizedCudaFFTV2 {
private:
    int fft_size_;
    int max_batch_size_;
    
    // 使用cuFFT的批量计划
    cufftHandle plan_batch_forward_ = 0;
    cufftHandle plan_batch_inverse_ = 0;
    
    // GPU内存池 - 预分配大块连续内存
    cufftComplex* d_data_batch_;      // 批量数据
    float* d_window_;                 // 窗函数
    cufftComplex* d_temp_;            // 临时数据
    
    // 固定主机内存（pinned memory）用于快速传输
    cufftComplex* h_pinned_data_;
    
    bool initialized_;
    bool debug_enabled_;
    
    // 性能统计
    mutable float total_transfer_time_;
    mutable float total_compute_time_;
    mutable int batch_count_;

public:
    OptimizedCudaFFTV2(int fft_size, int max_batch_size = 2048, bool enable_debug = false);
    ~OptimizedCudaFFTV2();
    
    // 禁用拷贝
    OptimizedCudaFFTV2(const OptimizedCudaFFTV2&) = delete;
    OptimizedCudaFFTV2& operator=(const OptimizedCudaFFTV2&) = delete;
    
    void forward_batch_optimized(std::vector<std::vector<std::complex<float>>>& data_batch);
    
    // 批量FFT + 窗函数 + FFTShift - 全部在GPU上完成
    void forward_batch_with_window_and_fftshift(
        std::vector<std::vector<std::complex<float>>>& data_batch,
        const std::vector<float>& window,
        bool apply_fftshift = false
    );
    
    // 性能统计
    void print_performance_stats() const;
    void reset_performance_stats();
    
    int get_fft_size() const { return fft_size_; }
    int get_max_batch_size() const { return max_batch_size_; }
};

// 优化的CUDA核函数
extern "C" {
    // 批量应用窗函数
    void launch_optimized_batch_window_kernel(cufftComplex* data, const float* window,
                                             int fft_size, int batch_size);
    
    // 使用共享内存执行批量 FFTShift
    void launch_optimized_batch_fftshift_kernel(cufftComplex* data, int fft_size, int batch_size);
    
    // 组合核函数：窗函数 + FFTShift 合并执行
    void launch_combined_window_fftshift_optimized(cufftComplex* data, const float* window,
                                                   int fft_size, int batch_size,
                                                   bool apply_window, bool apply_fftshift);
}

#endif // FFT_CUDA_OPTIMIZED_V2_HPP
