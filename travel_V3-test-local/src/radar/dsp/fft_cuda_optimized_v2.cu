#include "fft_cuda_optimized_v2.hpp"
#include "fft.hpp"
#include <iostream>
#include <iomanip>
#include <chrono>
#include <cmath>
#include <cstring>   

__global__ void optimized_batch_window_kernel(cufftComplex* data, const float* window,
                                             int fft_size, int batch_size) {
    int batch_idx = blockIdx.y;
    int elem_idx = blockIdx.x * blockDim.x + threadIdx.x;
    
    if (batch_idx >= batch_size || elem_idx >= fft_size) return;
    
    int global_idx = batch_idx * fft_size + elem_idx;
    
    // 使用寄存器缓存窗函数值，减少全局内存访问
    float window_val = window[elem_idx];
    
    data[global_idx].x *= window_val;
    data[global_idx].y *= window_val;
}

__global__ void optimized_batch_fftshift_kernel(cufftComplex* data, int fft_size, int batch_size) {
    int batch_idx = blockIdx.y;
    int elem_idx = blockIdx.x * blockDim.x + threadIdx.x;
    
    if (batch_idx >= batch_size || elem_idx >= fft_size) return;
    
    int half_size = fft_size / 2;
    
    // 只处理前一半元素，避免重复交换
    if (elem_idx >= half_size) return;
    
    int src_idx = batch_idx * fft_size + elem_idx;
    int dst_idx = batch_idx * fft_size + elem_idx + half_size;
    
    cufftComplex temp = data[src_idx];
    data[src_idx] = data[dst_idx];
    data[dst_idx] = temp;
}

__global__ void combined_window_fftshift_optimized_kernel(cufftComplex* data, const float* window,
                                                        int fft_size, int batch_size,
                                                        bool apply_window, bool apply_fftshift) {
    int batch_idx = blockIdx.y;
    int elem_idx = blockIdx.x * blockDim.x + threadIdx.x;
    
    if (batch_idx >= batch_size || elem_idx >= fft_size) return;
    
    int global_idx = batch_idx * fft_size + elem_idx;
    
    // 应用窗函数
    if (apply_window) {
        float window_val = window[elem_idx];
        data[global_idx].x *= window_val;
        data[global_idx].y *= window_val;
    }
    
    // FFTShift - 使用更高效的实现
    if (apply_fftshift) {
        int half_size = fft_size / 2;
        if (elem_idx < half_size) {
            int swap_idx = batch_idx * fft_size + elem_idx + half_size;
            cufftComplex temp = data[global_idx];
            data[global_idx] = data[swap_idx];
            data[swap_idx] = temp;
        }
    }
}

// 核函数启动器
extern "C" {
    void launch_optimized_batch_window_kernel(cufftComplex* data, const float* window,
                                             int fft_size, int batch_size) {
        // 优化的网格和块配置
        int threads_per_block = 256;
        dim3 block_size(threads_per_block);
        dim3 grid_size((fft_size + threads_per_block - 1) / threads_per_block, batch_size);
        
        optimized_batch_window_kernel<<<grid_size, block_size>>>(data, window, fft_size, batch_size);
        // 不等待同步，让后续操作并行
    }
    
    void launch_optimized_batch_fftshift_kernel(cufftComplex* data, int fft_size, int batch_size) {
        int threads_per_block = 256;
        dim3 block_size(threads_per_block);
        dim3 grid_size((fft_size + threads_per_block - 1) / threads_per_block, batch_size);
        
        optimized_batch_fftshift_kernel<<<grid_size, block_size>>>(data, fft_size, batch_size);
    }
    
    void launch_combined_window_fftshift_optimized(cufftComplex* data, const float* window,
                                                   int fft_size, int batch_size,
                                                   bool apply_window, bool apply_fftshift) {
        int threads_per_block = 256;
        dim3 block_size(threads_per_block);
        dim3 grid_size((fft_size + threads_per_block - 1) / threads_per_block, batch_size);
        
        combined_window_fftshift_optimized_kernel<<<grid_size, block_size>>>(
            data, window, fft_size, batch_size, apply_window, apply_fftshift);
    }
}

// OptimizedCudaFFTV2 类实现
OptimizedCudaFFTV2::OptimizedCudaFFTV2(int fft_size, int max_batch_size, bool enable_debug)
    : fft_size_(fft_size), max_batch_size_(max_batch_size),
      d_data_batch_(nullptr), d_window_(nullptr), d_temp_(nullptr), h_pinned_data_(nullptr),
      initialized_(false), debug_enabled_(enable_debug),
      total_transfer_time_(0), total_compute_time_(0), batch_count_(0) {
    
    if (max_batch_size <= 0 || fft_size <= 0 || (fft_size & (fft_size - 1)) != 0) {
        throw std::runtime_error("FFT size must be a positive power of 2");
    }
    
    try {
        // 分配GPU内存 - 大块连续内存
        size_t batch_data_size = static_cast<size_t>(max_batch_size) * fft_size * sizeof(cufftComplex);
        CUDA_CHECK(cudaMalloc((void**)&d_data_batch_, batch_data_size));
        CUDA_CHECK(cudaMalloc((void**)&d_window_, fft_size * sizeof(float)));
        CUDA_CHECK(cudaMalloc((void**)&d_temp_, batch_data_size));
        
        // 分配固定主机内存（pinned memory）- 提高传输速度
        CUDA_CHECK(cudaMallocHost((void**)&h_pinned_data_, batch_data_size));
        
        int rank = 1;
        int n[] = {fft_size};
        int istride = 1, ostride = 1;
        int idist = fft_size, odist = fft_size;
        int inembed[] = {fft_size}, onembed[] = {fft_size};
        
        // 批量前向FFT计划
        CUFFT_CHECK(cufftPlanMany(&plan_batch_forward_, rank, n,
                                 inembed, istride, idist,
                                 onembed, ostride, odist,
                                 CUFFT_C2C, max_batch_size));
        
        // 批量逆向FFT计划
        CUFFT_CHECK(cufftPlanMany(&plan_batch_inverse_, rank, n,
                                 inembed, istride, idist,
                                 onembed, ostride, odist,
                                 CUFFT_C2C, max_batch_size));
        
        // Use the same stream for copies, windowing, FFT and fftshift.
        CUFFT_CHECK(cufftSetStream(plan_batch_forward_, 0));
        CUFFT_CHECK(cufftSetStream(plan_batch_inverse_, 0));

        initialized_ = true;
        
        if (debug_enabled_) {
            std::cout << "[DEBUG] 高度优化CUDA FFT V2初始化成功:" << std::endl;
            std::cout << "  FFT大小: " << fft_size << std::endl;
            std::cout << "  最大批量: " << max_batch_size << std::endl;
            std::cout << "  批量数据内存: " << batch_data_size / (1024*1024) << " MB" << std::endl;
            std::cout << "  固定主机内存: " << batch_data_size / (1024*1024) << " MB" << std::endl;
        }
        
    } catch (const std::exception& e) {
        // 清理已分配的资源
        if (d_data_batch_) cudaFree(d_data_batch_);
        if (d_window_) cudaFree(d_window_);
        if (d_temp_) cudaFree(d_temp_);
        if (h_pinned_data_) cudaFreeHost(h_pinned_data_);
        if (plan_batch_forward_) cufftDestroy(plan_batch_forward_);
        if (plan_batch_inverse_) cufftDestroy(plan_batch_inverse_);
        throw;
    }
}

OptimizedCudaFFTV2::~OptimizedCudaFFTV2() {
    if (initialized_) {
        if (debug_enabled_) {
            std::cout << "[DEBUG] 销毁高度优化CUDA FFT V2资源" << std::endl;
        }
        
        if (d_data_batch_) cudaFree(d_data_batch_);
        if (d_window_) cudaFree(d_window_);
        if (d_temp_) cudaFree(d_temp_);
        if (h_pinned_data_) cudaFreeHost(h_pinned_data_);
        
        cufftDestroy(plan_batch_forward_);
        cufftDestroy(plan_batch_inverse_);
    }
}

void OptimizedCudaFFTV2::forward_batch_optimized(std::vector<std::vector<std::complex<float>>>& data_batch) {
    if (!initialized_) {
        throw std::runtime_error("OptimizedCudaFFTV2 not properly initialized");
    }
    
    int batch_size = data_batch.size();
    if (batch_size <= 0 || batch_size > max_batch_size_) {
        throw std::runtime_error("Batch size exceeds maximum");
    }
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // 1. 快速数据打包到固定内存
    for (int i = 0; i < batch_size; ++i) {
        if (data_batch[i].size() != fft_size_) {
            throw std::runtime_error("Input size does not match FFT size");
        }
        memcpy(h_pinned_data_ + i * fft_size_, data_batch[i].data(),
                   fft_size_ * sizeof(cufftComplex));
    }
    
    // 2. 异步传输到GPU（使用固定内存提高传输速度）
    CUDA_CHECK(cudaMemcpyAsync(d_data_batch_, h_pinned_data_,
                               batch_size * fft_size_ * sizeof(cufftComplex),
                               cudaMemcpyHostToDevice, 0));
    
    auto transfer_to_gpu_time = std::chrono::high_resolution_clock::now();
    
    // 3. 执行批量FFT（一次cuFFT调用处理所有数据）
    CUFFT_CHECK(cufftExecC2C(plan_batch_forward_, d_data_batch_, d_data_batch_, CUFFT_FORWARD));
    
    auto compute_time = std::chrono::high_resolution_clock::now();
    
    // 4. 异步传输回CPU
    CUDA_CHECK(cudaMemcpyAsync(h_pinned_data_, d_data_batch_,
                               batch_size * fft_size_ * sizeof(cufftComplex),
                               cudaMemcpyDeviceToHost, 0));
    
    // 5. 等待所有异步操作完成
    CUDA_CHECK(cudaStreamSynchronize(0));
    
    auto transfer_from_gpu_time = std::chrono::high_resolution_clock::now();
    
    // 6. 快速数据解包
    for (int i = 0; i < batch_size; ++i) {
        memcpy(data_batch[i].data(), h_pinned_data_ + i * fft_size_,
                   fft_size_ * sizeof(cufftComplex));
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    
    // 更新性能统计
    float transfer_ms = std::chrono::duration<float, std::milli>(transfer_to_gpu_time - start_time).count() +
                       std::chrono::duration<float, std::milli>(end_time - transfer_from_gpu_time).count();
    float compute_ms = std::chrono::duration<float, std::milli>(compute_time - transfer_to_gpu_time).count();
    
    total_transfer_time_ += transfer_ms;
    total_compute_time_ += compute_ms;
    batch_count_++;
    
    if (debug_enabled_) {
        float total_ms = std::chrono::duration<float, std::milli>(end_time - start_time).count();
        std::cout << "[DEBUG] 高度优化批量FFT (size=" << fft_size_ << ", batch=" << batch_size << "):" << std::endl;
        std::cout << "  数据传输: " << std::fixed << std::setprecision(3) << transfer_ms << " ms" << std::endl;
        std::cout << "  FFT计算: " << compute_ms << " ms" << std::endl;
        std::cout << "  总时间: " << total_ms << " ms" << std::endl;
        std::cout << "  平均每个FFT: " << total_ms / batch_size << " ms" << std::endl;
        std::cout << "  效率比: " << (compute_ms / total_ms * 100) << "%" << std::endl;
    }
}

void OptimizedCudaFFTV2::forward_batch_with_window_and_fftshift(
    std::vector<std::vector<std::complex<float>>>& data_batch,
    const std::vector<float>& window,
    bool apply_fftshift) {
    
    if (!initialized_ || data_batch.empty() || data_batch.size() > static_cast<std::size_t>(max_batch_size_))
        throw std::invalid_argument("Invalid FFT batch count");
    for (const auto& values : data_batch)
        if (values.size() != static_cast<std::size_t>(fft_size_))
            throw std::invalid_argument("FFT batch element size mismatch");
    if (window.size() != fft_size_) {
        throw std::runtime_error("Window size does not match FFT size");
    }
    
    int batch_size = data_batch.size();
    auto start_time = std::chrono::high_resolution_clock::now();
    
    // 上传窗函数到GPU（只需一次）
    CUDA_CHECK(cudaMemcpyAsync(d_window_, window.data(), fft_size_ * sizeof(float),
                               cudaMemcpyHostToDevice, 0));
    
    // 数据打包和传输 - 使用更高效的方法，减少内存复制
    cufftComplex* h_ptr = h_pinned_data_;
    if (batch_size <= max_batch_size_) {
        for (int i = 0; i < batch_size; ++i) {
            const auto& data_vec = data_batch[i];
            std::memcpy(h_ptr, data_vec.data(), fft_size_ * sizeof(cufftComplex));
            h_ptr += fft_size_;
        }
    }
    
    CUDA_CHECK(cudaMemcpyAsync(d_data_batch_, h_pinned_data_,
                               batch_size * fft_size_ * sizeof(cufftComplex),
                               cudaMemcpyHostToDevice, 0));
    
    auto transfer_time = std::chrono::high_resolution_clock::now();
    
    // 在GPU上执行：窗函数 + FFT + FFTShift
    launch_combined_window_fftshift_optimized(d_data_batch_, d_window_, fft_size_, batch_size,
                                             true, false);  // 先应用窗函数
    
    // 同步确保窗函数完成
    CUDA_CHECK(cudaDeviceSynchronize());
    
    // 批量FFT
    CUFFT_CHECK(cufftExecC2C(plan_batch_forward_, d_data_batch_, d_data_batch_, CUFFT_FORWARD));
    
    // FFTShift（如果需要）
    if (apply_fftshift) {
        launch_optimized_batch_fftshift_kernel(d_data_batch_, fft_size_, batch_size);
    }
    
    auto compute_time = std::chrono::high_resolution_clock::now();
    
    // 传输回CPU
    CUDA_CHECK(cudaMemcpyAsync(h_pinned_data_, d_data_batch_,
                               batch_size * fft_size_ * sizeof(cufftComplex),
                               cudaMemcpyDeviceToHost, 0));
    CUDA_CHECK(cudaStreamSynchronize(0));
    
    // 数据解包 - 使用更高效的方法
    cufftComplex* h_ptr_unpack = h_pinned_data_;
    for (int i = 0; i < batch_size; ++i) {
        memcpy(data_batch[i].data(), h_ptr_unpack, fft_size_ * sizeof(cufftComplex));
        h_ptr_unpack += fft_size_;
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    
    if (debug_enabled_) {
        float transfer_ms = std::chrono::duration<float, std::milli>(transfer_time - start_time).count() +
                           std::chrono::duration<float, std::milli>(end_time - compute_time).count();
        float compute_ms = std::chrono::duration<float, std::milli>(compute_time - transfer_time).count();
        float total_ms = std::chrono::duration<float, std::milli>(end_time - start_time).count();
        
        std::cout << "[DEBUG] 一体化窗函数+FFT+FFTShift (batch=" << batch_size << "):" << std::endl;
        std::cout << "  总时间: " << std::fixed << std::setprecision(3) << total_ms << " ms" << std::endl;
        std::cout << "  计算占比: " << (compute_ms / total_ms * 100) << "%" << std::endl;
    }
}

void OptimizedCudaFFTV2::print_performance_stats() const {
    if (batch_count_ == 0) {
        std::cout << "[性能统计] 未执行任何批量操作" << std::endl;
        return;
    }
    
    std::cout << "\n[性能统计] 高度优化CUDA FFT V2:" << std::endl;
    std::cout << "  总批次数: " << batch_count_ << std::endl;
    std::cout << "  平均传输时间: " << std::fixed << std::setprecision(3)
              << total_transfer_time_ / batch_count_ << " ms/batch" << std::endl;
    std::cout << "  平均计算时间: " << total_compute_time_ / batch_count_ << " ms/batch" << std::endl;
    std::cout << "  计算效率: " << (total_compute_time_ / (total_transfer_time_ + total_compute_time_) * 100)
              << "%" << std::endl;
}

void OptimizedCudaFFTV2::reset_performance_stats() {
    total_transfer_time_ = 0;
    total_compute_time_ = 0;
    batch_count_ = 0;
}
