#include "fft.hpp"

int FastFFT::bit_reverse(int num, int bits) {
    int result = 0;
    for (int i = 0; i < bits; i++) {
        result = (result << 1) | (num & 1);
        num >>= 1;
    }
    return result;
}

void FastFFT::fft(std::vector<std::complex<float>>& x, bool inverse) {
    int N = x.size();
    
    if (N <= 0 || (N & (N - 1)) != 0) {
        throw std::invalid_argument("FFT size must be power of 2");
    }
    
    int bits = static_cast<int>(std::log2(N));
    
    // 位反转重排
    for (int i = 0; i < N; i++) {
        int j = bit_reverse(i, bits);
        if (i < j) {
            std::swap(x[i], x[j]);
        }
    }
    
    // 迭代FFT蝶形运算
    for (int len = 2; len <= N; len <<= 1) {
        float angle = 2 * M_PI / len * (inverse ? 1 : -1);
        std::complex<float> wlen(std::cos(angle), std::sin(angle));
        
        for (int i = 0; i < N; i += len) {
            std::complex<float> w(1);
            for (int j = 0; j < len / 2; j++) {
                std::complex<float> u = x[i + j];
                std::complex<float> v = x[i + j + len / 2] * w;
                x[i + j] = u + v;
                x[i + j + len / 2] = u - v;
                w *= wlen;
            }
        }
    }
    
    // 逆变换归一化
    if (inverse) {
        for (auto& val : x) {
            val /= N;
        }
    }
}

void FastFFT::forward(std::vector<std::complex<float>>& x) {
    fft(x, false);
}

void FastFFT::inverse(std::vector<std::complex<float>>& x) {
    fft(x, true);
}

OptimizedFFT::OptimizedFFT(int size) : size_(size) {
    if (size <= 0 || (size & (size - 1)) != 0) {
        throw std::invalid_argument("FFT size must be power of 2");
    }
    precompute_factors();
}

void OptimizedFFT::precompute_factors() {
    int bits = static_cast<int>(std::log2(size_));
    
    // 预计算位反转表
    bit_reverse_table_.resize(size_);
    for (int i = 0; i < size_; i++) {
        int result = 0;
        int num = i;
        for (int j = 0; j < bits; j++) {
            result = (result << 1) | (num & 1);
            num >>= 1;
        }
        bit_reverse_table_[i] = result;
    }
    
    // 预计算旋转因子
    twiddle_factors_.resize(size_);
    for (int i = 0; i < size_; i++) {
        float angle = -2 * M_PI * i / size_;
        twiddle_factors_[i] = std::complex<float>(std::cos(angle), std::sin(angle));
    }
}

void OptimizedFFT::forward(std::vector<std::complex<float>>& x) {
    if (x.size() != size_) {
        throw std::invalid_argument("Input size mismatch");
    }
    
    // 位反转重排
    std::vector<std::complex<float>> temp(size_);
    for (int i = 0; i < size_; i++) {
        temp[i] = x[bit_reverse_table_[i]];
    }
    x = std::move(temp);
    
    // 迭代FFT
    for (int len = 2; len <= size_; len <<= 1) {
        int step = size_ / len;
        for (int i = 0; i < size_; i += len) {
            for (int j = 0; j < len / 2; j++) {
                std::complex<float> u = x[i + j];
                std::complex<float> v = x[i + j + len / 2] * twiddle_factors_[j * step];
                x[i + j] = u + v;
                x[i + j + len / 2] = u - v;
            }
        }
    }
}

void OptimizedFFT::inverse(std::vector<std::complex<float>>& x) {
    if (x.size() != size_) {
        throw std::invalid_argument("Input size mismatch");
    }
    
    // 共轭输入
    for (auto& val : x) {
        val = std::conj(val);
    }
    
    // 执行正向FFT
    forward(x);
    
    // 共轭结果并归一化
    for (auto& val : x) {
        val = std::conj(val) / static_cast<float>(size_);
    }
}

void OptimizedFFT::fftshift(std::vector<std::complex<float>>& x) {
    int N = x.size();
    int shift = N / 2;  // 如果N是偶数，就是N/2；如果N是奇数，则是(N/2)的整数部分
    if (N % 2) shift++;  

    // 标准做法：不论奇偶，都是将前半部分（0到shift-1）和后半部分（shift到末尾）交换
    std::vector<std::complex<float>> temp;
    temp.reserve(N);
    // 将原数组的后半部分放入临时向量
    temp.insert(temp.end(), x.begin() + shift, x.end());
    // 将原数组的前半部分放入临时向量
    temp.insert(temp.end(), x.begin(), x.begin() + shift);
    x = temp;
}


// 计算修正贝塞尔函数 I0(x) - 使用级数展开近似
static float modifiedBessel0(float x) {
    if (x == 0.0f) return 1.0f;
    
    float tolerance = 1e-8f;
    float sum = 1.0f;
    float term = 1.0f;
    float x_half_squared = (x * 0.5f) * (x * 0.5f);
    
    for (int k = 1; k < 50; k++) {  // 最多50项，通常10项就足够了
        term *= x_half_squared / (k * k);
        sum += term;
        
        if (term < tolerance * sum) {
            break;  // 收敛了
        }
    }

    return sum;
}

namespace WindowFunctions {
    
    // 生成Kaiser窗
    std::vector<float> kaiser(int window_len, float beta) {
        if (window_len <= 0) {
            throw std::invalid_argument("Kaiser window length must be positive");
        }
        
        if (window_len == 1) {
            return std::vector<float>(1, 1.0f);
        }
        
        std::vector<float> window(window_len);
        float i0_beta = modifiedBessel0(beta);  // 分母
        
        for (int n = 0; n < window_len; n++) {
            // 计算归一化位置：从-1到1
            float alpha = 2.0f * n / (window_len - 1.0f) - 1.0f;
            
            // 计算Kaiser窗参数
            float arg = beta * std::sqrt(1.0f - alpha * alpha);
            
            // 计算窗系数
            window[n] = modifiedBessel0(arg) / i0_beta;
        }
        
        return window;
    }
    
    // 对复数数据应用窗函数
    void apply_window(std::vector<std::complex<float>>& data, 
                     const std::vector<float>& window) {
        if (data.size() != window.size()) {
            throw std::invalid_argument("Data size must match window size");
        }
        
        for (size_t i = 0; i < data.size(); i++) {
            data[i] *= window[i];
        }
    }
}



