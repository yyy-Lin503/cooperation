#ifndef FFT_HPP
#define FFT_HPP

#include <complex>
#include <vector>
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <numeric>
#include <map>
#include <cmath>
#include <iostream>
#include <fstream>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// 雷达参数结构体
struct RadarParams {
    int num_ADC = 256;      // ADC采样点数
    int num_chirp = 64;     // chirp数
    int num_RX = 4;         // 接收天线数
    int num_TX = 2;         // 发射天线数
};

// 四维数组结构体，用于存储雷达数据
struct FourDArray {
    std::vector<std::complex<float>> data;
    int dim0, dim1, dim2, dim3;  // ADC, chirp, RX, TX
    
    FourDArray(int d0, int d1, int d2, int d3) : 
        dim0(d0), dim1(d1), dim2(d2), dim3(d3), 
        data(d0 * d1 * d2 * d3) 
    {}
    
    // 索引访问函数
    std::complex<float>& operator()(int i, int j, int k, int l) {
        int index = i * (dim1*dim2*dim3) + j * (dim2*dim3) + k * dim3 + l;
        return data[index];
    }
    
    const std::complex<float>& operator()(int i, int j, int k, int l) const {
        int index = i * (dim1*dim2*dim3) + j * (dim2*dim3) + k * dim3 + l;
        return data[index];
    }
    
    // 获取总大小
    size_t size() const { return data.size(); }
};

int mode(const std::vector<int>& data);

// 快速FFT类（静态方法）
class FastFFT {
private:
    // 位反转函数
    static int bit_reverse(int num, int bits);

public:
    // 快速FFT实现（基2，迭代法）
    static void fft(std::vector<std::complex<float>>& x, bool inverse = false);
    
    // 便利函数：正向FFT
    static void forward(std::vector<std::complex<float>>& x);
    
    // 便利函数：逆向FFT
    static void inverse(std::vector<std::complex<float>>& x);
};

// 预计算旋转因子，供相同长度的 FFT 重复使用。
class OptimizedFFT {
private:
    int size_;
    std::vector<std::complex<float>> twiddle_factors_;
    std::vector<int> bit_reverse_table_;
    
    void precompute_factors();
    
public:
    // 构造函数
    OptimizedFFT(int size);
    
    // 正向FFT
    void forward(std::vector<std::complex<float>>& x);
    
    // 逆向FFT
    void inverse(std::vector<std::complex<float>>& x);

    // fftshift
    void fftshift(std::vector<std::complex<float>>& x);
    
    // 获取FFT大小
    int size() const { return size_; }
};

namespace WindowFunctions {
    // 生成Kaiser窗
    std::vector<float> kaiser(int window_len, float beta = 10.0f);
    
    // 对复数数据应用窗函数
    void apply_window(std::vector<std::complex<float>>& data, 
                     const std::vector<float>& window);
}





#endif // FFT_HPP
