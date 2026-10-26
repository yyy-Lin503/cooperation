#ifndef CFAR_HPP
#define CFAR_HPP

#include "fft.hpp"





namespace RadarProcessing {
    // 运行时可配置的 CFAR 参数
    struct CFARRuntimeConfig {
        int N_ex = 8;           // 边界扩展/窗口半径
        int K = 5;              // 第K大
        int N_protect_r = 5;    // 距离维保护半径
        int N_protect_d = 5;    // 多普勒维保护半径
        float alpha = 3.5f;     // 阈值比例
    };

    // 设置/获取全局 CFAR 参数（线程安全）
    void set_cfar_params(const CFARRuntimeConfig& cfg);
    CFARRuntimeConfig get_cfar_params();
    // 非相关叠加函数：计算距离-多普勒图
    std::vector<float> computeRDM(const FourDArray& frame0, 
                                  int num_ADC, int num_chirp, 
                                  int num_RX, int num_TX);
    // 多普勒中心估计：沿距离维求和
    std::vector<float> sumOverRange(const std::vector<float>& RDM, 
                                   int num_ADC, int num_chirp);
    inline float getRDMValue(const std::vector<float>& RDM, 
                            int adc, int chirp, int num_chirp) {
        return RDM[adc * num_chirp + chirp];
    }

    // CFAR检测结果结构体
    struct CFARResult {
        int num_targets;
        std::vector<int> range_indices;
        std::vector<int> doppler_indices;
        std::vector<float> snrs;
    };
    
    // OSCA-CFAR目标检测函数（使用 set_cfar_params 配置的参数）
    CFARResult osca_cfar(const std::vector<float>& RDM, 
                               int num_ADC, int num_chirp);

}



#endif 