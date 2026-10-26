#include "project_paths.hpp"
#include "CFAR.hpp"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <algorithm>   
#include <limits>      
#include <mutex>

// 工具函数：返回数组中第K大的值（K>=1，降序第K大）
// 等价于 partial_sort 前K项后取第K项或 nth_element 的比较结果，但更轻量
namespace {
    constexpr bool SAVE_CFAR_T_MATRIX = false; // 控制矩阵是否保存

    inline float kth_largest_desc(const float* data, int n, int k) {
        if (k <= 0 || n <= 0) return 0.0f;
        if (k > n) k = n;

        std::vector<float> heap(static_cast<std::size_t>(k)); // supports larger validated windows
        // 复制前k个元素
        for (int i = 0; i < k; ++i) heap[i] = data[i];

        // 建立最小堆（堆顶为当前第K大）
        auto sift_down = [&](int idx) {
            while (true) {
                int l = idx * 2 + 1;
                if (l >= k) break;
                int r = l + 1;
                int smallest = (r < k && heap[r] < heap[l]) ? r : l;
                if (heap[idx] <= heap[smallest]) break;
                float t = heap[idx]; heap[idx] = heap[smallest]; heap[smallest] = t;
                idx = smallest;
            }
        };
        for (int i = (k >> 1) - 1; i >= 0; --i) sift_down(i);

        // 扫描剩余元素：仅当值大于堆顶时替换并下滤
        for (int i = k; i < n; ++i) {
            float v = data[i];
            if (v > heap[0]) {
                heap[0] = v;
                sift_down(0);
            }
        }
        return heap[0]; // 最小堆堆顶即第K大
    }
}

namespace RadarProcessing {
    // 全局运行时 CFAR 参数与互斥保护
    namespace {
        CFARRuntimeConfig g_cfar_cfg;
        std::mutex g_cfar_mtx;
    }

    void set_cfar_params(const CFARRuntimeConfig& cfg) {
        std::lock_guard<std::mutex> lock(g_cfar_mtx);
        g_cfar_cfg = cfg;
    }

    CFARRuntimeConfig get_cfar_params() {
        std::lock_guard<std::mutex> lock(g_cfar_mtx);
        return g_cfar_cfg;
    }
    
    std::vector<float> computeRDM(const FourDArray& frame0, 
                                int num_ADC, int num_chirp, 
                                int num_RX, int num_TX) {
        std::vector<float> RDM_sum(num_ADC * num_chirp, 0.0f);
        
        for (int rx = 0; rx < num_RX; ++rx) {
            for (int tx = 0; tx < num_TX; ++tx) {
                for (int chirp = 0; chirp < num_chirp; ++chirp) {
                    // 预计算chirp的偏移
                    for (int adc = 0; adc < num_ADC; ++adc) {
                        // 计算索引
                        const int index = adc  + chirp*num_ADC;
                        
                        // 连续访问frame0
                        const auto& complex_val = frame0(adc, chirp, rx, tx);
                        RDM_sum[index] += std::abs(complex_val);
                    }
                }
            }
        }
        
        return RDM_sum;
    }

    std::vector<float> sumOverRange(const std::vector<float>& RDM, 
                                   int num_ADC, int num_chirp) {
        std::vector<float> doppler_profile(num_chirp, 0.0f);
        
        // 按行优先遍历RDM，对每个多普勒单元累加所有距离单元的值
        for (int adc = 0; adc < num_ADC; ++adc) {
            // 预计算当前行的起始索引，减少重复乘法
            const int row_offset = adc * num_chirp;
            
            // 遍历当前距离单元的所有多普勒单元
            for (int chirp = 0; chirp < num_chirp; ++chirp) {
                // 累加到对应的多普勒单元
                doppler_profile[chirp] += RDM[row_offset + chirp];
            }
        }
        
        return doppler_profile;
    }

    CFARResult osca_cfar(const std::vector<float>& RDM, 
                               int num_ADC, int num_chirp) {
        // 读取并校验运行时参数
        CFARRuntimeConfig cfg = get_cfar_params();
        // 合理取值的边界，避免越界和非法窗口
        // N_ex 至少 1，最多不超过 min(num_ADC, num_chirp)/2 - 1，以保证窗口>保护区
        int max_half = std::max(1, std::min(num_ADC, num_chirp) / 2 - 1);
        int N_ex = std::min(std::max(1, cfg.N_ex), std::max(1, max_half));
        int K = std::min(std::max(1, cfg.K), 2 * N_ex + 1);
        int N_protect_r = std::min(std::max(0, cfg.N_protect_r), N_ex);
        int N_protect_d = std::min(std::max(0, cfg.N_protect_d), N_ex);
        float alpha = (std::isfinite(cfg.alpha) && cfg.alpha > 0.f) ? cfg.alpha : 3.5f;
        CFARResult result;
        
        std::vector<float> T_matrix; // (num_ADC * num_chirp, 0.0f);
        if (SAVE_CFAR_T_MATRIX) {
            T_matrix.assign(num_ADC * num_chirp, 0.0f);
        }

        // 计算RDM均值
        float RDM_sum = 0.0f;
        for (const auto& val : RDM) RDM_sum += val;
        float RDM_mean = RDM_sum / static_cast<float>(RDM.size());
        
    int expanded_rows = num_ADC + 2 * N_ex;
    int expanded_cols = num_chirp + 2 * N_ex;
        std::vector<float> RDM_ex(expanded_rows * expanded_cols, RDM_mean);
        
        // RDM存储顺序为[adc + chirp * num_ADC]，即先ADC后chirp
        for (int adc = 0; adc < num_ADC; ++adc)
            for (int chirp = 0; chirp < num_chirp; ++chirp)
                RDM_ex[(adc + N_ex) + (chirp + N_ex) * expanded_rows] = RDM[adc + chirp * num_ADC];
     
    int window_size = 2 * N_ex + 1;
        std::vector<float> cell(window_size * window_size);
        std::vector<float> column(window_size);
        std::vector<float> temp_values;
        temp_values.reserve(window_size);

        bool first_cell_saved = false; // 标记是否已保存第一个cell

        for (int adc_idx = N_ex; adc_idx < N_ex + num_ADC; ++adc_idx) {
            for (int chirp_idx = N_ex; chirp_idx < N_ex + num_chirp; ++chirp_idx) {
                // 提取窗口到二维cell (对应MATLAB的cell = RDM_ex(ii-N_ex: ii+N_ex, jj-N_ex : jj+N_ex))
                for (int i = 0; i < window_size; ++i) {
                    int row = adc_idx - N_ex + i;
                    for (int j = 0; j < window_size; ++j) {
                        int col = chirp_idx - N_ex + j;
                        cell[i + j * window_size] = RDM_ex[row + col * expanded_rows];
                    }
                }

                float center_value = cell[N_ex * window_size + N_ex];
                
                // 设置保护单元为0 (对应MATLAB的protect逻辑)
                // MATLAB: cell( N_ex-N_protect_r+1 : N_ex+N_protect_r+1,  N_ex-N_protect_d+1 : N_ex+N_protect_d+1) = 0;
                for (int i = N_ex - N_protect_r; i <= N_ex + N_protect_r; ++i) {
                    if (i < 0 || i >= window_size) continue;
                    for (int j = N_ex - N_protect_d; j <= N_ex + N_protect_d; ++j) {
                        if (j < 0 || j >= window_size) continue;
                        cell[i + j * window_size] = 0.0f;
                    }
                }


                // OSCA-CFAR处理 (对应MATLAB的按列排序和选择逻辑)
                temp_values.clear();
                //         column[i] = cell[i * window_size + j];
                //         // 替换 nth_element/partial_sort：仅取第K大，结果不变，开销更低
                //         temp_values.emplace_back(kth);
                for (int row = 0; row < window_size; ++row) {
                    for (int col = 0; col < window_size; ++col)
                        column[col] = cell[row + col * window_size]; 
                    float kth = kth_largest_desc(column.data(), window_size, K);
                    temp_values.emplace_back(kth);
                }

                if (!temp_values.empty()) {
                    // 计算Z和T (对应MATLAB的mean(temp,'all'))
                    float Z = 0.0f;
                    for (const auto& val : temp_values) Z += val;
                    Z /= static_cast<float>(temp_values.size());
                    float T = Z * alpha;
                    
                    // 保存T值到矩阵中
                    if (SAVE_CFAR_T_MATRIX) {
                        int matrix_idx = (adc_idx - N_ex) + (chirp_idx - N_ex) * num_ADC;
                        T_matrix[matrix_idx] = T;
                    }
                    
                    // 检测逻辑 (对应MATLAB的 T < cell_center，即center_value > T)
                    if (center_value > T) {
                        result.range_indices.emplace_back(adc_idx - N_ex);
                        result.doppler_indices.emplace_back(chirp_idx - N_ex);
                        result.snrs.emplace_back(center_value / T);
                    }
                }
            }
        }
        
        // 保存T矩阵到文件
        if (SAVE_CFAR_T_MATRIX) {
            std::filesystem::create_directories(RadarPaths::resolve().diagnostics());
            std::ofstream ofs_T(RadarPaths::resolve().diagnostics() / "CFAR_T_matrix.txt");
            if (ofs_T.is_open()) {
                // 保存为矩阵格式，每行对应一个距离单元，每列对应一个多普勒单元
                for (int range_idx = 0; range_idx < num_ADC; ++range_idx) {
                    for (int doppler_idx = 0; doppler_idx < num_chirp; ++doppler_idx) {
                        int linear_idx = range_idx + doppler_idx * num_ADC;
                        ofs_T << std::fixed << std::setprecision(6) << T_matrix[linear_idx];
                        if (doppler_idx < num_chirp - 1) {
                            ofs_T << "\t";
                        }
                    }
                    ofs_T << "\n";
                }
                ofs_T.close();
                std::cout << "已保存CFAR阈值矩阵到 CFAR_T_matrix.txt" << std::endl;
            } else {
                std::cerr << "无法创建CFAR阈值矩阵文件 CFAR_T_matrix.txt" << std::endl;
            }
        }        
        result.num_targets = static_cast<int>(result.range_indices.size());
        return result;
    }
}