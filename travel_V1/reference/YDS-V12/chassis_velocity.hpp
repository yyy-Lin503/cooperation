#ifndef CHASSIS_VELOCITY_HPP
#define CHASSIS_VELOCITY_HPP

#include <chrono>
#include <cmath>
#include <cstdint>

#ifdef __linux__
#include <sys/ipc.h>
#include <sys/shm.h>
#endif

struct ChassisVelocitySample {
    float vx_mps = 0.0f;        // Forward chassis velocity from /odom, m/s.
    float wz_radps = 0.0f;      // Chassis yaw rate from /odom, rad/s.
    double timestamp_sec = 0.0; // Python time.time() when the sample was written.
    std::uint32_t seq = 0;      // Monotonic writer sequence number.
    bool valid = false;
    bool has_timestamp = false;
    bool fresh = false;
};

class ChassisVelocityReader {
public:
    ChassisVelocityReader() = default;
    ~ChassisVelocityReader()
    {
#ifdef __linux__
        detach();
#endif
    }

    ChassisVelocityReader(const ChassisVelocityReader&) = delete;
    ChassisVelocityReader& operator=(const ChassisVelocityReader&) = delete;

    ChassisVelocitySample read()
    {
        ChassisVelocitySample sample;
#ifdef __linux__
        if (!attachIfNeeded()) {
            return sample;
        }

        if (shm_size_bytes_ >= kStampedShmSizeBytes) {
            double timestamp = 0.0;
            float vx = 0.0f;
            float wz = 0.0f;
            std::uint32_t seq = 0;
            copyFromSharedMemory(0, &timestamp, sizeof(timestamp));
            copyFromSharedMemory(8, &vx, sizeof(vx));
            copyFromSharedMemory(12, &wz, sizeof(wz));
            copyFromSharedMemory(16, &seq, sizeof(seq));

            sample.timestamp_sec = timestamp;
            sample.vx_mps = vx;
            sample.wz_radps = wz;
            sample.seq = seq;
            sample.has_timestamp = timestamp > 0.0;
            sample.fresh = sample.has_timestamp &&
                           std::abs(nowSeconds() - sample.timestamp_sec) <= kFreshTimeoutSec;
            sample.valid = sample.fresh;
        } else {
            float values[2] = {};
            copyFromSharedMemory(0, values, sizeof(values));
            sample.vx_mps = values[0];
            sample.wz_radps = values[1];
            sample.valid = true;
            sample.fresh = true;
        }
#endif
        return sample;
    }

private:
    static constexpr int kShmKey = 123456;
    static constexpr int kLegacyShmSizeBytes = 8;  // struct.pack("ff", vx, wz).
    static constexpr int kStampedShmSizeBytes = 24; // struct.pack("dffII", timestamp, vx, wz, seq, 0).
    static constexpr double kFreshTimeoutSec = 0.5;

    static double nowSeconds()
    {
        using clock = std::chrono::system_clock;
        return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
    }

#ifdef __linux__
    void* shm_addr_ = nullptr;
    int shm_id_ = -1;
    int shm_size_bytes_ = 0;

    bool attachIfNeeded()
    {
        if (shm_addr_) {
            return true;
        }

        shm_id_ = shmget(static_cast<key_t>(kShmKey), kStampedShmSizeBytes, 0444);
        shm_size_bytes_ = kStampedShmSizeBytes;
        if (shm_id_ < 0) {
            shm_id_ = shmget(static_cast<key_t>(kShmKey), kLegacyShmSizeBytes, 0444);
            shm_size_bytes_ = kLegacyShmSizeBytes;
            if (shm_id_ < 0) {
                shm_size_bytes_ = 0;
                return false;
            }
        }

        shm_addr_ = shmat(shm_id_, nullptr, SHM_RDONLY);
        if (shm_addr_ == reinterpret_cast<void*>(-1)) {
            shm_addr_ = nullptr;
            shm_id_ = -1;
            shm_size_bytes_ = 0;
            return false;
        }
        return true;
    }

    void copyFromSharedMemory(int offset, void* dst, int len) const
    {
        const volatile unsigned char* src =
            reinterpret_cast<const volatile unsigned char*>(shm_addr_) + offset;
        unsigned char* out = reinterpret_cast<unsigned char*>(dst);
        for (int i = 0; i < len; ++i) {
            out[i] = src[i];
        }
    }

    void detach()
    {
        if (shm_addr_) {
            shmdt(shm_addr_);
            shm_addr_ = nullptr;
        }
        shm_id_ = -1;
        shm_size_bytes_ = 0;
    }
#endif
};

#endif // CHASSIS_VELOCITY_HPP
