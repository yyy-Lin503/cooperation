#ifndef CHASSIS_VELOCITY_HPP
#define CHASSIS_VELOCITY_HPP
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <limits>
#include <stdexcept>
#include <string>
#ifdef __linux__
#include <sys/ipc.h>
#include <sys/shm.h>
#endif

struct ChassisVelocitySample {
    float vx_mps=0,wz_radps=0;
    double timestamp_sec=0;
    std::uint32_t seq=0;
    bool valid=false,has_timestamp=false,fresh=false;
    std::string status="missing";
};
namespace OdomWire {
constexpr std::uint32_t kMagic=0x384d444f; // ODM8. FNV-1a covers bytes 0..19 and 24..31.
inline std::uint32_t checksum(const unsigned char* bytes) {
    std::uint32_t hash=2166136261u;
    for(int i=0;i<32;++i)if(i<20||i>=24){hash^=bytes[i];hash*=16777619u;}
    return hash;
}
inline int key() {
    const char* configured=std::getenv("RADAR_ODOM_SHM_KEY");
    if(!configured)return 123458;  
    char* end=nullptr;const long value=std::strtol(configured,&end,10);
    if(!end||*end||value<1||value>std::numeric_limits<int>::max())throw std::invalid_argument("Invalid RADAR_ODOM_SHM_KEY");
    return int(value);
}
inline ChassisVelocitySample decode(const unsigned char* bytes,std::size_t size,double now) {
    ChassisVelocitySample s;
    if(size<32){s.status=size<24?"legacy_without_timestamp":"protocol_mismatch";return s;}
    std::uint32_t marker=0,check=0;
    std::memcpy(&marker,bytes+24,4);std::memcpy(&check,bytes+20,4);
    if(marker!=kMagic){s.status="protocol_mismatch";return s;}
    if(check!=checksum(bytes)){s.status="checksum_mismatch";return s;}
    std::memcpy(&s.timestamp_sec,bytes,8);std::memcpy(&s.vx_mps,bytes+8,4);
    std::memcpy(&s.wz_radps,bytes+12,4);std::memcpy(&s.seq,bytes+16,4);
    s.has_timestamp=std::isfinite(s.timestamp_sec)&&s.timestamp_sec>0;
    if(!s.has_timestamp||!std::isfinite(s.vx_mps)||!std::isfinite(s.wz_radps)){s.status="invalid_value";return s;}
    const double age=now-s.timestamp_sec;
    if(age<-.05){s.status="future_timestamp";return s;}
    if(age>.5){s.status="stale";return s;}
    s.valid=s.fresh=true;s.status="fresh";return s;
}
}

class ChassisVelocityReader {
public:
    ChassisVelocityReader()=default;
    ~ChassisVelocityReader(){detach();}
    ChassisVelocityReader(const ChassisVelocityReader&)=delete;
    ChassisVelocityReader& operator=(const ChassisVelocityReader&)=delete;
    ChassisVelocitySample read() {
        ChassisVelocitySample sample;
#ifdef __linux__
        const int current=shmget(static_cast<key_t>(OdomWire::key()),1,0444);
        if(current<0){detach();return sample;}
        if(current!=id_){
            detach();struct shmid_ds info{};
            if(shmctl(current,IPC_STAT,&info)<0)return sample;
            if(info.shm_segsz<32){sample.status="legacy_or_small_segment";return sample;}
            address_=shmat(current,nullptr,SHM_RDONLY);
            if(address_==reinterpret_cast<void*>(-1)){address_=nullptr;return sample;}
            id_=current;
        }
        std::array<unsigned char,32> a{},b{};
        for(int attempt=0;attempt<4;++attempt){
            copy(a.data());std::atomic_thread_fence(std::memory_order_acquire);copy(b.data());
            if(a==b){
                const double now=std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
                return OdomWire::decode(a.data(),a.size(),now);
            }
        }
        sample.status="inconsistent_snapshot";
#endif
        return sample;
    }
private:
#ifdef __linux__
    void* address_=nullptr;int id_=-1;
    void copy(unsigned char* out)const{
        const auto* source=static_cast<const volatile unsigned char*>(address_);
        for(int i=0;i<32;++i)out[i]=source[i];
    }
#endif
    void detach(){
#ifdef __linux__
        if(address_)shmdt(address_);address_=nullptr;id_=-1;
#endif
    }
};
#endif
