// Jetson/Linux only. Allocates an isolated test segment, never production keys.
#include "chassis_velocity.hpp"
#include <sys/shm.h>
#include <unistd.h>
#include <cassert>
#include <array>
#include <iostream>
static void publish(int id,unsigned seq){
    auto*p=static_cast<unsigned char*>(shmat(id,nullptr,0));assert(p!=reinterpret_cast<void*>(-1));
    std::array<unsigned char,32>b{};double stamp=std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
    float vx=.25,wz=-.3;unsigned magic=OdomWire::kMagic,pid=getpid();
    std::memcpy(b.data(),&stamp,8);std::memcpy(b.data()+8,&vx,4);std::memcpy(b.data()+12,&wz,4);
    std::memcpy(b.data()+16,&seq,4);std::memcpy(b.data()+24,&magic,4);std::memcpy(b.data()+28,&pid,4);
    unsigned crc=OdomWire::checksum(b.data());std::memcpy(b.data()+20,&crc,4);std::memcpy(p,b.data(),32);shmdt(p);
}
int main(){
    int first=shmget(IPC_PRIVATE,32,0600);assert(first>=0);struct shmid_ds info{};
    // Recreate a public, exclusive key in a test-only range, with no deletion on collision.
    int key=0x41000000+(getpid()&0xffffff),id=shmget(key,32,IPC_CREAT|IPC_EXCL|0600);
    shmctl(first,IPC_RMID,nullptr);assert(id>=0);
    const auto value=std::to_string(key);setenv("RADAR_ODOM_SHM_KEY",value.c_str(),1);
    ChassisVelocityReader reader;publish(id,1);assert(reader.read().seq==1);
    assert(shmctl(id,IPC_RMID,nullptr)==0);int second=shmget(key,32,IPC_CREAT|IPC_EXCL|0600);assert(second>=0);
    publish(second,2);auto s=reader.read();assert(s.valid&&s.seq==2);
    assert(shmctl(second,IPC_RMID,nullptr)==0);assert(!reader.read().valid);
    std::cout<<"PASS Linux IPC reconnect after segment recreation and disappearance\n";
}
