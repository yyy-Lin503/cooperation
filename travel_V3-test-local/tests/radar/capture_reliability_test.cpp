#include "realtime_capture.hpp"
#include "recording_frame_index.hpp"
#include <arpa/inet.h>
#include <atomic>
#include <chrono>
#include <experimental/filesystem>
#include <iostream>
#include <stdexcept>
using namespace std::chrono_literals;
std::chrono::system_clock::time_point capture_start_time;
static void check(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }
static std::vector<char> packet(std::uint32_t sequence, std::uint64_t offset, std::size_t count) {
    std::vector<char> bytes(10 + count, 0x37);
    for (int i=0;i<4;++i) bytes[i]=static_cast<char>(sequence>>(8*i));
    for (int i=0;i<6;++i) bytes[4+i]=static_cast<char>(offset>>(8*i));
    return bytes;
}
template<class Predicate> static void waitFor(Predicate predicate) {
    auto until=std::chrono::steady_clock::now()+2s;
    while (!predicate() && std::chrono::steady_clock::now()<until) std::this_thread::sleep_for(2ms);
    check(predicate(), "Timed out waiting for capture progress");
}
int main() {
    DcaPacketAssembler assembler(16);
    check(assembler.feed("bad",3).empty(), "Short packet accepted");
    auto a=packet(1,0,8); check(assembler.feed(a.data(),a.size()).empty(),"Partial frame emitted");
    auto b=packet(2,8,8); auto frames=assembler.feed(b.data(),b.size());
    check(frames.size()==1 && frames[0].number==1 && frames[0].bytes.size()==16,"Frame assembly failed");
    check(assembler.feed(b.data(),b.size()).empty() && assembler.counters.out_of_order==1,"Duplicate accepted");
    auto c=packet(4,24,24); frames=assembler.feed(c.data(),c.size());
    check(frames.size()==1 && frames[0].number==3 && assembler.counters.missing_packets==1,"Gap did not realign");
    DcaPacketAssembler mid(16); auto d=packet(9,5,43); frames=mid.feed(d.data(),d.size());
    check(frames.size()==2 && frames[0].number==2 && frames[1].number==3,"Mid-frame start was not aligned");
    std::queue<std::vector<char>> unused;
    CaptureOptions options;
    std::atomic<int> calls{0};
    std::atomic<bool> entered{false}, release{false};
    options.max_queue_frames=2;
    options.max_frame_age_sec=0.015;
    options.output_directory="capture-test-output";
    options.processor=[&](const auto&,const auto&,const auto&) {
        ++calls; entered=true;
        while (!release) std::this_thread::sleep_for(1ms);
        ProcessingResult result; result.timestamp="test"; return result;
    };
    UdpListener listener("test",unused,8,"127.0.0.1",0,2048,{},"127.0.0.1",9,options);
    listener.start();
    check(listener.status().state==CaptureStatus::State::WaitingForData,"Incorrect startup state");
    bool duplicate=false;
    try { listener.start(); } catch (const std::logic_error&) { duplicate=true; }
    check(duplicate,"Repeated start accepted");
    bool conflict=false;
    try { UdpListener second("test",unused,8,"127.0.0.1",listener.listening_port(),2048,{},"127.0.0.1",9,options); second.start(); }
    catch(const std::runtime_error&) { conflict=true; }
    check(conflict,"Bind failure not propagated");
    listener.start_saving();
    int sender=socket(AF_INET,SOCK_DGRAM,0); sockaddr_in destination{};
    destination.sin_family=AF_INET; destination.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
    destination.sin_port=htons(listener.listening_port());
    auto send=[&](int seq,int offset) { auto bytes=packet(seq,offset,16);
        check(sendto(sender,bytes.data(),bytes.size(),0,reinterpret_cast<sockaddr*>(&destination),sizeof(destination))>0,"Local send failed"); };
    send(1,0); waitFor([&]{return entered.load();});
    for(int i=2;i<=10;++i) send(i,(i-1)*16);
    waitFor([&]{return listener.status().frames_received==10;});
    check(listener.status().queue_depth<=2 && listener.status().queue_dropped>=7,"Queue is not bounded");
    check(!listener.join(1),"Timed join reported success for running workers");
    std::this_thread::sleep_for(25ms); release=true;
    waitFor([&]{return listener.status().stale_dropped>=1;});
    listener.stop_saving(); listener.stop_saving();
    listener.stop_collector(); listener.stop_collector();
    check(listener.join(1000),"Stop failed to join workers");
    close(sender);
    RecordingFrameIndex index; index.load(listener.output_filename(),10);
    check(index.frame(0).frame_number==1 && index.frame(9).frame_number==10,"Recording index lost source frames");
    // A stopped object owns/join-cleans all threads without an explicit join call.
    { UdpListener scoped("test",unused,8,"127.0.0.1",0,2048,{},"127.0.0.1",9,options); scoped.start(); }
    std::experimental::filesystem::remove_all(options.output_directory);
    std::cout << "PASS: packet boundaries/gaps, startup failures, bounded queue, stale frames, timed join, repeated stop and destructor\n";
}
