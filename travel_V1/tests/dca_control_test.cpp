#include "radar_config.hpp"
#include <thread>
#include <iostream>
#include <stdexcept>
static void check_reply_source(bool different_port, bool different_ip = false) {
    int peer=socket(AF_INET,SOCK_DGRAM,0);
    sockaddr_in address{};address.sin_family=AF_INET;address.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
    bind(peer,reinterpret_cast<sockaddr*>(&address),sizeof(address));
    socklen_t size=sizeof(address);getsockname(peer,reinterpret_cast<sockaddr*>(&address),&size);
    timeval timeout{1,0};setsockopt(peer,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));
    int responder=peer;
    if(different_port || different_ip) {
        responder=socket(AF_INET,SOCK_DGRAM,0);
        sockaddr_in reply_address{};reply_address.sin_family=AF_INET;
        inet_pton(AF_INET,different_ip?"127.0.0.2":"127.0.0.1",&reply_address.sin_addr);
        if(bind(responder,reinterpret_cast<sockaddr*>(&reply_address),sizeof(reply_address))<0) {
            close(responder);close(peer);throw std::runtime_error("reply socket bind failed");
        }
    }
    std::thread server([&]{
        for(int i=0;i<(different_ip?1:7);++i){
            unsigned char request[2048];sockaddr_in sender{};socklen_t n=sizeof(sender);
            int received=recvfrom(peer,request,sizeof(request),0,reinterpret_cast<sockaddr*>(&sender),&n);
            if(received<8)break;
            unsigned char reply[]={0x5a,0xa5,request[2],request[3],0,0,0xaa,0xee};
            // Real hardware can emit an asynchronous status before an ACK.
            if(!different_ip && (i==0 || i==4 || i==6)) {
                unsigned char status[]={0x5a,0xa5,0x0a,0,0,2,0xaa,0xee};
                if(i==4){status[4]=1;status[5]=0;}
                if(i==6){status[4]=0x80;status[5]=0;}
                sendto(responder,status,sizeof(status),0,reinterpret_cast<sockaddr*>(&sender),n);
            }
            if(request[2]==0x0e)reply[4]=0x42; // version reply is not a status code
            if(i==4)reply[4]=1;
            if(i==5)reply[2]=0x09;
            sendto(responder,reply,sizeof(reply),0,reinterpret_cast<sockaddr*>(&sender),n);
        }
    });
    bool failed_status=false,failed_echo=false,stopped=false,foreign_ip_rejected=false;
    bool async_preserved=false;
    try {
        DCA1000Config config("test","127.0.0.1",0,"127.0.0.1",ntohs(address.sin_port));
        const auto startup_status=config.pollAsyncStatus();
        async_preserved=startup_status==std::vector<std::uint16_t>{0x0200} &&
            config.pollAsyncStatus().empty() &&
            DCA1000Config::describeAsyncStatus(startup_status.front())=="STS_LVDS_BUFFER_FULL";
        std::string error;
        failed_status=!config.DCA1000_start(&error) && !error.empty();
        async_preserved=async_preserved && config.pollAsyncStatus()==std::vector<std::uint16_t>{1};
        failed_echo=!config.DCA1000_start(&error);
        stopped=config.DCA1000_close(&error);
        async_preserved=async_preserved && config.pollAsyncStatus()==std::vector<std::uint16_t>{0x80};
    } catch(const std::runtime_error& error) {
        foreign_ip_rejected=different_ip && std::string(error.what()).find(
            "unexpected response endpoint 127.0.0.2:")!=std::string::npos &&
            std::string(error.what()).find("expected device IP 127.0.0.1")!=std::string::npos;
        if(!foreign_ip_rejected) {
            server.join();if(responder!=peer)close(responder);close(peer);throw;
        }
    }
    server.join();
    if(responder!=peer)close(responder);
    close(peer);
    if(different_ip) {
        if(!foreign_ip_rejected)throw std::runtime_error("foreign device IP accepted");
        return;
    }
    bool timeout_rejected=false;
    try { DCA1000Config missing("test","127.0.0.1",0,"127.0.0.1",ntohs(address.sin_port)); }
    catch(const std::runtime_error&) {timeout_rejected=true;}
    if(!timeout_rejected || !failed_status || !failed_echo || !stopped || !async_preserved)
        throw std::runtime_error("DCA acknowledgement/async status validation failed");
}
int main() {
    check_reply_source(false);
    check_reply_source(true);
    check_reply_source(true,true);
    std::cout<<"PASS: DCA handshake with same/different reply ports, foreign IP rejection, "
                "asynchronous device status, device failure, mismatched echo, timeout and stop acknowledgement\n";
}
