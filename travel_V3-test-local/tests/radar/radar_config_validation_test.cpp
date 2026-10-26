#include "iwr6843param.hpp"
#include "radar_frame_layout.hpp"
#include <fstream>
#include <stdexcept>
#include <iostream>
static void write(const std::string& text) {std::ofstream file("cfg-validation-test.cfg");file<<text;}
static void reject(const std::string& text) {
    write(text);bool failed=false;try{IWR6843AOP_TLV cfg("cfg-validation-test.cfg");}catch(const std::exception&){failed=true;}
    if(!failed)throw std::runtime_error("Invalid cfg accepted");
}
int main(){
    const std::string profile="profileCfg 0 60.5 10.5 6 60 0 0 60.25 1 256 6667 0 0 30\n";
    reject("channelCfg 15 0 0\n"+profile+"frameCfg 0 1 64 0 100 1 0\n");
    reject("channelCfg 15 5 0\nprofileCfg 0\nframeCfg 0 1 64 0 100 1 0\n");
    reject("channelCfg 15 5 0\n"+profile+"frameCfg 0 2 64 0 100 1 0\n");
    reject("channelCfg 15 5 0\n"+profile+"frameCfg 0 1 63 0 100 1 0\n");
    write("channelCfg 15 5 0\n"+profile+"frameCfg 0 1 64 0 100 1 0\n");
    IWR6843AOP_TLV cfg("cfg-validation-test.cfg");auto params=cfg.get_params();
    if(params.at("fc")!=60.5 || params.at("idleTime")!=10.5 || params.at("slopeConst")!=60.25 ||
        RadarFrameLayout::frameBytes(RadarFrameLayout::paramsFromMap(params))!=524288)
        throw std::runtime_error("Physical parameters were truncated or frame dimensions changed");
    std::remove("cfg-validation-test.cfg");
    std::cout<<"PASS: invalid/missing cfg fields, zero antennas, unsupported chirp layout and fractional parameters\n";
}
