#include "camera_worker.hpp"
#include "frame_context.hpp"
#include <QCoreApplication>
#include <QCommandLineParser>
#include <QDir>
#include <iostream>
#include <thread>
int main(int argc,char**argv){
    QCoreApplication app(argc,argv);QCommandLineParser parser;parser.addHelpOption();
    QDir ids("/dev/v4l/by-id");const auto cameras=ids.entryList({"*video-index0"},QDir::Files|QDir::System);
    const auto device=cameras.empty()?QString("/dev/video0"):ids.filePath(cameras.front());
    parser.addOption({"device","V4L2 camera path","path",device});
    parser.addOption({"model","COCO YOLO11 ONNX, input 640","path",qEnvironmentVariable("RADAR_CAMERA_MODEL",QDir::home().filePath("ultralytics/ultralytics/yolo11n.onnx"))});
    parser.addOption({"cpu","Use CPU inference"});parser.process(app);
    CameraAssist::CameraSettings settings;
    settings.device=parser.value("device").toStdString();settings.model=parser.value("model").toStdString();
    settings.cuda=!parser.isSet("cpu");settings.record=false;
    CameraAssist::CameraWorker worker;worker.start(settings);
    std::uint64_t previous=0;int count=0;const double end=RadarTime::monotonicSeconds()+30;
    while(RadarTime::monotonicSeconds()<end) {
        const auto packet=worker.latest();
        if(packet&&!packet->error.empty()){std::cerr<<packet->error<<'\n';worker.stop();return 1;}
        if(packet&&packet->frame.sequence!=previous){previous=packet->frame.sequence;++count;
            const auto& f=packet->frame;
            std::cout<<"frame="<<f.sequence<<" size="<<f.width<<'x'<<f.height<<" people="<<f.people.size()
                <<" inference_ms="<<packet->inference_ms<<" age_ms="<<(RadarTime::monotonicSeconds()-f.captured_monotonic)*1000
                <<" time_quality="<<f.time_quality<<" backend="<<packet->status<<std::endl;
            if(count>=5){worker.stop();std::cout<<"PASS: live camera, detector, timestamps and cooperative stop; no image files saved\n";return 0;}}
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    worker.stop();std::cerr<<"Camera/model probe timed out\n";return 1;
}
