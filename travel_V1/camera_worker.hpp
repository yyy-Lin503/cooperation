#ifndef V3_CAMERA_WORKER_HPP
#define V3_CAMERA_WORKER_HPP
#include "camera_association.hpp"
#include "camera_recording.hpp"
#include <opencv2/dnn.hpp>
#include <QImage>
#include <atomic>
#include <mutex>
#include <thread>
#include <memory>

namespace CameraAssist {
struct CameraSettings { std::string device, model; bool cuda = true; int width=640,height=480; bool record=true; };
struct CameraPacket {
    QImage image;
    VisionFrame frame;
    std::string status, error;
    double inference_ms = 0;
};
class CameraWorker {
public:
    ~CameraWorker();
    void start(const CameraSettings&);
    QString recordingStatus() const { return recorder_.status(); }
    void recordEvidence(const QJsonObject& event){recorder_.evidence(event);}
    void stop();
    bool running() const { return running_; }
    std::shared_ptr<const CameraPacket> latest() const;
private:
    CameraRecorder recorder_;
    void run(CameraSettings);
    void publish(std::shared_ptr<CameraPacket>);
    std::thread worker_;
    std::atomic<bool> running_{false};
    mutable std::mutex mutex_;
    std::shared_ptr<const CameraPacket> latest_;
};
std::vector<Person> detectPeople(cv::dnn::Net& net,const cv::Mat& bgr);
}
#endif
