#ifndef RADAR_CAMERA_RECORDING_HPP
#define RADAR_CAMERA_RECORDING_HPP
#include "camera_association.hpp"
#include <QImage>
#include <QByteArray>
#include <QJsonObject>
#include <QString>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>
// Bounded asynchronous archive. Original MJPEG bytes are retained when supplied.
class CameraRecorder {
public:
    ~CameraRecorder(){stop();}
    bool start(const QString& root);
    void stop();
    void frame(const CameraAssist::VisionFrame&,const QImage&,QByteArray jpeg={});
    void evidence(const QJsonObject&);
    QString status() const;
private:
    struct Job {CameraAssist::VisionFrame frame;QImage image;QByteArray jpeg;QJsonObject event;};
    mutable std::mutex mutex_;std::condition_variable cv_;std::deque<Job> queue_;
    std::thread thread_;bool stopping_=false,accepting_=false;int queued_frames_=0;
    unsigned long dropped_=0;QString status_;
    void run(QString folder);
};
#endif
