#include "camera_recording.hpp"
#include <QDir>
#include <QFile>
#include <QDateTime>
#include <QUuid>
#include <QJsonDocument>
#include <QJsonArray>
#include <QBuffer>
#include <QStorageInfo>
#include <algorithm>
namespace {
QByteArray line(const QJsonObject& o){return QJsonDocument(o).toJson(QJsonDocument::Compact)+'\n';}
}
bool CameraRecorder::start(const QString& root){
    stop();
    const QString folder=QDir(root).filePath("camera_"+QDateTime::currentDateTimeUtc().toString("yyyyMMdd_HHmmss_zzz")+"_"+QUuid::createUuid().toString(QUuid::Id128).left(8));
    if(!QDir().mkpath(folder)){std::lock_guard<std::mutex> lock(mutex_);status_="相机记录目录不可写";return false;}
    {std::lock_guard<std::mutex> lock(mutex_);stopping_=false;accepting_=true;queued_frames_=0;dropped_=0;status_="录制中："+folder;}
    thread_=std::thread(&CameraRecorder::run,this,folder);return true;
}
void CameraRecorder::stop(){
    {std::lock_guard<std::mutex> lock(mutex_);stopping_=true;accepting_=false;cv_.notify_all();}
    if(thread_.joinable())thread_.join();
}
void CameraRecorder::frame(const CameraAssist::VisionFrame& f,const QImage& image,QByteArray jpeg){
    std::lock_guard<std::mutex> lock(mutex_);if(!accepting_)return;
    if(queued_frames_>=3||queue_.size()>=16){++dropped_;return;}
    Job job;job.frame=f;job.image=image;job.jpeg=std::move(jpeg);queue_.push_back(std::move(job));++queued_frames_;cv_.notify_one();
}
void CameraRecorder::evidence(const QJsonObject& event){
    std::lock_guard<std::mutex> lock(mutex_);if(!accepting_)return;
    if(queue_.size()>=16){++dropped_;return;}
    Job job;job.event=event;queue_.push_back(std::move(job));cv_.notify_one();
}
QString CameraRecorder::status()const{std::lock_guard<std::mutex> lock(mutex_);return status_+QString("；记录丢弃 %1").arg(dropped_);}
void CameraRecorder::run(QString folder){
    QFile video(folder+"/camera.mjpg"),index(folder+"/frames.jsonl"),events(folder+"/events.jsonl");
    bool ok=video.open(QIODevice::WriteOnly)&&index.open(QIODevice::WriteOnly)&&events.open(QIODevice::WriteOnly);
    quint64 saved=0;
    const qint64 limit=std::max(16,qEnvironmentVariableIntValue("RADAR_CAMERA_RECORD_LIMIT_MB")>0?qEnvironmentVariableIntValue("RADAR_CAMERA_RECORD_LIMIT_MB"):512)*qint64(1024*1024);
    while(ok){
        Job job;unsigned long dropped;
        {std::unique_lock<std::mutex> lock(mutex_);cv_.wait(lock,[this]{return stopping_||!queue_.empty();});
         if(queue_.empty()&&stopping_)break;
         job=std::move(queue_.front());queue_.pop_front();if(!job.image.isNull())--queued_frames_;dropped=dropped_;}
        if(!job.event.isEmpty()){const auto bytes=line(job.event);ok=events.write(bytes)==bytes.size();continue;}
        if(job.jpeg.isEmpty()){QBuffer buf(&job.jpeg);buf.open(QIODevice::WriteOnly);if(!job.image.save(&buf,"JPEG",85)){ok=false;break;}}
        QStorageInfo storage(folder);storage.refresh();
        if(video.pos()+job.jpeg.size()>limit||(storage.isValid()&&storage.bytesAvailable()<1024LL*1024*1024)){
            std::lock_guard<std::mutex> lock(mutex_);status_="相机录制已暂停：达到单段容量上限或磁盘不足1GiB；已有记录保留";accepting_=false;queue_.clear();queued_frames_=0;break;
        }
        QJsonArray people;for(const auto& p:job.frame.people)people.append(QJsonObject{{"id",QString::number(p.track_id)},{"confidence",p.confidence},{"stable",p.stable},{"x",p.box.x},{"y",p.box.y},{"width",p.box.width},{"height",p.box.height}});
        QJsonObject row{{"schema",1},{"session",QString::number(job.frame.session)},{"sequence",QString::number(job.frame.sequence)},
            {"captured_wall",job.frame.captured_wall},{"captured_monotonic",job.frame.captured_monotonic},{"completed_monotonic",job.frame.completed_monotonic},
            {"time_quality",QString::fromStdString(job.frame.time_quality)},{"width",job.frame.width},{"height",job.frame.height},
            {"offset",double(video.pos())},{"length",job.jpeg.size()},{"dropped",double(dropped)},{"people",people}};
        const auto bytes=line(row);ok=video.write(job.jpeg)==job.jpeg.size()&&index.write(bytes)==bytes.size();++saved;
        if(saved%10==0){video.flush();index.flush();events.flush();}
    }
    video.flush();index.flush();events.flush();
    std::lock_guard<std::mutex> lock(mutex_);accepting_=false;queue_.clear();queued_frames_=0;
    QFile summary(folder+"/summary.json");
    if(summary.open(QIODevice::WriteOnly))summary.write(line(QJsonObject{{"saved_frames",double(saved)},{"dropped_jobs",double(dropped_)},{"write_ok",ok},{"status",status_}}));
    if(!ok)status_="相机录制写入失败；相机检测继续，已有记录保留";
    else if(stopping_)status_="相机记录已保存："+folder;
}
