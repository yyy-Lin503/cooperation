#include "camera_recording.hpp"
#include <QCoreApplication>
#include <QTemporaryDir>
#include <QDir>
#include <QFile>
#include <QJsonDocument>
#include <QJsonArray>
#include <stdexcept>
#include <iostream>
void check(bool b,const char*m){if(!b)throw std::runtime_error(m);}
int main(int argc,char**argv){QCoreApplication app(argc,argv);QTemporaryDir root;check(root.isValid(),"temp");
 CameraRecorder recorder;check(recorder.start(root.path()),"start");
 CameraAssist::VisionFrame frame;frame.session=7;frame.sequence=1;frame.width=640;frame.height=480;frame.captured_wall=100;frame.captured_monotonic=10;frame.completed_monotonic=10.03;
 frame.people={{{10,20,50,80},.9f,8,true}};QImage image(640,480,QImage::Format_RGB888);image.fill(Qt::blue);
 recorder.frame(frame,image);recorder.evidence(QJsonObject{{"frame",4},{"sample_time_sec",100.0},{"motion_reason","accepted_measurement"}});recorder.stop();recorder.stop();
 const auto dirs=QDir(root.path()).entryList(QDir::Dirs|QDir::NoDotAndDotDot);check(dirs.size()==1,"session overwrite");const auto path=QDir(root.path()).filePath(dirs[0]);
 QFile index(path+"/frames.jsonl"),video(path+"/camera.mjpg"),events(path+"/events.jsonl");check(index.open(QIODevice::ReadOnly)&&video.open(QIODevice::ReadOnly)&&events.open(QIODevice::ReadOnly),"archive missing");
 auto data=QJsonDocument::fromJson(index.readLine()).object();check(data["schema"].toInt()==1&&data["sequence"].toString()=="1"&&data["people"].toArray().size()==1,"index schema");
 const auto offset=qint64(data["offset"].toDouble()),length=qint64(data["length"].toDouble());check(offset==0&&length==video.size(),"frame boundaries");
 check(!QImage::fromData(video.read(length),"JPEG").isNull(),"JPEG decode failed");check(!QJsonDocument::fromJson(events.readLine()).isNull(),"events missing");
 // Original MJPEG bytes must survive byte-for-byte, avoiding extra encoding.
 const auto original=QFile(path+"/camera.mjpg").fileName();QFile raw(original);raw.open(QIODevice::ReadOnly);auto jpeg=raw.readAll();
 check(recorder.start(root.path()),"restart");recorder.frame(frame,image,jpeg);recorder.stop();const auto after=QDir(root.path()).entryList(QDir::Dirs|QDir::NoDotAndDotDot);check(after.size()==2,"restart overwrote old recording");
 for(const auto& dir:after)if(dir!=dirs[0]){QFile copied(QDir(root.path()).filePath(dir+"/camera.mjpg"));check(copied.open(QIODevice::ReadOnly)&&copied.readAll()==jpeg,"MJPEG recompressed");}
 std::cout<<"PASS: async camera archive, frame index, event alignment, original JPEG and restart preservation\n";
}
