#include "project_paths.hpp"
#include "camera_replay.hpp"
#include "camera_assist_panel.hpp"
#include <QDialog>
#include <QFileDialog>
#include <QFile>
#include <QFileInfo>
#include <QDir>
#include <QJsonDocument>
#include <QJsonArray>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QSlider>
#include <QComboBox>
#include <QTimer>
#include <QMessageBox>
#include <QSignalBlocker>
#include <algorithm>
#include <cmath>
void openCameraReplay(QWidget* parent){
    const QString path=QFileDialog::getOpenFileName(parent,"选择相机记录的 frames.jsonl",QString::fromStdString(RadarPaths::resolve().cameraRecordings().u8string()),"相机帧索引 (frames.jsonl)");
    if(!path.isEmpty())openCameraReplayFile(path,parent);
}
void openCameraReplayFile(const QString& path,QWidget* parent){
    QFile source(path);if(!source.open(QIODevice::ReadOnly))return;
    QVector<QJsonObject> rows,events;
    while(!source.atEnd()){const auto row=QJsonDocument::fromJson(source.readLine()).object();if(row["schema"].toInt()==1)rows.push_back(row);}
    QFile video(QFileInfo(path).dir().filePath("camera.mjpg"));
    if(rows.isEmpty()||!video.open(QIODevice::ReadOnly)){QMessageBox::warning(parent,"无法回放","记录为空，或同目录 camera.mjpg 缺失。");return;}
    QFile history(QFileInfo(path).dir().filePath("events.jsonl"));if(history.open(QIODevice::ReadOnly))while(!history.atEnd()){
        const auto e=QJsonDocument::fromJson(history.readLine()).object();if(e["sample_time_sec"].toDouble()>0)events.push_back(e);
    }
    std::sort(events.begin(),events.end(),[](const QJsonObject&a,const QJsonObject&b){return a["sample_time_sec"].toDouble()<b["sample_time_sec"].toDouble();});
    QDialog dialog(parent);dialog.setObjectName("cameraReplayDialog");dialog.setWindowTitle("相机离线回放 · 不连接底盘");dialog.resize(1000,740);
    auto* layout=new QVBoxLayout(&dialog);auto* canvas=new CameraCanvas(&dialog);canvas->setReplayMode(true);layout->addWidget(canvas,1);
    auto* label=new QLabel(&dialog);label->setObjectName("cameraReplayStatus");label->setWordWrap(true);label->setTextInteractionFlags(Qt::TextSelectableByMouse);layout->addWidget(label);
    auto* slider=new QSlider(Qt::Horizontal,&dialog);slider->setObjectName("cameraReplayTimeline");slider->setRange(0,rows.size()-1);layout->addWidget(slider);
    auto* buttons=new QHBoxLayout;layout->addLayout(buttons);
    auto* previous=new QPushButton("上一帧",&dialog);auto* play=new QPushButton("播放",&dialog);auto* next=new QPushButton("下一帧",&dialog);
    next->setObjectName("cameraReplayNext");previous->setObjectName("cameraReplayPrevious");
    auto* speed=new QComboBox(&dialog);speed->addItem("0.5倍",.5);speed->addItem("1倍",1.0);speed->setCurrentIndex(1);
    buttons->addWidget(previous);buttons->addWidget(play);buttons->addWidget(next);buttons->addWidget(speed);
    QTimer timer(&dialog);int current=0;
    auto show=[&](int i){
        current=std::clamp(i,0,rows.size()-1);const auto& row=rows[current];
        const qint64 offset=qint64(row["offset"].toDouble()),length=qint64(row["length"].toDouble());
        if(offset<0||length<=0||length>16*1024*1024||offset+length>video.size()||!video.seek(offset)){
            timer.stop();play->setText("播放");label->setText("记录损坏或末帧未写完整，已停止回放。");return;
        }
        auto packet=std::make_shared<CameraAssist::CameraPacket>();packet->image=QImage::fromData(video.read(length),"JPEG");
        auto& f=packet->frame;f.session=row["session"].toString().toULongLong();f.sequence=row["sequence"].toString().toULongLong();
        f.width=packet->image.width();f.height=packet->image.height();f.captured_wall=row["captured_wall"].toDouble();
        for(const auto& value:row["people"].toArray()){
            const auto p=value.toObject();CameraAssist::Person person;person.track_id=p["id"].toString().toULongLong();person.stable=p["stable"].toBool();person.confidence=p["confidence"].toDouble();
            person.box=cv::Rect2f(p["x"].toDouble(),p["y"].toDouble(),p["width"].toDouble(),p["height"].toDouble());f.people.push_back(person);
        }
        QJsonObject event;double delta=.35;
        const auto it=std::lower_bound(events.begin(),events.end(),f.captured_wall-.35,[](const QJsonObject& e,double stamp){return e["sample_time_sec"].toDouble()<stamp;});
        for(auto j=it;j!=events.end()&&(*j)["sample_time_sec"].toDouble()<=f.captured_wall+.35;++j){
            const double d=std::abs((*j)["sample_time_sec"].toDouble()-f.captured_wall);if(d<delta){delta=d;event=*j;}
        }
        const auto guard=event["vision_guard"].toObject();
        const auto selected=guard["camera_track"].toString().toULongLong();
        canvas->display(packet,selected,event["target_id"].toInt(-1),event["selection_epoch"].toString().toULongLong());
        QString info=QString("离线帧 %1/%2 · 采集时刻 %3 · V人数 %4 · 录制丢弃 %5").arg(current+1).arg(rows.size()).arg(f.captured_wall,0,'f',3).arg(f.people.size()).arg(row["dropped"].toDouble());
        if(!event.isEmpty())info+=QString("\n最近雷达帧 %1 / P%2 · 时间差 %3 ms · %4 · 视觉 %5").arg(event["frame"].toInt()).arg(event["target_id"].toInt()).arg(delta*1000,0,'f',1).arg(event["motion_reason"].toString()).arg(guard["evidence_reason"].toString(guard["reason"].toString()));
        else info+="\n此时没有可配对的雷达控制记录（未锁定时也可能没有控制输出）。";
        label->setText(info);{const QSignalBlocker blocker(slider);slider->setValue(current);}
        double ms=100;if(current+1<rows.size())ms=(rows[current+1]["captured_wall"].toDouble()-f.captured_wall)*1000;
        timer.setInterval(std::clamp(int(ms/speed->currentData().toDouble()),20,2000));
    };
    QObject::connect(slider,&QSlider::valueChanged,&dialog,[&](int i){show(i);});
    QObject::connect(previous,&QPushButton::clicked,&dialog,[&]{timer.stop();play->setText("播放");show(current-1);});
    QObject::connect(next,&QPushButton::clicked,&dialog,[&]{timer.stop();play->setText("播放");show(current+1);});
    QObject::connect(play,&QPushButton::clicked,&dialog,[&]{if(timer.isActive()){timer.stop();play->setText("播放");}else{if(current+1>=rows.size())show(0);timer.start();play->setText("暂停");}});
    QObject::connect(&timer,&QTimer::timeout,&dialog,[&]{if(current+1>=rows.size()){timer.stop();play->setText("播放");}else show(current+1);});
    show(0);dialog.exec();
}
