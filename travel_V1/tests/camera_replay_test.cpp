#include "camera_replay.hpp"
#include "camera_recording.hpp"
#include <QApplication>
#include <QDialog>
#include <QTemporaryDir>
#include <QTimer>
#include <QSlider>
#include <QLabel>
#include <QDir>
#include <QPushButton>
#include <iostream>
int main(int argc,char**argv){QApplication app(argc,argv);QTemporaryDir temp;CameraRecorder archive;if(!archive.start(temp.path()))return 1;
 CameraAssist::VisionFrame f;f.session=1;f.sequence=1;f.width=640;f.height=480;f.captured_wall=100;QImage image(640,480,QImage::Format_RGB888);image.fill(Qt::blue);
 archive.frame(f,image);++f.sequence;f.captured_wall=100.1;archive.frame(f,image);archive.stop();const auto dirs=QDir(temp.path()).entryList(QDir::Dirs|QDir::NoDotAndDotDot);if(dirs.size()!=1)return 2;
 bool passed=false;QTimer::singleShot(100,[&]{
  for(auto* widget:QApplication::topLevelWidgets())if(auto* dialog=qobject_cast<QDialog*>(widget))if(dialog->objectName()=="cameraReplayDialog"){
   auto* slider=dialog->findChild<QSlider*>("cameraReplayTimeline");auto* label=dialog->findChild<QLabel*>("cameraReplayStatus");
   auto* previous=dialog->findChild<QPushButton*>("cameraReplayPrevious");auto* next=dialog->findChild<QPushButton*>("cameraReplayNext");
   if(slider&&label&&previous&&next){slider->setValue(1);bool last=label->text().contains("2/2");previous->click();bool first=label->text().contains("1/2");next->click();passed=last&&first&&label->text().contains("2/2");}
   dialog->accept();return;
  }
  app.quit();
 });
 openCameraReplayFile(QDir(temp.path()).filePath(dirs[0]+"/frames.jsonl"),nullptr);
 if(!passed)return 3;std::cout<<"PASS: real JPEG archive playback, seek and frame stepping without sensor/control objects\n";
}
