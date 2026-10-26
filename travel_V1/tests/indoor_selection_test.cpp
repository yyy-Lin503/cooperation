#include "radar_gui.hpp"
#include "ui_radar_gui_v2.h"
#include <QApplication>
#include <QTemporaryDir>
#include <QDir>
#include <QMouseEvent>
#include <stdexcept>
#include <iostream>
void check(bool b,const char*m){if(!b)throw std::runtime_error(m);}
struct RadarGUIIntegrationAccess {
 static void seed(RadarGUI&g){
  for(int f=1;f<=5;++f){std::vector<PointCloud> points;
   for(float dx:{-.1f,-.05f,0.f,.05f,.1f}){PointCloud p;p.x=dx;p.y=3+.3f*dx;p.source_frame=f;p.snr=10;p.is_moving=true;points.push_back(p);}
   g.targetTracker.updateCandidates(points,nullptr,f);
  }
  check(!g.targetTracker.candidates().empty(),"seed candidates");
  g.refreshTargetTable();g.drawTrackingOverlay();
 }
 static QPoint target(RadarGUI&g){const auto& c=g.targetTracker.candidates()[0];return g.ui->graphicsView->mapFromScene(QPointF(g.sceneXFromMeters(c.x),g.sceneYFromMeters(c.y)));}
 static QWidget* viewport(RadarGUI&g){return g.ui->graphicsView->viewport();}
 static bool preview(RadarGUI&g){return g.previewTargetUid==g.targetTracker.candidates()[0].internal_uid;}
 static void clear(RadarGUI&g){g.clearTargetPreview();g.refreshTargetTable();}
 static QTableWidget* table(RadarGUI&g){return g.targetTable;}
 static QPushButton* confirm(RadarGUI&g){return g.ui->autoSelectButton;}
 static bool pending(RadarGUI&g){return g.targetTracker.lockedTarget().valid&&g.targetTracker.manualLockConfirmationPending()&&!g.targetTracker.lockedTarget().control_allow_translation;}
};
void click(QWidget*w,QPoint p){QMouseEvent down(QEvent::MouseButtonPress,QPointF(p),Qt::LeftButton,Qt::LeftButton,Qt::NoModifier);QApplication::sendEvent(w,&down);QMouseEvent up(QEvent::MouseButtonRelease,QPointF(p),Qt::LeftButton,Qt::NoButton,Qt::NoModifier);QApplication::sendEvent(w,&up);}
int main(int argc,char**argv){QApplication app(argc,argv);QTemporaryDir temp;check(temp.isValid(),"temp");QDir::setCurrent(temp.path());QUdpSocket probe;check(probe.bind(QHostAddress(QHostAddress::LocalHost),quint16(0)),"free port");const auto port=probe.localPort();probe.close();
 qputenv("RADAR_RESULT_IP","127.0.0.1");qputenv("RADAR_RESULT_PORT",QByteArray::number(port));
 RadarGUI gui;gui.resize(1100,740);gui.show();app.processEvents();RadarGUIIntegrationAccess::seed(gui);app.processEvents();
 click(RadarGUIIntegrationAccess::viewport(gui),RadarGUIIntegrationAccess::target(gui));check(RadarGUIIntegrationAccess::preview(gui),"Plot click did not preview candidate");
 RadarGUIIntegrationAccess::clear(gui);auto* table=RadarGUIIntegrationAccess::table(gui);check(table->rowCount()>0,"table empty");
 click(table->viewport(),table->visualItemRect(table->item(0,1)).center());check(RadarGUIIntegrationAccess::preview(gui),"ID column did not preview candidate");
 RadarGUIIntegrationAccess::confirm(gui)->click();check(RadarGUIIntegrationAccess::pending(gui),"Confirmation bypassed observation gate");
 std::cout<<"PASS: point-cloud hit testing, ID-column selection and confirmation motion gate\n";
}
