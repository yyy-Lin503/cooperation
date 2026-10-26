#include "camera_assist_panel.hpp"
#include "frame_context.hpp"
#include <QApplication>
#include <QTemporaryDir>
#include <QDir>
#include <QMouseEvent>
#include <QPushButton>
#include <iostream>
#include <stdexcept>
static void check(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
struct CameraPanelTestAccess {
    static void configure(CameraAssistPanel& panel,std::shared_ptr<CameraAssist::CameraPacket> packet,CameraAssist::RadarFrame radar) {
        panel.acceptPacket(packet);panel.setRadarFrame(radar);
    }
    static void guard(CameraAssistPanel& panel){panel.guard_->setChecked(true);}
    static CameraCanvas* canvas(CameraAssistPanel& panel){return panel.canvas_;}
};
static std::uint64_t bound(const CameraAssistPanel& panel,std::uint64_t uid=42,std::uint64_t epoch=3){
    return panel.metadata(uid,epoch)["camera_track"].toString().toULongLong();
}
static void click(CameraCanvas* canvas,QPointF where,Qt::MouseButton button=Qt::LeftButton){
    QMouseEvent event(QEvent::MouseButtonPress,where,button,button,Qt::NoModifier);
    QApplication::sendEvent(canvas,&event);
}
int main(int argc,char**argv){
    QApplication app(argc,argv);QTemporaryDir temp;check(temp.isValid(),"No temp directory");QDir::setCurrent(temp.path());
    // Construct/layout widgets before stamping the simulated live evidence;
    // first-time font discovery can take longer than its freshness budget.
    CameraAssistPanel panel;
    // A configured source need not appear in Linux's /dev enumeration.
    const auto originalDevice=qgetenv("RADAR_CAMERA_DEVICE");
    qputenv("RADAR_CAMERA_DEVICE","tcp://127.0.0.1:32123");
    {
        CameraAssistPanel localPanel;
        auto* devices=localPanel.findChild<QComboBox*>("cameraDeviceCombo");
        check(devices&&devices->currentData().toString()=="tcp://127.0.0.1:32123",
              "Configured Windows camera was not selectable without a Linux video node");
    }
    if(originalDevice.isNull())qunsetenv("RADAR_CAMERA_DEVICE");else qputenv("RADAR_CAMERA_DEVICE",originalDevice);
    const auto emptyEvidence=panel.displayEvidence();
    check(!emptyEvidence["available"].toBool()&&!emptyEvidence["bound"].toBool()&&
          !emptyEvidence["verified"].toBool(),"Empty camera advertised visual evidence");
    auto* canvas=CameraPanelTestAccess::canvas(panel);canvas->setFixedSize(800,500);
    const double now=RadarTime::monotonicSeconds();
    auto packet=std::make_shared<CameraAssist::CameraPacket>();
    packet->image=QImage(640,480,QImage::Format_RGB888);packet->image.fill(Qt::black);
    auto& f=packet->frame;f.session=2;f.sequence=1;f.device="camera";f.width=640;f.height=480;
    f.captured_monotonic=now;f.completed_monotonic=now;f.captured_wall=RadarTime::wallSeconds();
    f.people={{{270,180,100,180},.9,7,true},{{450,180,100,180},.9,8,true}};
    CameraAssist::RadarFrame radar{1,now,{{42,1,-8,12},{43,2,8,2}}};
    CameraPanelTestAccess::configure(panel,packet,radar);
    check(!panel.findChild<QPushButton*>("cameraAddPairButton"),"Calibration UI remains");
    const QPointF first(400,300),second(590,300);
    click(canvas,first);check(bound(panel)==0,"Camera click bound without a radar lock");
    panel.setLockedTarget(42,1,3,1);
    check(bound(panel)==7,"Previously clicked camera person was not associated after manual radar selection");
    panel.findChild<QPushButton*>("cameraUnbindButton")->click();
    click(canvas,{10,200});check(bound(panel)==0,"Letterbox click bound a person");
    click(canvas,first);check(bound(panel)==7,"Click failed to bind existing radar lock without calibration");
    check(!panel.decision(42,3).enabled,"Binding silently enabled optional protection");
    const auto evidence=panel.displayEvidence();
    check(evidence["available"].toBool()&&evidence["bound"].toBool()&&evidence["verified"].toBool()&&
          evidence["reason"].toString()=="verified"&&evidence["captured_monotonic"].toDouble()==now,
          "Display evidence did not verify a fresh manual binding with protection disabled");
    // Reuse this exact canvas in the compact view; no additional camera or
    // binding is created, and clicks remain relative to the letterboxed image.
    canvas->setMaximumSize(QWIDGETSIZE_MAX,QWIDGETSIZE_MAX);
    panel.setCompactView(true);
    check(CameraPanelTestAccess::canvas(panel)==canvas&&panel.findChildren<CameraCanvas*>().size()==1,
          "Compact layout replaced or duplicated the camera canvas");
    check(canvas->minimumSize()==QSize(200,140)&&bound(panel)==7,
          "Compact layout changed binding or retained the large minimum size");
    canvas->resize(260,150);
    panel.findChild<QPushButton*>("cameraUnbindButton")->click();
    click(canvas,{10,80});check(bound(panel)==0,"Compact letterbox click selected a person");
    click(canvas,{130,85});check(bound(panel)==7,"Compact image click did not bind the same person");
    panel.setCompactView(false);
    check(canvas->minimumSize()==QSize(300,225)&&CameraPanelTestAccess::canvas(panel)==canvas&&bound(panel)==7,
          "Single-camera layout did not restore size and preserve the existing binding");
    canvas->setFixedSize(800,500);
    check(!panel.decision(42,3).enabled,"Compact view silently enabled visual protection");
    auto meta=panel.metadata(42,3);
    check(meta["radar_uid"].toString()=="42"&&meta["binding_mode"].toString()=="manual_uncalibrated","Manual identity metadata missing");
    CameraPanelTestAccess::guard(panel);check(panel.decision(42,3).permit,"Bound person not permitted");
    check(!panel.decision(43,3).permit&&!panel.decision(42,4).permit,"Another lock/epoch inherited binding");
    panel.setLockedTarget(42,1,3,1);check(bound(panel)==7,"Same lock refresh erased binding");
    click(canvas,second,Qt::RightButton);check(bound(panel)==7,"Right click rebound person");
    click(canvas,{100,20});check(bound(panel)==7,"Empty click erased binding");
    click(canvas,second);check(bound(panel)==8,"Explicit click failed to rebind person");
    auto next=std::make_shared<CameraAssist::CameraPacket>(*packet);next->frame.sequence=2;
    next->frame.people.push_back({{455,180,100,180},.9,9,false});
    CameraPanelTestAccess::configure(panel,next,radar);
    click(canvas,second);check(bound(panel)==8,"Overlapping unstable box changed binding");
    next=std::make_shared<CameraAssist::CameraPacket>(*packet);next->frame.sequence=3;
    next->frame.people.erase(next->frame.people.begin()+1);
    CameraPanelTestAccess::configure(panel,next,radar);
    check(bound(panel)==8&&!panel.decision(42,3).permit,"Strict mode allowed a missed person or erased its task binding");
    CameraPanelTestAccess::configure(panel,packet,radar);
    click(canvas,second);check(bound(panel)==8&&!panel.decision(42,3).permit,"Old frame restored visual permission");
    next=std::make_shared<CameraAssist::CameraPacket>(*packet);next->frame.sequence=4;
    CameraPanelTestAccess::configure(panel,next,radar);
    check(bound(panel)==8&&panel.decision(42,3).permit,"Same visual track did not recover existing association");
    click(canvas,second);check(bound(panel)==8&&panel.decision(42,3).permit,"Explicit binding after loss failed");
    next=std::make_shared<CameraAssist::CameraPacket>(*next);next->frame.session=3;next->frame.sequence=1;
    CameraPanelTestAccess::configure(panel,next,radar);
    check(bound(panel)==0&&!panel.decision(42,3).permit,"Camera restart retained binding");
    click(canvas,first);check(bound(panel)==7,"Explicit binding after camera restart failed");
    panel.setLockedTarget(43,1,3,2);check(bound(panel,43)==7,"Confirmed same-task radar recovery erased visual association");
    click(canvas,first);check(bound(panel,43)==7&&panel.decision(43,3).permit,"New lock binding failed");
    panel.setLockedTarget(43,1,4,2);check(bound(panel,43,4)==0,"New epoch inherited binding");
    click(canvas,first);check(bound(panel,43,4)==7,"Rebinding new epoch failed");
    panel.findChild<QPushButton*>("cameraUnbindButton")->click();
    check(bound(panel,43,4)==0&&panel.decision(43,4).enabled&&!panel.decision(43,4).permit,"Unbind disabled protection");
    click(canvas,first);check(bound(panel,43,4)==7,"Unbind changed existing radar context");
    // A frozen image can remain visible and bound, but may not be presented as
    // verified. Use timestamped fixtures rather than waiting on real hardware.
    const double staleNow=RadarTime::monotonicSeconds();
    next=std::make_shared<CameraAssist::CameraPacket>(*next);++next->frame.sequence;
    next->frame.captured_monotonic=staleNow-.8;next->frame.completed_monotonic=staleNow-.8;
    radar.captured_monotonic=staleNow;
    CameraPanelTestAccess::configure(panel,next,radar);
    const auto staleEvidence=panel.displayEvidence();
    check(staleEvidence["available"].toBool()&&staleEvidence["bound"].toBool()&&
          !staleEvidence["verified"].toBool()&&staleEvidence["reason"].toString()=="camera_stale",
          "Stale camera frame was shown as verified or erased the stored binding");
    next=std::make_shared<CameraAssist::CameraPacket>(*next);++next->frame.sequence;
    const double freshNow=RadarTime::monotonicSeconds();
    next->frame.captured_monotonic=freshNow;next->frame.completed_monotonic=freshNow;
    radar.captured_monotonic=freshNow;
    CameraPanelTestAccess::configure(panel,next,radar);
    check(panel.displayEvidence()["verified"].toBool()&&panel.decision(43,4).permit,
          "Fresh evidence did not restore the existing continuous binding");
    if(app.arguments().size()==3&&app.arguments()[1]=="--screenshot") {
        canvas->setMinimumSize(300,225);canvas->setMaximumSize(QWIDGETSIZE_MAX,QWIDGETSIZE_MAX);
        panel.resize(1100,460);panel.show();app.processEvents();
        check(panel.grab().save(app.arguments()[2]),"Panel screenshot failed");
    }
    panel.stopCamera();check(panel.decision(43,4).enabled&&!panel.decision(43,4).permit,"Camera stop disabled protection");
    const auto stoppedEvidence=panel.displayEvidence();
    check(!stoppedEvidence["available"].toBool()&&!stoppedEvidence["bound"].toBool()&&
          !stoppedEvidence["verified"].toBool()&&stoppedEvidence["captured_monotonic"].toDouble()==0,
          "Camera stop retained display evidence");
    panel.resetRadar();panel.stopCamera();
    // Failure and repeated cleanup without opening a physical camera.
    CameraAssist::CameraWorker worker;
    for(int i=0;i<2;++i){worker.start({"/nonexistent-camera","/nonexistent-model.onnx",false});
        const auto end=std::chrono::steady_clock::now()+std::chrono::seconds(2);
        while(worker.running()&&std::chrono::steady_clock::now()<end)std::this_thread::sleep_for(std::chrono::milliseconds(5));
        check(worker.latest()&&!worker.latest()->error.empty(),"Worker error not reported");worker.stop();worker.stop();check(!worker.latest(),"Stopped worker retained stale image");}
    std::cout<<"PASS: manual binding, compact/full canvas click mapping, display evidence freshness, restart/selection gates and worker cleanup\n";
}
