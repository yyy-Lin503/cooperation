#include "ros_runtime_panel.hpp"
#include "radar_project_root.hpp"
#include <QApplication>
#include <QTemporaryDir>
#include <QFile>
#include <QDir>
#include <QThread>
#include <QElapsedTimer>
#include <QJsonDocument>
#include <QJsonArray>
#include <iostream>
#include <stdexcept>

static void check(bool ok,const char* text){if(!ok)throw std::runtime_error(text);}
static void writeFile(const QString& path,const QByteArray& content){QFile file(path);check(file.open(QIODevice::WriteOnly),"Cannot create test fixture");check(file.write(content)==content.size(),"Fixture write failed");}
static QByteArray readFile(const QString& path){QFile file(path);check(file.open(QIODevice::ReadOnly),"Missing test output");return file.readAll();}
template<class Test>static void waitFor(Test predicate){QElapsedTimer elapsed;elapsed.start();while(!predicate()&&elapsed.elapsed()<5000){QApplication::processEvents();QThread::msleep(2);}check(predicate(),"Qt runtime timed out");}
struct RosRuntimePanelTestAccess {
    static QString folder(const RosRuntimePanel& panel){return panel.folder_;}
    static bool ready(const RosRuntimePanel& panel){return panel.ready_;}
    static QString status(const RosRuntimePanel& panel){return panel.status_->text();}
    static bool settingsLocked(const RosRuntimePanel& panel){return !panel.setup_->isEnabled();}
    static void kill(RosRuntimePanel& panel){panel.process_.kill();}
    static void event(RosRuntimePanel& panel,const QJsonObject& value){panel.handleEvent(value);}
};
int main(int argc,char** argv){
    QApplication app(argc,argv);check(argc>=2,"Need test peer script path");
    QTemporaryDir temporary;check(temporary.isValid(),"No temporary directory");
    const auto root=QDir(temporary.path()).filePath("项目 with spaces");QDir().mkpath(root+"/scripts");QDir().mkpath(root+"/build-v8-jetson");
    writeFile(root+"/VERSION.txt","V8\n");
    writeFile(root+"/scripts/run_gui_runtime.sh","#!/bin/bash\nexec /usr/bin/python3 \"$(dirname \"$0\")/../fake.py\" \"$@\"\n");
    check(QFile::copy(QString::fromLocal8Bit(argv[1]),root+"/fake.py"),"Cannot copy test peer");
    const auto cleanupFolder=root+"/功能包/my_radar_follower/my_radar_follower";QDir().mkpath(cleanupFolder);
    const auto sourceRoot=QFileInfo(QString::fromLocal8Bit(argv[1])).dir().absoluteFilePath("..");
    check(QFile::copy(sourceRoot+"/功能包/my_radar_follower/my_radar_follower/gui_processes.py",cleanupFolder+"/gui_processes.py"),"Cannot copy cleanup test helper");
    const auto executable=root+"/build-v8-jetson/radar_gui";writeFile(executable,"");
    QProcessEnvironment env;
    check(radarProjectRoot(executable,temporary.path(),env)==QDir(root).canonicalPath(),"Executable root resolution failed");
    env.insert("RADAR_PROJECT_ROOT",temporary.path());bool refused=false;
    try{radarProjectRoot(executable,root,env);}catch(const std::runtime_error&){refused=true;}
    check(refused,"Conflicting project environment accepted");
    const auto setup=root+"/底盘 setup.bash";writeFile(setup,"");
    QString firstFolder;
    {
        RosRuntimePanel panel(root);panel.findChild<QLineEdit*>("chassis_setup")->setText(setup);
        const auto initialDisplay=panel.displayState();
        check(initialDisplay["phase"].toString()=="unknown"&&initialDisplay["age_seconds"].toDouble()==-1&&
              !initialDisplay["process_running"].toBool()&&!initialDisplay["follow_requested"].toBool(),
              "New runtime advertised a cached running state");
        check(panel.findChild<QDoubleSpinBox*>("max_linear_speed_mps")->value()==.15,"Wrong default linear speed");
        check(panel.findChild<QDoubleSpinBox*>("max_angular_speed_radps")->value()==.25,"Wrong default yaw speed");
        bool canStart=true,canStop=false;
        QObject::connect(&panel,&RosRuntimePanel::controlsChanged,[&](bool start,bool stop){canStart=start;canStop=stop;});
        panel.setCaptureSession(1234567890123456789ULL);panel.startFollow();panel.startFollow();
        waitFor([&]{return RosRuntimePanelTestAccess::status(panel).contains("等待选择");});
        const auto followingDisplay=panel.displayState();
        check(followingDisplay["phase"].toString()=="following"&&followingDisplay["age_seconds"].toDouble()>=0&&
              followingDisplay["process_running"].toBool()&&followingDisplay["ready"].toBool()&&
              followingDisplay["follow_requested"].toBool()&&!followingDisplay["stopping"].toBool(),
              "Fresh helper state was not exposed with its lifecycle gates");
        check(followingDisplay["control"].toObject()["reason"].toString()=="waiting_for_target"&&
              followingDisplay["components"].toObject()["follow"].toString()=="owned",
              "Display state lost controller waiting reason or component ownership");
        QThread::msleep(25);
        check(panel.displayState()["age_seconds"].toDouble()>=followingDisplay["age_seconds"].toDouble()+.015,
              "Frozen helper display state did not age without new events");
        check(!canStart&&canStop&&RosRuntimePanelTestAccess::settingsLocked(panel),"Active controls/config not locked");
        firstFolder=RosRuntimePanelTestAccess::folder(panel);
        const auto settings=QJsonDocument::fromJson(readFile(firstFolder+"/received-settings.json")).object();
        check(settings["chassis_setup"].toString()==setup,"Spaces/Chinese path corrupted");
        check(readFile(firstFolder+"/commands.jsonl").contains("1234567890123456789"),"64-bit session lost precision");
        check(readFile(firstFolder+"/commands.jsonl").count("\"action\":\"start\"")==1,"Repeated click sent duplicate start");
        panel.stopFollow();
        const auto stoppingDisplay=panel.displayState();
        check(!stoppingDisplay["follow_requested"].toBool()&&stoppingDisplay["stopping"].toBool()&&
              stoppingDisplay["phase"].toString()=="unknown"&&stoppingDisplay["age_seconds"].toDouble()==-1,
              "Local stop retained a cached following display");
        RosRuntimePanelTestAccess::event(panel,{{"type","state"},{"phase","following"},
            {"control",QJsonObject{{"reason","tracking_command"}}}});
        check(panel.displayState()["phase"].toString()!="following"&&
              !panel.displayState()["follow_requested"].toBool(),
              "Late helper event restored following after a local stop");
        waitFor([&]{return canStart&&!canStop;});
        check(panel.active()&&RosRuntimePanelTestAccess::settingsLocked(panel),"Stopping follow shut down dependencies/unlocked live config");
        const auto idleDisplay=panel.displayState();
        check(idleDisplay["process_running"].toBool()&&idleDisplay["ready"].toBool()&&
              !idleDisplay["follow_requested"].toBool()&&idleDisplay["phase"].toString()=="idle",
              "Retained ROS dependencies were mistaken for an active follow session");
        panel.startFollow();
        check(panel.displayState()["phase"].toString()=="unknown"&&
              panel.displayState()["age_seconds"].toDouble()==-1,
              "Restart reused the previous helper state before a new event");
        waitFor([&]{return canStop&&panel.displayState()["phase"].toString()=="following";});
        check(RosRuntimePanelTestAccess::folder(panel)==firstFolder,"Restart launched a second runtime");
        panel.setCaptureSession(0);
        check(panel.shutdownBlocking(),"Normal shutdown did not finish");
        const auto closedDisplay=panel.displayState();
        check(closedDisplay["phase"].toString()=="unknown"&&closedDisplay["age_seconds"].toDouble()==-1&&
              !closedDisplay["process_running"].toBool()&&!closedDisplay["ready"].toBool()&&
              !closedDisplay["follow_requested"].toBool(),"Shutdown retained a live follow display");
        check(!RosRuntimePanelTestAccess::settingsLocked(panel),"Shutdown left settings locked");
        if(app.arguments().size()==4&&app.arguments()[2]=="--screenshot"){
            panel.resize(480,600);panel.show();app.processEvents();check(panel.grab().save(app.arguments()[3]),"Cannot save ROS panel screenshot");
        }
    }
    {
        RosRuntimePanel panel(root);
        check(panel.findChild<QLineEdit*>("chassis_setup")->text()==setup,"Settings not persisted");
        panel.startFollow();waitFor([&]{return RosRuntimePanelTestAccess::ready(panel);});
        RosRuntimePanelTestAccess::kill(panel);waitFor([&]{return !panel.active();});
        check(panel.displayState()["phase"].toString()=="unknown"&&
              panel.displayState()["age_seconds"].toDouble()==-1&&
              !panel.displayState()["process_running"].toBool(),"Runtime crash retained display state");
        check(!RosRuntimePanelTestAccess::settingsLocked(panel),"Runtime crash left config locked");
        panel.startFollow();waitFor([&]{return RosRuntimePanelTestAccess::ready(panel);});
        // Destructor exercises window-close cleanup of an active runtime.
    }
    {
        RosRuntimePanel panel(root);
        panel.findChild<QLineEdit*>("chassis_package")->setText("test_owned_child");panel.startFollow();
        waitFor([&]{return QFileInfo::exists(RosRuntimePanelTestAccess::folder(panel)+"/owned.json");});
        const auto registry=RosRuntimePanelTestAccess::folder(panel)+"/owned.json";
        RosRuntimePanelTestAccess::kill(panel);
        waitFor([&]{return !panel.active();});
        check(QJsonDocument::fromJson(readFile(registry)).object()["owned"].toObject().isEmpty(),"Qt did not clean up dead helper's owned child");
        check(!RosRuntimePanelTestAccess::settingsLocked(panel),"Emergency cleanup did not unlock settings");
    }
    {
        writeFile(root+"/scripts/run_gui_runtime.sh","#!/bin/bash\necho 'simulated missing ROS installation' >&2\nexit 7\n");
        RosRuntimePanel panel(root);QString messages;
        QObject::connect(&panel,&RosRuntimePanel::diagnostic,[&](const QString& text){messages+=text;});
        panel.startFollow();waitFor([&]{return !panel.active();});
        check(messages.contains("simulated missing ROS installation"),"Bootstrap failure detail not shown in GUI log");
        check(RosRuntimePanelTestAccess::status(panel).contains("失败"),"Bootstrap failure not shown as a failure");
        check(!RosRuntimePanelTestAccess::settingsLocked(panel),"Failed startup locked settings");
        check(panel.displayState()["phase"].toString()=="unknown"&&
              !panel.displayState()["follow_requested"].toBool(),"Failed startup advertised following");
    }
    std::cout<<"PASS: root resolution, helper display freshness/stop/restart/crash gates, GUI configuration and process cleanup\n";
}
