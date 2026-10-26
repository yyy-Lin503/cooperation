#include "radar_gui.hpp"
#include <QApplication>
#include <QTemporaryDir>
#include <QPushButton>
#include <QJsonDocument>
#include <QJsonObject>
#include <arpa/inet.h>
#include <atomic>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <thread>
#include <stdexcept>
#if RADAR_ENABLE_CAMERA
struct CameraPanelTestAccess {
    static void enableUnboundGuard(CameraAssistPanel& panel){panel.guard_->setChecked(true);}
    static void disableAndRefresh(CameraAssistPanel& panel){panel.guard_->setChecked(false);emit panel.evidenceChanged();}
    static bool clickPerson(CameraAssistPanel& panel,std::uint64_t uid,std::uint64_t session,std::uint64_t epoch){
        auto packet=std::make_shared<CameraAssist::CameraPacket>();
        packet->image=QImage(640,480,QImage::Format_RGB888);packet->image.fill(Qt::black);
        auto& frame=packet->frame;frame.session=2026;frame.sequence=1;frame.width=640;frame.height=480;
        frame.captured_monotonic=frame.completed_monotonic=RadarTime::monotonicSeconds();
        frame.captured_wall=RadarTime::wallSeconds();frame.people={{{270,180,100,180},.9,7,true}};
        panel.acceptPacket(packet);panel.refresh();
        emit panel.canvas_->personClicked(7);
        const auto bound=panel.binding_;
        const bool ok=bound.radar_uid==uid&&bound.radar_session==session&&
            bound.selection_epoch==epoch&&bound.camera_track==7;
        panel.clearBinding();return ok;
    }
};
#endif
struct RadarGUIIntegrationAccess {
    static void start(RadarGUI& gui,const char* cfg) {gui.radar_config=IWR6843AOP_TLV(cfg).get_params();gui.config_file=cfg;gui.start_capture();}
    static std::uint64_t session(RadarGUI& gui) {return gui.collector?gui.collector->session_id():0;}
    static bool stopped(RadarGUI& gui) {return !gui.collector && !gui.is_capturing;}
    static bool pending(RadarGUI& gui) {return gui.targetTracker.manualLockConfirmationPending() && !gui.targetTracker.lockedTarget().control_allow_translation;}
#if RADAR_ENABLE_CAMERA
    static void enableUnboundGuard(RadarGUI& gui){CameraPanelTestAccess::enableUnboundGuard(*gui.cameraAssist_);}
    static void refreshCamera(RadarGUI& gui){CameraPanelTestAccess::disableAndRefresh(*gui.cameraAssist_);}
    static void markStopped(RadarGUI& gui){gui.writeTrackingMetadata(gui.currentFrameContext.frame_number,"test_producer_stop");}
    static bool bindCurrentLock(RadarGUI& gui){
        const auto before=gui.targetTracker.output();
        const bool ok=CameraPanelTestAccess::clickPerson(*gui.cameraAssist_,before.target.matched_internal_uid,
            before.frame.session_id,before.selection.selection_epoch);
        const auto after=gui.targetTracker.output();
        return ok&&before.target.matched_internal_uid==after.target.matched_internal_uid&&
            before.selection.selection_epoch==after.selection.selection_epoch&&
            before.target.x==after.target.x&&before.target.y==after.target.y;
    }
#endif
};
static void check(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
static int freePort(){QUdpSocket socket;check(socket.bind(QHostAddress(QHostAddress::LocalHost),static_cast<quint16>(0)),"Port allocation failed");return socket.localPort();}
struct Peer {
    int socket_fd;
    int port;
    std::atomic<bool> running{true};std::thread worker;
    Peer(){
        socket_fd=socket(AF_INET,SOCK_DGRAM,0);sockaddr_in address{};address.sin_family=AF_INET;address.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
        check(bind(socket_fd,reinterpret_cast<sockaddr*>(&address),sizeof(address))==0,"Mock DCA bind failed");
        socklen_t length=sizeof(address);getsockname(socket_fd,reinterpret_cast<sockaddr*>(&address),&length);port=ntohs(address.sin_port);
        timeval timeout{0,100000};setsockopt(socket_fd,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));
        worker=std::thread([this]{while(running){unsigned char data[2048];sockaddr_in sender{};socklen_t n=sizeof(sender);
            int size=recvfrom(socket_fd,data,sizeof(data),0,reinterpret_cast<sockaddr*>(&sender),&n);
            if(size<8)continue;
            unsigned char reply[]={0x5a,0xa5,data[2],data[3],0,0,0xaa,0xee};
            sendto(socket_fd,reply,sizeof(reply),0,reinterpret_cast<sockaddr*>(&sender),n);
            if(data[2]==5) {
                const unsigned char status[]={0x5a,0xa5,0x0a,0,0,2,0xaa,0xee};
                sendto(socket_fd,status,sizeof(status),0,reinterpret_cast<sockaddr*>(&sender),n);
            }
        }});
    }
    ~Peer(){running=false;worker.join();close(socket_fd);}
};
template<class Predicate> static void waitFor(Predicate test){
    const auto end=std::chrono::steady_clock::now()+std::chrono::seconds(2);
    while(!test() && std::chrono::steady_clock::now()<end){QApplication::processEvents();std::this_thread::sleep_for(std::chrono::milliseconds(2));}
    check(test(),"GUI transport timed out");
}
static QJsonObject metadata(const QString& file){QFile input(file);check(input.open(QIODevice::ReadOnly),"Missing metadata");return QJsonDocument::fromJson(input.readAll()).object();}
static int columns(const std::string& row){bool quoted=false;int count=1;for(std::size_t i=0;i<row.size();++i){if(row[i]=='"'){if(quoted && i+1<row.size() && row[i+1]=='"')++i;else quoted=!quoted;}else if(row[i]==','&&!quoted)++count;}return count;}
int main(int argc,char**argv){
    check(argc==2,"Missing cfg argument");
    QApplication app(argc,argv);QTemporaryDir directory;check(directory.isValid(),"Temporary dir failed");QDir::setCurrent(directory.path());
    Peer peer;
    qputenv("RADAR_HOST_IP","127.0.0.1");qputenv("RADAR_FPGA_IP","127.0.0.1");
    qputenv("RADAR_FPGA_PORT",QByteArray::number(peer.port));
    qputenv("RADAR_CONFIG_PORT",QByteArray::number(freePort()));qputenv("RADAR_DATA_PORT",QByteArray::number(freePort()));
    const int result_port=freePort();qputenv("RADAR_RESULT_PORT",QByteArray::number(result_port));
    RadarGUI gui;RadarGUIIntegrationAccess::start(gui,argv[1]);
    const auto session=RadarGUIIntegrationAccess::session(gui);check(session!=0,"Capture startup failed");
    auto* log=gui.findChild<QTextEdit*>("logTextEdit");check(log,"Missing GUI log");
    waitFor([&]{return log->toPlainText().contains("0x0200") &&
        log->toPlainText().contains("STS_LVDS_BUFFER_FULL");});
    QUdpSocket source;
    auto send=[&](int frame,bool points){
        const double now=RadarTime::wallSeconds();std::ostringstream text;text<<std::setprecision(17);
        std::vector<std::string> fields(26,"0");fields[0]="META";fields[1]=std::to_string(frame);fields[2]="test";fields[24]="1";fields[25]="";
        for(std::size_t i=0;i<fields.size();++i)text<<(i?",":"")<<fields[i];
        if(points)for(float x:{-.1f,0.f,.1f})text<<'\n'<<frame<<','<<x<<",3,0,0,0,test,"<<frame<<",5,32,10,1";
        text<<"\nFRAME,"<<frame<<','<<session<<','<<now<<','<<now<<",receive_estimate,0,"<<now;
        const auto bytes=QByteArray::fromStdString(text.str());
        check(source.writeDatagram(bytes,QHostAddress(QHostAddress::LocalHost),result_port)==bytes.size(),"Point datagram send failed");
        // GUI is asynchronous: process its UDP event before sending the next frame.
        const auto end=std::chrono::steady_clock::now()+std::chrono::milliseconds(15);
        while(std::chrono::steady_clock::now()<end){app.processEvents();std::this_thread::sleep_for(std::chrono::milliseconds(1));}
    };
    for(int frame=1;frame<=5;++frame)send(frame,true);
    auto* table=gui.findChild<QTableWidget*>("tableWidget");check(table,"Missing target table");
    waitFor([&]{return table->rowCount()>0 && table->item(0,1) && table->item(0,1)->data(Qt::UserRole).toInt()>0;});
    QMetaObject::invokeMethod(table,"cellClicked",Qt::DirectConnection,Q_ARG(int,0),Q_ARG(int,0));
    auto* confirm=gui.findChild<QPushButton*>("autoSelectButton");check(confirm && confirm->isEnabled(),"Target confirm unavailable");confirm->click();
    check(RadarGUIIntegrationAccess::pending(gui),"Manual selection did not hold motion");
    send(6,true);
    QDir tracks(directory.path()+"/var/tracking/txt");QStringList files;
    waitFor([&]{files=tracks.entryList({"*.txt"},QDir::Files);return !files.empty();});
    const QString track=tracks.filePath(files.front());
    auto first=metadata(track+".meta.json");check(first["frame"].toInt()==6 && first["last_measurement_time_sec"].toDouble()>0,"Accepted observation time not exported");
#if RADAR_ENABLE_CAMERA
    check(RadarGUIIntegrationAccess::bindCurrentLock(gui),"Camera click failed to bind GUI radar lock or changed radar tracking");
#endif
    QFile txt(track);check(txt.open(QIODevice::ReadOnly),"Missing TXT");QByteArray last;
    for(auto line:txt.readAll().split('\n'))if(!line.trimmed().isEmpty())last=line;
    check(last.simplified().split(' ').size()==21,"Legacy TXT column count changed");
    send(7,false);
    auto predicted=metadata(track+".meta.json");
    check(predicted["frame"].toInt()==7 && predicted["predicted"].toBool() &&
        predicted["last_measurement_time_sec"]==first["last_measurement_time_sec"],"Prediction refreshed measurement age");
#if RADAR_ENABLE_CAMERA
    RadarGUIIntegrationAccess::enableUnboundGuard(gui);
    check(metadata(track+".meta.json")["force_stop"].toBool(),"Visual guard did not stop cached command immediately");
    send(8,true);
    const auto protected_meta=metadata(track+".meta.json");
    check(protected_meta["vision_guard"].toObject()["enabled"].toBool() && protected_meta["force_stop"].toBool(),"Visual guard metadata missing");
    QFile protected_txt(track);check(protected_txt.open(QIODevice::ReadOnly),"Missing protected TXT");
    for(auto line:protected_txt.readAll().split('\n'))if(!line.trimmed().isEmpty())last=line;
    const auto fields=last.simplified().split(' ');
    check(fields.size()==21 && fields[16]=="STOPPED" && fields[17]=="0.000" && fields[18]=="0" && fields[19]=="0", "Visual guard did not clear both legacy motion permissions");
    RadarGUIIntegrationAccess::markStopped(gui);
    RadarGUIIntegrationAccess::refreshCamera(gui);
    const auto after_camera_refresh=metadata(track+".meta.json");
    check(after_camera_refresh["force_stop"].toBool()&&after_camera_refresh["motion_reason"].toString()=="test_producer_stop",
          "Asynchronous visual refresh cleared explicit producer stop");
    send(9,true);
    check(!metadata(track+".meta.json")["force_stop"].toBool(),"New radar output could not end producer stop");
#endif
    QMetaObject::invokeMethod(&gui,"on_action_3_triggered",Qt::DirectConnection);
    check(RadarGUIIntegrationAccess::stopped(gui),"Mode switch left capture workers alive");
    check(metadata(track+".meta.json")["force_stop"].toBool(),"Mode switch did not stop control output");
    QDir diagnostics(directory.path()+"/var/diagnostics");
    for(const auto& name:diagnostics.entryList({"*.csv"},QDir::Files)){
        std::ifstream file(diagnostics.filePath(name).toStdString());std::string header,row;std::getline(file,header);
        while(std::getline(file,row))check(columns(row)==columns(header),"Diagnostic CSV header/row mismatch");
    }
    gui.close();
    std::cout<<"PASS: Qt UDP session, manual confirmation, unchanged TXT, metadata age, diagnostics and mode-switch shutdown\n";
}
