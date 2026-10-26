#include "camera_worker.hpp"
#include "frame_context.hpp"
#include <QBuffer>
#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QTcpServer>
#include <QTcpSocket>
#include <QTemporaryDir>
#include <QtEndian>
#include <future>
#include <iostream>
#include <stdexcept>

using namespace CameraAssist;
static void check(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
enum class Mode { Good, Disconnect, Oversize, BadJpeg, Empty, Stalled };

class Server {
    std::atomic<bool> stopping_{false};
    std::thread thread_;
public:
    quint16 port=0;
    std::atomic<int> requests{0};
    explicit Server(Mode mode) {
        std::promise<quint16> ready;auto result=ready.get_future();
        thread_=std::thread([this,mode,ready=std::move(ready)]()mutable {
            QTcpServer server;
            if(!server.listen(QHostAddress::LocalHost,0)){ready.set_value(0);return;}
            ready.set_value(server.serverPort());
            QImage image(320,240,QImage::Format_RGB888);image.fill(Qt::blue);
            QByteArray jpeg;QBuffer buffer(&jpeg);buffer.open(QIODevice::WriteOnly);image.save(&buffer,"JPEG");
            while(!stopping_) {
                if(!server.waitForNewConnection(100))continue;
                std::unique_ptr<QTcpSocket> socket(server.nextPendingConnection());
                while(!stopping_&&socket->state()==QAbstractSocket::ConnectedState) {
                    if(!socket->bytesAvailable()&&!socket->waitForReadyRead(100))continue;
                    if(socket->read(1)!="F")break;
                    const int count=++requests;
                    if(mode==Mode::Disconnect)break;
                    QByteArray payload=mode==Mode::BadJpeg?QByteArray("bad"):jpeg;
                    const quint32 length=mode==Mode::Empty?0:mode==Mode::Oversize?8*1024*1024+1:payload.size();
                    QByteArray header(4,0);qToBigEndian(length,reinterpret_cast<uchar*>(header.data()));
                    if(mode==Mode::Good&&count==1) {
                        // Cross the worker's 100ms polling boundary twice.
                        socket->write(header.left(2));socket->waitForBytesWritten(100);
                        std::this_thread::sleep_for(std::chrono::milliseconds(150));
                        socket->write(header.mid(2)+payload.left(10));socket->waitForBytesWritten(100);
                        std::this_thread::sleep_for(std::chrono::milliseconds(150));
                        socket->write(payload.mid(10));
                    } else if(mode==Mode::Stalled) {
                        socket->write(header+payload.left(2));socket->waitForBytesWritten(100);
                        while(!stopping_)std::this_thread::sleep_for(std::chrono::milliseconds(10));
                    } else {
                        socket->write(header);
                        if(length&&mode!=Mode::Oversize)socket->write(payload);
                    }
                    socket->waitForBytesWritten(100);
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                }
            }
        });
        port=result.get();check(port!=0,"Test server failed to listen");
    }
    ~Server(){stopping_=true;if(thread_.joinable())thread_.join();}
};

template<class Predicate>static void until(Predicate predicate,double seconds,const char* why) {
    const double deadline=RadarTime::monotonicSeconds()+seconds;
    while(!predicate()) {
        check(RadarTime::monotonicSeconds()<deadline,why);
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}
int main(int argc,char**argv) {
    QCoreApplication app(argc,argv);QTemporaryDir temp;check(temp.isValid(),"No temporary directory");
    // Compressed synthetic ONNX: GAP -> zero-weight 1x1 Conv with a constant
    // person bias -> [1,84,1]. Exercises the real worker without downloading
    // weights; it is only a protocol/tracking fixture, never an accuracy model.
    const auto model=qUncompress(QByteArray::fromBase64(
        "AAAGdnic42C3es3DpcTFmplXUFoixFaQn5+TmqIk6J6Tn5SY41iWWpSYnhoAFOSy4YJKcrGXp2amZ5QUc7EkZSYWC7GVJeaUphYrsTjn55VpiXLxZKcW5aXmxBdnJBakOjA6MC5gZOdS4IIq42IFiwux5ZeWAG1UYg9KhQiIJOfnFZck5pXoFqQWFefn6ZakFpdotXFwhHAwczByMAowOsEs9vrAzjAKRsEoGDFA6y4TRwiwCAAXOV4XmBgYFjhD8AcnBoYC57S0NPuBduNQBlqqHMwC7E6QwtlLghEqHAKlYfwoRVhNIcElxsEoJMTFxMEIxMxczBwNrGAiSYYLWrYDZQWAanigakJAtBMLF4MANwCSWU54"));
    QFile file(temp.filePath("fixture.onnx"));check(file.open(QIODevice::WriteOnly),"Cannot write model fixture");
    check(file.write(model)==model.size(),"Cannot write model bytes");file.close();
    CameraWorker worker;
    const auto settings=[&](quint16 port){CameraSettings s;s.device="tcp://127.0.0.1:"+std::to_string(port);s.model=file.fileName().toStdString();s.cuda=false;s.record=false;return s;};
    for(int restart=0;restart<2;++restart) {
        Server server(Mode::Good);worker.start(settings(server.port));
        until([&]{auto p=worker.latest();return p&&(!p->error.empty()||p->frame.sequence>=3);},5,"Fresh bridge frames did not arrive");
        auto packet=worker.latest();check(packet->error.empty(),packet->error.c_str());
        check(packet->image.size()==QSize(320,240)&&packet->frame.width==320&&packet->frame.height==240,"Actual bridge dimensions lost");
        check(packet->frame.people.size()==1&&packet->frame.people[0].stable,"Existing person tracker did not stabilize");
        check(packet->frame.time_quality=="receive_estimate"&&RadarTime::monotonicSeconds()-packet->frame.captured_monotonic<.5,"Bridge timestamp is not a fresh receive estimate");
        worker.stop();check(!worker.latest(),"Stop retained bridge frame");
    }
    for(auto mode:{Mode::Disconnect,Mode::Oversize,Mode::BadJpeg,Mode::Empty}) {
        Server server(mode);worker.start(settings(server.port));
        until([&]{return !worker.running();},5,"Invalid bridge did not fail promptly");
        auto packet=worker.latest();check(packet&&!packet->error.empty(),"Bridge failure not reported");
        worker.stop();
    }
    {
        Server server(Mode::Stalled);worker.start(settings(server.port));
        until([&]{return server.requests.load()>0;},3,"No request before stop test");
        const double start=RadarTime::monotonicSeconds();worker.stop();
        check(RadarTime::monotonicSeconds()-start<.7,"Partial JPEG blocked cooperative stop");
    }
    check(QDir(temp.path()).entryList(QDir::Dirs|QDir::NoDotAndDotDot).isEmpty(),"Bridge test unexpectedly recorded images");
    std::cout<<"PASS: fragmented JPEG, real worker/tracker, timestamps, restart, invalid payload, disconnect, timeout and stop\n";
}
