#include "radar_gui.hpp"
#include <QApplication>
#include <QTemporaryDir>
#include <QUdpSocket>
#include <QDir>
#include <QTabWidget>
#include <cmath>
#include <iostream>
static int autoConnectWarnings = 0;
static QtMessageHandler previousHandler = nullptr;
static void checkQtMessage(QtMsgType type, const QMessageLogContext& context, const QString& message) {
    if (message.contains("QMetaObject::connectSlotsByName: No matching signal")) ++autoConnectWarnings;
    if (previousHandler) previousHandler(type, context, message);
    else std::cerr << message.toStdString() << '\n';
}
int main(int argc,char** argv) {
    QApplication app(argc,argv);
    previousHandler = qInstallMessageHandler(checkQtMessage);
    QTemporaryDir directory;
    if (!directory.isValid() || !QDir::setCurrent(directory.path())) return 1;
    QUdpSocket probe; probe.bind(QHostAddress(QHostAddress::LocalHost), static_cast<quint16>(0));
    const auto port=probe.localPort();probe.close();
    qputenv("RADAR_RESULT_PORT",QByteArray::number(port));
    for(int i=0;i<3;++i) {
        RadarGUI window;
        window.show();app.processEvents();
        auto* view = window.findChild<QGraphicsView*>("graphicsView");
        auto* zoomIn = window.findChild<QPushButton*>("zoomInButton");
        auto* zoomOut = window.findChild<QPushButton*>("zoomOutButton");
        auto* exit = window.findChild<QPushButton*>("pushButton");
        if (!view || !zoomIn || !zoomOut || !exit) return 3;
        // Check real button signals, including a single zoom step per click.
        zoomOut->click();
        const double before = view->transform().m11();
        zoomIn->click();
        if (std::abs(view->transform().m11() / before - 1.2) > 1e-6) return 4;
        zoomOut->click();
        if (std::abs(view->transform().m11() / before - 1.0) > 1e-6) return 5;
        if(i==0&&app.arguments().size()==3&&app.arguments()[1]=="--screenshot"){
            window.resize(1024,720);
            if(auto* tabs=window.findChild<QTabWidget*>("workspaceControlTabs"))
                for(int index=0;index<tabs->count();++index)if(tabs->tabText(index)=="ROS/底盘")tabs->setCurrentIndex(index);
            app.processEvents();if(!window.grab().save(app.arguments()[2]))return 2;
        }
        if (i==1) {
            auto* resultSocket = window.findChild<QUdpSocket*>();
            if (!resultSocket || resultSocket->state() != QAbstractSocket::BoundState) return 6;
            exit->click();
            // quit() ends the normal application event loop; shutdown happens synchronously.
            if (resultSocket->state() != QAbstractSocket::UnconnectedState) return 6;
        }
        window.close();app.processEvents();
    }
    qInstallMessageHandler(previousHandler);
    if (autoConnectWarnings) return 7;
    std::cout<<"PASS: headless GUI create/close/explicit exit/repeated cleanup without device commands\n";
}
