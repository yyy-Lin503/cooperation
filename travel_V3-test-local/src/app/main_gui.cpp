#include <QApplication>
#include <iostream>
#include <QStyleFactory>
#include "radar_gui.hpp"
#include "radar_project_root.hpp"
#include "project_paths.hpp"

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#endif

int main(int argc, char *argv[])
{
#ifdef _WIN32
    WSADATA wsaData;
    WSAStartup(MAKEWORD(2, 2), &wsaData);
#endif

    QApplication a(argc, argv);
    a.setStyle(QStyleFactory::create("Fusion"));

    int result = 1;
    try {
        const auto root=radarProjectRoot(QCoreApplication::applicationFilePath(),QDir::currentPath(),QProcessEnvironment::systemEnvironment());
        if(!QDir::setCurrent(root))throw std::runtime_error("Cannot enter V8 project directory");
        qputenv("RADAR_PROJECT_ROOT",root.toUtf8());qputenv("RADAR_BASELINE_ROOT",root.toUtf8());
        const auto paths = RadarPaths::resolve(std::filesystem::u8path(root.toUtf8().constData()));
        paths.migrateSettings();
        qputenv("RADAR_DATA_ROOT",QByteArray::fromStdString(paths.data.u8string()));
        qputenv("RADAR_TRACK_DIR",QByteArray::fromStdString(paths.track.u8string()));
        RadarGUI w;
        w.show();
        result = a.exec();
    } catch (const std::exception& error) {
        std::cerr << "Radar GUI startup failed: " << error.what() << std::endl;
    }

#ifdef _WIN32
    WSACleanup();
#endif

    return result;
}
