#ifndef RADAR_PROJECT_ROOT_HPP
#define RADAR_PROJECT_ROOT_HPP
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QProcessEnvironment>
#include <stdexcept>

inline QString radarProjectRoot(const QString& executable,const QString& current,
                                const QProcessEnvironment& environment) {
    auto valid=[](const QString& path){QFile version(QDir(path).filePath("VERSION.txt"));return version.open(QIODevice::ReadOnly)&&version.readAll().trimmed()=="V8"&&QFileInfo::exists(QDir(path).filePath("scripts/runtime/run_gui_runtime.sh"));};
    QString result;QDir directory(QFileInfo(executable).absolutePath());
    for(int depth=0;depth<5;++depth){if(valid(directory.absolutePath())){result=directory.canonicalPath();break;}if(!directory.cdUp())break;}
    if(result.isEmpty()&&valid(current))result=QDir(current).canonicalPath();
    if(result.isEmpty())throw std::runtime_error("Cannot find this V8 project root next to the executable or working directory");
    for(const auto& name:{QString("RADAR_PROJECT_ROOT"),QString("RADAR_BASELINE_ROOT")}){
        const auto configured=environment.value(name);
        if(!configured.isEmpty()&&QDir(configured).canonicalPath()!=result)
            throw std::runtime_error((name+" points to another project: "+configured).toStdString());
    }
    return result;
}
#endif
