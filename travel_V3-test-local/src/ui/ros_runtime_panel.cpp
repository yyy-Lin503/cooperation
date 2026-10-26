#include "project_paths.hpp"
#include "ros_runtime_panel.hpp"
#include <QVBoxLayout>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QFileDialog>
#include <QFileInfo>
#include <QDir>
#include <QSettings>
#include <QSaveFile>
#include <QJsonDocument>
#include <QJsonParseError>
#include <QProcessEnvironment>
#include <QUuid>
#include <QEventLoop>
#include <cmath>

namespace {
QString phaseText(const QString& phase) {
    if(phase=="discovering")return "正在检查 ROS 环境和已有节点…";
    if(phase=="starting_chassis")return "正在启动/连接底盘，等待里程计…";
    if(phase=="starting_odom")return "正在启动/连接本项目里程计接收器…";
    if(phase=="starting_follow")return "正在启动随行控制器…";
    if(phase=="following")return "随行控制器已运行";
    if(phase=="stopping")return "正在停车并清理所启动的进程…";
    if(phase=="error")return "启动或运行失败";
    return "追踪已停止；可保留底盘、里程计和采集";
}
QString reasonText(const QString& reason) {
    if(reason=="waiting_for_capture")return "等待开始雷达采集（停车）";
    if(reason=="waiting_for_target"||reason=="no_tracking_file"||reason=="waiting_for_record")return "等待选择并确认雷达目标（停车）";
    if(reason=="gui_paused")return "等待 GUI 允许当前采集会话（停车）";
    if(reason=="tracking_command")return "追踪运行：按雷达及视觉运动许可输出速度";
    if(reason=="tracker_stopped")return "目标追踪状态要求停车";
    if(reason.contains("Visual")||reason.contains("vision_"))return "视觉保护禁止运动："+reason;
    if(reason.contains("stale")||reason.contains("expired"))return "数据过期，保持停车："+reason;
    return "停车原因："+reason;
}
QString componentText(const QString& state) {
    return state=="owned"?"GUI 管理":state=="external"?"外部管理（复用）":"未启动";
}
}

RosRuntimePanel::RosRuntimePanel(const QString& projectRoot,QWidget* parent)
    :QWidget(parent),root_(QDir(projectRoot).absolutePath()),configPath_(QString::fromStdString(RadarPaths::resolve(std::filesystem::u8path(root_.toUtf8().constData())).settings().u8string())) {
    setObjectName("rosRuntimePanel");
    setMinimumWidth(0);setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Preferred);
    auto* layout=new QVBoxLayout(this);
    auto* instructions=new QLabel("“开始追踪”自动启动底盘、里程计接收和随行控制。未选目标时停车等待。\n“停止追踪”保留采集与相机；“关闭底盘与 ROS”只关闭本界面启动的进程。",this);
    instructions->setWordWrap(true);instructions->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Preferred);layout->addWidget(instructions);
    auto* form=new QFormLayout();form->setFieldGrowthPolicy(QFormLayout::AllNonFixedFieldsGrow);
    form->setRowWrapPolicy(QFormLayout::WrapAllRows);
    RadarPaths::resolve(std::filesystem::u8path(root_.toUtf8().constData())).migrateSettings();
    QSettings stored(configPath_,QSettings::IniFormat);stored.setIniCodec("UTF-8");
    auto addText=[&](const QString& key,const QString& title,const QString& fallback){
        auto* edit=new QLineEdit(stored.value(key,fallback).toString(),this);
        edit->setObjectName(key);edit->setMinimumWidth(0);edit->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);
        settings_.append(edit);form->addRow(title,edit);return edit;
    };
    setup_=addText("chassis_setup","底盘 setup",qEnvironmentVariable("WHEELTEC_SETUP",QDir::home().filePath("xiaofang/install/setup.bash")));
    auto* browse=new QPushButton("选择底盘 setup.bash…",this);browse->setObjectName("rosBrowseSetup");
    settings_.append(browse);form->addRow(QString(),browse);
    connect(browse,&QPushButton::clicked,this,[this]{const auto path=QFileDialog::getOpenFileName(this,"选择底盘工作空间 setup.bash",setup_->text(),"Bash (*.bash)");if(!path.isEmpty())setup_->setText(path);});
    package_=addText("chassis_package","底盘包","turn_on_wheeltec_robot");
    launch_=addText("chassis_launch","启动文件","turn_on_wheeltec_robot.launch.py");
    node_=addText("chassis_node","底盘节点名","wheeltec_robot");
    odom_=addText("odom_topic","里程计话题","/odom");
    cmd_=addText("cmd_vel_topic","速度指令话题","/cmd_vel");
    auto addSpeed=[&](const QString& key,const QString& title,double fallback){
        auto* spin=new QDoubleSpinBox(this);spin->setObjectName(key);spin->setRange(0,5);spin->setDecimals(2);spin->setSingleStep(.05);
        spin->setValue(stored.value(key,fallback).toDouble());settings_.append(spin);form->addRow(title,spin);return spin;
    };
    linear_=addSpeed("max_linear_speed_mps","最大线速度 m/s",.15);
    angular_=addSpeed("max_angular_speed_radps","最大角速度 rad/s",.25);
    auto* distance=new QLabel("2.0 m（保持原控制参数）",this);distance->setWordWrap(true);
    form->addRow("期望距离",distance);layout->addLayout(form);
    auto* save=new QPushButton("保存 ROS 设置",this);save->setObjectName("rosSaveSettings");settings_.append(save);layout->addWidget(save);
    connect(save,&QPushButton::clicked,this,&RosRuntimePanel::saveConfiguration);
    status_=new QLabel("ROS 随行尚未启动",this);status_->setObjectName("rosRuntimeStatus");status_->setWordWrap(true);layout->addWidget(status_);
    components_=new QLabel(this);components_->setObjectName("rosComponentsStatus");components_->setWordWrap(true);layout->addWidget(components_);
    odomStatus_=new QLabel(this);odomStatus_->setObjectName("rosOdomStatus");odomStatus_->setWordWrap(true);layout->addWidget(odomStatus_);
    shutdown_=new QPushButton("关闭底盘与 ROS",this);shutdown_->setObjectName("rosShutdownButton");layout->addWidget(shutdown_);layout->addStretch();
    for(auto* widget:{static_cast<QWidget*>(status_),static_cast<QWidget*>(components_),static_cast<QWidget*>(odomStatus_),static_cast<QWidget*>(shutdown_)})layout->removeWidget(widget);
    layout->insertWidget(1,status_);layout->insertWidget(2,components_);layout->insertWidget(3,odomStatus_);layout->insertWidget(4,shutdown_);
    connect(shutdown_,&QPushButton::clicked,this,&RosRuntimePanel::shutdownRos);
    connect(&process_,&QProcess::readyReadStandardOutput,this,&RosRuntimePanel::readEvents);
    connect(&process_,&QProcess::readyReadStandardError,this,[this]{const auto text=QString::fromUtf8(process_.readAllStandardError()).trimmed();if(!text.isEmpty())emit diagnostic("[ROS 环境] "+text);});
    connect(&process_,QOverload<int,QProcess::ExitStatus>::of(&QProcess::finished),this,&RosRuntimePanel::finished);
    connect(&process_,&QProcess::errorOccurred,this,[this](QProcess::ProcessError error){
        emit diagnostic("ROS 运行管理进程："+process_.errorString());
        if(error==QProcess::FailedToStart){clearDisplayState();startupTimeout_.stop();heartbeat_.stop();ready_=false;followRequested_=false;stopping_=false;lockSettings(false);setStatus("ROS 启动失败："+process_.errorString());updateControls();}
    });
    heartbeat_.setInterval(400);connect(&heartbeat_,&QTimer::timeout,this,[this]{send({{"action","heartbeat"}});});
    startupTimeout_.setSingleShot(true);startupTimeout_.setInterval(15000);
    connect(&startupTimeout_,&QTimer::timeout,this,[this]{emit diagnostic("ROS 环境准备超时，正在停止启动进程。");const auto attempt=folder_;process_.terminate();QTimer::singleShot(2000,this,[this,attempt]{if(folder_==attempt&&!ready_&&process_.state()!=QProcess::NotRunning)process_.kill();});});
    connect(&cleanup_,QOverload<int,QProcess::ExitStatus>::of(&QProcess::finished),this,[this](int code,QProcess::ExitStatus exit){
        const auto output=QString::fromUtf8(cleanup_.readAllStandardError());
        cleanupFailed_=code!=0||exit!=QProcess::NormalExit;
        if(cleanupFailed_){setStatus("进程清理失败，请检查日志或点击关闭底盘与 ROS 重试");emit diagnostic("所属进程清理失败："+output);}
        else {emit diagnostic("ROS 所属进程清理完成。");setStatus(exitFailed_?"ROS 启动或运行失败，所属进程已清理；详见日志，可重新启动。":"底盘与 ROS 已关闭；外部管理的节点保持原状态。");components_->setText("本界面启动的 ROS 进程已退出；外部进程不变。");odomStatus_->setText("里程计状态监测已停止");}
        lockSettings(cleanupFailed_);updateControls();
    });
    connect(&cleanup_,&QProcess::errorOccurred,this,[this](QProcess::ProcessError error){if(error==QProcess::FailedToStart){cleanupFailed_=true;setStatus("无法启动所属进程清理，请检查日志");emit diagnostic(cleanup_.errorString());updateControls();}});
    updateControls();
}

RosRuntimePanel::~RosRuntimePanel(){shutdownBlocking();}
void RosRuntimePanel::setStatus(const QString& text){status_->setText(text);emit statusChanged(text);}
bool RosRuntimePanel::active()const{return process_.state()!=QProcess::NotRunning||cleanup_.state()!=QProcess::NotRunning;}
void RosRuntimePanel::clearDisplayState(){displayEvent_={};displayEventAge_.invalidate();}
QJsonObject RosRuntimePanel::displayState() const {
    QJsonObject result=displayEvent_;
    const bool running=process_.state()==QProcess::Running;
    QString phase=result["phase"].toString("unknown");
    // A queued helper event cannot undo a local stop or process-exit display.
    if(phase=="following"&&(!running||!ready_||!followRequested_||stopping_||closing_))phase="unknown";
    result["phase"]=phase;
    result["age_seconds"]=displayEventAge_.isValid()?displayEventAge_.elapsed()/1000.0:-1.0;
    result["process_running"]=running;result["ready"]=ready_;
    result["follow_requested"]=followRequested_;result["stopping"]=stopping_;
    result["closing"]=closing_;
    return result;
}
void RosRuntimePanel::lockSettings(bool locked){for(auto* widget:settings_)widget->setEnabled(!locked);}
QJsonObject RosRuntimePanel::configuration()const{
    QString setup=setup_->text().trimmed();if(setup.startsWith("~/"))setup=QDir::home().filePath(setup.mid(2));
    return {{"chassis_setup",setup},{"chassis_package",package_->text().trimmed()},
        {"chassis_launch",launch_->text().trimmed()},{"chassis_node",node_->text().trimmed()},
        {"odom_topic",odom_->text().trimmed()},{"cmd_vel_topic",cmd_->text().trimmed()},
        {"max_linear_speed_mps",linear_->value()},{"max_angular_speed_radps",angular_->value()}};
}
void RosRuntimePanel::saveConfiguration(){
    QSettings stored(configPath_,QSettings::IniFormat);stored.setIniCodec("UTF-8");
    const auto config=configuration();for(auto it=config.begin();it!=config.end();++it)stored.setValue(it.key(),it.value().toVariant());
    stored.sync();if(stored.status()!=QSettings::NoError)emit diagnostic("无法保存 GUI ROS 设置："+configPath_);
    else emit diagnostic("ROS 设置已保存，下一次启动整套 ROS 时生效。");
}
void RosRuntimePanel::updateControls(){
    const bool cleaning=cleanup_.state()!=QProcess::NotRunning;
    emit controlsChanged(!closing_&&!cleaning&&!cleanupFailed_&&!stopping_&&!followRequested_,!closing_&&!cleaning&&followRequested_);
    shutdown_->setEnabled((active()||cleanupFailed_)&&!closing_&&!cleaning);
}
void RosRuntimePanel::send(QJsonObject message){
    if(process_.state()==QProcess::Running)process_.write(QJsonDocument(message).toJson(QJsonDocument::Compact)+'\n');
}
void RosRuntimePanel::setCaptureSession(quint64 session){session_=QString::number(session);if(ready_)send({{"action","session"},{"session_id",session_}});}
void RosRuntimePanel::startFollow(){
    if(followRequested_||closing_||stopping_||cleanupFailed_||cleanup_.state()!=QProcess::NotRunning)return;
    if(process_.state()!=QProcess::NotRunning){
        clearDisplayState();followRequested_=true;if(ready_)send({{"action","start"}});updateControls();
        return;
    }
    const auto config=configuration();
    if(!QFileInfo::exists(config["chassis_setup"].toString())){setStatus("底盘 setup 文件不存在，请在 ROS/底盘 页选择正确路径。");emit diagnostic(status_->text());return;}
    const QString launcher=QDir(root_).filePath("scripts/runtime/run_gui_runtime.sh");
    if(!QFileInfo::exists(launcher)){setStatus("未找到 GUI ROS 启动脚本："+launcher);emit diagnostic(status_->text());return;}
    saveConfiguration();folder_=QDir(QString::fromStdString(RadarPaths::resolve(std::filesystem::u8path(root_.toUtf8().constData())).track.u8string())).filePath("gui_runtime/"+QUuid::createUuid().toString(QUuid::WithoutBraces));
    if(!QDir().mkpath(folder_)){setStatus("无法创建 ROS 运行记录目录");return;}
    auto environment=QProcessEnvironment::systemEnvironment();
    environment.insert("WHEELTEC_SETUP",config["chassis_setup"].toString());
    environment.insert("RADAR_PROJECT_ROOT",root_);environment.insert("RADAR_BASELINE_ROOT",root_);
    const auto paths = RadarPaths::resolve(std::filesystem::u8path(root_.toUtf8().constData()));
    environment.insert("RADAR_DATA_ROOT",QString::fromStdString(paths.data.u8string()));
    environment.insert("RADAR_TRACK_DIR",QString::fromStdString(paths.track.u8string()));
    if(!environment.contains("RADAR_ODOM_SHM_KEY"))environment.insert("RADAR_ODOM_SHM_KEY","123458");
    process_.setProcessEnvironment(environment);process_.setWorkingDirectory(root_);
    process_.setProgram("bash");process_.setArguments({launcher,"--runtime-dir",folder_,"--settings",QString::fromUtf8(QJsonDocument(config).toJson(QJsonDocument::Compact))});
    clearDisplayState();ready_=false;followRequested_=true;closing_=false;expectedExit_=false;stopping_=false;exitFailed_=false;pending_.clear();
    lockSettings(true);setStatus("正在加载本项目 ROS 环境…");process_.start();startupTimeout_.start();heartbeat_.start();updateControls();
}
void RosRuntimePanel::stopFollow(){
    if(!active()&&!followRequested_)return;
    clearDisplayState();
    followRequested_=false;
    stopping_=ready_;
    // Revoke locally as well, even while the helper is busy starting a node.
    if(!folder_.isEmpty()){QSaveFile file(QDir(folder_).filePath("context.json"));if(file.open(QIODevice::WriteOnly)){file.write("{\"permit\":false}");file.commit();}}
    if(ready_)send({{"action","stop_follow"}});
    setStatus("正在停止追踪；采集和相机保持运行");updateControls();
}
void RosRuntimePanel::shutdownRos(){
    if(cleanupFailed_&&!active()){emergencyCleanup();return;}
    if(!active())return;
    stopFollow();closing_=true;expectedExit_=true;updateControls();
    if(ready_)send({{"action","shutdown"}});
    else {const auto attempt=folder_;process_.terminate();QTimer::singleShot(2000,this,[this,attempt]{if(folder_==attempt&&!ready_&&process_.state()!=QProcess::NotRunning)process_.kill();});}
}
void RosRuntimePanel::handleEvent(const QJsonObject& event){
    const auto type=event["type"].toString();
    if(type=="ready"){
        ready_=true;startupTimeout_.stop();send({{"action","session"},{"session_id",session_}});
        if(closing_)send({{"action","shutdown"}});else if(followRequested_)send({{"action","start"}});
    }else if(type=="log")emit diagnostic("[ROS/"+event["component"].toString()+"] "+event["message"].toString());
    else if(type=="state"){
        displayEvent_=event;displayEventAge_.restart();
        const auto phase=event["phase"].toString();QString text=phaseText(phase);
        if(phase=="stopping")stopping_=true;
        if(phase=="idle"||phase=="error")stopping_=false;
        if(phase=="error"){closing_=true;exitFailed_=true;}
        const auto control=event["control"].toObject();if(phase=="following"&&!control.isEmpty())text=reasonText(control["reason"].toString());
        if(!event["message"].toString().isEmpty())text+="\n"+event["message"].toString();
        setStatus(text);
        const auto components=event["components"].toObject();
        components_->setText(QString("底盘：%1\n里程计接收：%2\n随行控制：%3").arg(componentText(components["chassis"].toString())).arg(componentText(components["odom"].toString())).arg(componentText(components["follow"].toString())));
        odomStatus_->setText(event["odom_fresh"].toBool()?"里程计：新鲜":"里程计：未就绪或暂时过期（运行期间仅告警）");
        if(phase=="idle"||phase=="error")followRequested_=false;
        if(phase=="error")emit diagnostic("ROS 运行失败："+event["message"].toString());
        updateControls();
    }
}
void RosRuntimePanel::readEvents(){
    pending_+=process_.readAllStandardOutput();
    while(true){const int end=pending_.indexOf('\n');if(end<0)break;const auto line=pending_.left(end);pending_.remove(0,end+1);
        QJsonParseError error;const auto doc=QJsonDocument::fromJson(line,&error);
        if(error.error==QJsonParseError::NoError&&doc.isObject())handleEvent(doc.object());
        else if(!line.trimmed().isEmpty())emit diagnostic("[ROS 输出] "+QString::fromUtf8(line));
    }
    if(pending_.size()>1024*1024){pending_.clear();emit diagnostic("ROS 输出行过长，已丢弃。");}
}
void RosRuntimePanel::finished(int code,QProcess::ExitStatus exit){
    readEvents();heartbeat_.stop();startupTimeout_.stop();ready_=false;followRequested_=false;stopping_=false;
    clearDisplayState();
    exitFailed_=exitFailed_||!expectedExit_||code!=0||exit!=QProcess::NormalExit;
    if(exitFailed_){setStatus("ROS 运行管理进程退出，正在核对并清理所属进程");emit diagnostic(QString("ROS 运行管理进程退出：code=%1").arg(code));}
    emergencyCleanup();closing_=false;
    if(cleanup_.state()==QProcess::NotRunning){lockSettings(cleanupFailed_);setStatus(exitFailed_?QString("ROS 启动或运行失败（退出码 %1），详见日志。").arg(code):"底盘与 ROS 已关闭；外部管理的节点保持原状态。");components_->setText("本界面启动的 ROS 进程已退出；外部进程不变。");odomStatus_->setText("里程计状态监测已停止");}updateControls();
}
void RosRuntimePanel::emergencyCleanup(){
    if(folder_.isEmpty())return;
    QSaveFile context(QDir(folder_).filePath("context.json"));if(context.open(QIODevice::WriteOnly)){context.write("{\"permit\":false}");context.commit();}
    const auto registry=QDir(folder_).filePath("owned.json");if(!QFileInfo::exists(registry))return;
    cleanup_.setProgram("/usr/bin/python3");cleanup_.setArguments({"-B",QDir(root_).filePath("ros2_ws/src/my_radar_follower/my_radar_follower/gui_processes.py"),"--cleanup-registry",registry});
    cleanup_.setWorkingDirectory(root_);cleanup_.start();
}
bool RosRuntimePanel::shutdownBlocking(int timeoutMs){
    if(!active())return !cleanupFailed_;
    shutdownRos();QEventLoop loop;QTimer timer;timer.setSingleShot(true);
    connect(&timer,&QTimer::timeout,&loop,&QEventLoop::quit);
    QTimer poll;poll.setInterval(20);connect(&poll,&QTimer::timeout,&loop,[this,&loop]{if(!active())loop.quit();});
    timer.start(timeoutMs);poll.start();loop.exec();
    if(process_.state()!=QProcess::NotRunning){process_.kill();process_.waitForFinished(1000);}
    if(cleanup_.state()!=QProcess::NotRunning){
        QEventLoop cleanupLoop;QTimer limit;limit.setSingleShot(true);
        connect(&cleanup_,QOverload<int,QProcess::ExitStatus>::of(&QProcess::finished),&cleanupLoop,&QEventLoop::quit);
        connect(&limit,&QTimer::timeout,&cleanupLoop,&QEventLoop::quit);limit.start(12000);cleanupLoop.exec();
    }
    return !active()&&!cleanupFailed_;
}
