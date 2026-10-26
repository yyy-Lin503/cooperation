#include "camera_replay.hpp"
#include "camera_assist_panel.hpp"
#include "frame_context.hpp"
#include "project_paths.hpp"
#include <QPainter>
#include <QMouseEvent>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QFileDialog>
#include <QFileInfo>
#include <QDir>
#include <QSet>
#include <map>
#include <algorithm>
#include <cmath>

using namespace CameraAssist;
namespace {
QString reasonText(const std::string& reason) {
    static const std::map<std::string,QString> names={
        {"disabled","视觉保护未启用"},{"not_bound","请先锁定雷达目标，再点击画面人物绑定"},
        {"no_radar_lock","请先在雷达界面锁定目标"},{"camera_stale","图像尚未就绪或已过期"},
        {"radar_stale","雷达数据尚未就绪或已过期"},{"unsynchronized","图像与雷达时间差过大"},
        {"ambiguous","人物框重叠或编号不唯一"},{"person_missing","所绑定的视觉人物已丢失，请重新点击绑定"},
        {"person_unstable","人物编号尚未稳定，请稍后点击"},{"radar_target_missing","锁定雷达目标当前无有效观测"},
        {"selection_changed","雷达目标已变化，请重新点击绑定"},{"camera_session_changed","摄像头已重启，请重新点击绑定"},
        {"verified","所绑定人物的视觉连续性通过"}};
    const auto it=names.find(reason);return it==names.end()?QString::fromStdString(reason):it->second;
}
}
CameraCanvas::CameraCanvas(QWidget* parent):QWidget(parent){setMinimumSize(300,225);setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);}
QRectF CameraCanvas::imageRect()const {
    if(!packet_||packet_->image.isNull())return {};
    const auto size=packet_->image.size().scaled(this->size(),Qt::KeepAspectRatio);
    return QRectF((width()-size.width())/2.,(height()-size.height())/2.,size.width(),size.height());
}
void CameraCanvas::display(std::shared_ptr<const CameraPacket> packet,std::uint64_t selected,int radar_display_id,std::uint64_t task_epoch){task_epoch_=task_epoch;packet_=std::move(packet);selected_=selected;radar_display_id_=radar_display_id;update();}
void CameraCanvas::paintEvent(QPaintEvent*) {
    QPainter p(this);p.fillRect(rect(),QColor(24,24,24));
    if(!packet_||packet_->image.isNull()){p.setPen(Qt::white);p.drawText(rect(),Qt::AlignCenter,"摄像头未开启");return;}
    const auto target=imageRect();p.drawImage(target,packet_->image);
    const qreal sx=target.width()/packet_->image.width(),sy=target.height()/packet_->image.height();
    QFont font=p.font();font.setPixelSize(15);font.setBold(true);p.setFont(font);
    QVector<QRectF> labelRects;
    for(const auto& person:packet_->frame.people) {
        const bool selected=person.track_id==selected_;
        const QColor color=selected?QColor(255,215,55):(person.stable?QColor(55,230,100):QColor(175,175,175));
        const auto& b=person.box;
        const QRectF box(target.x()+b.x*sx,target.y()+b.y*sy,b.width*sx,b.height*sy);
        p.setPen(QPen(color,selected?3:2));p.drawRect(box);
        QString title=QString("V%1 · %2").arg(static_cast<qulonglong>(person.track_id)).arg(person.confidence,0,'f',2);
        if(selected&&radar_display_id_>0)
            title=task_epoch_?QString("T%1 · V%2 / P%3").arg(static_cast<qulonglong>(task_epoch_)).arg(static_cast<qulonglong>(person.track_id)).arg(radar_display_id_):QString("V%1 / P%2").arg(static_cast<qulonglong>(person.track_id)).arg(radar_display_id_);
        else if(selected)title=QString("V%1 · 待选雷达").arg(static_cast<qulonglong>(person.track_id));
        const qreal w=std::min(target.width(),qreal(p.fontMetrics().horizontalAdvance(title)+12));
        const qreal x=std::max(target.left(),std::min(box.left(),target.right()-w));
        qreal y=std::max(target.top(),box.top()-25);
        for(int attempt=0;attempt<4;++attempt){
            const QRectF proposed(x,y,w,24);
            if(std::none_of(labelRects.begin(),labelRects.end(),[&](const QRectF& old){return old.intersects(proposed);}))break;
            y=std::min(target.bottom()-24,y+25);
        }
        labelRects.push_back(QRectF(x,y,w,24));
        p.fillRect(QRectF(x,y,w,24),QColor(0,0,0,185));p.setPen(color);
        p.drawText(QRectF(x+5,y,w-10,24),Qt::AlignVCenter|Qt::AlignLeft,title);
    }
    if(!replay_&&RadarTime::monotonicSeconds()-packet_->frame.captured_monotonic>.5){p.fillRect(QRect(0,0,width(),30),QColor(150,0,0,200));p.setPen(Qt::white);p.drawText(QRect(0,0,width(),30),Qt::AlignCenter,"画面已过期，请等待新图像");}
}
void CameraCanvas::mousePressEvent(QMouseEvent* event) {
    if(event->button()!=Qt::LeftButton||!packet_||packet_->image.isNull()||RadarTime::monotonicSeconds()-packet_->frame.captured_monotonic>.5)return;
    const auto target=imageRect();if(!target.contains(event->localPos()))return;
    const auto q=event->localPos()-target.topLeft();const cv::Point2f point(q.x()*packet_->image.width()/target.width(),q.y()*packet_->image.height()/target.height());
    const Person* selected=nullptr;int count=0;
    for(const auto& person:packet_->frame.people)if(person.box.contains(point)){selected=&person;++count;}
    if(count==1&&selected->stable&&selected->track_id)emit personClicked(selected->track_id);
}

CameraAssistPanel::CameraAssistPanel(QWidget* parent):QWidget(parent) {
    setObjectName("cameraAssistPanel");
    auto* layout=new QHBoxLayout(this);canvas_=new CameraCanvas(this);canvas_->setObjectName("cameraCanvas");layout->addWidget(canvas_,3);
    controls_widget_=new QWidget(this);controls_widget_->setObjectName("cameraControls");
    auto* controls=new QVBoxLayout(controls_widget_);layout->addWidget(controls_widget_,2);
    auto* devices=new QHBoxLayout();device_=new QComboBox(this);device_->setObjectName("cameraDeviceCombo");devices->addWidget(device_,1);
    device_->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);device_->setMinimumContentsLength(12);
    device_->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Fixed);
    auto* refresh_button=new QPushButton("刷新",this);devices->addWidget(refresh_button);controls->addLayout(devices);
    auto* model_row=new QHBoxLayout();model_=new QLineEdit(this);model_->setObjectName("cameraModelPath");
    const auto configured=qEnvironmentVariable("RADAR_CAMERA_MODEL");
    const auto localModel=QString::fromStdString((RadarPaths::resolve().data/"models/yolo11n.onnx").u8string());
    model_->setText(configured.isEmpty()?(QFileInfo::exists(localModel)?localModel:QDir::home().filePath("ultralytics/ultralytics/yolo11n.onnx")):configured);
    model_->setPlaceholderText("COCO YOLO11 检测 ONNX，输入 640");model_row->addWidget(model_,1);
    model_->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Fixed);
    auto* browse=new QPushButton("模型…",this);model_row->addWidget(browse);controls->addLayout(model_row);
    auto* buttons=new QHBoxLayout();auto* start=new QPushButton("开启摄像头",this);start->setObjectName("cameraStartButton");
    auto* stop=new QPushButton("关闭摄像头",this);stop->setObjectName("cameraStopButton");cuda_=new QCheckBox("使用 GPU",this);cuda_->setChecked(true);
    buttons->addWidget(start);buttons->addWidget(stop);buttons->addWidget(cuda_);controls->addLayout(buttons);
    auto* recordingRow=new QHBoxLayout();record_=new QCheckBox("录制相机供回放",this);record_->setObjectName("cameraRecordCheck");record_->setChecked(true);recordingRow->addWidget(record_);
    auto* replay=new QPushButton("回放记录…",this);replay->setObjectName("cameraReplayButton");recordingRow->addWidget(replay);controls->addLayout(recordingRow);
    replay->setToolTip("回放前停止随行、采集和相机；已有记录保存后再打开离线播放器。");
    connect(replay,&QPushButton::clicked,this,&CameraAssistPanel::replayRequested);
    status_=new QLabel("摄像头未开启",this);status_->setWordWrap(true);status_->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Preferred);status_->setObjectName("cameraStatus");controls->addWidget(status_);
    auto* instructions=new QLabel("先点击相机人物，再点击对应雷达目标并确认锁定；也可先选雷达再点击人物。\n当前未标定，两路对应由操作者核对；相机选中不会自动启动随行。",this);
    instructions->setWordWrap(true);controls->addWidget(instructions);
    binding_status_=new QLabel(this);binding_status_->setObjectName("cameraBindingStatus");binding_status_->setWordWrap(true);status_->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Preferred);controls->addWidget(binding_status_);
    auto* unbind=new QPushButton("解除相机绑定",this);unbind->setObjectName("cameraUnbindButton");controls->addWidget(unbind);
    guard_=new QCheckBox("严格双传感器保护",this);guard_->setObjectName("cameraGuardCheck");
    guard_->setToolTip("默认雷达主导：视觉暂不可用时保留任务，运动由雷达观测控制。勾选后视觉过期或人物不可见立即禁止运动。");controls->addWidget(guard_);controls->addStretch();
    connect(refresh_button,&QPushButton::clicked,this,&CameraAssistPanel::refreshDevices);
    connect(browse,&QPushButton::clicked,this,[this]{const auto path=QFileDialog::getOpenFileName(this,"选择 COCO YOLO11 检测模型",model_->text(),"ONNX (*.onnx)");if(!path.isEmpty())model_->setText(path);});
    connect(start,&QPushButton::clicked,this,&CameraAssistPanel::startCamera);connect(stop,&QPushButton::clicked,this,&CameraAssistPanel::stopCamera);
    connect(unbind,&QPushButton::clicked,this,&CameraAssistPanel::clearBinding);
    connect(guard_,&QCheckBox::toggled,this,[this]{refresh();emit evidenceChanged();});
    connect(canvas_,&CameraCanvas::personClicked,this,&CameraAssistPanel::bindClickedPerson);
    connect(&timer_,&QTimer::timeout,this,&CameraAssistPanel::tick);timer_.start(100);
    refreshDevices();refresh();
}
CameraAssistPanel::~CameraAssistPanel(){timer_.stop();worker_.stop();}
QWidget* CameraAssistPanel::takeControlsWidget() {
    layout()->removeWidget(controls_widget_);
    layout()->setContentsMargins(0,0,0,0);
    return controls_widget_;
}
void CameraAssistPanel::setCompactView(bool compact) {
    canvas_->setMinimumSize(compact ? QSize(200,140) : QSize(300,225));
    updateGeometry();
}
QJsonObject CameraAssistPanel::displayEvidence() const {
    const auto frame=vision();
    const auto evidence=motionDecision(true,binding_,radar_,frame,locked_uid_,locked_epoch_,RadarTime::monotonicSeconds());
    const bool available=shown_&&!shown_->image.isNull()&&frame.session!=0&&
        std::isfinite(frame.captured_monotonic)&&frame.captured_monotonic>0;
    return {{"available",available},{"captured_monotonic",available?frame.captured_monotonic:0.0},
        {"bound",binding_.camera_track!=0&&binding_.camera_session!=0&&binding_.radar_uid!=0},
        {"verified",available&&evidence.permit},{"reason",QString::fromStdString(evidence.reason)},
        {"pending_camera_track",QString::number(pending_camera_track_)},{"camera_track",QString::number(binding_.camera_track)},
        {"strict",guard_->isChecked()}};
}
void CameraAssistPanel::refreshDevices() {
    const auto selected=device_->currentData().toString().isEmpty()?qEnvironmentVariable("RADAR_CAMERA_DEVICE"):device_->currentData().toString();device_->clear();
    QDir ids("/dev/v4l/by-id");QSet<QString> canonical;
    for(const auto& name:ids.entryList({"*video-index0"},QDir::Files|QDir::System)) {
        const auto path=ids.filePath(name);canonical.insert(QFileInfo(path).canonicalFilePath());device_->addItem(name,path);
    }
    QDir devices("/dev");for(const auto& name:devices.entryList({"video*"},QDir::System)) {
        const auto path=devices.filePath(name);if(!canonical.contains(QFileInfo(path).canonicalFilePath()))device_->addItem(path,path);
    }
    const auto configured=qEnvironmentVariable("RADAR_CAMERA_DEVICE");
    for(const auto& path:{configured,selected})if(!path.isEmpty()&&device_->findData(path)<0)
        device_->addItem(path.startsWith("tcp://")?QString("本机摄像头 (%1)").arg(path):path,path);
    const int index=device_->findData(selected);if(index>=0)device_->setCurrentIndex(index);
}
void CameraAssistPanel::startCamera() {
    if(worker_.running()){emit diagnostic("摄像头已经开启。");return;}
    if(device_->currentData().toString().isEmpty()||!QFileInfo::exists(model_->text())){emit diagnostic("请选择摄像头及存在的 YOLO11 ONNX 检测模型。");return;}
    shown_.reset();binding_={};pending_camera_track_=pending_camera_session_=0;last_error_.clear();
    worker_.start({device_->currentData().toString().toStdString(),model_->text().toStdString(),cuda_->isChecked(),640,480,record_->isChecked()});
    emit diagnostic("摄像头正在启动，人物检测在独立线程运行。");refresh();emit evidenceChanged();
}
void CameraAssistPanel::stopCamera() {
    worker_.stop();shown_.reset();binding_={};pending_camera_track_=pending_camera_session_=0;
    canvas_->display({},0);refresh();emit evidenceChanged();
}
void CameraAssistPanel::acceptPacket(std::shared_ptr<const CameraPacket> packet) {
    if(!packet)return;
    if(!packet->error.empty()) {
        if(last_error_!=QString::fromStdString(packet->error)){last_error_=QString::fromStdString(packet->error);emit diagnostic("摄像头失败："+last_error_);}
        shown_.reset();binding_={};return;
    }
    if(shown_&&shown_->frame.session==packet->frame.session&&shown_->frame.sequence>=packet->frame.sequence)return;
    const auto& f=packet->frame;
    if(binding_.camera_track&&f.session!=binding_.camera_session) {
        binding_={};emit diagnostic("相机会话已变化，人工关联已解除。");
    }
    if(pending_camera_session_&&pending_camera_session_!=f.session)
        pending_camera_track_=pending_camera_session_=0;
    // Missing detections do not erase the task. Strict mode still vetoes each
    // frame without current evidence; default radar-primary mode degrades.
    // Display the latest image. Old matching-time frames must never restore a
    // visual ID after a newer image has already reported it lost.
    shown_=std::move(packet);last_error_.clear();
}
void CameraAssistPanel::tick(){
    const auto before=shown_;acceptPacket(worker_.latest());refresh();
    // Freshness is independently checked by the bridge. Avoid rewriting control
    // metadata ten times/second when there is no new image.
    if(before!=shown_)emit evidenceChanged();
}
VisionFrame CameraAssistPanel::vision()const{return shown_?shown_->frame:VisionFrame();}
void CameraAssistPanel::refresh() {
    if(refreshing_)return;refreshing_=true;
    const auto f=vision();
    const auto state=motionDecision(true,binding_,radar_,f,locked_uid_,locked_epoch_,RadarTime::monotonicSeconds());
    canvas_->display(shown_,binding_.camera_track?binding_.camera_track:pending_camera_track_,binding_.camera_track?locked_display_id_:-1,locked_epoch_);
    guard_->setEnabled(guard_->isChecked()||binding_.camera_track!=0);
    QString text=shown_?QString("%1；人物 %2；推理 %3 ms；%4").arg(QString::fromStdString(shown_->status)).arg(f.people.size()).arg(shown_->inference_ms,0,'f',1).arg(reasonText(state.reason)):
        (last_error_.isEmpty()?"摄像头未开启或正在初始化":last_error_);
    text+=guard_->isChecked()?"\n严格双传感器：视觉失效暂停运动。":"\n雷达主导：视觉不可用时降级显示，雷达可靠则续跟。";
    if(record_->isChecked())text+="\n"+worker_.recordingStatus();
    status_->setText(text);
    binding_status_->setText(binding_.camera_track?
        QString("T%1 · V%2 ↔ P%3（人工确认）；%4").arg(static_cast<qulonglong>(locked_epoch_)).arg(static_cast<qulonglong>(binding_.camera_track)).arg(locked_display_id_).arg(state.permit?"视觉当前可见":"视觉暂不可用，保留关联"):
        pending_camera_track_?QString("已选 V%1：请在雷达图或目标列表预选对应目标，再确认锁定。").arg(static_cast<qulonglong>(pending_camera_track_)):
        locked_uid_?QString("已锁定 P%1；点击画面对应人物完成关联。").arg(locked_display_id_):"点击相机人物或雷达候选开始选人；绿框仅表示视觉轨迹稳定。");
    refreshing_=false;
}
void CameraAssistPanel::setRadarFrame(const RadarFrame& radar) {
    if(radar_.session!=radar.session)binding_={};
    radar_=radar;refresh();
}
void CameraAssistPanel::setLockedTarget(std::uint64_t uid,std::uint64_t session,std::uint64_t epoch,int display_id) {
    const bool changed=uid!=locked_uid_||session!=locked_session_||epoch!=locked_epoch_;
    const bool sameTask=uid&&locked_uid_&&session==locked_session_&&epoch==locked_epoch_;
    if(changed&&sameTask&&binding_.radar_uid)binding_.radar_uid=uid;
    else if(changed)binding_={};
    locked_uid_=uid;locked_session_=session;locked_epoch_=epoch;locked_display_id_=display_id;
    if(uid&&pending_camera_track_&&!binding_.camera_track&&pending_camera_session_==vision().session) {
        Binding candidate;std::string reason;
        if(bindPerson(radar_,vision(),uid,epoch,pending_camera_track_,RadarTime::monotonicSeconds(),candidate,reason)) {
            binding_=candidate;pending_camera_track_=pending_camera_session_=0;
            emit diagnostic(QString("人工确认关联：T%1 · P%2 ↔ V%3。").arg(static_cast<qulonglong>(epoch)).arg(display_id).arg(static_cast<qulonglong>(binding_.camera_track)));
        }
    }
    refresh();if(changed)emit evidenceChanged();
}
void CameraAssistPanel::resetRadar(){radar_={};binding_={};locked_uid_=locked_session_=locked_epoch_=0;locked_display_id_=-1;refresh();emit evidenceChanged();}
void CameraAssistPanel::clearBinding(){binding_={};pending_camera_track_=pending_camera_session_=0;refresh();emit evidenceChanged();emit diagnostic("已解除相机绑定；雷达目标锁定保持原状态。");}
void CameraAssistPanel::bindClickedPerson(std::uint64_t id) {
    if(!locked_uid_) {
        pending_camera_track_=id;pending_camera_session_=vision().session;
        refresh();emit evidenceChanged();
        emit diagnostic(QString("已选视觉 V%1：请人工选择并确认对应雷达目标。").arg(static_cast<qulonglong>(id)));return;
    }
    Binding candidate;std::string reason;
    if(locked_session_!=radar_.session||!bindPerson(radar_,vision(),locked_uid_,locked_epoch_,id,RadarTime::monotonicSeconds(),candidate,reason)) {
        if(locked_session_!=radar_.session)reason="no_radar_lock";
        emit diagnostic("相机绑定失败："+reasonText(reason));return;
    }
    binding_=candidate;pending_camera_track_=pending_camera_session_=0;refresh();emit evidenceChanged();
    emit diagnostic(QString("已人工绑定 V%1 ↔ P%2；相机点击不改变雷达锁定目标。").arg(static_cast<qulonglong>(id)).arg(locked_display_id_));
}
MotionDecision CameraAssistPanel::decision(std::uint64_t uid,std::uint64_t epoch)const {
    return motionDecision(guard_->isChecked(),binding_,radar_,vision(),uid,epoch,RadarTime::monotonicSeconds());
}
QJsonObject CameraAssistPanel::metadata(std::uint64_t uid,std::uint64_t epoch)const {
    const auto d=decision(uid,epoch);const auto f=vision();QJsonObject object;
    object["evidence_reason"]=QString::fromStdString(motionDecision(true,binding_,radar_,f,uid,epoch,RadarTime::monotonicSeconds()).reason);
    object["enabled"]=d.enabled;object["permit"]=d.permit;object["reason"]=QString::fromStdString(d.reason);
    object["camera_session"]=QString::number(f.session);object["camera_track"]=QString::number(binding_.camera_track);
    object["binding_mode"]="manual_uncalibrated";object["mode"]=guard_->isChecked()?"strict_dual":"radar_primary";
    object["pending_camera_track"]=QString::number(pending_camera_track_);object["radar_uid"]=QString::number(binding_.radar_uid);
    object["radar_session"]=QString::number(binding_.radar_session);object["selection_epoch"]=QString::number(binding_.selection_epoch);
    object["captured_time_sec"]=f.captured_wall;object["time_quality"]=QString::fromStdString(f.time_quality);
    const double now=RadarTime::monotonicSeconds(),wall=RadarTime::wallSeconds();
    object["expires_time_sec"]=wall+std::min(f.captured_monotonic+.5-now,radar_.captured_monotonic+.35-now);
    return object;
}
