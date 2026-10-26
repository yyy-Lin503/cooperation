#if RADAR_ENABLE_CAMERA
#include "camera_replay.hpp"
#endif
#include <QMouseEvent>
#include "radar_gui.hpp"
#include "ui_radar_gui_v2.h"
#include <QApplication>
#include <QMessageBox>
#include <QFile>
#include <QTextStream>
#include <QDir>
#include <QStandardPaths>
#include <QDebug>
#include <QDateTime>
#include <QStringList>
#include <QPainter>
#include <QMenuBar>
#include <QMenu>
#include <QStatusBar>
#include <iostream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <complex>
#include <fstream>
#include <thread>
#include <chrono>
#include <limits>   
#include <QFileInfo>
#include <QByteArray>
#include <fcntl.h>  
#include <streambuf>
#include <QDoubleSpinBox>
#include <QTableWidgetItem>
#include <QHeaderView>
#include <QScrollBar>
#include <QAbstractItemView>
#include <QSignalBlocker>
#include <QGraphicsRectItem>
#include <QScreen>
#include <QLayout>
#include "CFAR.hpp"
#include "chassis_velocity.hpp"
#include "radar_mount.hpp"
#include <QRegularExpression>
#include <cstdlib>  
#include <QThread>  
#include <QProcess>
#include <QSaveFile>
#include <QToolBar>
#include <QSplitter>
#include <QScrollArea>
#include <QTabWidget>
#include <QJsonDocument>
#include <QJsonObject>
#include "radar_motion.hpp"
#include "radar_frame_layout.hpp"
#include "gui_icons.hpp"

namespace {
static ChassisVelocityReader g_gui_chassis_velocity_reader;

QString autoSelectRuleName(TargetTracking::AutoSelectRule rule)
{
    switch (rule) {
    case TargetTracking::AutoSelectRule::Nearest:
        return "距离最近";
    case TargetTracking::AutoSelectRule::Fastest:
        return "速度最快";
    case TargetTracking::AutoSelectRule::MovingFirst:
        return "运动点优先";
    }
    return "未知策略";
}

QString followControlStateName(TargetTracking::FollowControlState state)
{
    switch (state) {
    case TargetTracking::FollowControlState::NormalFollow:
        return "正常跟随";
    case TargetTracking::FollowControlState::LimitedFollow:
        return "限速跟随";
    case TargetTracking::FollowControlState::YawOnly:
        return "低速方向微调";
    case TargetTracking::FollowControlState::StopAndReacquire:
        return "停车等待重捕获";
    case TargetTracking::FollowControlState::ReacquireConfirming:
        return "锁定/重捕获连续确认中";
    }
    return "未知";
}

QString followControlStateToken(TargetTracking::FollowControlState state)
{
    switch (state) {
    case TargetTracking::FollowControlState::NormalFollow: return "TRACKING";
    case TargetTracking::FollowControlState::LimitedFollow: return "PREDICTED";
    case TargetTracking::FollowControlState::YawOnly: return "STOPPED";
    case TargetTracking::FollowControlState::StopAndReacquire: return "STOPPED";
    // The bridge also checks the explicit motion-permission fields.
    case TargetTracking::FollowControlState::ReacquireConfirming: return "PREDICTED";
    }
    return "STOPPED";
}

const char* candidatePositionModeToken(
    TargetTracking::CandidatePositionMode mode)
{
    switch (mode) {
    case TargetTracking::CandidatePositionMode::WindowCentroid:
        return "window";
    case TargetTracking::CandidatePositionMode::CurrentFrameCentroid:
        return "current";
    case TargetTracking::CandidatePositionMode::TemporalWeightedCentroid:
        return "weighted";
    case TargetTracking::CandidatePositionMode::NoCurrentFrameSupport:
        return "no_current";
    }
    return "unknown";
}
}
class QtLogTeeBuf : public std::streambuf {
public:
    QtLogTeeBuf(std::streambuf* original, RadarGUI* gui) : original_(original), gui_(gui) {}
    ~QtLogTeeBuf() override { sync(); }
protected:
    int_type overflow(int_type ch) override {
        if (ch == traits_type::eof()) return traits_type::not_eof(ch);
        // 转发到原控制台
        if (original_) original_->sputc(static_cast<char>(ch));
        // 累积一行并实时推送
        char c = static_cast<char>(ch);
        line_.push_back(c);
        if (c == '\n') {
            flushLine();
        }
        return ch;
    }
    int sync() override {
        if (original_) original_->pubsync();
        if (!line_.empty()) flushLine();
        return 0;
    }
private:
    void flushLine() {
        // 去掉末尾换行，避免重复空行
        std::string out = line_;
        while (!out.empty() && (out.back() == '\n' || out.back() == '\r')) out.pop_back();
        if (!out.empty()) {
            // 在 GUI 线程中调用（configure_radar 在主线程里执行）
            gui_->appendLog(QString::fromStdString(out));
        }
        line_.clear();
    }
    std::streambuf* original_;
    RadarGUI* gui_;
    std::string line_;
};

std::chrono::system_clock::time_point capture_start_time;

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "ws2_32.lib")
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <poll.h>
#include <dirent.h>
#include <cstring>
#define closesocket close
#define INVALID_SOCKET -1
#define SOCKET int
#endif

RadarGUI::RadarGUI(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::RadarGUIRedesignV2)
{
    (void)RadarMount::xOffsetM(); (void)RadarMount::yOffsetM(); (void)RadarMount::yawSign();
    ui->setupUi(this);
    rosRuntime_=new RosRuntimePanel(QDir::currentPath(),this);
    connect(rosRuntime_,&RosRuntimePanel::diagnostic,this,&RadarGUI::appendLog);
    connect(rosRuntime_,&RosRuntimePanel::controlsChanged,this,[this](bool canStart,bool canStop){
        if(ui&&ui->pushButton_6)ui->pushButton_6->setEnabled(canStart);
        if(stopFollowButton_)stopFollowButton_->setEnabled(canStop);
    });
#if RADAR_ENABLE_CAMERA
    cameraAssist_=new CameraAssistPanel(this);
    connect(cameraAssist_,&CameraAssistPanel::diagnostic,this,&RadarGUI::appendLog);
    connect(cameraAssist_,&CameraAssistPanel::evidenceChanged,this,[this]{
        if(!shutdown_)writeTrackingMetadata(currentFrameContext.frame_number);
    });
#endif
    setupWorkspaceLayout();
    setWindowTitle(windowTitle() + QStringLiteral(" · V1024-2158"));

    if (const char* value = std::getenv("RADAR_POINT_WINDOW_FRAMES")) {
        try {
            trackingWindowFrames = std::clamp(std::stoi(value), 1, 10);
        } catch (...) {
            trackingWindowFrames = std::clamp(RadarRuntimeConfig::kPointWindowFrames, 1, 10);
        }
    }
    trackingWindowFrames = std::clamp(trackingWindowFrames, 1, 10);
    if (const char* level = std::getenv("RADAR_LOG_LEVEL")) {
        const std::string value(level);
        verboseTrackingLogs = value == "debug" || value == "trace" || value == "2";
    }

    // 清除任何窗口级样式，恢复默认调色板（避免整体泛蓝）

    offlineGroup = ui->offlinePointCloudGroup;
    offlineBinCombo = ui->offlineBinCombo;
    offlineProcessButton = ui->offlineProcessButton;
    offlineTrackGroup = ui->offlinePointCloudGroup_2;
    offlineTrackCombo = ui->offlineBinCombo_2;
    offlineTrackButton = ui->offlineProcessButton_2;
    if (offlineGroup) offlineGroup->hide();
    if (offlineTrackGroup) offlineTrackGroup->hide();
    if (offlineProcessButton) {
        connect(offlineProcessButton, &QPushButton::clicked, this, &RadarGUI::on_offlineProcessButton_clicked, Qt::UniqueConnection);
    }
    if (offlineTrackButton) {
        connect(offlineTrackButton, &QPushButton::clicked, this, &RadarGUI::on_offlineProcessButton_clicked, Qt::UniqueConnection);
    }
    ui->graphicsView->viewport()->installEventFilter(this);
    if (ui->targetModeCombo) {
        // Safety default: never acquire or switch a target without an explicit
        // preview plus confirmation (index 0 = automatic, index 1 = manual).
        ui->targetModeCombo->setCurrentIndex(1);
        connect(ui->targetModeCombo,
                qOverload<int>(&QComboBox::currentIndexChanged),
                this, [this](int index) {
            if (index == 0) {
                clearTargetPreview();
            }
            updateTargetSelectionControls();
            refreshTargetTable();
            drawTrackingOverlay();
        }, Qt::UniqueConnection);
    }
    if (ui->targetRuleCombo) {
        ui->targetRuleCombo->setCurrentIndex(2);
    }
    if (ui->autoSelectButton) {
        connect(ui->autoSelectButton, &QPushButton::clicked, this, [this]() {
            const bool manualMode = ui->targetModeCombo &&
                                    ui->targetModeCombo->currentIndex() == 1;
            if (manualMode) {
                const auto selected = std::find_if(
                    targetTracker.candidates().begin(),
                    targetTracker.candidates().end(),
                    [this](const TargetTracking::CandidateTarget& candidate) {
                        return candidate.id > 0 &&
                               candidate.internal_uid == previewTargetUid;
                    });
                if (selected == targetTracker.candidates().end()) {
                    logMessage("预选人员已不在当前帧，请重新勾选");
                    clearTargetPreview();
                } else if (selectFollowTargetByUid(previewTargetUid)) {
                    clearTargetPreview();
                    ui->targetModeCombo->setCurrentIndex(1);
                }
            } else if (autoSelectFollowTarget(currentAutoSelectRule()) &&
                       ui->targetModeCombo) {
                // Automatic selection is still an explicit one-shot command.
                // Return to manual mode so a later loss cannot switch people.
                ui->targetModeCombo->setCurrentIndex(1);
            }
            refreshTargetTable();
            refreshLockedTargetInfo();
            drawTrackingOverlay();
        }, Qt::UniqueConnection);
    }
    if (ui->tableWidget) {
        connect(ui->tableWidget, &QTableWidget::cellClicked, this,
                &RadarGUI::onTargetTableCellClicked, Qt::UniqueConnection);
    }
    updateTargetSelectionControls();
    if (ui->backgroundStartButton) {
        connect(ui->backgroundStartButton, &QPushButton::clicked, this, [this]() {
            if (targetTracker.backgroundModelingActive()) {
                targetTracker.stopBackgroundModeling();
                ui->backgroundStartButton->setText("开始建模");
                logMessage(QString("背景建模：手动停止，已学习 %1 帧，背景簇=%2")
                               .arg(targetTracker.backgroundModelFrameCount())
                               .arg(targetTracker.backgroundModelTrackCount()));
                return;
            }
            targetTracker.beginBackgroundModeling();
            targetTracker.setBackgroundFilterEnabled(ui->backgroundFilterCheck &&
                                                     ui->backgroundFilterCheck->isChecked());
            ui->backgroundStartButton->setText("停止建模");
            logMessage("背景建模：开始。请保持小车/雷达静止，并尽量保证场景无人。");
        }, Qt::UniqueConnection);
    }
    if (ui->backgroundClearButton) {
        connect(ui->backgroundClearButton, &QPushButton::clicked, this, [this]() {
            targetTracker.clearBackgroundModel();
            if (ui->backgroundStartButton) ui->backgroundStartButton->setText("开始建模");
            logMessage("背景建模：已清除背景模型");
            refreshTargetTable();
            refreshLockedTargetInfo();
        }, Qt::UniqueConnection);
    }
    if (ui->backgroundFilterCheck) {
        connect(ui->backgroundFilterCheck, &QCheckBox::toggled, this, [this](bool checked) {
            targetTracker.setBackgroundFilterEnabled(checked);
            logMessage(QString("背景过滤：%1。当前背景簇=%2")
                           .arg(checked ? "启用" : "关闭")
                           .arg(targetTracker.backgroundModelTrackCount()));
        }, Qt::UniqueConnection);
    }

    // 显式连接菜单动作，避免自动连接失效
    if (ui->globalPointcloudCheck) {
        ui->globalPointcloudCheck->setChecked(true);
        connect(ui->globalPointcloudCheck, &QCheckBox::toggled, this, [this](bool checked) {
            logMessage(QString("全局点云显示：%1").arg(checked ? "开启" : "只显示锁定目标"));
            for (auto* item : pointItems) {
                scene->removeItem(item);
                delete item;
            }
            pointItems.clear();
            updateSceneRectFromRadarConfig(false);
            drawTrackingOverlay();
        }, Qt::UniqueConnection);
    }
    if (ui->action) {
        connect(ui->action, &QAction::triggered, this, &RadarGUI::on_action_triggered, Qt::UniqueConnection);
    }
    if (ui->action_3) {
        connect(ui->action_3, &QAction::triggered, this, &RadarGUI::on_action_3_triggered, Qt::UniqueConnection);
    }
    if (ui->action_5) {
        connect(ui->action_5, &QAction::triggered, this, &RadarGUI::on_action_5_triggered, Qt::UniqueConnection);
    }
    if (ui->action_4) {
        connect(ui->action_4, &QAction::triggered, this, &RadarGUI::on_action_4_triggered, Qt::UniqueConnection);
    }

    // Hardware control is created only on capture startup. Offline use and
    // GUI lifecycle tests do not send configuration commands to a device.
    int deviceCount = 0;
    cudaError_t cudaError = cudaGetDeviceCount(&deviceCount);
    if (cudaError != cudaSuccess || deviceCount == 0) {
        logMessage("[主程序] CUDA不可用，将使用CPU进行FFT计算");
    } else {
        logMessage(QString("[主程序] 检测到 %1 个CUDA设备，将使用GPU加速FFT").arg(deviceCount));
    }

    scene = new QGraphicsScene(this);
    ui->graphicsView->setScene(scene);
    ui->graphicsView->setRenderHint(QPainter::Antialiasing);
    ui->graphicsView->setBackgroundBrush(Qt::white);

    // 初始使用默认画布；雷达配置加载后会按 maxRange(m) 自动更新坐标范围。
    updateSceneRectFromRadarConfig(true);

    infoText = scene->addText(" ", QFont("Arial", 35));
    infoText->setDefaultTextColor(Qt::black);
    infoText->setPos(10, 10);

    connect(ui->pushButton_2, &QPushButton::clicked, this, &RadarGUI::handleStartCaptureClicked);  // 开始采集
    connect(ui->pushButton_3, &QPushButton::clicked, this, &RadarGUI::handleStopCaptureClicked);          // 停止采集
    connect(ui->pushButton,   &QPushButton::clicked, this, &RadarGUI::handleExitClicked);          // 退出
    if (ui->pushButton_4) {
        connect(ui->pushButton_4, &QPushButton::clicked, this, &RadarGUI::on_pushButton_4_clicked, Qt::UniqueConnection);
    }
    if (ui->pushButton_6) {
    connect(ui->pushButton_6, &QPushButton::clicked, this, &RadarGUI::on_pushButton_6_clicked, Qt::UniqueConnection);
    }
    udpSocket = new QUdpSocket(this);
    if (!udpSocket->bind(QHostAddress(QString::fromStdString(connection_.result_ip)), connection_.result_port))
        throw std::runtime_error("Cannot bind point-cloud endpoint: " + udpSocket->errorString().toStdString());
    connect(udpSocket, &QUdpSocket::readyRead, this, &RadarGUI::processUdpData);

    // 定时器用于更新点云显示
    updateTimer = new QTimer(this);
    connect(updateTimer, &QTimer::timeout, this, &RadarGUI::updatePointCloudDisplay);
    updateTimer->start(100);  // 每100ms更新一次，以匹配帧率

    updatePortList();
    updateConfigList();
    updateBinList();  // 同时刷新bin文件列表（供离线处理使用）
    updateTrackBinList();
    initTargetTable();

    // 读取 UI 中的 CFAR 参数并设置到算法模块
    auto pushCfarParams = [this]() {
        RadarProcessing::CFARRuntimeConfig cfg;
        cfg.N_ex = ui->spinNex ? ui->spinNex->value() : 8;
        cfg.K = ui->spinK ? ui->spinK->value() : 5;
        cfg.N_protect_r = ui->spinProtectR ? ui->spinProtectR->value() : 5;
        cfg.N_protect_d = ui->spinProtectD ? ui->spinProtectD->value() : 5;
        cfg.alpha = ui->spinAlpha ? static_cast<float>(ui->spinAlpha->value()) : 3.5f;
        RadarProcessing::set_cfar_params(cfg);
        if (ui->textEdit) {
            ui->textEdit->append("CFAR参数设置成功");
        }
        logMessage(QString("已保存CFAR参数: N_ex=%1, K=%2, N_protect_r=%3, N_protect_d=%4, alpha=%5")
                   .arg(cfg.N_ex).arg(cfg.K).arg(cfg.N_protect_r).arg(cfg.N_protect_d).arg(cfg.alpha));
    };
    if (ui->cfarSaveButton) {
        connect(ui->cfarSaveButton, &QPushButton::clicked, this, [pushCfarParams]() {
            pushCfarParams();
        });
    }

    // 将按钮放在 graphicsView 的 viewport 上，更容易精确定位在右下角
    QWidget* vp = ui->graphicsView->viewport();

    positionZoomButtons();

    // 目标聚类模块
    cluster = new Cluster(scene, this);
    cluster->setWindowSize(trackingWindowFrames);
    connect(cluster, &Cluster::playbackFinished, this, &RadarGUI::onClusterPlaybackFinished);

    std::cout << "[CONFIG] point_window_frames=" << trackingWindowFrames
              << ", track_id_mode=" << targetTracker.trackIdModeName()
              << ", log_level=" << (verboseTrackingLogs ? "debug" : "summary")
              << std::endl;

    // 初始为"原始点云"模式：隐藏"点云数据"与"离线点云"选择框
    if (ui->pointCloudGroup) ui->pointCloudGroup->hide();
    if (offlineGroup) offlineGroup->hide();
    if (offlineTrackGroup) offlineTrackGroup->hide();

    offlineTimer = new QTimer(this);
    connect(offlineTimer, &QTimer::timeout, this, &RadarGUI::processOfflineFrame);
}

RadarGUI::~RadarGUI()
{
    shutdownRuntime();
    delete cluster;
    cluster = nullptr;
    delete ui;
}

void RadarGUI::closeEvent(QCloseEvent* event)
{
    shutdownRuntime();
    event->accept();
}

void RadarGUI::setupWorkspaceLayout()
{
    const QScreen* screen = QApplication::primaryScreen();
    const QRect available = screen ? screen->availableGeometry() : QRect(0, 0, 1024, 600);
    const bool compact = available.width() <= 1100 || available.height() <= 650;
    setStyleSheet("background-color: rgb(198, 225, 239);");
    setWindowIcon(RadarUi::icon(RadarUi::Icon::Radar));
    QFont readableFont = font();
    readableFont.setPointSize(compact ? 9 : 10);
    setFont(readableFont);
    for (QWidget* widget : centralWidget()->findChildren<QWidget*>()) widget->setFont(readableFont);
#if RADAR_ENABLE_CAMERA
    cameraAssist_->setFont(readableFont);
    for (QWidget* widget : cameraAssist_->findChildren<QWidget*>()) widget->setFont(readableFont);
#endif

    // Preserve the generated widgets and their signal connections. Only move
    // them into a layout that fits the available screen instead of four columns.
    ui->mainLayout->removeWidget(ui->navFrame);
    ui->mainLayout->removeWidget(ui->configPanel);
    ui->mainLayout->removeWidget(ui->groupBox);
    ui->mainLayout->removeWidget(ui->taskPanel);
    auto* operations = new QToolBar("常用操作", this);
    operations->setObjectName("captureToolBar");
    operations->setMovable(false);
    operations->setFloatable(false);
    addToolBar(Qt::TopToolBarArea, operations);
    const RadarUi::Icon operationIcons[] = {RadarUi::Icon::Connect, RadarUi::Icon::Configure,
        RadarUi::Icon::Start, RadarUi::Icon::Track, RadarUi::Icon::Stop, RadarUi::Icon::Exit};
    int operationIndex = 0;
    for (QPushButton* button : {ui->connectButton, ui->configureButton, ui->pushButton_2,
                               ui->pushButton_6, ui->pushButton_3, ui->pushButton}) {
        ui->navLayout->removeWidget(button);
        button->setMinimumSize(76, 32);
        button->setMaximumSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX);
        QFont buttonFont = readableFont;
        buttonFont.setBold(true);
        button->setFont(buttonFont);
        button->setIcon(RadarUi::icon(operationIcons[operationIndex++]));
        button->setIconSize(QSize(18, 18));
        operations->addWidget(button);
    }
    ui->navFrame->hide();
    stopFollowButton_=new QPushButton("停止追踪",this);
    stopFollowButton_->setObjectName("stopFollowButton");
    stopFollowButton_->setIcon(RadarUi::icon(RadarUi::Icon::Stop));
    stopFollowButton_->setMinimumSize(76,32);stopFollowButton_->setEnabled(false);
    stopFollowButton_->setFont(ui->pushButton_6->font());
    QAction* beforeStopCapture=nullptr;
    for(auto* action:operations->actions())if(operations->widgetForAction(action)==ui->pushButton_3){beforeStopCapture=action;break;}
    operations->insertWidget(beforeStopCapture,stopFollowButton_);
    connect(stopFollowButton_,&QPushButton::clicked,rosRuntime_,&RosRuntimePanel::stopFollow);
    ui->rootLayout->setContentsMargins(4, 4, 4, 4);
    ui->rootLayout->setSpacing(4);
    ui->mainLayout->setContentsMargins(0, 0, 0, 0);
    ui->mainLayout->setSpacing(0);

    auto* splitter = new QSplitter(Qt::Horizontal, centralWidget());
    splitter->setObjectName("workspaceSplitter");
    splitter->setChildrenCollapsible(false);
    splitter->setHandleWidth(6);
    auto* views = new QTabWidget(splitter);
    views->setObjectName("sensorViewTabs");
    views->setMinimumWidth(280);
    views->setFont(readableFont);
    views->setIconSize(QSize(16, 16));
    ui->groupBox->setTitle(QString());
    ui->pointCloudLayout->setContentsMargins(0, 0, 0, 0);
    ui->groupBox->setMinimumSize(0, 0);
    ui->graphicsView->setMinimumSize(200, 120);
    ui->graphicsView->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    auto* cooperativePage = new QWidget(views);
    cooperativePage->setObjectName("cooperativePage");
    auto* cooperativeLayout = new QHBoxLayout(cooperativePage);
    cooperativeLayout->setContentsMargins(0, 0, 0, 0);
    auto* cooperativeSplit = new QSplitter(Qt::Horizontal, cooperativePage);
    cooperativeSplit->setObjectName("cooperativeSplitter");
    cooperativeSplit->setChildrenCollapsible(false);
    cooperativeSplit->setHandleWidth(5);
    cooperativeLayout->addWidget(cooperativeSplit);
    cooperativeView_ = new RadarCooperative::CooperativeView(cooperativeSplit);
    auto* sensorSplit = new QSplitter(Qt::Vertical, cooperativeSplit);
    sensorSplit->setObjectName("cooperativeSensorSplitter");
    sensorSplit->setChildrenCollapsible(false);
    sensorSplit->setHandleWidth(5);
    auto sensorPane = [sensorSplit](const QString& title, const QString& name) {
        auto* pane = new QWidget(sensorSplit);
        pane->setObjectName(name);
        auto* layout = new QVBoxLayout(pane);
        layout->setContentsMargins(0, 0, 0, 0);
        layout->setSpacing(0);
        auto* label = new QLabel(title, pane);
        label->setMargin(4);label->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Fixed);
        label->setStyleSheet("QLabel { background: #e7f0f8; color: #354b60; font-weight: bold; }");
        layout->addWidget(label);
        return layout;
    };
    auto* cooperativeCameraHost = sensorPane("摄像头 · 点击选人", "cooperativeCameraPane");
    auto* cooperativeRadarHost = sensorPane("雷达点云 · 点击目标预选", "cooperativeRadarPane");
    selectionHintLabel_=new QLabel("点雷达目标预选，再确认；两路由人工核对。",sensorSplit);
    selectionHintLabel_->setObjectName("radarSelectionHint");selectionHintLabel_->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Preferred);selectionHintLabel_->setWordWrap(true);selectionHintLabel_->setMaximumHeight(30);
    selectionHintLabel_->setToolTip("红点只表示多普勒运动观测；当前帧支持、点数、稳定轨迹和1.8m近场过滤均影响可选性。不可把单个速度点直接当作已识别人。建议在2.5–4m处缓慢横移后选择P编号。");
    selectionHintLabel_->setStyleSheet("color:#6b4b09;font-size:12px;");
    cooperativeRadarHost->addWidget(selectionHintLabel_);
    auto* radarPage = new QWidget(views);
    radarPage->setObjectName("radarSinglePage");
    auto* radarHost = new QVBoxLayout(radarPage);
    radarHost->setContentsMargins(0, 0, 0, 0);
    views->addTab(cooperativePage, RadarUi::icon(RadarUi::Icon::Track), "雷视协同");
    views->addTab(radarPage, RadarUi::icon(RadarUi::Icon::Radar), "雷达点云");
#if RADAR_ENABLE_CAMERA
    auto* cameraPage = new QWidget(views);
    cameraPage->setObjectName("cameraSinglePage");
    auto* cameraHost = new QVBoxLayout(cameraPage);
    cameraHost->setContentsMargins(0, 0, 0, 0);
    views->addTab(cameraPage, RadarUi::icon(RadarUi::Icon::Camera), "摄像头");
#else
    cooperativeCameraHost->addWidget(new QLabel("此构建未启用摄像头", sensorSplit));
#endif
    auto mountSensors = [=] {
        const bool combined = views->currentWidget() == cooperativePage;
        (combined ? cooperativeRadarHost : radarHost)->addWidget(ui->groupBox, 1);
        ui->groupBox->show();
#if RADAR_ENABLE_CAMERA
        cameraAssist_->setCompactView(combined);
        (combined ? cooperativeCameraHost : cameraHost)->addWidget(cameraAssist_, 1);
        cameraAssist_->show();
#endif
        QTimer::singleShot(0, this, [this] { positionZoomButtons(); applyZoom(); });
    };
    connect(views, &QTabWidget::currentChanged, this, [mountSensors](int) { mountSensors(); });
    mountSensors();
    cooperativeSplit->setStretchFactor(0, 55);
    cooperativeSplit->setStretchFactor(1, 45);
    cooperativeSplit->setSizes({350, 280});
    sensorSplit->setSizes({220, 220});
    for (auto* inner : {cooperativeSplit, sensorSplit})
        connect(inner, &QSplitter::splitterMoved, this, [this] { positionZoomButtons(); applyZoom(); });
    cooperativeTimer_ = new QTimer(this);
    cooperativeTimer_->setInterval(100);
    connect(cooperativeTimer_, &QTimer::timeout, this, &RadarGUI::refreshCooperativeView);
    cooperativeTimer_->start();
#if RADAR_ENABLE_CAMERA
    if(cameraAssist_)connect(cameraAssist_,&CameraAssistPanel::replayRequested,this,[this]{
        if(rosRuntime_)rosRuntime_->stopFollow();
        if(collector||is_capturing)stop_capture();
        cameraAssist_->stopCamera();
        writeTrackingMetadata(currentFrameContext.frame_number,"camera_replay");
        openCameraReplay(this);
    });
#endif

    auto* sidebar = new QWidget(splitter);
    sidebar->setObjectName("workspaceSidebar");
    const int sidebarWidth = compact ? 300 : 340;
    sidebar->setMinimumWidth(sidebarWidth);
    auto* sideLayout = new QVBoxLayout(sidebar);
    sideLayout->setContentsMargins(0, 0, 0, 0);
    sideLayout->setSpacing(3);
    auto* pages = new QTabWidget(sidebar);
    pages->setObjectName("workspaceControlTabs");
    pages->setFont(readableFont);
    pages->setIconSize(QSize(14, 14));
    sideLayout->addWidget(pages, 1);
    auto* rosSummary=new QLabel("ROS：随行尚未启动",sidebar);rosSummary->setObjectName("rosRuntimeSummary");
    rosSummary->setWordWrap(true);rosSummary->setTextFormat(Qt::PlainText);
    rosSummary->setSizePolicy(QSizePolicy::Ignored,QSizePolicy::Preferred);sideLayout->addWidget(rosSummary);
    connect(rosRuntime_,&RosRuntimePanel::statusChanged,rosSummary,[rosSummary](const QString& text){rosSummary->setText("ROS："+text.section('\n',0,0).left(120));rosSummary->setToolTip(text);});
    auto addPage = [pages](QWidget* content, const QString& title) {
        auto* scroll = new QScrollArea(pages);
        scroll->setFrameShape(QFrame::NoFrame);
        scroll->setWidgetResizable(true);
        scroll->setWidget(content);
        return pages->addTab(scroll, title);
    };
    for (QFrame* panel : {ui->configPanel, ui->taskPanel}) {
        panel->setMinimumSize(0, 0);
        panel->setMaximumSize(QWIDGETSIZE_MAX, QWIDGETSIZE_MAX);
        panel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        panel->setFrameShape(QFrame::NoFrame);
    }
    for (QVBoxLayout* layout : {ui->configPanelLayout, ui->taskPanelLayout}) {
        layout->setContentsMargins(3, 3, 3, 3);
        layout->setSpacing(5);
    }
    // Infrequent setup no longer takes space from the target table.
    ui->taskPanelLayout->removeWidget(ui->backgroundModelGroup);
    ui->configPanelLayout->addWidget(ui->backgroundModelGroup);
    ui->configPanelLayout->addStretch();
    // A label/value row uses less height than separate label and combo rows.
    while (auto* item = ui->targetTrackingLayout->takeAt(0)) delete item;
    ui->targetTrackingLayout->setSpacing(4);
    auto* selectionForm = new QFormLayout();
    selectionForm->setFieldGrowthPolicy(QFormLayout::ExpandingFieldsGrow);
    selectionForm->setVerticalSpacing(4);
    selectionForm->addRow(ui->targetModeLabel, ui->targetModeCombo);
    selectionForm->addRow(ui->targetRuleLabel, ui->targetRuleCombo);
    ui->targetTrackingLayout->addLayout(selectionForm);
    ui->targetTrackingLayout->addWidget(ui->autoSelectButton);
    ui->targetTrackingLayout->addWidget(ui->tableWidget, 2);
    ui->targetTrackingLayout->addWidget(ui->lockedTargetInfoText, 1);
    // CFAR's old two-pair grid exceeded the sidebar width; keep one pair per row.
    while (auto* item = ui->cfarGrid->takeAt(0)) delete item;
    ui->cfarGrid->addWidget(ui->labelNex, 0, 0);
    ui->cfarGrid->addWidget(ui->spinNex, 0, 1);
    ui->cfarGrid->addWidget(ui->labelK, 1, 0);
    ui->cfarGrid->addWidget(ui->spinK, 1, 1);
    ui->cfarGrid->addWidget(ui->labelAlpha, 2, 0);
    ui->cfarGrid->addWidget(ui->spinAlpha, 2, 1);
    ui->cfarGrid->addWidget(ui->labelPr, 3, 0);
    ui->cfarGrid->addWidget(ui->spinProtectR, 3, 1);
    ui->cfarGrid->addWidget(ui->labelPd, 4, 0);
    ui->cfarGrid->addWidget(ui->spinProtectD, 4, 1);
    ui->cfarGrid->addWidget(ui->cfarSaveButton, 5, 0, 1, 2);
    pages->setTabIcon(addPage(ui->taskPanel, "目标"), RadarUi::icon(RadarUi::Icon::Person));
    pages->setTabIcon(addPage(ui->configPanel, "配置"), RadarUi::icon(RadarUi::Icon::Configure));
    pages->setTabIcon(addPage(rosRuntime_, "ROS/底盘"), RadarUi::icon(RadarUi::Icon::Configure));
#if RADAR_ENABLE_CAMERA
    QWidget* cameraControls = cameraAssist_->takeControlsWidget();
    cameraControls->setFont(readableFont);
    const int cameraSettingsIndex = addPage(cameraControls, "视觉");
    pages->setTabIcon(cameraSettingsIndex, RadarUi::icon(RadarUi::Icon::Camera));
    auto* cameraAction = menuBar()->addAction("视觉协同");
    connect(cameraAction, &QAction::triggered, this, [views, pages, cameraSettingsIndex, cooperativePage] {
        views->setCurrentWidget(cooperativePage);
        pages->setCurrentIndex(cameraSettingsIndex);
    });
    auto* visionState = new QLabel("视觉保护未启用", sidebar);
    visionState->setObjectName("visualGuardSummary");
    visionState->setFont(readableFont);
    visionState->setWordWrap(true);
    sideLayout->addWidget(visionState);
    connect(cameraAssist_, &CameraAssistPanel::evidenceChanged, this, [this, visionState] {
        const auto output = targetTracker.output();
        const auto state = cameraAssist_->decision(output.target.matched_internal_uid,
                                                   output.selection.selection_epoch);
        visionState->setText(!state.enabled ? "视觉保护未启用" :
                            state.permit ? "视觉保护：绑定人物连续可见" : "视觉保护：禁止运动，请检查视觉页");
        visionState->setStyleSheet(!state.enabled ? QString() :
                                  state.permit ? "color: #146c2e;" : "color: #b42318;");
    });
#endif
    auto* logs = new QWidget(pages);
    auto* logLayout = new QVBoxLayout(logs);
    logLayout->setContentsMargins(3, 3, 3, 3);
    ui->configPanelLayout->removeWidget(ui->captureGroup);
    ui->configPanelLayout->removeWidget(ui->logGroup);
    ui->captureGroup->setMaximumHeight(140);
    ui->textEdit->setMinimumHeight(55);
    ui->logTextEdit->setMinimumHeight(90);
    logLayout->addWidget(ui->captureGroup);
    logLayout->addWidget(ui->logGroup, 1);
    pages->setTabIcon(addPage(logs, "日志"), RadarUi::icon(RadarUi::Icon::Log));

    ui->tableWidget->setMinimumSize(0, 85);
    ui->tableWidget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    ui->tableWidget->verticalHeader()->setVisible(false);
    ui->tableWidget->horizontalHeader()->setMinimumSectionSize(38);
    ui->tableWidget->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    ui->lockedTargetInfoText->setMinimumHeight(70);
    ui->lockedTargetInfoText->setMaximumHeight(155);
    ui->serialPortCombo->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
    ui->serialPortCombo->setMinimumContentsLength(8);
    ui->serialPortCombo->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);
    ui->configFileCombo->setSizeAdjustPolicy(QComboBox::AdjustToMinimumContentsLengthWithIcon);
    ui->configFileCombo->setMinimumContentsLength(10);
    ui->configFileCombo->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Fixed);

    splitter->setStretchFactor(0, 1);
    splitter->setStretchFactor(1, 0);
    const int width = std::min(available.width(), 1440);
    splitter->setSizes({std::max(280, width - sidebarWidth - 12), sidebarWidth});
    connect(splitter, &QSplitter::splitterMoved, this, [this] {positionZoomButtons();applyZoom();});
    ui->mainLayout->addWidget(splitter);
    if (statusBar()) statusBar()->hide();
    setMinimumSize(620, 380);
    resize(std::min(available.width(), 1440), std::min(available.height() - 32, 900));
}

void RadarGUI::refreshCooperativeView()
{
    if (shutdown_ || !cooperativeView_) return;
    using namespace RadarCooperative;
    Snapshot state;
    const bool realtime = !isOfflineMode && !isOfflineTrackingMode && !isTestMode;
    state.acquisitionActive = is_capturing && realtime;
    // Radar tracking runs during acquisition; the toolbar starts ROS following.
    state.trackingEnabled = state.acquisitionActive;
    const double now = RadarTime::monotonicSeconds();
    const double stamp = currentFrameContext.received_monotonic_sec;
    state.radar = {state.acquisitionActive && stamp > 0 && std::isfinite(stamp),
                   stamp > 0 ? now - stamp : -1.0, 0.35};
    const auto output = targetTracker.output();
    const auto& locked = output.target;
    state.targetSelected = state.acquisitionActive && locked.valid;
    state.targetPositionValid = state.targetSelected && std::isfinite(locked.x) && std::isfinite(locked.y);
    state.targetPosition = QPointF(locked.x, locked.y);
    state.targetLabel = QString("T%1 · P%2").arg(static_cast<qulonglong>(output.selection.selection_epoch)).arg(locked.display_id);
    state.observation = !locked.valid ? ObservationState::Unknown : locked.has_measurement
        ? ObservationState::Measured : ObservationState::Predicted;
    if (state.acquisitionActive) {
        for (const auto& person : targetTracker.candidates()) {
            if (person.id > 0 && person.has_current_frame_support &&
                (!locked.valid || person.internal_uid != locked.matched_internal_uid))
                state.otherPeople.push_back({QString("P%1").arg(person.id), QPointF(person.x, person.y)});
        }
    }
#if RADAR_ENABLE_CAMERA
    if (cameraAssist_) {
        const auto evidence = cameraAssist_->displayEvidence();
        const double captured = evidence["captured_monotonic"].toDouble();
        state.camera = {evidence["available"].toBool(), captured > 0 ? now - captured : -1.0, 0.5};
        state.targetPersonConfirmed = state.targetSelected&&evidence["bound"].toBool();
        state.visualEvidence = evidence["verified"].toBool() ? VisualEvidence::Verified :
            evidence["bound"].toBool() ? VisualEvidence::Bound : VisualEvidence::None;
    }
#endif
    state.followState = FollowState::NotStarted;
    if (rosRuntime_) {
        const auto runtime = rosRuntime_->displayState();
        const auto phase = runtime["phase"].toString();
        const double age = runtime["age_seconds"].toDouble(-1);
        if (runtime["stopping"].toBool()) state.followState = FollowState::Stopped;
        else if (runtime["follow_requested"].toBool()) {
            state.followState = FollowState::Requested;
            if (runtime["process_running"].toBool() && runtime["ready"].toBool() &&
                age >= 0 && age <= 1.5 && phase == "following") {
                const auto control = runtime["control"].toObject();
                const double published = control["published_time_sec"].toDouble();
                const double controlAge = RadarTime::wallSeconds() - published;
                if (!std::isfinite(controlAge) || published <= 0 || controlAge < -0.05 || controlAge > 1.0)
                    state.followState = FollowState::Unknown;
                else if (control["reason"].toString() != "tracking_command" ||
                         (!control["allow_translation"].toBool() && !control["allow_yaw"].toBool())) {
                    state.followState = FollowState::Stopped;
                    state.detail = QStringLiteral("随行等待运动许可 · 详情见 ROS/底盘");
                } else state.followState = control["allow_translation"].toBool()
                    ? FollowState::Running : FollowState::Turning;
            }
            else if (runtime["ready"].toBool() && (age < 0 || age > 1.5))
                state.followState = FollowState::Unknown;
        }
    }
    if (!realtime) state.detail = QStringLiteral("当前为离线模式；协同页仅显示实时采集");
    if(!state.targetSelected&&!selectionHintText_.isEmpty())state.detail=selectionHintText_;
#if RADAR_ENABLE_CAMERA
    if(cameraAssist_&&!state.targetSelected) {
        const auto e=cameraAssist_->displayEvidence();
        if(e["pending_camera_track"].toString().toULongLong())
            state.detail=QString("已选视觉 V%1 · 请人工确认对应雷达目标").arg(e["pending_camera_track"].toString());
    }
#endif
    cooperativeView_->setSnapshot(state);
}

void RadarGUI::logMessage(const QString& message)
{
    QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
    ui->logTextEdit->append(QString("[%1] %2").arg(timestamp, message));
    if (trackingEventStream.is_open()) {
        const QByteArray utf8 = message.toUtf8();
        trackingEventStream << QDateTime::currentDateTime()
                                   .toString("yyyy-MM-ddThh:mm:ss.zzz")
                                   .toStdString()
                            << " " << utf8.constData() << '\n';
        // Event messages are sparse and often contain the reason immediately
        // before a stop/crash, so favor durability over buffering here.
        trackingEventStream.flush();
    }
    QTextCursor cursor = ui->logTextEdit->textCursor();
    cursor.movePosition(QTextCursor::End);
    ui->logTextEdit->setTextCursor(cursor);
    // 立刻处理事件，确保在耗时操作（如配置雷达）开始前文本已刷新到界面
    // Do not re-enter event handlers while publishing/logging a frame.
}

void RadarGUI::updatePortList()
{
    ui->serialPortCombo->clear();
    auto ports = listSerialPorts();
    for (const auto& port : ports) {
        ui->serialPortCombo->addItem(QString::fromStdString(port));
    }

    // 自动检测雷达串口并默认选择
    std::string detectedPort = detectRadarPort();
    if (!detectedPort.empty()) {
        int index = ui->serialPortCombo->findText(QString::fromStdString(detectedPort));
        if (index >= 0) {
            ui->serialPortCombo->setCurrentIndex(index);
            logMessage(QString("自动选择雷达串口：%1").arg(QString::fromStdString(detectedPort)));
        }
    }
}

void RadarGUI::updateConfigList()
{
    ui->configFileCombo->clear();
    auto files = list_config_files();
    for (const auto& file : files) {
        ui->configFileCombo->addItem(QString::fromStdString(file));
    }
    if (!files.empty()) {
        config_file = "./config/" + files[0];
    }

    // 同步填充点云下拉框（./txt 目录）
    if (ui->pointCloudCombo) {
        ui->pointCloudCombo->clear();
        QDir txtDir("./txt");
        if (txtDir.exists()) {
            QStringList txts = txtDir.entryList({"point_cloud_*.txt"}, QDir::Files, QDir::Name);
            for (const QString& f : txts) ui->pointCloudCombo->addItem(f);
        }
    }
}

std::vector<std::string> RadarGUI::listSerialPorts()
{
    std::vector<std::string> ports;

#ifdef _WIN32
    HKEY hKey;
    LONG result = RegOpenKeyExA(HKEY_LOCAL_MACHINE, "HARDWARE\\DEVICEMAP\\SERIALCOMM", 0, KEY_READ, &hKey);
    if (result != ERROR_SUCCESS) {
        return ports;
    }

    DWORD index = 0;
    char valueName[256];
    DWORD valueNameSize = sizeof(valueName);
    char data[256];
    DWORD dataSize = sizeof(data);
    DWORD type;
    while (RegEnumValueA(hKey, index, valueName, valueNameSize, NULL, &type, (LPBYTE)data, dataSize) == ERROR_SUCCESS) {
        if (type == REG_SZ) {
            ports.push_back(std::string(data));
        }
        valueNameSize = sizeof(valueName);
        dataSize = sizeof(data);
        index++;
    }

    RegCloseKey(hKey);
#else
    DIR* dir = opendir("/dev");
    if (!dir) {
        return ports;
    }

    struct dirent* entry;
    while ((entry = readdir(dir)) != nullptr) {
        std::string name = entry->d_name;
        if (name.find("ttyS") == 0 || name.find("ttyUSB") == 0 || name.find("ttyACM") == 0) {
            std::string fullPath = "/dev/" + name;
            int fd = open(fullPath.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
            if (fd >= 0) {
                ports.push_back(fullPath);
                ::close(fd);
            }
        }
    }

    closedir(dir);
#endif

    return ports;
}

std::string RadarGUI::selectSerialPort()
{
    if (ui->serialPortCombo->currentIndex() >= 0) {
        return ui->serialPortCombo->currentText().toStdString();
    }
    return "";
}

std::string RadarGUI::detectRadarPort()
{
    auto start_time = std::chrono::high_resolution_clock::now();
    try {
        std::vector<std::string> ports = listSerialPorts();
        if (ports.empty()) {
            logMessage("没有找到串口。");
            return "";
        }

        for (const auto& port : ports) {
            try {
                AWR2944SerialConfig test_radar(port, 115200);
                // 发送测试命令 "version\n" 并检查响应（首次启动时可靠）
                std::string test_cmd = "version\n";
#ifdef _WIN32
                DWORD bytes_written;
                WriteFile(test_radar.get_cli_port(), test_cmd.c_str(), test_cmd.size(), &bytes_written, NULL);  // 使用 getter
#else
                ssize_t bytes_written = write(test_radar.get_fd(), test_cmd.c_str(), test_cmd.size());  // 使用 getter
                if (bytes_written < 0) {
                    continue; // 写入失败，跳过
                }
#endif
                // 读取响应，带超时
                std::string response;
                auto start = std::chrono::steady_clock::now();
                while (std::chrono::steady_clock::now() - start < std::chrono::milliseconds(100)) {
                    char buf[1024];
#ifdef _WIN32
                    DWORD bytes_read;
                    ReadFile(test_radar.get_cli_port(), buf, sizeof(buf), &bytes_read, NULL);  // 使用 getter
                    if (bytes_read > 0) response.append(buf, bytes_read);
#else
                    ssize_t bytes_read = read(test_radar.get_fd(), buf, sizeof(buf));  // 使用 getter
                    if (bytes_read > 0) response.append(buf, bytes_read);
#endif
                    std::this_thread::sleep_for(std::chrono::milliseconds(2));
                }
                // 检查响应是否非空（可靠检测）
                logMessage(QString("[DEBUG] Port %1 response: '%2'").arg(QString::fromStdString(port), QString::fromStdString(response)));
                // 检查响应是否包含雷达版本信息（更严格的验证）
                if (!response.empty() && response.find("version") != std::string::npos) {
                    logMessage(QString("检测到雷达串口：%1 (响应: %2)").arg(QString::fromStdString(port), QString::fromStdString(response)));
                    auto end_time = std::chrono::high_resolution_clock::now();
                    logMessage(QString("detectRadarPort()耗时: %1 ms").arg(std::chrono::duration<float, std::milli>(end_time - start_time).count()));
                    return port; // 找到有效响应，返回此串口
                } else if (!response.empty()) {
                    logMessage(QString("[DEBUG] Port %1 有响应但不是雷达: '%2'").arg(QString::fromStdString(port), QString::fromStdString(response)));
                }
                test_radar.close(); // 关闭测试实例
            } catch (const std::exception& e) {
                // 忽略异常，继续下一个
                continue;
            }
        }
        logMessage("未检测到雷达串口。");
    } catch (const std::exception& e) {
        logMessage(QString("Error: %1").arg(e.what()));
        return "";
    }
    auto end_time = std::chrono::high_resolution_clock::now();
    logMessage(QString("detectRadarPort()耗时: %1 ms").arg(std::chrono::duration<float, std::milli>(end_time - start_time).count()));
    return "";
}

void RadarGUI::on_connectButton_clicked()
{
    connect_radar();
}

void RadarGUI::on_configureButton_clicked()
{
    select_config_file();
}

void RadarGUI::handleStartCaptureClicked()
{
    if (isTestMode) {
        logMessage("离线聚类模式请使用'开始'按钮");
        return;
    }
    start_capture();
}

void RadarGUI::handleStopCaptureClicked()
{
    if (isTestMode) {
        logMessage("离线聚类模式请使用'停止'按钮");
        return;
    }
    stop_capture();
}

void RadarGUI::handleExitClicked()
{
    exit_program();
}

void RadarGUI::on_pushButton_4_clicked()
{
    if (!isTestMode) {
        logMessage("当前不是离线聚类模式");
        return;
    }
    if (is_capturing) {
        stop_capture();
        if (ui->pushButton_4) ui->pushButton_4->setText("开始");
        return;
    }
    start_capture();
    if (is_capturing && ui->pushButton_4) {
        ui->pushButton_4->setText("停止");
    }
}

void RadarGUI::on_refreshPortsButton_clicked()
{
    updatePortList();
}

void RadarGUI::connect_radar()
{
    logMessage("正在连接雷达...");

    if (radar != nullptr && radar->is_open()) {
        logMessage("已连接到雷达，无需重复连接。");
        // 不再重复在"当前状态"里提示"连接成功"，避免重复显示
        return;
    }

    std::string selected_port = selectSerialPort();
    if (!selected_port.empty()) {
        try {
            radar = new AWR2944SerialConfig(selected_port, 115200);
            logMessage(QString("已连接到 %1").arg(QString::fromStdString(selected_port)));
            ui->textEdit->append("雷达连接成功\n");
            // 成功连接后禁用"连接雷达"按钮，防止重复点击造成重复日志
        } catch (const std::exception& e) {
            logMessage(QString("连接失败：%1").arg(e.what()));
        }
    } else {
        logMessage("未选择串口。");
    }
}

std::vector<std::string> RadarGUI::list_config_files()
{
    std::vector<std::string> files;
    std::string config_dir = "./config";
    if (!fs::exists(config_dir)) {
        logMessage("配置文件目录不存在。");
    } else {
        for (const auto& entry : fs::directory_iterator(config_dir)) {
            if (fs::is_regular_file(entry.status())) {
                files.push_back(entry.path().filename().string());
            }
        }
        if (files.empty()) {
            logMessage("没有找到配置文件。");
        }
    }
    return files;
}

void RadarGUI::configure_radar()
{
    if (collector || is_capturing) { logMessage("采集期间不能修改雷达配置，请先停止。"); return; }
    // 防重入：如果正在配置，直接返回，避免重复输出参数块
    if (is_configuring) {
        logMessage("配置流程已在进行中，忽略重复请求。");
        return;
    }

    logMessage("正在配置雷达...");

    if (radar == nullptr || !radar->is_open()) {
        logMessage("雷达未连接。");
        return;
    }
    if (config_file.empty()) {
        logMessage("未选择配置文件。");
        return;
    }
    if (!fs::exists(config_file)) {
        logMessage("所选配置文件不存在。");
        return;
    }
    // 通过前置校验后再进入"配置中"状态，这样早退不会影响按钮可用性
    is_configuring = true;
    if (ui->configureButton) ui->configureButton->setEnabled(false);
    // RAII 保护：无论何种返回路径，都会恢复按钮状态与 is_configuring 标志
    struct ConfigGuard { RadarGUI* self; ~ConfigGuard(){ if (self && self->ui && self->ui->configureButton) self->ui->configureButton->setEnabled(true); if (self) self->is_configuring = false; } } _cfgGuard{this};

    try {
        // 实时 Tee：同时保留终端输出并实时写入 GUI 日志
        std::streambuf* oldCout = std::cout.rdbuf();
        std::streambuf* oldCerr = std::cerr.rdbuf();
        QtLogTeeBuf coutTee(oldCout, this);
        QtLogTeeBuf cerrTee(oldCerr, this);
        std::cout.rdbuf(&coutTee);
        std::cerr.rdbuf(&cerrTee);
        // RAII 保护：异常也能恢复 rdbuf
        struct RdbufGuard { std::streambuf* oc; std::streambuf* oe; RdbufGuard(std::streambuf* c, std::streambuf* e): oc(c), oe(e) {} ~RdbufGuard(){ std::cout.rdbuf(oc); std::cerr.rdbuf(oe); } } _rbGuard(oldCout, oldCerr);

        // Configure the capture card before sensorStart in the radar cfg, as in
        // the original hardware startup order. Keep GUI construction device-free.
        if (!dca1000_cfg) dca1000_cfg = new DCA1000Config("DCA1000", connection_.host_ip,
            connection_.config_port, connection_.fpga_ip, connection_.fpga_port);
        radar->SendConfig(config_file);
        IWR6843AOP_TLV tlv(config_file);

        radar_config = tlv.get_params();
        updateSceneRectFromRadarConfig(true);
        logMessage(QString("已配置雷达 %1").arg(QString::fromStdString(config_file)));
        // 在"当前状态"textEdit里追加展示雷达参数（不覆盖）
        QString params;
        params += "\n==== 配置完成，雷达参数 ====:\n";
        for (const auto& kv : radar_config) {
            params += QString("%1: %2\n").arg(QString::fromStdString(kv.first)).arg(kv.second);
        }
        if (ui->textEdit) ui->textEdit->append(params);
    } catch (const std::exception& e) {
        logMessage(QString("配置雷达失败：%1").arg(e.what()));
    }
}

void RadarGUI::start_capture()
{
    if (shutdown_ || collector || is_capturing) { logMessage("采集已经启动，忽略重复开始。"); return; }
    logMessage("正在开始捕获数据...");

    // 离线处理模式：不允许通过"开始采集"按钮触发，应使用"处理"按钮
    if (isOfflineMode) {
        logMessage("离线处理模式下请使用'处理'按钮开始离线处理");
        return;
    }

    // 目标聚类：点击"开始采集"即从 ./txt 下文件绘制点云，不进行实时采集
    if (isTestMode) {
        QDir txtDir("./txt");
        if (!txtDir.exists()) {
            logMessage("目标聚类：未找到 ./txt 目录");
            return;
        }
        QString filePath;
        if (ui->pointCloudCombo && ui->pointCloudCombo->currentIndex() >= 0) {
            const QString chosen = ui->pointCloudCombo->currentText();
            if (!chosen.isEmpty()) filePath = txtDir.filePath(chosen);
        }
        if (filePath.isEmpty()) {
            const QString targetName = "point_cloud_20251026_1639_37.txt";
            if (txtDir.exists(targetName)) {
                filePath = txtDir.filePath(targetName);
            } else {
                QStringList files = txtDir.entryList({"point_cloud_*.txt"}, QDir::Files, QDir::Name);
                if (!files.isEmpty()) filePath = txtDir.filePath(files.first());
            }
        }
        if (filePath.isEmpty()) {
            logMessage("目标聚类：未找到 point_cloud_*.txt 文件");
            return;
        }
        if (!cluster->loadFile(filePath)) {
            logMessage("目标聚类：无法加载点云文件");
            return;
        }
        if (cluster->frameCount() < trackingWindowFrames) {
            logMessage(QString("目标聚类：文件有效帧不足 %1 帧，无法进行滑窗")
                           .arg(trackingWindowFrames));
            return;
        }
        // 启动播放前清空画布上残留的点云和文字
        for (auto* item : pointItems) {
            scene->removeItem(item);
            delete item;
        }
        pointItems.clear();
        if (infoText) {
            infoText->hide();
        }
        cluster->startPlayback();
        is_capturing = true;
        return;
    }

    if (radar_config.empty()) {
        logMessage("雷达未配置。");
        return;
    }
    try {
        const int frame_length = static_cast<int>(RadarFrameLayout::frameBytes(
            RadarFrameLayout::paramsFromMap(radar_config)) / 2);
        if (!dca1000_cfg) dca1000_cfg = new DCA1000Config("DCA1000", connection_.host_ip,
            connection_.config_port, connection_.fpga_ip, connection_.fpga_port);
    // 新会话开始前，重置信号处理会话，确保新文件与帧号从头开始
    reset_signal_processing_session();
    resetRealtimeClusterBuffer();
    targetTracker.reset();
    resetTrackTxtOutput();
    syncBackgroundUiAfterTrackerReset();
    refreshTargetTable();
    refreshLockedTargetInfo();

    CaptureOptions captureOptions;
    captureOptions.max_frame_age_sec = RadarEnvironment::number("RADAR_MAX_FRAME_AGE_SEC", 0.5, 0.01, 30);
    collector = new UdpListener("Listener", bin_data, frame_length, connection_.host_ip,
        connection_.data_port, 65536, radar_config, connection_.result_ip, connection_.result_port, captureOptions);
        collector->start();
        collector->start_saving();
        trackingStats.input_path = collector->output_filename();
        ensureTrackingDiagnosticLogOpen();
        ::capture_start_time = std::chrono::system_clock::now();
        std::string startError;
        if (!dca1000_cfg->DCA1000_start(&startError)) throw std::runtime_error(startError);
        lastCaptureState_ = CaptureStatus::State::WaitingForData;
        nextCaptureDiagnosticSec_ = RadarTime::monotonicSeconds() + 5;
        auto time_stamp = std::chrono::system_clock::to_time_t(::capture_start_time);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      ::capture_start_time.time_since_epoch()) % 1000;
        QDateTime dt = QDateTime::fromTime_t(time_stamp);
        QString timeStr = dt.toString("yyyy-MM-dd hh:mm:ss") + "." + QString::number(ms.count()).rightJustified(3, '0');
        logMessage(QString("数据捕获开始时间: %1").arg(timeStr));
        is_capturing = true;
        logMessage("采集已绑定，等待完整 ADC 帧；收到数据后才确认收帧。");
        if(rosRuntime_)rosRuntime_->setCaptureSession(collector->session_id());
        ui->textEdit->append("采集 socket 就绪，等待完整数据帧\n");
    } catch (const std::exception& e) {
        logMessage(QString("开始捕获失败：%1").arg(e.what()));
        if (collector) {
            collector->stop_saving();
            collector->stop_collector();
            delete collector;
            collector = nullptr;
        }
        // 启动失败也复位一次，避免产生残留文件句柄
        reset_signal_processing_session();
        finalizeTrackingSession("realtime_start_failed");
        if (dca1000_cfg) { dca1000_cfg->DCA1000_close(); delete dca1000_cfg; dca1000_cfg = nullptr; }
        is_capturing = false;
    }
}

void RadarGUI::select_config_file()
{
    if (collector || is_capturing) { logMessage("请先停止采集再选择配置。"); return; }
    auto files = list_config_files();
    if (files.empty()) return;
    config_file = "./config/" + (ui->configFileCombo && ui->configFileCombo->currentIndex() >= 0
        ? ui->configFileCombo->currentText().toStdString() : files[0]);
    logMessage(QString("已选择配置文件：%1").arg(QString::fromStdString(config_file)));
    configure_radar();
}

void RadarGUI::stop_capture()
{
    if(rosRuntime_){rosRuntime_->setCaptureSession(0);rosRuntime_->stopFollow();}
    logMessage("正在停止捕获数据...");
    writeTrackingMetadata(currentFrameContext.frame_number, "capture_stopped");

    if (!is_capturing && !collector) {
        logMessage("当前未捕获数据。");
        return;
    }
    try {
        if (isTestMode) {
            finalizeTrackingSession("playback_stopped");
            cluster->stopPlayback();
            is_capturing = false;
            if (ui->pushButton_4) ui->pushButton_4->setText("开始");
            logMessage("捕获已停止");
            ui->textEdit->append("捕获已停止\n");
            return;
        }
        if (dca1000_cfg) {
            dca1000_cfg->DCA1000_close();
        }
        if (collector) {
            const auto stats = collector->status();
            if (trackingEventStream.is_open()) {
                trackingEventStream << "CAPTURE_SUMMARY packets=" << stats.packets.packets
                    << " frames=" << stats.frames_received << " short=" << stats.packets.short_packets
                    << " gaps=" << stats.packets.discontinuities << " missing=" << stats.packets.missing_packets
                    << " out_of_order=" << stats.packets.out_of_order << " queue_dropped=" << stats.queue_dropped
                    << " stale_dropped=" << stats.stale_dropped << " send_dropped=" << stats.send_dropped << '\n';
                trackingEventStream.flush();
            }
            collector->stop_saving();
            collector->stop_collector();
            if (!collector->join(1000)) logMessage("等待正在执行的 DSP 帧完成后释放采集器。");
            delete collector;
            collector = nullptr;
        }
        if (dca1000_cfg) { delete dca1000_cfg; dca1000_cfg = nullptr; }
        // 关闭并复位点云输出文件与帧计数
        reset_signal_processing_session();
    finalizeTrackingSession("realtime_capture_stopped");
    is_capturing = false;
    logMessage("已停止捕获数据。");
    ui->textEdit->append("已停止捕获数据\n");
    // 记录停止时间
    auto stop_time = std::chrono::system_clock::now();
    auto stop_tt = std::chrono::system_clock::to_time_t(stop_time);
    auto stop_ms = std::chrono::duration_cast<std::chrono::milliseconds>(stop_time.time_since_epoch()) % 1000;
    QDateTime stop_dt = QDateTime::fromTime_t(stop_tt);
    QString timeStr = stop_dt.toString("yyyy-MM-dd hh:mm:ss") + "." + QString::number(stop_ms.count()).rightJustified(3, '0');
    logMessage(QString("数据捕获停止时间: %1").arg(timeStr));
    } catch (const std::exception& e) {
        logMessage(QString("停止捕获失败：%1").arg(e.what()));
        if (collector) { collector->stop_collector(); delete collector; collector = nullptr; }
        if (dca1000_cfg) { delete dca1000_cfg; dca1000_cfg = nullptr; }
        reset_signal_processing_session();
        finalizeTrackingSession("realtime_stop_exception");
        is_capturing = false;
    }
}

void RadarGUI::shutdownRuntime()
{
    if (shutdown_) return;
    shutdown_ = true;
    if (cooperativeTimer_) cooperativeTimer_->stop();
#if RADAR_ENABLE_CAMERA
    if(cameraAssist_)cameraAssist_->stopCamera();
#endif
    writeTrackingMetadata(currentFrameContext.frame_number, "program_exit");
    if(rosRuntime_){rosRuntime_->setCaptureSession(0);rosRuntime_->stopFollow();}
    if (updateTimer) updateTimer->stop();
    if (offlineTimer) offlineTimer->stop();
    if (offlineBinFile.is_open()) offlineBinFile.close();
    if (is_capturing || collector) stop_capture();
    if(rosRuntime_&&!rosRuntime_->shutdownBlocking())logMessage("ROS 所属进程未完全退出，请检查 ROS/底盘 日志。");
    if (collector) { delete collector; collector = nullptr; }
    finalizeTrackingSession("program_exit");
    reset_signal_processing_session();
    if (radar) { delete radar; radar = nullptr; }
    if (dca1000_cfg) { delete dca1000_cfg; dca1000_cfg = nullptr; }
    if (udpSocket) udpSocket->close();
}

void RadarGUI::exit_program()
{
    shutdownRuntime();
    QApplication::quit();
}

void RadarGUI::processUdpData()
{
    // 目标聚类模式下不处理UDP数据
    if (shutdown_ || isTestMode || isOfflineMode || !is_capturing || !collector) return;

    while (udpSocket->hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(udpSocket->pendingDatagramSize());
        udpSocket->readDatagram(datagram.data(), datagram.size());

        QString data = QString::fromUtf8(datagram);
        QStringList lines = data.split('\n');

        std::map<int, std::vector<TestPoint>> framePoints;
        std::map<int, ProcessingDiagnostics> frameDiagnostics;
        std::map<int, std::string> frameTimestamps;
        std::map<int, RadarFrameContext> contexts;
        int maxFrame = -1;

        for (const QString& line : lines) {
            if (line.isEmpty()) continue;
            QStringList parts = line.split(',');
            if (parts.size() >= 8 && parts[0].trimmed() == "FRAME") {
                RadarFrameContext f;
                f.frame_number = parts[1].toInt(); f.session_id = parts[2].toULongLong();
                f.received_time_sec = parts[3].toDouble(); f.sample_time_sec = parts[4].toDouble();
                f.time_quality = parts[5].toStdString(); f.queue_delay_ms = parts[6].toDouble();
                f.processing_completed_sec = parts[7].toDouble();
                // Local acquisition and GUI share the same monotonic clock.
                // Older 8-field producers only provide a receive-time estimate.
                f.received_monotonic_sec = parts.size()>=18 ? parts[17].toDouble() :
                    RadarTime::monotonicSeconds()-(RadarTime::wallSeconds()-f.received_time_sec);
                contexts[f.frame_number] = f;
                continue;
            }
            if (!parts.isEmpty() && parts[0].trimmed() == "META" &&
                parts.size() >= 21) {
                bool okFrame = false;
                const int frame = parts[1].trimmed().toInt(&okFrame);
                if (!okFrame) continue;
                ProcessingDiagnostics diag;
                diag.processing_time_ms = parts[3].trimmed().toFloat();
                diag.cfar_target_count = parts[4].trimmed().toInt();
                diag.pre_doa_moving_count = parts[5].trimmed().toInt();
                diag.pre_doa_static_count = parts[6].trimmed().toInt();
                diag.doa_input_count = parts[7].trimmed().toInt();
                diag.point_count = parts[8].trimmed().toInt();
                diag.ego_motion_compensation_enabled = parts[9].trimmed().toInt() != 0;
                diag.odom_valid = parts[10].trimmed().toInt() != 0;
                diag.odom_fresh = parts[11].trimmed().toInt() != 0;
                diag.odom_vx_mps = parts[12].trimmed().toFloat();
                diag.odom_wz_radps = parts[13].trimmed().toFloat();
                diag.odom_timestamp_sec = parts[14].trimmed().toDouble();
                diag.odom_age_sec = parts[15].trimmed().toDouble();
                diag.all_point_abs_velocity_median_mps = parts[16].trimmed().toFloat();
                diag.velocity_sample_count = parts[17].trimmed().toInt();
                diag.moving_ratio = parts[18].trimmed().toFloat();
                diag.compensation_warning = parts[19].trimmed().toInt() != 0;
                diag.velocity_resolution_mps = parts[20].trimmed().toFloat();
                if (parts.size() >= 22) {
                    diag.odom_seq = parts[21].trimmed().toUInt();
                }
                // New protocol appends two near-field counters before the
                // processing status.  Keep accepting the older 24-column META
                // row so GUI and acquisition binaries can be upgraded safely.
                if (parts.size() >= 26) {
                    diag.near_field_range_rejected_count =
                        parts[22].trimmed().toInt();
                    diag.near_field_burst_rejected_count =
                        parts[23].trimmed().toInt();
                    diag.processing_ok = parts[24].trimmed().toInt() != 0;
                    diag.error_message = parts[25].trimmed().toStdString();
                } else {
                    if (parts.size() >= 23) {
                        diag.processing_ok = parts[22].trimmed().toInt() != 0;
                    }
                    if (parts.size() >= 24) {
                        diag.error_message = parts[23].trimmed().toStdString();
                    }
                }
                frameDiagnostics[frame] = diag;
                frameTimestamps[frame] = parts[2].trimmed().toStdString();
                // Preserve zero-point frames so lost/reacquire timing remains
                // correct even when the DSP emits no detections.
                framePoints[frame];
                if (frame > maxFrame) maxFrame = frame;
                continue;
            }
            if (parts.size() >= 6) {  // Frame,X,Y,RelativeVelocity,AbsoluteVelocity,IsMoving,Timestamp
                bool okFrame = false;
                bool okX = false;
                bool okY = false;
                bool okRv = false;
                bool okAv = false;
                int frame = parts[0].toInt(&okFrame);
                float x = parts[1].toFloat(&okX);
                float y = parts[2].toFloat(&okY);
                float rv = parts[3].toFloat(&okRv);
                float av = parts[4].toFloat(&okAv);
                bool isMoving = parts[5].toInt() == 1;
                if (okFrame && okX && okY) {
                    TestPoint tp;
                    tp.x = x;
                    tp.y = y;
                    tp.relVel = okRv ? rv : 0.0f;
                    tp.absVel = okAv ? av : std::abs(rv);
                    tp.moving = isMoving;
                    tp.sourceFrame = parts.size() >= 8
                        ? parts[7].trimmed().toInt() : frame;
                    tp.rangeBin = parts.size() >= 9
                        ? parts[8].trimmed().toInt() : -1;
                    tp.dopplerBin = parts.size() >= 10
                        ? parts[9].trimmed().toInt() : -1;
                    tp.snr = parts.size() >= 11
                        ? parts[10].trimmed().toFloat() : 0.0f;
                    tp.doaPeakConfidence = parts.size() >= 12
                        ? parts[11].trimmed().toFloat() : 0.0f;
                    framePoints[frame].push_back(tp);
                    if (frame > maxFrame) maxFrame = frame;
                }
            }
        }

        if (framePoints.empty()) {
            continue;
        }

        for (const auto& kv : framePoints) {
            int frame = kv.first;
            const auto& points = kv.second;
            const auto context = contexts.find(frame);
            // A delayed datagram from an older session must never control a new one.
            if (context == contexts.end() || context->second.session_id != collector->session_id() ||
                frame <= lastRealtimeFrame_ || !std::isfinite(context->second.received_time_sec)) continue;
            const double age = RadarTime::wallSeconds() - context->second.received_time_sec;
            if (age < -0.05 || age > RadarEnvironment::number("RADAR_MAX_FRAME_AGE_SEC", 0.5, 0.01, 30)) continue;
            lastRealtimeFrame_ = frame;
            if (realtimeFrameBuffer.find(frame) == realtimeFrameBuffer.end()) {
                realtimeFrameOrder.push_back(frame);
            }
            RealtimeFrameData frameData;
            frameData.points = points;
            frameData.frame = context->second;
            frameData.chassis = g_gui_chassis_velocity_reader.read();
            auto diagIt = frameDiagnostics.find(frame);
            if (diagIt != frameDiagnostics.end()) {
                frameData.processing = diagIt->second;
            }
            auto tsIt = frameTimestamps.find(frame);
            if (tsIt != frameTimestamps.end()) {
                frameData.radar_timestamp = tsIt->second;
            }
            realtimeFrameBuffer[frame] = frameData;
        }

        if (lastRealtimeFrame_ != maxFrame) continue;
        while (realtimeFrameOrder.size() > static_cast<std::size_t>(trackingWindowFrames)) {
            int oldFrame = realtimeFrameOrder.front();
            realtimeFrameOrder.pop_front();
            realtimeFrameBuffer.erase(oldFrame);
        }

        if (realtimeFrameOrder.size() >= static_cast<std::size_t>(trackingWindowFrames)) {
            std::vector<TestPoint> windowPts = compensatedRealtimeWindowPoints();

            std::vector<PointCloud> trackingPoints;
            trackingPoints.reserve(windowPts.size());
            for (const auto& p : windowPts) {
                PointCloud point;
                point.x = p.x;
                point.y = p.y;
                point.relative_velocity = p.relVel;
                point.absolute_velocity = p.absVel;
                point.is_moving = p.moving;
                point.source_frame = p.sourceFrame;
                point.range_bin = p.rangeBin;
                point.doppler_bin = p.dopplerBin;
                point.snr = p.snr;
                point.doa_peak_confidence = p.doaPeakConfidence;
                trackingPoints.push_back(point);
            }

            for (auto* item : pointItems) {
                scene->removeItem(item);
                delete item;
            }
            pointItems.clear();

            updateSceneRectFromRadarConfig(false);
            const bool showGlobalPointCloud = !ui->globalPointcloudCheck || ui->globalPointcloudCheck->isChecked();
            if (showGlobalPointCloud) {
                cluster->renderWindow(windowPts, realtimeFrameOrder.front(), realtimeFrameOrder.back(), false, false);
            } else {
                cluster->stopPlayback();
            }
            const auto currentFrameIt = realtimeFrameBuffer.find(realtimeFrameOrder.back());
            if (currentFrameIt != realtimeFrameBuffer.end()) {
                currentProcessingDiagnostics = currentFrameIt->second.processing;
                currentTrackingChassis = currentFrameIt->second.chassis;
                currentTrackingOdomAgeFromReplay = false;
                currentTrackingOdomAgeSec = 0.0;
                currentRadarTimestamp = currentFrameIt->second.radar_timestamp;
                currentFrameContext = currentFrameIt->second.frame;
            }
            updateTargetTrackingCandidates(trackingPoints, realtimeFrameOrder.back());
            drawTrackingOverlay();

            if (infoText) {
                infoText->setPlainText(QString("实时聚类 - 窗口: %1-%2\n点数(%3帧总计): %4")
                    .arg(realtimeFrameOrder.front())
                    .arg(realtimeFrameOrder.back())
                    .arg(trackingWindowFrames)
                    .arg(static_cast<int>(windowPts.size())));
                infoText->show();
            }

            continue;
        }

        // 只显示最新帧的点
        auto it = framePoints.find(maxFrame);
        if (it != framePoints.end()) {
            const auto& points = it->second;

            cluster->stopPlayback();

            for (auto* item : pointItems) {
                scene->removeItem(item);
                delete item;
            }
            pointItems.clear();

            for (const auto& p : points) {
                QColor color = p.moving ? Qt::red : Qt::black;  // 移动红色，静止黑色
                QGraphicsEllipseItem* ellipse = scene->addEllipse(
                    sceneXFromMeters(p.x) - 10,
                    sceneYFromMeters(p.y) - 10,
                    20, 20, QPen(color), QBrush(color));

                pointItems.push_back(ellipse);
            }

            infoText->setPlainText(QString("当前帧: %1\n点数: %2").arg(maxFrame).arg(points.size()));

            // 保持当前雷达配置对应的 sceneRect，但不要每帧重置视图缩放。
            updateSceneRectFromRadarConfig(false);
        }
    }
}

void RadarGUI::updatePointCloudDisplay()
{
    if (!collector || !is_capturing) return;
    if (dca1000_cfg) for (const auto bits : dca1000_cfg->pollAsyncStatus()) {
        QString message = QString("DCA 异步状态 0x%1：%2")
            .arg(static_cast<unsigned>(bits), 4, 16, QLatin1Char('0'))
            .arg(QString::fromStdString(DCA1000Config::describeAsyncStatus(bits)));
        if (bits & 0x0200) message += "；LVDS 缓冲区满，请检查 DCA 状态、LVDS 配置及以太网传输。";
        if (bits & 0x0001) message += "；DCA 未收到 LVDS 数据，请检查雷达输出及连接。";
        logMessage(message);
    }
    const auto status = collector->status();
    const double now = RadarTime::monotonicSeconds();
    if (status.frames_received == 0 && now >= nextCaptureDiagnosticSec_) {
        nextCaptureDiagnosticSec_ = now + 5;
        logMessage(QString("采集诊断 %1:%2：UDP 包=%3，完整帧=%4，短包=%5，缺包=%6。")
            .arg(QString::fromStdString(connection_.host_ip)).arg(connection_.data_port)
            .arg(static_cast<qulonglong>(status.packets.packets))
            .arg(static_cast<qulonglong>(status.frames_received))
            .arg(static_cast<qulonglong>(status.packets.short_packets))
            .arg(static_cast<qulonglong>(status.packets.missing_packets)));
    }
    if (status.state != lastCaptureState_) {
        lastCaptureState_ = status.state;
        if (status.state == CaptureStatus::State::Receiving)
            logMessage("已收到完整 ADC 帧，实时处理已开始。");
        else if (status.state == CaptureStatus::State::Failed) {
            logMessage(QString("采集失败：%1").arg(QString::fromStdString(status.error)));
            stop_capture();
            return;
        }
    }
}

void RadarGUI::onClusterPlaybackFinished()
{
    is_capturing = false;
    if (ui->pushButton_4) ui->pushButton_4->setText("开始");
    logMessage("目标聚类：播放完成");
    ui->textEdit->append("目标聚类：播放完成\n");
}

void RadarGUI::appendLog(const QString& message)
{
    logMessage(message);
}

void RadarGUI::resetRealtimeClusterBuffer()
{
#if RADAR_ENABLE_CAMERA
    if(cameraAssist_)cameraAssist_->resetRadar();
#endif
    realtimeFrameBuffer.clear();
    realtimeFrameOrder.clear();
    offlineTrackingWindow.clear();
    offlineOdomReplay.clear();
    resetTrackingRuntimeStats();
}

void RadarGUI::resetTrackingRuntimeStats()
{
    static std::int64_t lastSessionId = 0;
    finalizeTrackingSession("session_reset");
    trackingStats = TrackingRuntimeStats();
    const std::int64_t wallClockId = static_cast<std::int64_t>(
        QDateTime::currentMSecsSinceEpoch());
    trackingStats.session_id = std::max(wallClockId, lastSessionId + 1);
    lastSessionId = trackingStats.session_id;
    trackingStats.source = isOfflineTrackingMode ? "offline" : "realtime";
    currentDiagnosticCsvPath.clear();
    currentEventLogPath.clear();
    currentProcessingDiagnostics = ProcessingDiagnostics();
    currentTrackingChassis = ChassisVelocitySample();
    currentTrackingOdomAgeFromReplay = false;
    currentTrackingOdomAgeSec = 0.0;
    currentRadarTimestamp.clear();
    currentFrameContext = RadarFrameContext();
    lastRealtimeFrame_ = -1;
    targetTableCache.clear();
    previewTargetUid = 0;
    previewTargetDisplayId = -1;
    updateTargetSelectionControls();
    resetLockedTrajectoryHistory();
}

std::vector<PointCloud> RadarGUI::aggregateOfflineTrackingWindow() const
{
    return RadarMotion::aggregate(offlineTrackingWindow, realtimeFramePeriodSeconds(),
        [this](int frame) -> const ChassisVelocitySample* {
            const auto* motion = offlineOdomReplay.frame(frame);
            return motion && motion->has_tracking_sample ? &motion->tracking_sample : nullptr;
        });
}

void RadarGUI::recordTrackingRuntimeStats(std::size_t windowPointCount,
                                          double trackerMs,
                                          int frameNumber)
{
    auto& stats = trackingStats;
    if (stats.session_id == 0 || stats.finalized) {
        resetTrackingRuntimeStats();
    }
    if (stats.source.empty()) {
        stats.source = isOfflineTrackingMode ? "offline" : "realtime";
    }
    const auto& locked = targetTracker.lockedTarget();
    stats.updates++;
    stats.points_sum += static_cast<double>(windowPointCount);
    stats.candidates_sum += static_cast<double>(targetTracker.candidates().size());
    stats.clusters_sum += static_cast<double>(targetTracker.clusterDebugInfo().size());
    stats.active_tracks_sum += static_cast<double>(targetTracker.activeTrackCount());
    stats.raw_track_births = targetTracker.totalTrackBirths();
    stats.exposed_id_births = targetTracker.confirmedTrackBirths();
    stats.max_display_id = targetTracker.maxDisplayIdSeen();
    stats.tracker_times_ms.push_back(trackerMs);
    if (trackerMs > 100.0) stats.deadline_misses++;

    if (locked.valid) {
        stats.locked_updates++;
        // A reacquire-pending observation is visible in the detailed audit log,
        // but it has deliberately not been accepted by the main Kalman yet.
        const bool acceptedMeasurement = locked.has_measurement &&
            locked.control_state !=
                TargetTracking::FollowControlState::ReacquireConfirming;
        if (acceptedMeasurement) {
            stats.measurement_hits++;
            if (stats.have_last_measurement) {
                const double dx = locked.measured_x - stats.last_measured_x;
                const double dy = locked.measured_y - stats.last_measured_y;
                stats.measured_step_sq_sum += dx * dx + dy * dy;
                stats.measured_step_count++;
            }
            stats.last_measured_x = locked.measured_x;
            stats.last_measured_y = locked.measured_y;
            stats.have_last_measurement = true;
        }
        if (locked.control_state == TargetTracking::FollowControlState::LimitedFollow) {
            stats.predicted_updates++;
        } else if (locked.control_state == TargetTracking::FollowControlState::StopAndReacquire) {
            stats.stopped_updates++;
        } else if (locked.control_state == TargetTracking::FollowControlState::ReacquireConfirming) {
            stats.reacquire_updates++;
        }
    } else {
        stats.have_last_measurement = false;
    }

    appendTrackingDiagnosticRow(frameNumber, windowPointCount, trackerMs);
    if (stats.updates % 100 == 0) {
        writeTrackingStatsSnapshot(false);
    }
}

void RadarGUI::writeTrackingStatsSnapshot(bool finalSnapshot,
                                          const std::string& endReason)
{
    auto& stats = trackingStats;
    if (stats.updates == 0 || stats.tracker_times_ms.empty()) return;
    if (finalSnapshot && stats.last_written_updates == stats.updates) {
        if (trackingDiagnosticStream.is_open()) trackingDiagnosticStream.flush();
        return;
    }

    std::vector<double> sorted = stats.tracker_times_ms;
    std::sort(sorted.begin(), sorted.end());
    const double timeSum = std::accumulate(sorted.begin(), sorted.end(), 0.0);
    const double timeMean = timeSum / sorted.size();
    const std::size_t p95Index = std::min(sorted.size() - 1,
        static_cast<std::size_t>(std::ceil(0.95 * sorted.size())) - 1);
    const double timeP95 = sorted[p95Index];
    const double timeMax = sorted.back();
    const double n = static_cast<double>(stats.updates);
    const double hitRate = stats.locked_updates > 0
        ? static_cast<double>(stats.measurement_hits) / stats.locked_updates : 0.0;
    const double measuredStepRms = stats.measured_step_count > 0
        ? std::sqrt(stats.measured_step_sq_sum / stats.measured_step_count) : 0.0;
    const std::string source = stats.source.empty()
        ? (isOfflineTrackingMode ? "offline" : "realtime")
        : stats.source;

    std::cout << std::fixed << std::setprecision(3)
              << (finalSnapshot ? "[TRACK-STATS-FINAL] source="
                                : "[TRACK-STATS] source=")
              << source
              << " session=" << stats.session_id
              << " window=" << trackingWindowFrames
              << " id_mode=" << targetTracker.trackIdModeName()
              << " updates=" << stats.updates
              << " points_avg=" << stats.points_sum / n
              << " clusters_avg=" << stats.clusters_sum / n
              << " candidates_avg=" << stats.candidates_sum / n
              << " active_tracks_avg=" << stats.active_tracks_sum / n
              << " measurement_hit_rate=" << hitRate
              << " predicted=" << stats.predicted_updates
              << " stopped=" << stats.stopped_updates
              << " reacquire=" << stats.reacquire_updates
              << " raw_track_births=" << stats.raw_track_births
              << " exposed_id_births=" << stats.exposed_id_births
              << " max_display_id=" << stats.max_display_id
              << " measured_step_rms_m=" << measuredStepRms
              << " e2e_ms_mean=" << timeMean
              << " e2e_ms_p95=" << timeP95
              << " e2e_ms_max=" << timeMax
              << " deadline_miss_gt100ms=" << stats.deadline_misses;
    if (finalSnapshot) {
        std::cout << " end_reason=" << endReason;
    }
    std::cout
              << std::endl;

    const char* csvPath = "tracking_window_stats.csv";
    const bool writeHeader = !std::ifstream(csvPath).good();
    std::ofstream csv(csvPath, std::ios::app);
    if (csv) {
        if (writeHeader) {
            csv << "session_id,source,window_frames,id_mode,updates,points_avg,clusters_avg,candidates_avg,"
                   "active_tracks_avg,measurement_hit_rate,predicted,stopped,reacquire,raw_track_births,"
                   "exposed_id_births,max_display_id,measured_step_rms_m,e2e_ms_mean,e2e_ms_p95,e2e_ms_max,deadline_miss_gt100ms\n";
        }
        csv << stats.session_id << ',' << source << ',' << trackingWindowFrames << ','
            << targetTracker.trackIdModeName() << ','
            << stats.updates << ',' << stats.points_sum / n << ',' << stats.clusters_sum / n << ','
            << stats.candidates_sum / n << ',' << stats.active_tracks_sum / n << ',' << hitRate << ','
            << stats.predicted_updates << ',' << stats.stopped_updates << ',' << stats.reacquire_updates << ','
            << stats.raw_track_births << ',' << stats.exposed_id_births << ','
            << stats.max_display_id << ','
            << measuredStepRms << ',' << timeMean << ',' << timeP95 << ',' << timeMax << ','
            << stats.deadline_misses << '\n';
        csv.flush();
        stats.last_written_updates = stats.updates;
    }
}

void RadarGUI::finalizeTrackingSession(const std::string& endReason)
{
    if (trackingStats.session_id == 0 || trackingStats.finalized) {
        closeTrackingDiagnosticLog();
        return;
    }
    writeTrackingStatsSnapshot(true, endReason);
    if (trackingEventStream.is_open()) {
        trackingEventStream << "SESSION_END reason=" << endReason
                            << " updates=" << trackingStats.updates << '\n';
        trackingEventStream.flush();
    }
    trackingStats.finalized = true;
    closeTrackingDiagnosticLog();
}

std::string RadarGUI::csvEscape(const std::string& value)
{
    if (value.find_first_of(",\"\r\n") == std::string::npos) return value;
    std::string escaped = "\"";
    for (char c : value) {
        if (c == '\"') escaped += "\"\"";
        else escaped.push_back(c);
    }
    escaped.push_back('\"');
    return escaped;
}

bool RadarGUI::ensureTrackingDiagnosticLogOpen()
{
    if (trackingDiagnosticStream.is_open()) return true;
    QDir rootDir(QDir::currentPath());
    if (!rootDir.exists("tracking_diagnostics") &&
        !rootDir.mkpath("tracking_diagnostics")) {
        logMessage("诊断日志创建失败：无法创建 tracking_diagnostics 文件夹");
        return false;
    }
    QDir diagnosticDir(rootDir.filePath("tracking_diagnostics"));

    const QString source = QString::fromStdString(
        trackingStats.source.empty()
            ? (isOfflineTrackingMode ? "offline" : "realtime")
            : trackingStats.source);
    const QString stamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss_zzz");
    const QString base = QString("tracking_diag_%1_%2_%3")
        .arg(source)
        .arg(stamp)
        .arg(static_cast<qlonglong>(trackingStats.session_id));
    QString uniqueBase = base;
    int suffix = 1;
    while (QFileInfo::exists(diagnosticDir.filePath(uniqueBase + ".csv")) ||
           QFileInfo::exists(diagnosticDir.filePath(uniqueBase + ".log"))) {
        uniqueBase = QString("%1_%2").arg(base).arg(suffix++);
    }
    currentDiagnosticCsvPath = diagnosticDir.filePath(uniqueBase + ".csv");
    currentEventLogPath = diagnosticDir.filePath(uniqueBase + ".log");
    trackingDiagnosticStream.open(currentDiagnosticCsvPath.toStdString(),
                                  std::ios::out | std::ios::trunc);
    if (!trackingDiagnosticStream) {
        logMessage(QString("诊断日志创建失败：%1").arg(currentDiagnosticCsvPath));
        currentDiagnosticCsvPath.clear();
        return false;
    }

    trackingDiagnosticStream
        << "session_id,source,frame,wall_timestamp,radar_timestamp,window_frames,id_mode,"
           "input_path,motion_replay_csv,radar_config_file,frame_period_ms,kalman_dt_s,cluster_eps,min_cluster_points,"
           "min_range_m,near_field_filter_range_m,trajectory_association_enabled,"
           "candidate_track_unconfirmed_max_lost_frames,"
           "candidate_track_confirmed_max_lost_frames,locked_track_max_lost_frames,"
           "trajectory_unassigned_cost,trajectory_point_weight,trajectory_size_weight,"
           "trajectory_doppler_weight,trajectory_point_ratio_min,trajectory_point_ratio_max,"
           "candidate_track_gate_chi2,candidate_track_max_gate_m,"
           "reject_ambiguous_locked_uid,candidate_track_ambiguity_cost_margin,"
           "new_uid_reacquire_min_points,new_uid_reacquire_confirm_frames,"
           "new_uid_reacquire_window_frames,"
           "new_uid_reacquire_max_distance_m,max_measurement_speed_mps,kalman_gate_chi2,"
           "roi_gate_chi2,kalman_gate_radius_m,roi_radius_m,reacquire_confirm_frames,"
           "reacquire_window_frames,max_reacquire_hypotheses,reacquire_min_score_margin,"
           "prediction_decay_start_lost,prediction_stop_lost,prediction_decay_per_frame,"
           "prediction_max_speed_mps,prediction_max_step_m,"
           "manual_lock_confirm_frames,manual_lock_confirm_max_missed_frames,"
           "manual_lock_max_initial_speed_mps,"
           "manual_unlock_lost_frames,limited_follow_frames,"
           "window_points,clusters,candidates,active_tracks,trajectory_method,"
           "trajectory_tracks,trajectory_detections,trajectory_valid_pairs,trajectory_matches,"
           "trajectory_unassigned_tracks,trajectory_unassigned_detections,"
           "trajectory_locked_ambiguous,locked_uid_continuity_compromised,"
           "trajectory_locked_best_detection,"
           "trajectory_locked_best_cost,trajectory_locked_second_cost,trajectory_total_cost,"
           "lock_valid,locked_display_id,"
           "matched_candidate_id,matched_internal_uid,has_measurement,control_state,lost_frames,"
           "stable_frames,reacquire_hits,manual_lock_pending,manual_lock_confirm_misses,"
           "confidence,filtered_x_m,filtered_y_m,filtered_vx_mps,"
           "filtered_vy_mps,filtered_range_m,filtered_angle_deg,measured_x_m,measured_y_m,"
           "measured_range_m,measured_angle_deg,association_source,measurement_rejected,"
           "reject_reason,predicted_x_m,predicted_y_m,innovation_x_m,innovation_y_m,nis,"
           "implied_speed_mps,considered_candidates,gate_candidate_count,candidate_display_id,"
           "candidate_internal_uid,candidate_points,candidate_width_m,candidate_depth_m,"
           "candidate_avg_abs_velocity_mps,candidate_moving_ratio,candidate_score,"
           "candidate_background_score,candidate_current_frame_points,"
           "candidate_current_frame_ratio,candidate_dominant_range_bin,"
           "candidate_dominant_doppler_bin,candidate_unique_rd_cells,"
           "candidate_average_snr,candidate_average_doa_peak_confidence,"
           "pending_internal_uid,pending_hits,pending_window_frames,"
           "reacquire_hypothesis_count,reacquire_best_score,reacquire_second_score,"
           "reacquire_score_margin,dsp_processing_ok,"
           "dsp_error,dsp_processing_ms,"
           "tracker_ms,cfar_targets,pre_doa_moving,pre_doa_static,doa_inputs,current_frame_points,"
           "near_field_range_rejected,near_field_burst_rejected,"
           "velocity_resolution_mps,ego_compensation_enabled,dsp_odom_valid,dsp_odom_fresh,"
           "dsp_odom_vx_mps,dsp_odom_wz_radps,dsp_odom_timestamp_sec,dsp_odom_age_sec,dsp_odom_seq,"
           "all_point_abs_velocity_median_mps,velocity_samples,dsp_moving_ratio,compensation_warn,"
           "tracking_odom_valid,tracking_odom_fresh,tracking_odom_vx_mps,tracking_odom_wz_radps,"
           "tracking_odom_timestamp_sec,tracking_odom_age_sec,tracking_odom_seq,"
           "tracking_ego_enabled,tracking_ego_odom_valid,tracking_ego_applied,tracking_ego_dt_s,"
           "tracking_ego_dx_m,tracking_ego_dy_m,tracking_ego_dyaw_rad,"
           "tracking_ego_chassis_vx_mps,tracking_ego_chassis_wz_radps,"
           "candidate_snapshot,cluster_snapshot,use_current_frame_extent,short_gap_same_uid_max_lost,"
           "dropout_gate_expansion,dropout_gate_max_extra_m,ego_relative_dropout_decay,"
           "hold_motion_during_reacquire,candidate_gate_snapshot,frame_session_id,received_time_sec,sample_time_sec,"
           "time_quality,queue_delay_ms,processing_completed_sec,last_measurement_time_sec,selection_epoch,position_variance_x,position_variance_y,adc_packets,adc_short_packets,adc_missing_packets,"
           "adc_out_of_order,adc_discontinuities,capture_frames,queue_dropped,stale_dropped,send_dropped,queue_depth\n";
    trackingDiagnosticStream.flush();

    trackingEventStream.open(currentEventLogPath.toStdString(),
                             std::ios::out | std::ios::trunc);
    if (trackingEventStream) {
        const auto& cfg = targetTracker.config();
        const auto framePeriodIt = radar_config.find("framePeriodicity");
        const double framePeriodMs = framePeriodIt == radar_config.end()
            ? 0.0 : framePeriodIt->second;
        trackingEventStream << "CONNECTION host=" << connection_.host_ip << ':' << connection_.data_port
            << " fpga=" << connection_.fpga_ip << ':' << connection_.fpga_port
            << " result=" << connection_.result_ip << ':' << connection_.result_port
            << " mount_x=" << RadarMount::xOffsetM() << " mount_y=" << RadarMount::yOffsetM()
            << " yaw_sign=" << RadarMount::yawSign() << '\n';
        trackingEventStream << "SESSION_START id=" << trackingStats.session_id
                            << " source=" << trackingStats.source
                            << " input=" << trackingStats.input_path
                            << " motion_replay_csv="
                            << (offlineOdomReplay.loaded()
                                    ? offlineOdomReplay.sourcePath() : "none")
                            << " radar_config=" << config_file << '\n'
                            << "CONFIG point_window_frames=" << trackingWindowFrames
                            << " id_mode=" << targetTracker.trackIdModeName()
                            << " frame_period_ms=" << framePeriodMs
                            << " kalman_dt_s=" << cfg.kalman_dt_s
                            << " cluster_eps=" << cfg.cluster_eps
                            << " min_cluster_points=" << cfg.min_cluster_points
                            << " current_centroid_min_points="
                            << cfg.current_frame_centroid_min_points
                            << " current_single_weight="
                            << cfg.current_frame_single_point_weight
                            << " previous_weight="
                            << cfg.previous_frame_centroid_weight
                            << " second_previous_weight="
                            << cfg.second_previous_frame_centroid_weight
                            << " require_current_support="
                            << (cfg.require_current_frame_support ? 1 : 0)
                            << " min_range_m=" << cfg.min_range_m
                            << " near_field_filter_range_m="
                            << cfg.near_field_filter_range_m
                             << " trajectory_association="
                             << (RadarRuntimeConfig::kUseHungarianTrajectoryAssociation
                                     ? "hungarian" : "greedy")
                             << " candidate_track_unconfirmed_max_lost_frames="
                             << cfg.candidate_track_unconfirmed_max_lost_frames
                             << " candidate_track_confirmed_max_lost_frames="
                             << cfg.candidate_track_confirmed_max_lost_frames
                             << " locked_track_max_lost_frames="
                             << cfg.max_lost_frames
                            << " trajectory_unassigned_cost="
                            << cfg.trajectory_unassigned_cost
                            << " trajectory_point_weight="
                            << cfg.trajectory_point_count_cost_weight
                            << " trajectory_size_weight="
                            << cfg.trajectory_size_cost_weight
                            << " trajectory_doppler_weight="
                            << cfg.trajectory_signed_doppler_cost_weight
                            << " trajectory_point_ratio_min="
                            << cfg.trajectory_point_count_ratio_min
                            << " trajectory_point_ratio_max="
                            << cfg.trajectory_point_count_ratio_max
                            << " candidate_track_gate_chi2="
                            << cfg.candidate_track_gate_chi2
                            << " candidate_track_max_gate_m="
                            << cfg.candidate_track_max_gate_m
                            << " reject_ambiguous_locked_uid="
                            << (cfg.reject_ambiguous_locked_uid_assignment ? 1 : 0)
                            << " candidate_track_ambiguity_cost_margin="
                            << cfg.candidate_track_locked_ambiguity_cost_margin
                            << " new_uid_reacquire_min_points="
                            << cfg.new_uid_reacquire_min_points
                            << " new_uid_reacquire_confirm_frames="
                            << cfg.new_uid_reacquire_confirm_frames
                            << " new_uid_reacquire_window_frames="
                            << cfg.new_uid_reacquire_window_frames
                            << " new_uid_reacquire_max_distance_m="
                            << cfg.new_uid_reacquire_max_distance_m
                            << " max_measurement_speed_mps=" << cfg.max_measurement_speed_mps
                            << " kalman_gate_chi2=" << cfg.kalman_gate_chi2
                            << " roi_gate_chi2=" << cfg.roi_gate_chi2
                            << " kalman_gate_radius_m=" << cfg.kalman_gate_radius_m
                            << " roi_radius_m=" << cfg.roi_radius_m
                            << " reacquire_confirm_frames=" << cfg.reacquire_confirm_frames
                            << " reacquire_window_frames="
                            << cfg.reacquire_window_frames
                            << " max_reacquire_hypotheses="
                            << cfg.max_reacquire_hypotheses
                            << " reacquire_min_score_margin="
                            << cfg.reacquire_min_score_margin
                            << " prediction_decay_start_lost="
                            << cfg.prediction_velocity_decay_start_lost_frames
                            << " prediction_stop_lost="
                            << cfg.prediction_velocity_stop_lost_frames
                            << " prediction_decay_per_frame="
                            << cfg.prediction_velocity_decay_per_frame
                            << " prediction_max_speed_mps="
                            << cfg.prediction_max_speed_mps
                            << " prediction_max_step_m="
                            << cfg.prediction_max_step_m
                            << " manual_lock_confirm_frames="
                            << cfg.manual_lock_confirm_frames
                            << " manual_lock_confirm_max_missed_frames="
                            << cfg.manual_lock_confirm_max_missed_frames
                            << " manual_lock_max_initial_speed_mps="
                            << cfg.manual_lock_max_initial_speed_mps
                            << " manual_unlock_lost_frames=" << cfg.manual_unlock_lost_frames
                            << '\n';
        trackingEventStream.flush();
    } else {
        currentEventLogPath.clear();
    }
    logMessage(QString("诊断日志：%1").arg(currentDiagnosticCsvPath));
    return true;
}

std::string RadarGUI::candidateSnapshotForLog() const
{
    std::vector<TargetTracking::CandidateTarget> candidates = targetTracker.candidates();
    std::sort(candidates.begin(), candidates.end(),
              [](const auto& a, const auto& b) { return a.id < b.id; });
    std::ostringstream out;
    out << std::fixed << std::setprecision(3);
    for (std::size_t i = 0; i < candidates.size(); ++i) {
        if (i) out << ';';
        const auto& c = candidates[i];
        out << "id=" << c.id << "|uid=" << c.internal_uid
            << "|x=" << c.x << "|y=" << c.y
            << "|r=" << c.range_m << "|a=" << c.angle_deg
            << "|v=" << c.average_abs_velocity
            << "|signed_v=" << c.median_signed_velocity
            << "|mr=" << c.moving_ratio << "|pts=" << c.point_count
            << "|src=" << c.oldest_source_frame << ':'
            << c.newest_source_frame
            << "|current_pts=" << c.current_frame_point_count
            << "|current_ratio=" << c.current_frame_point_ratio
            << "|position_mode=" << candidatePositionModeToken(c.position_mode)
            << "|rb=" << c.dominant_range_bin
            << "|db=" << c.dominant_doppler_bin
            << "|rd_unique=" << c.unique_range_doppler_cells
            << "|snr=" << c.average_snr
            << "|doa_conf=" << c.average_doa_peak_confidence
            << "|w=" << c.width_m << "|d=" << c.depth_m
            << "|score=" << c.score << "|bg=" << c.background_score;
    }
    return out.str();
}

std::string RadarGUI::clusterSnapshotForLog() const
{
    const auto& clusters = targetTracker.clusterDebugInfo();
    std::ostringstream out;
    out << std::fixed << std::setprecision(3);
    for (std::size_t i = 0; i < clusters.size(); ++i) {
        if (i) out << ';';
        const auto& c = clusters[i];
        std::map<std::pair<int, int>, int> rdCells;
        std::map<int, int> sourceFrames;
        for (const auto& point : c.points) {
            if (point.range_bin >= 0 && point.doppler_bin >= 0) {
                rdCells[{point.range_bin, point.doppler_bin}]++;
            }
            if (point.source_frame >= 0) {
                sourceFrames[point.source_frame]++;
            }
        }
        out << "c=" << c.cluster_id << "|person=" << (c.person_like ? 1 : 0)
            << "|x=" << c.x << "|y=" << c.y
            << "|r=" << c.range_m << "|a=" << c.angle_deg
            << "|v=" << c.average_abs_velocity
            << "|signed_v=" << c.median_signed_velocity
            << "|mr=" << c.moving_ratio << "|pts=" << c.point_count
            << "|src=" << c.oldest_source_frame << ':'
            << c.newest_source_frame
            << "|current_pts=" << c.current_frame_point_count
            << "|current_ratio=" << c.current_frame_point_ratio
            << "|position_mode=" << candidatePositionModeToken(c.position_mode)
            << "|current_support=" << (c.has_current_frame_support ? 1 : 0)
            << "|rb=" << c.dominant_range_bin
            << "|db=" << c.dominant_doppler_bin
            << "|rd_unique=" << c.unique_range_doppler_cells
            << "|snr=" << c.average_snr
            << "|doa_conf=" << c.average_doa_peak_confidence
            << "|w=" << c.width_m << "|d=" << c.depth_m
            << "|score=" << c.score << "|bg=" << c.background_score
            << "|rd_cells=";
        bool firstCell = true;
        for (const auto& cell : rdCells) {
            if (!firstCell) out << '/';
            firstCell = false;
            out << cell.first.first << ':' << cell.first.second
                << ':' << cell.second;
        }
        out << "|frame_hist=";
        bool firstFrame = true;
        for (const auto& source : sourceFrames) {
            if (!firstFrame) out << '/';
            firstFrame = false;
            out << source.first << ':' << source.second;
        }
    }
    return out.str();
}

void RadarGUI::appendTrackingDiagnosticRow(int frameNumber,
                                           std::size_t windowPointCount,
                                           double trackerMs)
{
    if (!ensureTrackingDiagnosticLogOpen()) return;
    const auto& locked = targetTracker.lockedTarget();
    const auto& assoc = targetTracker.associationDiagnostics();
    const auto& trajectory = targetTracker.trajectoryAssociationDiagnostics();
    const auto& trackingEgo = targetTracker.egoMotionDiagnostics();
    const auto& dsp = currentProcessingDiagnostics;
    const auto& cfg = targetTracker.config();
    const auto framePeriodIt = radar_config.find("framePeriodicity");
    const double framePeriodMs = framePeriodIt == radar_config.end()
        ? 0.0 : framePeriodIt->second;
    const double trackingOdomAge = currentTrackingOdomAgeFromReplay
        ? currentTrackingOdomAgeSec
        : (currentTrackingChassis.has_timestamp
               ? std::abs(QDateTime::currentMSecsSinceEpoch() / 1000.0 -
                          currentTrackingChassis.timestamp_sec)
               : 0.0);
    const std::string wallTimestamp =
        QDateTime::currentDateTime().toString("yyyy-MM-ddThh:mm:ss.zzz").toStdString();
    const std::string state = locked.valid
        ? followControlStateToken(locked.control_state).toStdString()
        : "UNLOCKED";

    auto numberOrNan = [](bool valid, float value) -> std::string {
        if (!valid) return "nan";
        std::ostringstream out;
        out << std::fixed << std::setprecision(6) << value;
        return out.str();
    };

    trackingDiagnosticStream << std::fixed << std::setprecision(6)
        << trackingStats.session_id << ',' << trackingStats.source << ',' << frameNumber << ','
        << csvEscape(wallTimestamp) << ',' << csvEscape(currentRadarTimestamp) << ','
        << trackingWindowFrames << ',' << targetTracker.trackIdModeName() << ','
        << csvEscape(trackingStats.input_path) << ','
        << csvEscape(offlineOdomReplay.loaded()
                         ? offlineOdomReplay.sourcePath() : std::string()) << ','
        << csvEscape(config_file) << ','
        << framePeriodMs << ','
        << cfg.kalman_dt_s << ',' << cfg.cluster_eps << ',' << cfg.min_cluster_points << ','
        << cfg.min_range_m << ',' << cfg.near_field_filter_range_m << ','
        << (RadarRuntimeConfig::kUseHungarianTrajectoryAssociation ? 1 : 0) << ','
        << cfg.candidate_track_unconfirmed_max_lost_frames << ','
        << cfg.candidate_track_confirmed_max_lost_frames << ','
        << cfg.max_lost_frames << ','
        << cfg.trajectory_unassigned_cost << ','
        << cfg.trajectory_point_count_cost_weight << ','
        << cfg.trajectory_size_cost_weight << ','
        << cfg.trajectory_signed_doppler_cost_weight << ','
        << cfg.trajectory_point_count_ratio_min << ','
        << cfg.trajectory_point_count_ratio_max << ','
        << cfg.candidate_track_gate_chi2 << ','
        << cfg.candidate_track_max_gate_m << ','
        << (cfg.reject_ambiguous_locked_uid_assignment ? 1 : 0) << ','
        << cfg.candidate_track_locked_ambiguity_cost_margin << ','
        << cfg.new_uid_reacquire_min_points << ','
        << cfg.new_uid_reacquire_confirm_frames << ','
        << cfg.new_uid_reacquire_window_frames << ','
        << cfg.new_uid_reacquire_max_distance_m << ','
        << cfg.max_measurement_speed_mps << ',' << cfg.kalman_gate_chi2 << ','
        << cfg.roi_gate_chi2 << ',' << cfg.kalman_gate_radius_m << ','
        << cfg.roi_radius_m << ',' << cfg.reacquire_confirm_frames << ','
        << cfg.reacquire_window_frames << ','
        << cfg.max_reacquire_hypotheses << ','
        << cfg.reacquire_min_score_margin << ','
        << cfg.prediction_velocity_decay_start_lost_frames << ','
        << cfg.prediction_velocity_stop_lost_frames << ','
        << cfg.prediction_velocity_decay_per_frame << ','
        << cfg.prediction_max_speed_mps << ','
        << cfg.prediction_max_step_m << ','
        << cfg.manual_lock_confirm_frames << ','
        << cfg.manual_lock_confirm_max_missed_frames << ','
        << cfg.manual_lock_max_initial_speed_mps << ','
        << cfg.manual_unlock_lost_frames << ',' << cfg.lost_limited_follow_frames << ','
        << windowPointCount << ',' << targetTracker.clusterDebugInfo().size() << ','
        << targetTracker.candidates().size() << ',' << targetTracker.activeTrackCount() << ','
        << csvEscape(trajectory.method) << ',' << trajectory.track_count << ','
        << trajectory.detection_count << ',' << trajectory.valid_pair_count << ','
        << trajectory.matched_pair_count << ',' << trajectory.unassigned_track_count << ','
        << trajectory.unassigned_detection_count << ','
        << (trajectory.locked_assignment_ambiguous ? 1 : 0) << ','
        << (targetTracker.lockedUidContinuityCompromised() ? 1 : 0) << ','
        << trajectory.locked_best_detection_index << ','
        << trajectory.locked_best_cost << ',' << trajectory.locked_second_cost << ','
        << trajectory.total_assignment_cost << ','
        << (locked.valid ? 1 : 0) << ',' << (locked.valid ? locked.display_id : -1) << ','
        << (locked.valid ? locked.matched_candidate_id : -1) << ','
        << (locked.valid ? locked.matched_internal_uid : 0) << ','
        << (locked.valid && locked.has_measurement ? 1 : 0) << ',' << state << ','
        << (locked.valid ? locked.lost_frames : 0) << ','
        << (locked.valid ? locked.stable_frames : 0) << ','
        << (locked.valid ? locked.reacquire_hits : 0) << ','
        << (targetTracker.manualLockConfirmationPending() ? 1 : 0) << ','
        << targetTracker.manualLockConfirmationMisses() << ','
        << (locked.valid ? locked.confidence : 0.0f) << ','
        << numberOrNan(locked.valid, locked.x) << ','
        << numberOrNan(locked.valid, locked.y) << ','
        << numberOrNan(locked.valid, locked.vx) << ','
        << numberOrNan(locked.valid, locked.vy) << ','
        << numberOrNan(locked.valid, locked.range_m) << ','
        << numberOrNan(locked.valid, locked.angle_deg) << ','
        << numberOrNan(locked.valid && locked.has_measurement, locked.measured_x) << ','
        << numberOrNan(locked.valid && locked.has_measurement, locked.measured_y) << ','
        << numberOrNan(locked.valid && locked.has_measurement, locked.measured_range_m) << ','
        << numberOrNan(locked.valid && locked.has_measurement, locked.measured_angle_deg) << ','
        << csvEscape(assoc.source) << ',' << (assoc.measurement_rejected ? 1 : 0) << ','
        << csvEscape(assoc.reject_reason) << ',' << assoc.predicted_x << ',' << assoc.predicted_y << ','
        << assoc.innovation_x << ',' << assoc.innovation_y << ',' << assoc.nis << ','
        << assoc.implied_speed_mps << ',' << assoc.considered_candidates << ','
        << assoc.gate_candidate_count << ',' << assoc.candidate_display_id << ','
        << assoc.candidate_internal_uid << ',' << assoc.candidate_point_count << ','
        << assoc.candidate_width_m << ',' << assoc.candidate_depth_m << ','
        << assoc.candidate_average_abs_velocity << ',' << assoc.candidate_moving_ratio << ','
        << assoc.candidate_score << ',' << assoc.candidate_background_score << ','
        << assoc.candidate_current_frame_points << ','
        << assoc.candidate_current_frame_ratio << ','
        << assoc.candidate_dominant_range_bin << ','
        << assoc.candidate_dominant_doppler_bin << ','
        << assoc.candidate_unique_range_doppler_cells << ','
        << assoc.candidate_average_snr << ','
        << assoc.candidate_average_doa_peak_confidence << ','
        << assoc.pending_internal_uid << ',' << assoc.pending_hits << ','
        << assoc.pending_window_frames << ','
        << assoc.reacquire_hypothesis_count << ','
        << assoc.reacquire_best_score << ','
        << assoc.reacquire_second_score << ','
        << assoc.reacquire_score_margin << ','
        << (dsp.processing_ok ? 1 : 0) << ',' << csvEscape(dsp.error_message) << ','
        << dsp.processing_time_ms << ',' << trackerMs << ',' << dsp.cfar_target_count << ','
        << dsp.pre_doa_moving_count << ',' << dsp.pre_doa_static_count << ','
        << dsp.doa_input_count << ',' << dsp.point_count << ','
        << dsp.near_field_range_rejected_count << ','
        << dsp.near_field_burst_rejected_count << ','
        << dsp.velocity_resolution_mps << ','
        << (dsp.ego_motion_compensation_enabled ? 1 : 0) << ','
        << (dsp.odom_valid ? 1 : 0) << ',' << (dsp.odom_fresh ? 1 : 0) << ','
        << dsp.odom_vx_mps << ',' << dsp.odom_wz_radps << ','
        << dsp.odom_timestamp_sec << ',' << dsp.odom_age_sec << ',' << dsp.odom_seq << ','
        << dsp.all_point_abs_velocity_median_mps << ',' << dsp.velocity_sample_count << ','
        << dsp.moving_ratio << ',' << (dsp.compensation_warning ? 1 : 0) << ','
        << (currentTrackingChassis.valid ? 1 : 0) << ','
        << (currentTrackingChassis.fresh ? 1 : 0) << ','
        << currentTrackingChassis.vx_mps << ',' << currentTrackingChassis.wz_radps << ','
        << currentTrackingChassis.timestamp_sec << ',' << trackingOdomAge << ','
        << currentTrackingChassis.seq << ','
        << (trackingEgo.enabled ? 1 : 0) << ','
        << (trackingEgo.odom_valid ? 1 : 0) << ','
        << (trackingEgo.applied ? 1 : 0) << ','
        << trackingEgo.dt_s << ',' << trackingEgo.radar_dx_m << ','
        << trackingEgo.radar_dy_m << ',' << trackingEgo.radar_dyaw_rad << ','
        << trackingEgo.chassis_vx_mps << ',' << trackingEgo.chassis_wz_radps << ','
        << csvEscape(candidateSnapshotForLog()) << ','
        << csvEscape(clusterSnapshotForLog()) << ','
        << cfg.use_current_frame_extent << ',' << cfg.short_gap_same_uid_max_lost_frames << ','
        << cfg.enable_dropout_gate_expansion << ',' << cfg.dropout_gate_max_extra_m << ','
        << cfg.ego_relative_dropout_decay << ',' << cfg.hold_motion_during_reacquire << ',';
    std::ostringstream gates;
    gates << std::fixed << std::setprecision(5);
    for(const auto& gate:targetTracker.candidateGateAudit()) {
        gates << "uid=" << gate.candidate.internal_uid << "|d=" << gate.distance_m << "|nis=" << gate.nis
              << "|normal=" << gate.normal_reason << "|roi=" << gate.roi_reason
              << "|reacquire=" << gate.reacquire_reason << ';';
    }
    const auto snapshot = targetTracker.output();
    trackingDiagnosticStream << csvEscape(gates.str()) << ',' << std::setprecision(17)
        << currentFrameContext.session_id << ',' << currentFrameContext.received_time_sec << ','
        << currentFrameContext.sample_time_sec << ',' << currentFrameContext.time_quality << ','
        << currentFrameContext.queue_delay_ms << ',' << currentFrameContext.processing_completed_sec << ','
        << snapshot.last_measurement_time_sec << ',' << snapshot.selection.selection_epoch << ','
        << snapshot.position_variance_x << ',' << snapshot.position_variance_y;
    const auto capture = collector ? collector->status() : CaptureStatus();
    trackingDiagnosticStream << ',' << capture.packets.packets << ',' << capture.packets.short_packets << ','
        << capture.packets.missing_packets << ',' << capture.packets.out_of_order << ','
        << capture.packets.discontinuities << ',' << capture.frames_received << ','
        << capture.queue_dropped << ',' << capture.stale_dropped << ',' << capture.send_dropped << ','
        << capture.queue_depth << '\n';

    if (trackingEventStream.is_open() &&
        dsp.near_field_burst_rejected_count > 0) {
        trackingEventStream << "NEAR_FIELD_BURST frame=" << frameNumber
                            << " hard_range_rejected="
                            << dsp.near_field_range_rejected_count
                            << " burst_rejected="
                            << dsp.near_field_burst_rejected_count << '\n';
        trackingEventStream.flush();
    }
    if (trackingEventStream.is_open() &&
        trajectory.locked_assignment_ambiguous) {
        trackingEventStream << "TRAJECTORY_AMBIGUOUS frame=" << frameNumber
                            << " locked_uid=" << trajectory.locked_uid
                            << " best_detection="
                            << trajectory.locked_best_detection_index
                            << " best_cost=" << trajectory.locked_best_cost
                            << " second_cost=" << trajectory.locked_second_cost
                            << '\n';
        trackingEventStream.flush();
    }
    if (trackingEventStream.is_open() &&
        (assoc.source == "reacquire_pending" ||
         assoc.source == "reacquire_pending_gap" ||
         assoc.source == "reacquire_confirmed")) {
        trackingEventStream << "REACQUIRE frame=" << frameNumber
                            << " state=" << assoc.source
                            << " uid=" << assoc.pending_internal_uid
                            << " hits=" << assoc.pending_hits
                            << " window_frames="
                            << assoc.pending_window_frames << '\n';
        trackingEventStream.flush();
    }
    if (trackingEventStream.is_open() &&
        (assoc.source == "manual_lock_pending" ||
         assoc.source == "manual_lock_confirmed" ||
         assoc.source == "manual_lock_confirmation_failed")) {
        trackingEventStream << "MANUAL_LOCK frame=" << frameNumber
                            << " state=" << assoc.source
                            << " uid=" << assoc.pending_internal_uid
                            << " hits=" << assoc.pending_hits
                            << " misses="
                            << targetTracker.manualLockConfirmationMisses()
                            << " init_vx=" << (locked.valid ? locked.vx : 0.0f)
                            << " init_vy=" << (locked.valid ? locked.vy : 0.0f)
                            << '\n';
        trackingEventStream.flush();
    }

    const bool importantDiagnostic = !dsp.processing_ok ||
        dsp.compensation_warning || assoc.measurement_rejected ||
        trajectory.locked_assignment_ambiguous ||
        dsp.near_field_burst_rejected_count > 0 ||
        assoc.source == "reacquire_pending" ||
        assoc.source == "reacquire_pending_gap" ||
        assoc.source == "reacquire_confirmed" ||
        assoc.source == "manual_lock_pending" ||
        assoc.source == "manual_lock_confirmed" ||
        assoc.source == "manual_lock_confirmation_failed" ||
        assoc.source == "manual_lost_timeout" ||
        (locked.valid && locked.lost_frames > 0);
    if (importantDiagnostic || trackingStats.updates % 10 == 0) {
        trackingDiagnosticStream.flush();
    }
}

void RadarGUI::closeTrackingDiagnosticLog()
{
    if (trackingDiagnosticStream.is_open()) {
        trackingDiagnosticStream.flush();
        trackingDiagnosticStream.close();
    }
    if (trackingEventStream.is_open()) {
        trackingEventStream.flush();
        trackingEventStream.close();
    }
}

double RadarGUI::realtimeFramePeriodSeconds() const
{
    auto it = radar_config.find("framePeriodicity");
    if (it != radar_config.end() && it->second > 1.0) {
        return it->second / 1000.0;
    }
    return targetTracker.config().kalman_dt_s;
}

std::vector<TestPoint> RadarGUI::compensatedRealtimeWindowPoints() const
{
    std::vector<TestPoint> points;
    if (realtimeFrameOrder.empty()) return points;
    const auto current = realtimeFrameBuffer.find(realtimeFrameOrder.back());
    if (current == realtimeFrameBuffer.end()) return points;
    for (int frame : realtimeFrameOrder) {
        const auto source = realtimeFrameBuffer.find(frame);
        if (source == realtimeFrameBuffer.end()) continue;
        RadarMotion::appendCompensated(points, source->second.points, frame,
            realtimeFrameOrder.back(), realtimeFramePeriodSeconds(), &source->second.chassis,
            &current->second.chassis, false);
    }
    return points;
}

void RadarGUI::syncBackgroundUiAfterTrackerReset()
{
    if (ui->backgroundStartButton) {
        ui->backgroundStartButton->setText("开始建模");
    }
    targetTracker.setBackgroundFilterEnabled(ui->backgroundFilterCheck &&
                                             ui->backgroundFilterCheck->isChecked());
}

void RadarGUI::resetTrackTxtOutput()
{
    writeTrackingMetadata(currentFrameContext.frame_number, "output_session_reset");
    currentTrackTxtPath.clear();
    currentTrackCsvPath.clear();
    currentControlRecordLine_.clear();
}

bool RadarGUI::visualMotionAllowed() const
{
#if RADAR_ENABLE_CAMERA
    if(cameraAssist_){const auto output=targetTracker.output();return cameraAssist_->decision(output.target.matched_internal_uid,output.selection.selection_epoch).permit;}
#endif
    return true;
}

bool RadarGUI::writeTrackingMetadata(int frame, const QString& stopReason)
{
    if (currentTrackTxtPath.isEmpty()) return false;
    if (!stopReason.isEmpty()) trackingMetadataStopReason = stopReason;
    const QString effectiveStopReason = trackingMetadataStopReason;
    const auto snapshot = targetTracker.output();
    QJsonObject object;
    object["schema_version"] = 1;
    object["frame"] = frame;
    object["target_id"] = snapshot.target.display_id;
    object["session_id"] = QString::number(snapshot.frame.session_id);
    object["selection_epoch"] = QString::number(snapshot.selection.selection_epoch);
    object["uid"] = QString::number(snapshot.target.matched_internal_uid);
    object["received_time_sec"] = snapshot.frame.received_time_sec;
    object["sample_time_sec"] = snapshot.frame.sample_time_sec;
    object["time_quality"] = QString::fromStdString(snapshot.frame.time_quality);
    object["last_measurement_time_sec"] = snapshot.last_measurement_time_sec;
    object["queue_delay_ms"] = snapshot.frame.queue_delay_ms;
    object["processing_completed_sec"] = snapshot.frame.processing_completed_sec;
    object["predicted"] = snapshot.predicted;
    object["position_variance_x"] = snapshot.position_variance_x;
    object["position_variance_y"] = snapshot.position_variance_y;
    object["motion_reason"] = effectiveStopReason.isEmpty() ? QString::fromStdString(snapshot.motion_reason) : effectiveStopReason;
    object["force_stop"] = !effectiveStopReason.isEmpty();
#if RADAR_ENABLE_CAMERA
    if(cameraAssist_) {
        const auto guard=cameraAssist_->metadata(snapshot.target.matched_internal_uid,snapshot.selection.selection_epoch);
        object["vision_guard"]=guard;
        if(guard["enabled"].toBool()&&!guard["permit"].toBool()) {
            object["force_stop"]=true;
            if(effectiveStopReason.isEmpty())object["motion_reason"]="vision_"+guard["reason"].toString();
        }
    }
#endif
    const auto recordFields=currentControlRecordLine_.split(' ',Qt::SkipEmptyParts);
    if(recordFields.size()==21&&recordFields[11].toInt()==frame&&recordFields[0].toInt()==snapshot.target.display_id) {
        object["control_snapshot_version"]=1;
        object["record_line"]=currentControlRecordLine_;
    } else if(effectiveStopReason.isEmpty()) {
        // A camera callback cannot publish metadata for a radar frame whose
        // complete control record has not yet been assembled.
        return false;
    }
    QSaveFile file(currentTrackTxtPath + ".meta.json");
    const auto bytes = QJsonDocument(object).toJson(QJsonDocument::Compact);
    if (!file.open(QIODevice::WriteOnly) || file.write(bytes) != bytes.size() || !file.commit()) {
        logMessage("无法更新跟踪时间元数据；停止追加控制记录，等待桥接超时停车。");
        return false;
    }
#if RADAR_ENABLE_CAMERA
    if(cameraAssist_)cameraAssist_->recordRadarSnapshot(object);
#endif
    return true;
}

void RadarGUI::appendLockedTargetToTxt(int frameNumber)
{
    const auto exportSnapshot=targetTracker.output();
    const auto& locked = exportSnapshot.target;
    // Keep a per-frame heartbeat while the lock exists. The ROS2 controller
    // consumes the explicit state token instead of guessing from file silence.
    if (!locked.valid) return;

    QDir dir(QDir::currentPath());
    if (!dir.exists("track_csv") && !dir.mkpath("track_csv")) {
        logMessage("追踪坐标保存失败：无法创建 track_csv 文件夹");
        return;
    }
    if (!dir.exists("track_txt") && !dir.mkpath("track_txt")) {
        logMessage("追踪坐标保存失败：无法创建 track_txt 文件夹");
        return;
    }

    if (currentTrackCsvPath.isEmpty() || currentTrackTxtPath.isEmpty()) {
        const QString mode = isOfflineTrackingMode ? "offline" : "realtime";
        const QString stamp = QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss_zzz");
        const QString base = QString("track_%1_%2_%3")
                                 .arg(mode)
                                 .arg(stamp)
                                 .arg(static_cast<qlonglong>(trackingStats.session_id));
        QString uniqueBase = base;
        int suffix = 1;
        do {
            currentTrackCsvPath = dir.filePath(
                QString("track_csv/%1.csv").arg(uniqueBase));
            currentTrackTxtPath = dir.filePath(
                QString("track_txt/%1.txt").arg(uniqueBase));
            if (!QFileInfo::exists(currentTrackCsvPath) &&
                !QFileInfo::exists(currentTrackTxtPath)) {
                break;
            }
            uniqueBase = QString("%1_%2").arg(base).arg(suffix++);
        } while (true);

        QFile csvFile(currentTrackCsvPath);
        if (csvFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QTextStream out(&csvFile);
            out << "target_id,filtered_x_m,filtered_y_m,filtered_range_m,filtered_angle_deg,"
                   "measured_x_m,measured_y_m,measured_range_m,measured_angle_deg,"
                   "has_measurement,confidence,frame,lost_frames,stable_frames,timestamp,mode,"
                   "control_state,speed_scale,allow_translation,allow_yaw,reacquire_hits\n";
            out.flush();
            csvFile.flush();
        } else {
            logMessage(QString("追踪坐标保存失败：无法创建文件 %1").arg(currentTrackCsvPath));
            currentTrackCsvPath.clear();
            currentTrackTxtPath.clear();
            return;
        }

        QFile txtFile(currentTrackTxtPath);
        if (txtFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QTextStream out(&txtFile);
            out << QString("%1 %2 %3 %4 %5 %6 %7 %8 %9 %10 %11 %12 %13 %14 %15 %16 %17 %18 %19 %20 %21\n")
                       .arg("target_id", -9)
                       .arg("f_x_m", -10)
                       .arg("f_y_m", -10)
                       .arg("f_range", -10)
                       .arg("f_angle", -10)
                       .arg("m_x_m", -10)
                       .arg("m_y_m", -10)
                       .arg("m_range", -10)
                       .arg("m_angle", -10)
                       .arg("meas", -5)
                       .arg("conf", -7)
                       .arg("frame", -7)
                       .arg("lost", -6)
                       .arg("stable", -7)
                       .arg("timestamp", -23)
                       .arg("mode", -8)
                       .arg("state", -10)
                       .arg("scale", -7)
                       .arg("move", -5)
                       .arg("yaw", -5)
                       .arg("rehit", -6);
            out << QString(220, '-') << "\n";
            out.flush();
            txtFile.flush();
        } else {
            logMessage(QString("追踪坐标保存失败：无法创建文件 %1").arg(currentTrackTxtPath));
            currentTrackCsvPath.clear();
            currentTrackTxtPath.clear();
            return;
        }
    }

    // Keep the TXT protocol whitespace-safe: one timestamp must remain one token.
    const QString timestamp = QDateTime::currentDateTime().toString("yyyy-MM-ddThh:mm:ss.zzz");
    const QString mode = isOfflineTrackingMode ? "offline" : "realtime";
    const QString xText = QString::number(locked.x, 'f', 4);
    const QString yText = QString::number(locked.y, 'f', 4);
    const QString rangeText = QString::number(locked.range_m, 'f', 4);
    const QString angleText = QString::number(locked.angle_deg, 'f', 2);
    const QString measuredXText = locked.has_measurement ? QString::number(locked.measured_x, 'f', 4) : "nan";
    const QString measuredYText = locked.has_measurement ? QString::number(locked.measured_y, 'f', 4) : "nan";
    const QString measuredRangeText = locked.has_measurement ? QString::number(locked.measured_range_m, 'f', 4) : "nan";
    const QString measuredAngleText = locked.has_measurement ? QString::number(locked.measured_angle_deg, 'f', 2) : "nan";
    const QString hasMeasurementText = locked.has_measurement ? "1" : "0";
    const QString confidenceText = QString::number(locked.confidence, 'f', 3);
    const bool visionAllowed=visualMotionAllowed();
    const QString controlStateText = visionAllowed?followControlStateToken(locked.control_state):"STOPPED";
    const QString speedScaleText = QString::number(visionAllowed?locked.control_speed_scale:0, 'f', 3);

    const QString recordLine = QString("%1 %2 %3 %4 %5 %6 %7 %8 %9 %10 %11 %12 %13 %14 %15 %16 %17 %18 %19 %20 %21\n")
                  .arg(QString::number(locked.display_id), -9)
                  .arg(xText, -10)
                  .arg(yText, -10)
                  .arg(rangeText, -10)
                  .arg(angleText, -10)
                  .arg(measuredXText, -10)
                  .arg(measuredYText, -10)
                  .arg(measuredRangeText, -10)
                  .arg(measuredAngleText, -10)
                  .arg(hasMeasurementText, -5)
                  .arg(confidenceText, -7)
                  .arg(QString::number(frameNumber), -7)
                  .arg(QString::number(locked.lost_frames), -6)
                  .arg(QString::number(locked.stable_frames), -7)
                  .arg(timestamp, -23)
                  .arg(mode, -8)
                  .arg(controlStateText, -10)
                  .arg(speedScaleText, -7)
                  .arg(locked.control_allow_translation && visionAllowed ? "1" : "0", -5)
                  .arg(locked.control_allow_yaw && visionAllowed ? "1" : "0", -5)
                  .arg(QString::number(locked.reacquire_hits), -6);
    currentControlRecordLine_=recordLine.trimmed();
    // Only a newly exported radar observation can end an explicit producer stop.
    // The camera's periodic evidence refresh must preserve it.
    trackingMetadataStopReason.clear();
    if (!writeTrackingMetadata(frameNumber)) return;
    QFile csvFile(currentTrackCsvPath);
    if (!csvFile.open(QIODevice::Append | QIODevice::Text)) {
        logMessage(QString("追踪坐标保存失败：无法写入文件 %1").arg(currentTrackCsvPath));
        return;
    }

    QTextStream csvOut(&csvFile);
    csvOut << locked.display_id << ","
           << xText << ","
           << yText << ","
           << rangeText << ","
           << angleText << ","
           << measuredXText << ","
           << measuredYText << ","
           << measuredRangeText << ","
           << measuredAngleText << ","
           << hasMeasurementText << ","
           << confidenceText << ","
           << frameNumber << ","
           << locked.lost_frames << ","
           << locked.stable_frames << ","
           << timestamp << ","
           << mode << ","
           << controlStateText << ","
           << speedScaleText << ","
           << (locked.control_allow_translation && visionAllowed ? 1 : 0) << ","
           << (locked.control_allow_yaw && visionAllowed ? 1 : 0) << ","
           << locked.reacquire_hits << "\n";
    csvOut.flush();
    csvFile.flush();

    QFile txtFile(currentTrackTxtPath);
    if (!txtFile.open(QIODevice::Append | QIODevice::Text)) {
        logMessage(QString("追踪坐标保存失败：无法写入文件 %1").arg(currentTrackTxtPath));
        return;
    }

    QTextStream txtOut(&txtFile);
    txtOut << recordLine;
    txtOut.flush();
    txtFile.flush();
}
void RadarGUI::on_action_triggered()
{
    if(rosRuntime_){rosRuntime_->setCaptureSession(0);rosRuntime_->stopFollow();}
    if (collector || is_capturing) stop_capture();
    writeTrackingMetadata(currentFrameContext.frame_number, "mode_changed");
    ui->captureGroup->show();
    if (ui->pointCloudGroup) ui->pointCloudGroup->hide();
    if (ui->offlinePointCloudGroup) ui->offlinePointCloudGroup->hide();  // 隐藏离线点云
    if (ui->offlinePointCloudGroup_2) ui->offlinePointCloudGroup_2->hide();  // 隐藏离线追踪
    isTestMode = false;
    isOfflineMode = false;
    isOfflineTrackingMode = false;
    targetTracker.reset();
    resetTrackTxtOutput();
    cluster->stopPlayback();
    // 停止离线处理（如果正在运行）
    if (offlineTimer && offlineTimer->isActive()) {
        offlineTimer->stop();
        if (offlineProcessButton) offlineProcessButton->setText("处理");
        if (offlineTrackButton) offlineTrackButton->setText("追踪");
    }
    if (offlineBinFile.is_open()) {
        offlineBinFile.close();
    }
    // 恢复实时模式下的信息文字显示
    if (infoText) {
        infoText->show();
    }
    // 重新连接 UDP 以恢复实时显示（避免重复连接，先断开再连接）
    if (udpSocket) {
        QObject::disconnect(udpSocket, &QUdpSocket::readyRead, this, &RadarGUI::processUdpData);
        connect(udpSocket, &QUdpSocket::readyRead, this, &RadarGUI::processUdpData);
    }
    resetRealtimeClusterBuffer();
    logMessage("切换到原始点云（实时显示）");
}

void RadarGUI::on_action_3_triggered()
{
    if(rosRuntime_){rosRuntime_->setCaptureSession(0);rosRuntime_->stopFollow();}
    if (collector || is_capturing) stop_capture();
    writeTrackingMetadata(currentFrameContext.frame_number, "mode_changed");
    // 目标聚类：隐藏"当前状态"控件
    ui->captureGroup->hide();
    if (ui->pointCloudGroup) ui->pointCloudGroup->show();
    if (ui->offlinePointCloudGroup) ui->offlinePointCloudGroup->hide();  // 隐藏离线点云
    if (ui->offlinePointCloudGroup_2) ui->offlinePointCloudGroup_2->hide();  // 隐藏离线追踪
    isTestMode = true;
    isOfflineMode = false;
    isOfflineTrackingMode = false;
    targetTracker.reset();
    resetTrackTxtOutput();
    // 停止离线处理（如果正在运行）
    if (offlineTimer && offlineTimer->isActive()) {
        offlineTimer->stop();
        if (offlineProcessButton) offlineProcessButton->setText("处理");
        if (offlineTrackButton) offlineTrackButton->setText("追踪");
    }
    if (offlineBinFile.is_open()) {
        offlineBinFile.close();
    }
    // 断开 UDP 实时更新，改为从本地文件渲染
    if (udpSocket) {
        QObject::disconnect(udpSocket, &QUdpSocket::readyRead, this, &RadarGUI::processUdpData);
    }
    // 停止实时采集状态（如果存在）
    if (collector) {
        collector->stop_collector();
        collector->join(1000);
        delete collector; collector = nullptr;
    }
    if (dca1000_cfg) {
        dca1000_cfg->DCA1000_close();
    }
    cluster->stopPlayback();
    is_capturing = false;
    // 清空画布上实时模式的残留点云和文字，避免与聚类显示重叠
    for (auto* item : pointItems) {
        scene->removeItem(item);
        delete item;
    }
    pointItems.clear();
    if (infoText) {
        infoText->setPlainText("");
        infoText->hide();
    }
    // 刷新点云下拉列表
    updateConfigList();
    if (ui->pushButton_4) ui->pushButton_4->setText("开始");
    logMessage("切换到目标聚类（点击'开始'按钮后读取 ./txt 下点云文件并逐帧播放）");
    ui->textEdit->append("切换到目标聚类：点击'开始'按钮后读取 ./txt 下点云文件并逐帧播放\n");
}

// —— 缩放按钮实现 ——
void RadarGUI::applyZoom()
{
    if (!scene || !ui || !ui->graphicsView) return;
    // 先根据固定的 sceneRect 合理居中，然后应用缩放变换
    ui->graphicsView->resetTransform();
    // 以 sceneRect 居中显示并保持纵横比
    ui->graphicsView->fitInView(scene->sceneRect(), Qt::KeepAspectRatio);
    // 在 fitInView 基础上再叠加自定义缩放
    ui->graphicsView->scale(zoomFactor, zoomFactor);
}

void RadarGUI::handleZoomInClicked()
{
    zoomFactor = std::min(maxZoom, zoomFactor * zoomStep);
    applyZoom();
}

void RadarGUI::handleZoomOutClicked()
{
    zoomFactor = std::max(minZoom, zoomFactor / zoomStep);
    applyZoom();
}

void RadarGUI::positionZoomButtons()
{
    QWidget* vp = ui->graphicsView->viewport();
    if (!vp->findChild<QPushButton*>("zoomInButton")) {
        auto* zoomInButton = new QPushButton("+", vp);
        zoomInButton->setObjectName("zoomInButton");
        zoomInButton->setFixedSize(32, 32);
        zoomInButton->setStyleSheet("QPushButton{background:#f0f0f0;border:1px solid #aaa;border-radius:4px;}"
                                    "QPushButton:hover{background:#ffffff;}");
        connect(zoomInButton, &QPushButton::clicked, this, &RadarGUI::handleZoomInClicked, Qt::UniqueConnection);
    }
    if (!vp->findChild<QPushButton*>("zoomOutButton")) {
        auto* zoomOutButton = new QPushButton("-", vp);
        zoomOutButton->setObjectName("zoomOutButton");
        zoomOutButton->setFixedSize(32, 32);
        zoomOutButton->setStyleSheet("QPushButton{background:#f0f0f0;border:1px solid #aaa;border-radius:4px;}"
                                     "QPushButton:hover{background:#ffffff;}");
        connect(zoomOutButton, &QPushButton::clicked, this, &RadarGUI::handleZoomOutClicked, Qt::UniqueConnection);
    }
    // 将按钮放在点云图右下角空白处（graphicsView 的 viewport 内部）
    auto zoomIn = ui->graphicsView->viewport()->findChild<QPushButton*>("zoomInButton");
    auto zoomOut = ui->graphicsView->viewport()->findChild<QPushButton*>("zoomOutButton");
    if (!zoomIn || !zoomOut) return;

    // viewport 坐标系下定位
    QRect vrect = ui->graphicsView->viewport()->rect();
    int margin = 12;   // 距离右/下边缘的空隙
    int spacing = 8;   // 两个按钮之间的间距
    int btnW = zoomIn->width();
    int btnH = zoomIn->height();
    // 按钮垂直堆叠，靠右下；下方为 "-"，上方为 "+"，整体贴近最底部空白
    QPoint minusPos(vrect.right() - btnW - margin, vrect.bottom() - btnH - margin);
    QPoint plusPos(minusPos.x(), minusPos.y() - btnH - spacing);
    zoomOut->move(minusPos);
    zoomIn->move(plusPos);
    zoomOut->show();
    zoomIn->show();
    zoomOut->raise();
    zoomIn->raise();
}

void RadarGUI::resizeEvent(QResizeEvent* event)
{
    QMainWindow::resizeEvent(event);
    positionZoomButtons();
    // 窗口尺寸改变后重新应用缩放以适配新尺寸
    applyZoom();
    updatePinnedTargetRowChrome();
}

void RadarGUI::showEvent(QShowEvent* event)
{
    QMainWindow::showEvent(event);
    // 延迟到事件循环空闲时再定位，确保布局和视口尺寸已经稳定
    QTimer::singleShot(0, this, [this]() {
        positionZoomButtons();
        applyZoom();
        updatePinnedTargetRowChrome();
    });
}

void RadarGUI::updateSceneRectFromRadarConfig(bool fitView)
{
    if (!scene) return;

    constexpr qreal scale = 100.0; // 1m = 100 scene units
    double yMax = 40.0;
    auto it = radar_config.find("maxRange(m)");
    if (it != radar_config.end() && it->second > 1.0) {
        // maxRange(m) 中带有安全系数，例如 25m 配置会解析为 22.5m；
        // 画布向上取整到 5m 档位，显示更直观。
        yMax = std::ceil(it->second / 5.0) * 5.0;
    }
    yMax = std::clamp(yMax, 5.0, 80.0);

    // 横向范围保持左右对称。取前向距离的一半并按 5m 档位上取整，
    // 这样 25m 配置约为 X[-15,15]，旧 45m 配置约为 X[-25,25]。
    double xMax = std::ceil((yMax * 0.5) / 5.0) * 5.0;
    xMax = std::clamp(xMax, 5.0, 30.0);

    sceneYMaxMeters = yMax;
    sceneXMaxMeters = xMax;

    const QRectF rect(-sceneXMaxMeters * scale,
                      0.0,
                      sceneXMaxMeters * 2.0 * scale,
                      sceneYMaxMeters * scale);
    if (scene->sceneRect() != rect) {
        scene->setSceneRect(rect);
        drawAxes();
    }
    if (fitView && ui && ui->graphicsView) {
        ui->graphicsView->fitInView(rect, Qt::KeepAspectRatio);
    }
    if (ui && ui->graphicsView) {
        positionZoomButtons();
    }
}

qreal RadarGUI::sceneXFromMeters(float x) const
{
    constexpr qreal scale = 100.0;
    return static_cast<qreal>(x) * scale;
}

qreal RadarGUI::sceneYFromMeters(float y) const
{
    constexpr qreal scale = 100.0;
    return static_cast<qreal>(sceneYMaxMeters - y) * scale;
}

// —— 坐标轴绘制 ——
void RadarGUI::drawAxes()
{
    // 清除旧坐标轴元素
    for (auto* it : axisItems) {
        scene->removeItem(it);
        delete it;
    }
    axisItems.clear();

    if (!scene) return;

    const qreal scale = 100.0; // 1m = 100 scene units
    const QRectF r = scene->sceneRect();
    // 将物理范围还原成米
    const qreal xMin = r.left() / scale;
    const qreal xMax = r.right() / scale;
    const qreal yMin = r.top() / scale;   // 注意：视图坐标系 y 向下为正
    const qreal yMax = r.bottom() / scale;

    // 轴线颜色与样式（加粗一点以便更清晰）
    QPen axisPen(QColor(120,120,120));
    axisPen.setWidthF(2.0);
    axisPen.setCosmetic(true);
    QPen tickPen(QColor(160,160,160));
    tickPen.setWidthF(1.5);
    tickPen.setCosmetic(true);

    // X 轴放在底部边界（视觉上更符合"地面在下方"的直觉）
    // 原先按 y=0 绘制会落在顶部，因为 sceneRect 的 top=0
    qreal xAxisYScene = r.bottom();
    // Y 轴放在 x=0 位置（如果 0 在范围内），否则放在左侧边界
    qreal yAxisXScene = 0;
    if (0 >= xMin && 0 <= xMax) {
        yAxisXScene = 0 * scale; // x=0 的场景 x
    } else {
        yAxisXScene = r.left();
    }

    // 画主轴线
    auto* xAxis = scene->addLine(r.left(), xAxisYScene, r.right(), xAxisYScene, axisPen);
    auto* yAxis = scene->addLine(yAxisXScene, r.top(), yAxisXScene, r.bottom(), axisPen);
    axisItems.push_back(xAxis);
    axisItems.push_back(yAxis);

    // Keep labels readable in screen pixels when a large physical range is fitted.
    // Minor marks remain every metre; only major marks receive labels.
    const qreal tickLen = 16.0;
    QFont tickFont("Arial", 9);
    auto labelStep = [](double range) {
        const double desired = range / 8;
        for (int step : {1, 2, 5, 10, 20, 50}) if (step >= desired) return step;
        return 100;
    };
    const int xLabelStep = labelStep(xMax - xMin);
    const int yLabelStep = labelStep(r.height() / scale);

    // X 轴刻度与标签
    for (int xm = static_cast<int>(std::ceil(xMin)); xm <= static_cast<int>(std::floor(xMax)); ++xm) {
        qreal xs = xm * scale;
        auto* t = scene->addLine(xs, xAxisYScene - tickLen, xs, xAxisYScene + tickLen, tickPen);
        axisItems.push_back(t);
        if (xm == 0 || xm % xLabelStep != 0) continue;
        // 标签放在轴线之上，避免在底部被裁剪
        auto* txt = scene->addText(QString::number(xm), tickFont);
        txt->setFlag(QGraphicsItem::ItemIgnoresTransformations);
        txt->setDefaultTextColor(QColor(80,80,80));
        txt->setPos(xs, xAxisYScene - tickLen);
        txt->setTransform(QTransform::fromTranslate(-txt->boundingRect().width()/2, -txt->boundingRect().height()-2));
        axisItems.push_back(txt);
    }

    // Y 轴刻度与标签（数值用物理米，注意坐标系向下为正）
    const int yPhysMin = 0;
    const int yPhysMax = static_cast<int>(r.height() / scale);
    for (int ym = yPhysMin; ym <= yPhysMax; ++ym) {
        qreal ys = r.bottom() - ym * scale;
        auto* t = scene->addLine(yAxisXScene - tickLen, ys, yAxisXScene + tickLen, ys, tickPen);
        axisItems.push_back(t);
        if (ym == 0 || ym % yLabelStep != 0) continue;
        auto* txt = scene->addText(QString::number(ym), tickFont);
        txt->setFlag(QGraphicsItem::ItemIgnoresTransformations);
        txt->setDefaultTextColor(QColor(80,80,80));
        txt->setPos(yAxisXScene + tickLen, ys);
        txt->setTransform(QTransform::fromTranslate(4, ym == yPhysMax ? 0 : -txt->boundingRect().height()/2));
        axisItems.push_back(txt);
    }
    //     qreal ys = ym * scale;
    //     auto* t = scene->addLine(yAxisXScene - tickLen, ys, yAxisXScene + tickLen, ys, tickPen);
    //     auto* txt = scene->addText(QString::number(ym), tickFont);
    //     txt->setDefaultTextColor(QColor(80,80,80));
    //     txt->setPos(yAxisXScene + tickLen + 2, ys - 8);

    // 原点标签（放在交点附近，略微向上避免贴边）
    auto* originTxt = scene->addText("0", tickFont);
    originTxt->setFlag(QGraphicsItem::ItemIgnoresTransformations);
    originTxt->setDefaultTextColor(QColor(80,80,80));
    originTxt->setPos(yAxisXScene + tickLen, xAxisYScene - tickLen);
    originTxt->setTransform(QTransform::fromTranslate(4, -originTxt->boundingRect().height()-2));
    axisItems.push_back(originTxt);
}

// ===== 离线处理相关方法实现 =====

void RadarGUI::on_action_5_triggered()
{
    if(rosRuntime_){rosRuntime_->setCaptureSession(0);rosRuntime_->stopFollow();}
    if (collector || is_capturing) stop_capture();
    writeTrackingMetadata(currentFrameContext.frame_number, "mode_changed");
    // 切换到离线处理模式
    isOfflineMode = true;
    isTestMode = false;
    isOfflineTrackingMode = false;
    targetTracker.reset();
    resetTrackTxtOutput();

    // 停止实时采集（如果正在运行）
    if (is_capturing) {
        stop_capture();
    }

    // 停止目标聚类播放（如果正在运行）
    if (cluster) {
        cluster->stopPlayback();
    }

    // 停止离线处理定时器（如果正在运行）
    if (offlineTimer && offlineTimer->isActive()) {
        offlineTimer->stop();
        if (offlineProcessButton) offlineProcessButton->setText("处理");
        if (offlineTrackButton) offlineTrackButton->setText("追踪");
    }
    if (offlineBinFile.is_open()) {
        offlineBinFile.close();
    }

    // 断开UDP实时更新
    if (udpSocket) {
        QObject::disconnect(udpSocket, &QUdpSocket::readyRead, this, &RadarGUI::processUdpData);
    }

    // 显示/隐藏对应的选项框
    ui->captureGroup->hide();                           // 隐藏"当前状态"
    if (ui->pointCloudGroup) ui->pointCloudGroup->hide(); // 隐藏"点云数据"
    if (ui->offlinePointCloudGroup) ui->offlinePointCloudGroup->show();  // 显示"离线点云"
    if (ui->offlinePointCloudGroup_2) ui->offlinePointCloudGroup_2->hide();  // 隐藏"离线追踪"

    // 清空画布上残留的点云和文字
    for (auto* item : pointItems) {
        scene->removeItem(item);
        delete item;
    }
    pointItems.clear();

    // 显示提示信息
    if (infoText) {
        infoText->setPlainText("离线处理模式\n请选择bin文件并点击'处理'");
        infoText->show();
    }

    // 刷新bin文件列表
    updateBinList();

    logMessage("切换到离线处理（读取./bin目录下的bin文件进行离线信号处理）");
    if (ui->textEdit) ui->textEdit->append("切换到离线处理：选择bin文件后点击'处理'按钮开始\n");
}

void RadarGUI::on_action_4_triggered()
{
    if(rosRuntime_){rosRuntime_->setCaptureSession(0);rosRuntime_->stopFollow();}
    if (collector || is_capturing) stop_capture();
    writeTrackingMetadata(currentFrameContext.frame_number, "mode_changed");
    // 切换到离线追踪模式
    isOfflineMode = true;
    isTestMode = false;
    isOfflineTrackingMode = true;
    targetTracker.reset();
    resetTrackTxtOutput();

    // 停止实时采集（如果正在运行）
    if (is_capturing) {
        stop_capture();
    }

    // 停止目标聚类播放（如果正在运行）
    if (cluster) {
        cluster->stopPlayback();
    }

    // 停止离线处理定时器（如果正在运行）
    if (offlineTimer && offlineTimer->isActive()) {
        offlineTimer->stop();
        if (offlineProcessButton) offlineProcessButton->setText("处理");
        if (offlineTrackButton) offlineTrackButton->setText("追踪");
    }
    if (offlineBinFile.is_open()) {
        offlineBinFile.close();
    }

    // 断开UDP实时更新
    if (udpSocket) {
        QObject::disconnect(udpSocket, &QUdpSocket::readyRead, this, &RadarGUI::processUdpData);
    }

    // 显示/隐藏对应的选项框
    ui->captureGroup->hide();
    if (ui->pointCloudGroup) ui->pointCloudGroup->hide();
    if (ui->offlinePointCloudGroup) ui->offlinePointCloudGroup->hide();
    if (ui->offlinePointCloudGroup_2) ui->offlinePointCloudGroup_2->show();

    // 清空画布上残留的点云和文字
    for (auto* item : pointItems) {
        scene->removeItem(item);
        delete item;
    }
    pointItems.clear();

    // 显示提示信息
    if (infoText) {
        infoText->setPlainText("离线追踪模式\n请选择bin文件并点击'追踪'");
        infoText->show();
    }

    // 刷新bin文件列表
    updateTrackBinList();
    if (ui->targetModeCombo) ui->targetModeCombo->setCurrentIndex(1);
    if (ui->targetRuleCombo) ui->targetRuleCombo->setCurrentIndex(2);
    refreshTargetTable();
    refreshLockedTargetInfo();

    logMessage("切换到离线追踪（读取./bin目录下的bin文件进行离线追踪）");
    if (ui->textEdit) ui->textEdit->append("切换到离线追踪：选择bin文件后点击'追踪'按钮开始\n");
}

void RadarGUI::updateBinList()
{
    if (!offlineBinCombo) return;
    offlineBinCombo->clear();

    QDir binDir("./bin");
    if (binDir.exists()) {
        QStringList bins = binDir.entryList({"*.bin"}, QDir::Files, QDir::Name);
        for (const QString& f : bins) {
            offlineBinCombo->addItem(f);
        }
    }
    if (offlineBinCombo->count() == 0) {
        logMessage("离线处理：./bin目录下未找到bin文件");
    } else {
        logMessage(QString("离线处理：找到 %1 个bin文件").arg(offlineBinCombo->count()));
    }
}

void RadarGUI::updateTrackBinList()
{
    if (!offlineTrackCombo) return;
    offlineTrackCombo->clear();

    QDir binDir("./bin");
    if (binDir.exists()) {
        QStringList bins = binDir.entryList({"*.bin"}, QDir::Files, QDir::Name);
        for (const QString& f : bins) {
            offlineTrackCombo->addItem(f);
        }
    }
    if (offlineTrackCombo->count() == 0) {
        logMessage("离线追踪：./bin目录下未找到bin文件");
    } else {
        logMessage(QString("离线追踪：找到 %1 个bin文件").arg(offlineTrackCombo->count()));
    }
}

void RadarGUI::initTargetTable()
{
    targetTable = this->findChild<QTableWidget*>("tableWidget");
    if (!targetTable) return;

    targetTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    targetTable->setSelectionMode(QAbstractItemView::SingleSelection);
    targetTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
    targetTable->setSortingEnabled(false);
    targetTable->setHorizontalHeaderLabels(
        QStringList() << "选择" << "雷达ID" << "距离" << "速度");
    targetTable->setToolTip(
        "勾选后该人员固定在表头下方，其余人员仍按数字顺序滚动；取消勾选后全部一起滚动");

    if (!pinnedTargetTable && ui && ui->targetTrackingLayout) {
        pinnedTargetRowHost = new QWidget(targetTable->parentWidget());
        pinnedTargetRowHost->setObjectName("pinnedTargetRowHost");
        pinnedTargetRowHost->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        auto* hostLayout = new QHBoxLayout(pinnedTargetRowHost);
        hostLayout->setContentsMargins(0, 0, 0, 0);
        hostLayout->setSpacing(0);

        pinnedTargetTable = new QTableWidget(1, 4, pinnedTargetRowHost);
        pinnedTargetTable->setObjectName("pinnedTargetTable");
        pinnedTargetTable->setSelectionBehavior(QAbstractItemView::SelectRows);
        pinnedTargetTable->setSelectionMode(QAbstractItemView::SingleSelection);
        pinnedTargetTable->setEditTriggers(QAbstractItemView::NoEditTriggers);
        pinnedTargetTable->setSortingEnabled(false);
        pinnedTargetTable->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        pinnedTargetTable->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
        pinnedTargetTable->verticalHeader()->setVisible(false);
        pinnedTargetTable->horizontalHeader()->setMinimumSectionSize(38);
        pinnedTargetTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
        pinnedTargetTable->setHorizontalHeaderLabels(
            QStringList() << "选择" << "雷达ID" << "距离" << "速度");
        pinnedTargetTable->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
        hostLayout->addWidget(pinnedTargetTable, 1);

        pinnedScrollSpacer = new QWidget(pinnedTargetRowHost);
        pinnedScrollSpacer->setObjectName("pinnedScrollSpacer");
        pinnedScrollSpacer->setFixedWidth(0);
        hostLayout->addWidget(pinnedScrollSpacer);

        const int layoutIndex = ui->targetTrackingLayout->indexOf(targetTable);
        if (layoutIndex >= 0) {
            ui->targetTrackingLayout->insertWidget(layoutIndex, pinnedTargetRowHost);
        }
        pinnedTargetRowHost->hide();
        connect(pinnedTargetTable, &QTableWidget::cellClicked, this,
                &RadarGUI::onTargetTableCellClicked, Qt::UniqueConnection);
        connect(targetTable->verticalScrollBar(), &QScrollBar::rangeChanged, this,
                [this](int, int) { updatePinnedTargetRowChrome(); });
    }

    ensureTargetTableRows(4);
}

bool RadarGUI::eventFilter(QObject* watched,QEvent* event) {
    if(ui&&watched==ui->graphicsView->viewport()&&event->type()==QEvent::MouseButtonPress) {
        auto* mouse=static_cast<QMouseEvent*>(event);
        if(mouse->button()!=Qt::LeftButton)return QMainWindow::eventFilter(watched,event);
        const TargetTracking::CandidateTarget* best=nullptr;
        qreal bestDistance=32,secondDistance=1e9;
        for(const auto& c:targetTracker.candidates()) {
            if(c.id<=0||!c.has_current_frame_support)continue;
            const auto q=ui->graphicsView->mapFromScene(QPointF(sceneXFromMeters(c.x),sceneYFromMeters(c.y)));
            const qreal d=QLineF(QPointF(q),mouse->localPos()).length();
            if(d<bestDistance){if(best)secondDistance=bestDistance;bestDistance=d;best=&c;}
            else secondDistance=std::min(secondDistance,d);
        }
        if(best&&secondDistance-bestDistance>=6) {
            previewTargetUid=best->internal_uid;previewTargetDisplayId=best->id;
            if(ui->targetModeCombo)ui->targetModeCombo->setCurrentIndex(1);
            selectionHintText_=QString("已预选 P%1：核对黄框后点击“确认锁定”。").arg(best->id);
            refreshTargetTable();updateTargetSelectionControls();drawTrackingOverlay();
        } else if(best) {
            selectionHintText_="此处有相邻候选，请在右侧列表按 P 编号选择。";
        } else {
            clearTargetPreview();updateTargetSelectionControls();
            selectionHintText_="尚无稳定雷达轨迹；请在 2.5–4m 处缓慢横移。";
        }
        if(selectionHintLabel_)selectionHintLabel_->setText(selectionHintText_);
        refreshCooperativeView();return true;
    }
    return QMainWindow::eventFilter(watched,event);
}

void RadarGUI::onTargetTableCellClicked(int row, int column)
{
    Q_UNUSED(column); // All columns select the same immutable preview identity.
    auto* table = qobject_cast<QTableWidget*>(sender());
    if (!table) return;
    QTableWidgetItem* idItem = table->item(row, 1);
    if (!idItem) return;
    const bool current = idItem->data(Qt::UserRole + 2).toBool();
    const int displayId = idItem->data(Qt::UserRole).toInt();
    const std::uint64_t internalUid =
        idItem->data(Qt::UserRole + 1).toULongLong();
    const bool pinnedRow = table == pinnedTargetTable;
    if ((!current && !pinnedRow) || displayId <= 0 || internalUid == 0) {
        logMessage("该灰色行仅用于保持数字顺序，当前帧不可选");
        return;
    }

    const auto& locked = targetTracker.lockedTarget();
    if (locked.valid && locked.matched_internal_uid == internalUid) {
        clearTargetPreview();
        logMessage(QString("人员 P%1 已经处于锁定状态").arg(displayId));
    } else if (previewTargetUid == internalUid) {
        clearTargetPreview();
    } else {
        previewTargetUid = internalUid;
        previewTargetDisplayId = displayId;
        if (ui->targetModeCombo) ui->targetModeCombo->setCurrentIndex(1);
        logMessage(QString("已预选人员 P%1；请核对点云中的黄色标记，再点击确认锁定")
                       .arg(displayId));
        updateTargetSelectionControls();
    }
    refreshTargetTable();
    drawTrackingOverlay();
}

void RadarGUI::updatePinnedTargetRowChrome()
{
    if (!pinnedTargetRowHost || !pinnedTargetTable || !pinnedScrollSpacer || !targetTable) return;
    if (!pinnedTargetRowHost->isVisible()) return;
    const int barWidth = targetTable->verticalScrollBar()->isVisible()
        ? targetTable->verticalScrollBar()->width() : 0;
    pinnedScrollSpacer->setFixedWidth(barWidth);
    const int rowHeight = std::max(pinnedTargetTable->rowHeight(0),
                                   targetTable->verticalHeader()->defaultSectionSize());
    const int headerHeight = std::max(pinnedTargetTable->horizontalHeader()->sizeHint().height(),
                                      targetTable->horizontalHeader()->sizeHint().height());
    pinnedTargetRowHost->setFixedHeight(headerHeight + rowHeight + 2 * pinnedTargetTable->frameWidth());
}

void RadarGUI::ensureTargetTableRows(int requiredRows)
{
    if (!targetTable) return;
    if (requiredRows < 0) return;

    if (targetTable->rowCount() < requiredRows) {
        targetTable->setRowCount(requiredRows);
    }

    for (int row = 0; row < targetTable->rowCount(); ++row) {
        QTableWidgetItem* item = targetTable->item(row, 0);
        if (!item) {
            item = new QTableWidgetItem();
            targetTable->setItem(row, 0, item);
        }
        item->setFlags(Qt::ItemIsUserCheckable | Qt::ItemIsEnabled | Qt::ItemIsSelectable);
        item->setCheckState(Qt::Unchecked);
        item->setTextAlignment(Qt::AlignCenter);
    }
}

TargetTracking::AutoSelectRule RadarGUI::currentAutoSelectRule() const
{
    if (!ui || !ui->targetRuleCombo) {
        return TargetTracking::AutoSelectRule::MovingFirst;
    }

    switch (ui->targetRuleCombo->currentIndex()) {
    case 0:
        return TargetTracking::AutoSelectRule::Nearest;
    case 1:
        return TargetTracking::AutoSelectRule::Fastest;
    case 2:
    default:
        return TargetTracking::AutoSelectRule::MovingFirst;
    }
}

void RadarGUI::updateTargetTrackingCandidates(const std::vector<PointCloud>& points, int frameNumber)
{
    const auto trackingStart = std::chrono::steady_clock::now();
    latestTrackingPoints = points;
    // Offline BIN files do not contain synchronized odom samples. Reading a
    // live chassis sample during replay would apply compensation from the wrong time.
    targetTracker.setEgoMotionCompensationEnabled(
        !isOfflineTrackingMode || offlineOdomReplay.loaded());
    targetTracker.setBackgroundFilterEnabled(ui->backgroundFilterCheck &&
                                             ui->backgroundFilterCheck->isChecked());
    const bool wasModelingBackground = targetTracker.backgroundModelingActive();
    currentFrameContext.frame_number = frameNumber;
    if (!currentFrameContext.session_id) currentFrameContext.session_id = trackingStats.session_id;
    targetTracker.setFrameContext(currentFrameContext);
    RadarMotion::advance(targetTracker, points, currentTrackingChassis, frameNumber);
#if RADAR_ENABLE_CAMERA
    if(cameraAssist_) {
        CameraAssist::RadarFrame frame;
        if(!isOfflineMode&&!isOfflineTrackingMode&&!isTestMode) {
            frame.session=currentFrameContext.session_id;
            frame.captured_monotonic=currentFrameContext.received_monotonic_sec;
            for(const auto& candidate:targetTracker.candidates())
                if(candidate.id>0&&candidate.has_current_frame_support)
                    frame.people.push_back({candidate.internal_uid,candidate.id,candidate.x,candidate.y});
        }
        cameraAssist_->setRadarFrame(frame);
    }
#endif
    if (wasModelingBackground && !targetTracker.backgroundModelingActive()) {
        if (ui->backgroundStartButton) ui->backgroundStartButton->setText("开始建模");
        logMessage(QString("背景建模：完成 %1 帧，背景簇=%2。可勾选'启用背景过滤'用于降权。")
                       .arg(targetTracker.backgroundModelFrameCount())
                       .arg(targetTracker.backgroundModelTrackCount()));
    } else if (targetTracker.backgroundModelingActive() && frameNumber % 20 == 0) {
        logMessage(QString("背景建模中：%1/%2 帧，背景簇=%3")
                       .arg(targetTracker.backgroundModelFrameCount())
                       .arg(targetTracker.config().background_model_frames)
                       .arg(targetTracker.backgroundModelTrackCount()));
    }
    const bool autoMode = !ui->targetModeCombo || ui->targetModeCombo->currentIndex() == 0;
    if (!autoMode && targetTracker.lockedTarget().valid &&
        !targetTracker.lockedTarget().has_measurement &&
        targetTracker.lockedTarget().lost_frames >=
            std::max(1, targetTracker.config().manual_unlock_lost_frames)) {
        const int lostId = targetTracker.lockedTarget().display_id;
        const int lostFrames = targetTracker.lockedTarget().lost_frames;
        targetTracker.unlockTarget("manual_lost_timeout");
        logMessage(QString("手动模式：人员 P%1 连续丢失 %2 帧，已解除锁定；停止输出坐标，等待重新手动选择。")
                       .arg(lostId)
                       .arg(lostFrames));
    }
    if (autoMode && !targetTracker.lockedTarget().valid && !targetTracker.candidates().empty()) {
        autoSelectFollowTarget(currentAutoSelectRule());
    } else if (autoMode && targetTracker.lockedTarget().valid) {
        const auto& locked = targetTracker.lockedTarget();
        const auto& cfg = targetTracker.config();
        const auto motionPriority = [](const TargetTracking::CandidateTarget& c) {
            return c.moving_ratio * 3.0f + c.average_abs_velocity * 2.0f;
        };
        const auto isMovingCandidate = [&cfg](const TargetTracking::CandidateTarget& c) {
            return c.moving_ratio >= cfg.moving_ratio_select_threshold ||
                   c.average_abs_velocity >= cfg.moving_velocity_select_threshold;
        };

        const TargetTracking::CandidateTarget* lockedCandidate = nullptr;
        const TargetTracking::CandidateTarget* bestMovingCandidate = nullptr;
        float bestMovingPriority = -1.0f;

        for (const auto& candidate : targetTracker.candidates()) {
            if (candidate.internal_uid == locked.matched_internal_uid) {
                lockedCandidate = &candidate;
            }
            if (candidate.internal_uid != locked.matched_internal_uid &&
                isMovingCandidate(candidate)) {
                const float priority = motionPriority(candidate);
                if (priority > bestMovingPriority) {
                    bestMovingPriority = priority;
                    bestMovingCandidate = &candidate;
                }
            }
        }

        const float lockedPriority = lockedCandidate ? motionPriority(*lockedCandidate) : 0.0f;
        const bool lockedLooksStatic = !lockedCandidate || !isMovingCandidate(*lockedCandidate);
        const bool lockedLooksBackground = lockedCandidate &&
                                           lockedCandidate->background_score >= cfg.background_reacquire_threshold;
        const bool lockIsOldEnough = locked.stable_frames >= cfg.min_locked_frames_before_global_reacquire ||
                                     locked.lost_frames > 0;
        const bool lockedIsWeak = locked.lost_frames >= cfg.reacquire_min_lost_frames ||
                                  locked.confidence < cfg.hard_reacquire_confidence_threshold ||
                                  (lockIsOldEnough &&
                                   locked.confidence < cfg.reacquire_confidence_threshold) ||
                                  (lockIsOldEnough && lockedLooksBackground);
        const float absYawRate = currentTrackingChassis.valid
            ? std::abs(currentTrackingChassis.wz_radps) : 0.0f;
        const float turnReselectMargin =
            std::max(0.0f, absYawRate - cfg.high_turn_wz_threshold_radps) *
            cfg.turn_reselect_margin_gain;
        if (bestMovingCandidate &&
            lockedIsWeak &&
            lockedLooksStatic &&
            bestMovingPriority > lockedPriority + cfg.motion_switch_margin + turnReselectMargin) {
            // Do not perform the former single-frame global identity switch.
            // The normal tracker may only commit a reacquisition after the
            // independent hypothesis reaches reacquire_confirm_frames.  If the
            // old lock ultimately times out, auto mode can acquire an already
            // confirmed candidate as a new tracking session.
            if (verboseTrackingLogs && frameNumber % 10 == 0) {
                logMessage(QString("自动模式：发现更强候选 P%1，但拒绝单帧换人；继续使用门控/连续确认链路。")
                               .arg(bestMovingCandidate->id));
            }
        }
    }

#if RADAR_ENABLE_CAMERA
    syncCameraRadarSelection();
#endif
    updateLockedTrajectoryHistory(frameNumber);
    refreshTargetTable();
    refreshLockedTargetInfo();
    appendLockedTargetToTxt(frameNumber);

    const double trackerMs = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - trackingStart).count();
    recordTrackingRuntimeStats(points.size(), trackerMs, frameNumber);

    if (frameNumber % 10 == 0) {
        const auto& lock = targetTracker.lockedTarget();
        logMessage(QString("%1: frame=%2, window=%3, clusters=%4, candidates=%5, active_tracks=%6, lock=%7, measured=%8, lost=%9")
                       .arg(isOfflineTrackingMode ? "离线追踪" : "实时跟踪")
                       .arg(frameNumber)
                       .arg(trackingWindowFrames)
                       .arg(targetTracker.clusterDebugInfo().size())
                       .arg(targetTracker.candidates().size())
                       .arg(targetTracker.activeTrackCount())
                       .arg(lock.valid ? QString("P%1").arg(lock.display_id) : QString("NONE"))
                       .arg(lock.valid && lock.has_measurement ? 1 : 0)
                       .arg(lock.valid ? lock.lost_frames : 0));
    }
}

void RadarGUI::clearTargetPreview()
{
    previewTargetUid = 0;
    previewTargetDisplayId = -1;
    if (pinnedTargetRowHost) pinnedTargetRowHost->hide();
    if (targetTable) targetTable->horizontalHeader()->show();
    updateTargetSelectionControls();
}

void RadarGUI::updateTargetSelectionControls()
{
#if RADAR_ENABLE_CAMERA
    syncCameraRadarSelection();
#endif
    if (!ui || !ui->autoSelectButton) return;

    const bool manualMode = ui->targetModeCombo &&
                            ui->targetModeCombo->currentIndex() == 1;
    if (!manualMode) {
        ui->autoSelectButton->setText("执行自动选择");
        ui->autoSelectButton->setToolTip("按当前策略一次性选择人员");
        ui->autoSelectButton->setEnabled(true);
        return;
    }

    const auto& locked = targetTracker.lockedTarget();
    if (previewTargetUid != 0 && previewTargetDisplayId > 0) {
        const auto previewIt = targetTableCache.find(previewTargetDisplayId);
        const bool previewCurrent = previewIt != targetTableCache.end() &&
            previewIt->second.current &&
            previewIt->second.candidate.internal_uid == previewTargetUid;
        const bool alreadyLocked = locked.valid &&
            locked.matched_internal_uid == previewTargetUid;
        if (alreadyLocked) {
            ui->autoSelectButton->setText(
                QString("已锁定 P%1").arg(previewTargetDisplayId));
            ui->autoSelectButton->setToolTip("该人员已锁定");
            ui->autoSelectButton->setEnabled(false);
        } else if (!previewCurrent) {
            ui->autoSelectButton->setText(
                QString("等待 P%1 再出现").arg(previewTargetDisplayId));
            ui->autoSelectButton->setToolTip(
                "预选 UID 已保留，但灰色历史行当前不可确认");
            ui->autoSelectButton->setEnabled(false);
        } else {
            ui->autoSelectButton->setText(
                QString("确认锁定 P%1").arg(previewTargetDisplayId));
            ui->autoSelectButton->setToolTip(
                "将黄色预览人员的 internal_uid 提交给跟踪器");
            ui->autoSelectButton->setEnabled(true);
        }
        return;
    }

    if (locked.valid) {
        ui->autoSelectButton->setText(
            QString("已锁定 P%1").arg(locked.display_id));
        ui->autoSelectButton->setToolTip("勾选其他亮色人员可预览并切换");
    } else {
        ui->autoSelectButton->setText("请先勾选人员");
        ui->autoSelectButton->setToolTip("只有当前帧亮色行可勾选");
    }
    ui->autoSelectButton->setEnabled(false);
}

void RadarGUI::refreshTargetTable()
{
    if (!targetTable) initTargetTable();
    if (!targetTable) return;

    for (auto& kv : targetTableCache) {
        kv.second.current = false;
        kv.second.missed_updates++;
    }
    for (const auto& candidate : targetTracker.candidates()) {
        // ConfirmedOnly mode intentionally keeps tentative id=-1 tracks out of
        // the manual selection table. A visible person ID must be stable first.
        if (candidate.id <= 0) continue;
        auto& entry = targetTableCache[candidate.id];
        entry.candidate = candidate;
        entry.current = true;
        entry.missed_updates = 0;
    }

    const auto& locked = targetTracker.lockedTarget();
    for (auto it = targetTableCache.begin(); it != targetTableCache.end();) {
        const bool isLockedEntry = locked.valid &&
            it->second.candidate.internal_uid == locked.matched_internal_uid;
        if (it->first <= 0 ||
            (!isLockedEntry &&
             it->second.missed_updates > kTargetTableHoldUpdates)) {
            it = targetTableCache.erase(it);
        } else {
            ++it;
        }
    }

    if (previewTargetUid != 0) {
        // Preserve the preview UID while its disabled grey row is retained.
        // This gives the user time to verify the yellow marker and prevents a
        // one-frame sparse-point dropout from changing the intended identity.
        const bool previewStillKnown = std::any_of(
            targetTableCache.begin(), targetTableCache.end(),
            [this](const auto& item) {
                return item.second.candidate.internal_uid == previewTargetUid;
            });
        if (!previewStillKnown) {
            previewTargetUid = 0;
            previewTargetDisplayId = -1;
        }
    }

    std::uint64_t selectedUid = 0;
    if (QTableWidgetItem* currentItem = targetTable->currentItem()) {
        selectedUid = currentItem->data(Qt::UserRole + 1).toULongLong();
    }
    const QSignalBlocker blocker(targetTable);
    const QSignalBlocker pinnedBlocker(pinnedTargetTable);
    targetTable->setSortingEnabled(false);
    if (targetTable->columnCount() < 4) {
        targetTable->setColumnCount(4);
    }

    const TargetTableCacheEntry* pinnedEntry = nullptr;
    std::vector<const TargetTableCacheEntry*> scrollingEntries;
    scrollingEntries.reserve(targetTableCache.size());
    for (const auto& kv : targetTableCache) {
        if (previewTargetUid != 0 &&
            kv.second.candidate.internal_uid == previewTargetUid) {
            if (!pinnedEntry) pinnedEntry = &kv.second;
            continue;
        }
        scrollingEntries.push_back(&kv.second);
    }

    auto writeRow = [&](QTableWidget* table, int row, const TargetTableCacheEntry& entry,
                        bool pinnedRow) {
        const auto& c = entry.candidate;
        const bool isLocked = locked.valid &&
            c.internal_uid == locked.matched_internal_uid;
        const bool isPreviewIdentity = previewTargetUid != 0 &&
                                       c.internal_uid == previewTargetUid;
        const bool isPreview = entry.current && isPreviewIdentity;
        const bool interactive = entry.current || pinnedRow;

        QTableWidgetItem* selected = new QTableWidgetItem();
        selected->setFlags(interactive
            ? (Qt::ItemIsUserCheckable | Qt::ItemIsEnabled | Qt::ItemIsSelectable)
            : Qt::NoItemFlags);
        selected->setCheckState((isPreview || (pinnedRow && isPreviewIdentity) ||
                                 (previewTargetUid == 0 && isLocked))
                                    ? Qt::Checked : Qt::Unchecked);
        selected->setTextAlignment(Qt::AlignCenter);
        selected->setData(Qt::UserRole, c.id);
        selected->setData(Qt::UserRole + 1,
                          QVariant::fromValue<qulonglong>(
                              static_cast<qulonglong>(c.internal_uid)));
        selected->setData(Qt::UserRole + 2, entry.current);
        table->setItem(row, 0, selected);

        QTableWidgetItem* id = new QTableWidgetItem(QString("P%1").arg(c.id));
        id->setTextAlignment(Qt::AlignCenter);
        QFont idFont = id->font();
        idFont.setBold(pinnedRow);
        id->setFont(idFont);
        id->setData(Qt::UserRole, c.id);
        id->setData(Qt::UserRole + 1,
                    QVariant::fromValue<qulonglong>(
                        static_cast<qulonglong>(c.internal_uid)));
        id->setData(Qt::UserRole + 2, entry.current);
        id->setToolTip(pinnedRow
            ? QString("黄框人员 P%1 固定在表头下方，取消勾选后回到滚动列表。internal_uid=%2")
                  .arg(c.id)
                  .arg(static_cast<qulonglong>(c.internal_uid))
            : QString("人员 P%1, internal_uid=%2%3")
                  .arg(c.id)
                  .arg(static_cast<qulonglong>(c.internal_uid))
                  .arg(entry.current ? "" : "（当前帧未出现，不可选）"));
        table->setItem(row, 1, id);

        QTableWidgetItem* range = new QTableWidgetItem(QString::number(c.range_m, 'f', 2));
        range->setTextAlignment(Qt::AlignCenter);
        table->setItem(row, 2, range);

        QTableWidgetItem* velocity = new QTableWidgetItem(QString::number(c.average_abs_velocity, 'f', 2));
        velocity->setTextAlignment(Qt::AlignCenter);
        table->setItem(row, 3, velocity);

        for (int col = 0; col < 4; ++col) {
            QTableWidgetItem* item = table->item(row, col);
            if (!item) continue;
            item->setData(Qt::UserRole, c.id);
            item->setData(Qt::UserRole + 1,
                          QVariant::fromValue<qulonglong>(
                              static_cast<qulonglong>(c.internal_uid)));
            item->setData(Qt::UserRole + 2, entry.current);
            if (col > 0) {
                item->setFlags(interactive
                    ? (Qt::ItemIsEnabled | Qt::ItemIsSelectable)
                    : Qt::NoItemFlags);
            }
            if (!entry.current) {
                item->setForeground(QColor(150, 150, 150));
                item->setBackground(QColor(242, 242, 242));
            } else if (isPreview || pinnedRow) {
                item->setForeground(Qt::black);
                item->setBackground(QColor(255, 236, 135));
            } else if (isLocked) {
                item->setForeground(Qt::black);
                item->setBackground(QColor(205, 242, 214));
            } else {
                item->setForeground(Qt::black);
                item->setData(Qt::BackgroundRole, QVariant());
            }
        }
    };

    QScrollBar* scrollBar = targetTable->verticalScrollBar();
    const int scrollValue = scrollBar ? scrollBar->value() : 0;
    if (pinnedEntry && pinnedTargetTable && pinnedTargetRowHost) {
        targetTable->horizontalHeader()->hide();
        pinnedTargetTable->setRowCount(1);
        if (pinnedTargetTable->columnCount() < 4) pinnedTargetTable->setColumnCount(4);
        writeRow(pinnedTargetTable, 0, *pinnedEntry, true);
        pinnedTargetRowHost->show();
        updatePinnedTargetRowChrome();
    } else {
        if (pinnedTargetRowHost) pinnedTargetRowHost->hide();
        targetTable->horizontalHeader()->show();
    }

    targetTable->setRowCount(static_cast<int>(scrollingEntries.size()));
    int rowToRestore = -1;
    for (int row = 0; row < static_cast<int>(scrollingEntries.size()); ++row) {
        const auto& entry = *scrollingEntries[static_cast<std::size_t>(row)];
        writeRow(targetTable, row, entry, false);
        if (entry.candidate.internal_uid == selectedUid) rowToRestore = row;
    }
    if (pinnedEntry) {
        if (scrollBar) scrollBar->setValue(scrollValue);
    } else if (rowToRestore >= 0) {
        targetTable->selectRow(rowToRestore);
    }
    updateTargetSelectionControls();
}

void RadarGUI::refreshLockedTargetInfo()
{
    if (!ui || !ui->lockedTargetInfoText) return;

    const auto& locked = targetTracker.lockedTarget();
    if (!locked.valid) {
        ui->lockedTargetInfoText->setPlainText("当前未锁定人员");
        return;
    }

    float candidateVelocity = 0.0f;
    bool hasCandidateVelocity = false;
    for (const auto& c : targetTracker.candidates()) {
        if (c.internal_uid == locked.matched_internal_uid) {
            candidateVelocity = c.average_abs_velocity;
            hasCandidateVelocity = true;
            break;
        }
    }
    const float estimatedSpeed = std::sqrt(locked.vx * locked.vx + locked.vy * locked.vy);
    const QString measuredText = locked.has_measurement
        ? QString("观测坐标: x=%1 m, y=%2 m, range=%3 m, angle=%4 deg")
              .arg(locked.measured_x, 0, 'f', 2)
              .arg(locked.measured_y, 0, 'f', 2)
              .arg(locked.measured_range_m, 0, 'f', 2)
              .arg(locked.measured_angle_deg, 0, 'f', 1)
        : QString("观测坐标: 当前帧未匹配到候选目标，使用 Kalman 预测");
    const bool visionAllowed=visualMotionAllowed();
    const QString translationText = locked.control_allow_translation && visionAllowed ? "允许平移" : "禁止平移";
    const QString yawText = locked.control_allow_yaw && visionAllowed ? "允许转向" : "禁止转向";
    const auto& association = targetTracker.associationDiagnostics();

    ui->lockedTargetInfoText->setPlainText(
        QString("锁定人员 P%1  候选ID=%18  uid=%19\n滤波距离: %2 m  滤波角度: %3 deg\n滤波坐标: x=%4 m, y=%5 m\n%6\n点云速度: %7 m/s  目标估计速度: %8 m/s\n控制建议: %9  速度比例=%10  %11/%12\nlost=%13  stable=%14  rehit=%15/%16  confidence=%17\n关联=%20  NIS=%21  隐含速度=%22 m/s")
            .arg(locked.display_id)
            .arg(locked.range_m, 0, 'f', 2)
            .arg(locked.angle_deg, 0, 'f', 1)
            .arg(locked.x, 0, 'f', 2)
            .arg(locked.y, 0, 'f', 2)
            .arg(measuredText)
            .arg(hasCandidateVelocity ? candidateVelocity : 0.0f, 0, 'f', 2)
            .arg(estimatedSpeed, 0, 'f', 2)
            .arg(followControlStateName(locked.control_state))
            .arg(locked.control_speed_scale, 0, 'f', 2)
            .arg(translationText)
            .arg(yawText)
            .arg(locked.lost_frames)
            .arg(locked.stable_frames)
            .arg(locked.reacquire_hits)
            .arg(targetTracker.manualLockConfirmationPending()
                     ? targetTracker.config().manual_lock_confirm_frames
                     : targetTracker.config().reacquire_confirm_frames)
            .arg(locked.confidence, 0, 'f', 2)
            .arg(locked.matched_candidate_id)
            .arg(static_cast<qulonglong>(locked.matched_internal_uid))
            .arg(QString::fromStdString(association.source))
            .arg(association.nis, 0, 'f', 2)
             .arg(association.implied_speed_mps, 0, 'f', 2));
}

void RadarGUI::resetLockedTrajectoryHistory()
{
    lockedTrajectoryHistory.clear();
    lockedTrajectoryDisplayId = -1;
    lockedTrajectoryLastUid = 0;
    lockedTrajectoryLastFrame = -1;
}

void RadarGUI::updateLockedTrajectoryHistory(int frameNumber)
{
    // TargetTracker has already transformed its previous state into the
    // current radar frame. Apply exactly the same ego-motion transform to the
    // GUI-only history before adding this frame's sample.
    const auto& ego = targetTracker.egoMotionDiagnostics();
    if (ego.applied && !lockedTrajectoryHistory.empty()) {
        const float c = std::cos(ego.radar_dyaw_rad);
        const float s = std::sin(ego.radar_dyaw_rad);
        const auto transform = [=](float& x, float& y) {
            const float shiftedX = x - ego.radar_dx_m;
            const float shiftedY = y - ego.radar_dy_m;
            x = c * shiftedX + s * shiftedY;
            y = -s * shiftedX + c * shiftedY;
        };
        for (auto& sample : lockedTrajectoryHistory) {
            transform(sample.filtered_x, sample.filtered_y);
            if (sample.has_measurement) {
                transform(sample.measured_x, sample.measured_y);
            }
        }
    }

    const auto& locked = targetTracker.lockedTarget();
    if (!locked.valid) {
        resetLockedTrajectoryHistory();
        return;
    }

    // A new visible lock starts a new trail. A UID change under the same
    // visible lock is retained and marked explicitly instead of being hidden.
    if (lockedTrajectoryDisplayId != locked.display_id ||
        frameNumber <= lockedTrajectoryLastFrame) {
        resetLockedTrajectoryHistory();
        lockedTrajectoryDisplayId = locked.display_id;
    }

    LockedTrajectorySample sample;
    sample.frame = frameNumber;
    sample.display_id = locked.display_id;
    sample.internal_uid = locked.matched_internal_uid;
    sample.previous_internal_uid = lockedTrajectoryLastUid;
    sample.filtered_x = locked.x;
    sample.filtered_y = locked.y;
    sample.has_measurement = locked.has_measurement &&
        std::isfinite(locked.measured_x) && std::isfinite(locked.measured_y);
    if (sample.has_measurement) {
        sample.measured_x = locked.measured_x;
        sample.measured_y = locked.measured_y;
    }
    // Dashed means exactly "no committed radar measurement in this frame".
    // Control-state transitions (for example manual confirmation) must not make
    // a real measured sample look like a Kalman-only prediction.
    sample.prediction_only = !sample.has_measurement;
    sample.uid_changed = lockedTrajectoryLastUid != 0 &&
        locked.matched_internal_uid != 0 &&
        locked.matched_internal_uid != lockedTrajectoryLastUid;

    lockedTrajectoryHistory.push_back(sample);
    while (lockedTrajectoryHistory.size() > kLockedTrajectoryHistoryFrames) {
        lockedTrajectoryHistory.pop_front();
    }
    lockedTrajectoryDisplayId = locked.display_id;
    lockedTrajectoryLastUid = locked.matched_internal_uid;
    lockedTrajectoryLastFrame = frameNumber;
}

void RadarGUI::drawTrackingOverlay()
{
    if (!scene) return;

    constexpr qreal scale = 100.0;
    const bool showGlobalPointCloud = !ui->globalPointcloudCheck || ui->globalPointcloudCheck->isChecked();
    auto sceneX = [this](float x) -> qreal { return sceneXFromMeters(x); };
    auto sceneY = [this](float y) -> qreal { return sceneYFromMeters(y); };
    auto distance2 = [](float ax, float ay, float bx, float by) -> float {
        const float dx = ax - bx;
        const float dy = ay - by;
        return dx * dx + dy * dy;
    };

    QPen clusterPen(QColor(80, 130, 200, 150));
    clusterPen.setWidth(3);
    clusterPen.setStyle(Qt::DashLine);

    QPen candidatePen(QColor(30, 90, 220));
    candidatePen.setWidth(3);

    QPen previewPen(QColor(235, 170, 0));
    previewPen.setWidth(6);

    QPen backgroundPen(QColor(120, 120, 120, 180));
    backgroundPen.setWidth(3);
    backgroundPen.setStyle(Qt::DashLine);

    QPen lockedPen(QColor(20, 170, 80));
    lockedPen.setWidth(5);

    QPen filteredTrailPen(QColor(20, 180, 80, 220));
    filteredTrailPen.setWidth(5);
    QPen predictedTrailPen(QColor(20, 180, 80, 190));
    predictedTrailPen.setWidth(5);
    predictedTrailPen.setStyle(Qt::DashLine);
    QPen measuredTrailPen(QColor(245, 135, 35, 230));
    measuredTrailPen.setWidth(3);
    QPen uidSwitchPen(QColor(220, 35, 35));
    uidSwitchPen.setWidth(5);
    QPen velocityArrowPen(QColor(0, 120, 70));
    velocityArrowPen.setWidth(6);

    const auto& locked = targetTracker.lockedTarget();
    const std::uint64_t lockedUid = locked.valid ? locked.matched_internal_uid : 0;
    const auto& candidates = targetTracker.candidates();

    for (const auto& clusterInfo : targetTracker.clusterDebugInfo()) {
        const TargetTracking::CandidateTarget* matchedCandidate = nullptr;
        for (const auto& candidate : candidates) {
            if(candidate.id>0&&candidate.cluster_id==clusterInfo.cluster_id) {
                matchedCandidate=&candidate;break;
            }
        }

        const bool hasTarget = matchedCandidate != nullptr;
        const bool isLocked = hasTarget &&
                               matchedCandidate->internal_uid == lockedUid;
        const bool isPreview = hasTarget && previewTargetUid != 0 &&
                               matchedCandidate->internal_uid == previewTargetUid;
        if (!showGlobalPointCloud && !isLocked && !isPreview) {
            continue;
        }

        const bool isBackground = !isLocked && !isPreview &&
                                  clusterInfo.background_score >= 0.65f;
        const QColor overlayColor = !clusterInfo.has_current_frame_support ? QColor(160,170,180) : isPreview ? QColor(210, 145, 0) :
                                   (isLocked ? QColor(20, 150, 70) :
                                   (isBackground ? QColor(110, 110, 110) :
                                     (hasTarget ? QColor(30, 90, 220) : QColor(80, 130, 200))));
        const QPen boxPen = isPreview ? previewPen :
                            (isLocked ? lockedPen : (isBackground ? backgroundPen :
                            (hasTarget ? candidatePen : clusterPen)));

        if (!showGlobalPointCloud) {
            for (const auto& p : clusterInfo.points) {
                QColor pointColor = p.is_moving ? QColor(230, 40, 80) : QColor(20, 20, 20);
                QGraphicsEllipseItem* pointItem = scene->addEllipse(sceneX(p.x) - 12,
                                                                    sceneY(p.y) - 12,
                                                                    24, 24,
                                                                    QPen(pointColor),
                                                                    QBrush(pointColor));
                pointItem->setZValue(8);
                pointItems.push_back(pointItem);
            }
        }

        const qreal width = std::max<qreal>(clusterInfo.width_m * scale, 70.0);
        const qreal height = std::max<qreal>(clusterInfo.depth_m * scale, 70.0);
        const qreal left = sceneX(clusterInfo.x) - width / 2.0;
        const qreal top = sceneY(clusterInfo.y) - height / 2.0;

        QGraphicsRectItem* rect = scene->addRect(QRectF(left, top, width, height), boxPen);
        rect->setZValue(10);
        pointItems.push_back(rect);

        QString labelText = QString(clusterInfo.has_current_frame_support?"C%1":"历史 C%1").arg(clusterInfo.cluster_id);
        if (isBackground) {
            labelText = QString("BG C%1").arg(clusterInfo.cluster_id);
        } else if (hasTarget) {
            labelText = isPreview
                ? QString("C%1  PREVIEW P%2  uid=%3")
                      .arg(clusterInfo.cluster_id)
                      .arg(matchedCandidate->id)
                      .arg(static_cast<qulonglong>(matchedCandidate->internal_uid))
                : (isLocked
                ? QString("C%1  LOCK P%2  uid=%3")
                      .arg(clusterInfo.cluster_id)
                      .arg(locked.display_id)
                      .arg(static_cast<qulonglong>(matchedCandidate->internal_uid))
                : QString("C%1  P%2").arg(clusterInfo.cluster_id).arg(matchedCandidate->id));
        }

        QGraphicsTextItem* label = scene->addText(labelText, QFont("Arial", 11, QFont::Bold));
        label->setDefaultTextColor(overlayColor);
        label->setFlag(QGraphicsItem::ItemIgnoresTransformations);
        label->setPos(hasTarget?sceneX(matchedCandidate->x):left,
                      hasTarget?sceneY(matchedCandidate->y):top);
        label->setZValue(11);
        pointItems.push_back(label);

        if (hasTarget) {
            QGraphicsEllipseItem* marker = scene->addEllipse(sceneX(matchedCandidate->x) - 25,
                                                             sceneY(matchedCandidate->y) - 25,
                                                             50, 50,
                                                             isPreview ? previewPen :
                                                                 (isLocked ? lockedPen : candidatePen),
                                                             Qt::NoBrush);
            marker->setZValue(19);
            pointItems.push_back(marker);
        }
    }

    if (locked.valid) {
        // Filtered trajectory: solid green when a measurement is committed,
        // dashed green while the lock is propagated by prediction.
        for (std::size_t i = 1; i < lockedTrajectoryHistory.size(); ++i) {
            const auto& previous = lockedTrajectoryHistory[i - 1];
            const auto& current = lockedTrajectoryHistory[i];
            const QPen& pen = current.prediction_only
                ? predictedTrailPen : filteredTrailPen;
            auto* segment = scene->addLine(sceneX(previous.filtered_x),
                                           sceneY(previous.filtered_y),
                                           sceneX(current.filtered_x),
                                           sceneY(current.filtered_y), pen);
            segment->setZValue(21);
            pointItems.push_back(segment);
        }

        // Raw radar measurements remain visible as orange dots. They are kept
        // separate from the filtered line so a DOA/cluster jump is obvious.
        for (const auto& sample : lockedTrajectoryHistory) {
            if (sample.has_measurement) {
                constexpr qreal measurementDiameter = 18.0;
                auto* measurement = scene->addEllipse(
                    sceneX(sample.measured_x) - measurementDiameter / 2.0,
                    sceneY(sample.measured_y) - measurementDiameter / 2.0,
                    measurementDiameter, measurementDiameter,
                    measuredTrailPen, QBrush(QColor(245, 135, 35, 150)));
                measurement->setZValue(22);
                pointItems.push_back(measurement);
            }
            if (sample.uid_changed) {
                const qreal x = sceneX(sample.filtered_x);
                const qreal y = sceneY(sample.filtered_y);
                constexpr qreal markerRadius = 20.0;
                auto* slashA = scene->addLine(x - markerRadius, y - markerRadius,
                                              x + markerRadius, y + markerRadius,
                                              uidSwitchPen);
                auto* slashB = scene->addLine(x - markerRadius, y + markerRadius,
                                              x + markerRadius, y - markerRadius,
                                              uidSwitchPen);
                slashA->setZValue(24);
                slashB->setZValue(24);
                pointItems.push_back(slashA);
                pointItems.push_back(slashB);
                auto* uidLabel = scene->addText(
                    QString("UID %1>%2")
                        .arg(static_cast<qulonglong>(sample.previous_internal_uid))
                        .arg(static_cast<qulonglong>(sample.internal_uid)),
                    QFont("Arial", 18, QFont::Bold));
                uidLabel->setDefaultTextColor(QColor(220, 35, 35));
                uidLabel->setPos(x + markerRadius, y - markerRadius - 20.0);
                uidLabel->setZValue(25);
                pointItems.push_back(uidLabel);
            }
        }

        // Velocity direction is drawn from the current filtered position. The
        // length is clamped for visibility, so the arrow is directional rather
        // than an exact velocity scale.
        const float speed = std::hypot(locked.vx, locked.vy);
        if (std::isfinite(speed) && speed >= 0.08f) {
            const float arrowDistance = std::clamp(speed * 0.75f, 0.35f, 2.0f);
            const float endXMetres = locked.x + locked.vx / speed * arrowDistance;
            const float endYMetres = locked.y + locked.vy / speed * arrowDistance;
            const qreal startX = sceneX(locked.x);
            const qreal startY = sceneY(locked.y);
            const qreal endX = sceneX(endXMetres);
            const qreal endY = sceneY(endYMetres);
            auto* shaft = scene->addLine(startX, startY, endX, endY,
                                         velocityArrowPen);
            shaft->setZValue(23);
            pointItems.push_back(shaft);

            const qreal dx = endX - startX;
            const qreal dy = endY - startY;
            const qreal length = std::hypot(dx, dy);
            if (length > 1e-3) {
                const qreal ux = dx / length;
                const qreal uy = dy / length;
                constexpr qreal headLength = 28.0;
                constexpr qreal headHalfWidth = 12.0;
                const qreal baseX = endX - ux * headLength;
                const qreal baseY = endY - uy * headLength;
                auto* headA = scene->addLine(
                    endX, endY, baseX - uy * headHalfWidth,
                    baseY + ux * headHalfWidth, velocityArrowPen);
                auto* headB = scene->addLine(
                    endX, endY, baseX + uy * headHalfWidth,
                    baseY - ux * headHalfWidth, velocityArrowPen);
                headA->setZValue(23);
                headB->setZValue(23);
                pointItems.push_back(headA);
                pointItems.push_back(headB);
            }
        }

        if (!locked.has_measurement) {
            QGraphicsEllipseItem* predicted = scene->addEllipse(sceneX(locked.x) - 18,
                                                                sceneY(locked.y) - 18,
                                                                36, 36,
                                                                lockedPen,
                                                                QBrush(QColor(20, 170, 80, 80)));
            predicted->setZValue(20);
            pointItems.push_back(predicted);
        }

        const qreal roiRadius = static_cast<qreal>(targetTracker.config().roi_radius_m) * scale;
        QGraphicsEllipseItem* roi = scene->addEllipse(sceneX(locked.x) - roiRadius,
                                                      sceneY(locked.y) - roiRadius,
                                                      roiRadius * 2.0,
                                                      roiRadius * 2.0,
                                                      lockedPen,
                                                      Qt::NoBrush);
        roi->setZValue(18);
        pointItems.push_back(roi);
    }
}

#if RADAR_ENABLE_CAMERA
void RadarGUI::syncCameraRadarSelection()
{
    if(!cameraAssist_)return;
    const auto output=targetTracker.output();
    const bool valid=output.target.valid&&!isOfflineMode&&!isOfflineTrackingMode&&!isTestMode;
    cameraAssist_->setLockedTarget(valid?output.target.matched_internal_uid:0,
        output.frame.session_id,output.selection.selection_epoch,
        valid?output.target.display_id:-1);
}
#endif

bool RadarGUI::selectFollowTargetById(int candidateId)
{
    const bool ok = targetTracker.selectTargetById(candidateId);
    if (ok) {
#if RADAR_ENABLE_CAMERA
        syncCameraRadarSelection();
#endif
        const auto& locked = targetTracker.lockedTarget();
        logMessage(QString("已选择人员 P%1：正在连续确认 %2 帧；range=%3m, angle=%4deg")
                       .arg(locked.display_id)
                       .arg(targetTracker.config().manual_lock_confirm_frames)
                       .arg(locked.range_m, 0, 'f', 2)
                       .arg(locked.angle_deg, 0, 'f', 1));
    } else {
        logMessage(QString("人工选择失败: 未找到候选人员 P%1").arg(candidateId));
    }
    return ok;
}

bool RadarGUI::selectFollowTargetByUid(std::uint64_t internalUid)
{
    const bool ok = targetTracker.selectTargetByUid(internalUid);
    if (ok) {
#if RADAR_ENABLE_CAMERA
        syncCameraRadarSelection();
#endif
        const auto& locked = targetTracker.lockedTarget();
        logMessage(QString("已选择人员 P%1 / uid=%2：正在连续确认 %3 帧；range=%4m, angle=%5deg")
                       .arg(locked.display_id)
                       .arg(static_cast<qulonglong>(locked.matched_internal_uid))
                       .arg(targetTracker.config().manual_lock_confirm_frames)
                       .arg(locked.range_m, 0, 'f', 2)
                       .arg(locked.angle_deg, 0, 'f', 1));
    } else {
        logMessage(QString("人工选择失败: uid=%1 已不在当前候选列表")
                       .arg(static_cast<qulonglong>(internalUid)));
    }
    return ok;
}

bool RadarGUI::autoSelectFollowTarget(TargetTracking::AutoSelectRule rule)
{
    const bool ok = targetTracker.autoSelect(rule);
    if (ok) {
#if RADAR_ENABLE_CAMERA
        syncCameraRadarSelection();
#endif
        const auto& locked = targetTracker.lockedTarget();
        logMessage(QString("自动选择[%1]锁定 P%2: range=%3m, angle=%4deg")
                       .arg(autoSelectRuleName(rule))
                       .arg(locked.display_id)
                       .arg(locked.range_m, 0, 'f', 2)
                       .arg(locked.angle_deg, 0, 'f', 1));
    } else {
        logMessage(QString("自动选择失败: 当前没有候选目标，策略=%1").arg(autoSelectRuleName(rule)));
    }
    return ok;
}

void RadarGUI::printOfflineTrackingDebug(const ProcessingResult& result)
{
    const auto& clusters = targetTracker.clusterDebugInfo();
    const auto& candidates = targetTracker.candidates();
    const auto& locked = targetTracker.lockedTarget();

    std::cout << "\n[离线追踪] frame=" << result.frame_number
              << ", raw_points=" << result.point_cloud.size()
              << ", clusters=" << clusters.size()
              << ", candidates=" << candidates.size()
              << std::endl;

    for (const auto& clusterInfo : clusters) {
        std::cout << "[聚类簇] C" << clusterInfo.cluster_id
                  << " person_like=" << (clusterInfo.person_like ? 1 : 0)
                  << " x=" << clusterInfo.x
                  << " y=" << clusterInfo.y
                  << " range=" << clusterInfo.range_m
                  << " angle=" << clusterInfo.angle_deg
                  << " avg_abs_v=" << clusterInfo.average_abs_velocity
                  << " moving_ratio=" << clusterInfo.moving_ratio
                  << " width=" << clusterInfo.width_m
                  << " depth=" << clusterInfo.depth_m
                  << " points=" << clusterInfo.point_count
                  << " bg=" << clusterInfo.background_score
                  << " score=" << clusterInfo.score
                  << std::endl;
        for (size_t i = 0; i < clusterInfo.points.size(); ++i) {
            const auto& p = clusterInfo.points[i];
            std::cout << "  [簇内点] C" << clusterInfo.cluster_id
                      << "#" << i
                      << " x=" << p.x
                      << " y=" << p.y
                      << " rel_v=" << p.relative_velocity
                      << " abs_v=" << p.absolute_velocity
                      << " moving=" << (p.is_moving ? 1 : 0)
                      << std::endl;
        }
    }

    for (const auto& c : candidates) {
        std::cout << "[候选目标] T" << c.id
                  << " uid=" << c.internal_uid
                  << " x=" << c.x
                  << " y=" << c.y
                  << " range=" << c.range_m
                  << " angle=" << c.angle_deg
                  << " avg_abs_v=" << c.average_abs_velocity
                  << " moving_ratio=" << c.moving_ratio
                  << " width=" << c.width_m
                  << " depth=" << c.depth_m
                  << " points=" << c.point_count
                  << " bg=" << c.background_score
                  << " score=" << c.score
                  << std::endl;
    }

    if (locked.valid) {
        std::cout << "[锁定目标] T" << locked.display_id
                  << " x=" << locked.x
                  << " y=" << locked.y
                  << " range=" << locked.range_m
                  << " angle=" << locked.angle_deg
                  << " lost_frames=" << locked.lost_frames
                  << " stable_frames=" << locked.stable_frames
                  << " confidence=" << locked.confidence
                  << std::endl;
    } else {
        std::cout << "[锁定目标] 当前未锁定" << std::endl;
    }
}

bool RadarGUI::load_config_from_file(const std::string& path)
{
    try {
        IWR6843AOP_TLV tlv(path);
        radar_config = tlv.get_params();
        config_file = path;
        updateSceneRectFromRadarConfig(true);

        // 在"当前状态"中展示加载的雷达参数
        QString params;
        params += "\n==== 从文件加载的雷达参数 ====:\n";
        for (const auto& kv : radar_config) {
            params += QString("%1: %2\n").arg(QString::fromStdString(kv.first)).arg(kv.second);
        }
        if (ui->textEdit) ui->textEdit->append(params);
        logMessage(QString("已从配置文件加载雷达参数: %1").arg(QString::fromStdString(path)));
        return true;
    } catch (const std::exception& e) {
        logMessage(QString("加载配置文件失败：%1").arg(e.what()));
        return false;
    }
}

void RadarGUI::on_offlineProcessButton_clicked()
{
    if (collector || is_capturing) { logMessage("请先停止实时采集再进行离线处理。"); return; }
    if (!offlineTimer) {
        offlineTimer = new QTimer(this);
        connect(offlineTimer, &QTimer::timeout, this, &RadarGUI::processOfflineFrame);
    }

    QComboBox* activeCombo = isOfflineTrackingMode ? offlineTrackCombo : offlineBinCombo;
    QPushButton* activeButton = isOfflineTrackingMode ? offlineTrackButton : offlineProcessButton;
    if (!activeCombo || !activeButton) {
        logMessage("错误：离线处理控件未初始化");
        return;
    }

    // 如果定时器正在运行，则停止处理
    if (offlineTimer->isActive()) {
        offlineTimer->stop();
        activeButton->setText(isOfflineTrackingMode ? "追踪" : "处理");
        if (offlineBinFile.is_open()) {
            offlineBinFile.close();
        }
        reset_signal_processing_session();
        finalizeTrackingSession("offline_user_stopped");
        targetTracker.reset();
        offlineTrackingWindow.clear();
        resetTrackingRuntimeStats();
        resetTrackTxtOutput();
        const QString modeName = isOfflineTrackingMode ? "离线追踪" : "离线点云";
        logMessage(modeName + "已停止");
        if (ui->textEdit) ui->textEdit->append(modeName + "已停止\n");
        refreshTargetTable();
        refreshLockedTargetInfo();
        return;
    }

    // ========== 开始离线处理 ==========

    // 2. 检查bin文件选择
    if (activeCombo->currentIndex() < 0) {
        logMessage("错误：未选择bin文件");
        return;
    }

    // 3. 打开bin文件
    std::string binPath = "./bin/" + activeCombo->currentText().toStdString();
    offlineBinFile.open(binPath, std::ios::binary);
    if (!offlineBinFile) {
        logMessage(QString("错误：无法打开文件 %1").arg(QString::fromStdString(binPath)));
        return;
    }

    // Recover acquisition-time DSP odom and tracker-time odom from the
    // realtime diagnostic CSV that belongs to this BIN.  The environment
    // variable RADAR_OFFLINE_DIAGNOSTIC_CSV can select an explicit file.
    std::vector<std::string> diagnosticSearchDirectories;
    diagnosticSearchDirectories.push_back(
        QDir(QDir::currentPath()).filePath("tracking_diagnostics").toStdString());
    diagnosticSearchDirectories.push_back(
        QDir(QDir::currentPath()).filePath("tracking").toStdString());
    diagnosticSearchDirectories.push_back(
        QDir(QDir::currentPath()).filePath("t/tracking_diagnostics").toStdString());
    const QFileInfo binInfo(QString::fromStdString(binPath));
    diagnosticSearchDirectories.push_back(
        QDir(binInfo.absolutePath()).filePath("../tracking_diagnostics").toStdString());
    diagnosticSearchDirectories.push_back(
        QDir(binInfo.absolutePath()).filePath("../tracking").toStdString());
    std::string replayMessage;
    const bool replayLoaded = offlineOdomReplay.loadForBin(
        binPath, diagnosticSearchDirectories, &replayMessage);
    logMessage(QString::fromStdString(
        std::string("offline motion replay: ") + replayMessage));
    if (!replayLoaded) {
        offlineOdomReplay.clear();
    }

    // Reload for each recording: a previous live/default cfg must not remain cached.
    std::string replayCfg;
    if (replayLoaded && !offlineOdomReplay.recordedConfigPath().empty()) {
        QString recordedConfig = QString::fromStdString(offlineOdomReplay.recordedConfigPath());
        recordedConfig.replace('\\', '/');
        const QString name = QFileInfo(recordedConfig).fileName();
        replayCfg = QDir("config").filePath(name).toStdString();
        if (ui->configFileCombo) {
            const int index = ui->configFileCombo->findText(name);
            if (index >= 0) ui->configFileCombo->setCurrentIndex(index);
        }
    } else if (ui->configFileCombo && ui->configFileCombo->currentIndex() >= 0) {
        replayCfg = "./config/" + ui->configFileCombo->currentText().toStdString();
    }
    if (replayCfg.empty() || !load_config_from_file(replayCfg)) {
        offlineBinFile.close();
        logMessage("离线处理停止：无法加载采集时的配置；请将对应 cfg 放入 config。");
        return;
    }
    try {
        offlineFrameLength = static_cast<int>(RadarFrameLayout::frameBytes(
            RadarFrameLayout::paramsFromMap(radar_config)));
    } catch (const std::exception& error) {
        offlineBinFile.close();
        logMessage(QString("离线帧布局无效：%1").arg(error.what()));
        return;
    }
    offlineFrameCount = 0;

    // 5. 获取文件总大小，用于估算总帧数
    offlineBinFile.seekg(0, std::ios::end);
    auto totalBytes = offlineBinFile.tellg();
    offlineBinFile.seekg(0, std::ios::beg);
    try { offlineFrameIndex.load(binPath, static_cast<std::size_t>(totalBytes / offlineFrameLength)); }
    catch (const std::exception& error) {
        offlineBinFile.close(); logMessage(QString::fromStdString(error.what())); return;
    }
    ::capture_start_time = std::chrono::system_clock::now();

    // 6. 重置信号处理会话（清零帧计数、关闭旧的点云输出文件）
    reset_signal_processing_session();
    targetTracker.reset();
    offlineTrackingWindow.clear();
    resetTrackingRuntimeStats();
    trackingStats.input_path = binPath;
    if (isOfflineTrackingMode) {
        ensureTrackingDiagnosticLogOpen();
    }
    resetTrackTxtOutput();
    refreshTargetTable();
    refreshLockedTargetInfo();

    // 7. 启动定时器，每100ms处理一帧（约10fps显示速率）
    activeButton->setText("停止");
    offlineTimer->start(100);

    int totalFrames = (offlineFrameLength > 0) ? static_cast<int>(totalBytes / offlineFrameLength) : 0;
    const QString modeName = isOfflineTrackingMode ? "离线追踪" : "离线点云";
    logMessage(QString("开始%1: %2 (共约%3帧, 每帧%4字节)")
        .arg(modeName)
        .arg(QString::fromStdString(binPath))
        .arg(totalFrames)
        .arg(offlineFrameLength));
    std::cout << "[" << modeName.toStdString() << "] bin=" << binPath
              << ", bytes_per_frame=" << offlineFrameLength
              << ", total_frames=" << totalFrames
              << ", pipeline=bin -> signal_processing -> point_cloud";
    if (isOfflineTrackingMode) {
        std::cout << " -> clustering -> candidates -> tracking";
    }
    std::cout << std::endl;
    if (ui->textEdit) {
        ui->textEdit->append(QString("开始%1: %2\n总帧数: %3\n")
            .arg(modeName)
            .arg(QString::fromStdString(binPath))
            .arg(totalFrames));
    }
}

void RadarGUI::processOfflineFrame()
{
    QPushButton* activeButton = isOfflineTrackingMode ? offlineTrackButton : offlineProcessButton;
    const QString modeName = isOfflineTrackingMode ? "离线追踪" : "离线点云";

    if (!offlineBinFile.is_open() || offlineBinFile.eof()) {
        offlineTimer->stop();
        if (activeButton) activeButton->setText(isOfflineTrackingMode ? "追踪" : "处理");
        if (offlineBinFile.is_open()) {
            offlineBinFile.close();
        }
        reset_signal_processing_session();
        finalizeTrackingSession("offline_completed");
        logMessage(QString("%1完成，共处理 %2 帧").arg(modeName).arg(offlineFrameCount));
        if (ui->textEdit) ui->textEdit->append(QString("%1完成，共处理 %2 帧\n").arg(modeName).arg(offlineFrameCount));
        return;
    }

    std::vector<char> frame_data(offlineFrameLength);
    offlineBinFile.read(frame_data.data(), offlineFrameLength);
    auto bytes_read = offlineBinFile.gcount();

    // 最后一帧不完整则结束处理
    if (bytes_read < offlineFrameLength) {
        offlineTimer->stop();
        if (activeButton) activeButton->setText(isOfflineTrackingMode ? "追踪" : "处理");
        offlineBinFile.close();
        reset_signal_processing_session();
        finalizeTrackingSession(bytes_read > 0
                                    ? "offline_incomplete_last_frame"
                                    : "offline_completed");
        if (bytes_read > 0) {
            logMessage(QString("%1完成（最后帧不完整），共处理 %2 帧").arg(modeName).arg(offlineFrameCount));
        } else {
            logMessage(QString("%1完成，共处理 %2 帧").arg(modeName).arg(offlineFrameCount));
        }
        if (ui->textEdit) ui->textEdit->append(QString("%1完成，共处理 %2 帧\n").arg(modeName).arg(offlineFrameCount));
        return;
    }

    // Replay the exact per-frame DSP odom sample and radar timestamp when a
    // matching realtime diagnostics CSV was found.  Never read today's live
    // chassis state while processing an old recording.
    const auto indexContext = offlineFrameIndex.frame(static_cast<std::size_t>(offlineFrameCount));
    const int replayFrameNumber = indexContext.frame_number;
    const OfflineMotionFrame* replayMotion =
        offlineOdomReplay.frame(replayFrameNumber);
    SignalProcessingFrameContext processingContext;
    processingContext.source_frame_number = replayFrameNumber;
    processingContext.radar_timestamp = offlineFrameIndex.loaded() ? RadarTime::timestamp(indexContext.sample_time_sec) :
        offlineOdomReplay.radarTimestampForFrame(replayFrameNumber, radar_config.at("framePeriodicity"));
    processingContext.frame = indexContext;
    const SignalProcessingFrameContext* processingContextPtr = &processingContext;
    bool enableOfflineDspCompensation = false;
    if (replayMotion) {
        if (replayMotion->has_dsp_sample) {
            processingContext.use_chassis_override = true;
            processingContext.chassis = replayMotion->dsp_sample;
            processingContext.has_odom_age = replayMotion->has_dsp_age;
            processingContext.odom_age_sec = replayMotion->dsp_age_sec;
            enableOfflineDspCompensation = true;
        }
        processingContextPtr = &processingContext;
    }
    processingContext.frame.session_id = trackingStats.session_id;
    processingContext.frame.frame_number = replayFrameNumber;
    if (!offlineFrameIndex.loaded())
        processingContext.frame.time_quality = processingContext.radar_timestamp.empty() ? "unknown" : "recorded_or_estimated";
    if (!processingContext.radar_timestamp.empty()) {
        const auto parsed = QDateTime::fromString(QString::fromStdString(processingContext.radar_timestamp), "yyyy-MM-dd hh:mm:ss.zzz");
        if (parsed.isValid()) processingContext.frame.sample_time_sec = parsed.toMSecsSinceEpoch()/1000.0;
    }
    ProcessingResult result = run_signal_processing_pipeline(
        frame_data, radar_config, enableOfflineDspCompensation,
        processingContextPtr);
    offlineFrameCount++;

    if (isOfflineTrackingMode) {
        offlineTrackingWindow.emplace_back(result.frame_number, result.point_cloud);
        while (offlineTrackingWindow.size() > static_cast<std::size_t>(trackingWindowFrames)) {
            offlineTrackingWindow.pop_front();
        }

        if (offlineTrackingWindow.size() >= static_cast<std::size_t>(trackingWindowFrames)) {
            const std::vector<PointCloud> trackingPoints = aggregateOfflineTrackingWindow();
            std::vector<TestPoint> windowPoints;
            windowPoints.reserve(trackingPoints.size());
            for (const auto& point : trackingPoints) {
                TestPoint displayPoint;
                displayPoint.x = point.x;
                displayPoint.y = point.y;
                displayPoint.relVel = point.relative_velocity;
                displayPoint.absVel = point.absolute_velocity;
                displayPoint.moving = point.is_moving;
                displayPoint.sourceFrame = point.source_frame;
                displayPoint.rangeBin = point.range_bin;
                displayPoint.dopplerBin = point.doppler_bin;
                displayPoint.snr = point.snr;
                displayPoint.doaPeakConfidence = point.doa_peak_confidence;
                windowPoints.push_back(displayPoint);
            }

            for (auto* item : pointItems) {
                scene->removeItem(item);
                delete item;
            }
            pointItems.clear();
            updateSceneRectFromRadarConfig(false);
            const bool showGlobalPointCloud = !ui->globalPointcloudCheck ||
                                              ui->globalPointcloudCheck->isChecked();
            if (showGlobalPointCloud && cluster) {
                cluster->renderWindow(windowPoints,
                                      offlineTrackingWindow.front().first,
                                      offlineTrackingWindow.back().first,
                                      false);
            } else if (cluster) {
                cluster->stopPlayback();
            }

            currentProcessingDiagnostics = result.diagnostics;
            currentFrameContext = result.frame;
            currentTrackingChassis = ChassisVelocitySample();
            currentTrackingOdomAgeFromReplay = false;
            currentTrackingOdomAgeSec = 0.0;
            if (replayMotion) {
                if (replayMotion->has_tracking_sample) {
                    currentTrackingChassis = replayMotion->tracking_sample;
                    currentTrackingOdomAgeFromReplay =
                        replayMotion->has_tracking_age;
                    currentTrackingOdomAgeSec =
                        replayMotion->tracking_age_sec;
                } else if (replayMotion->has_dsp_sample) {
                    currentTrackingChassis = replayMotion->dsp_sample;
                    currentTrackingOdomAgeFromReplay = replayMotion->has_dsp_age;
                    currentTrackingOdomAgeSec = replayMotion->dsp_age_sec;
                }
            }
            currentRadarTimestamp = result.timestamp;
            updateTargetTrackingCandidates(trackingPoints, result.frame_number);
            drawTrackingOverlay();
            if (infoText) {
                infoText->setPlainText(QString("离线追踪 - 窗口: %1-%2\n点数(%3帧总计): %4")
                    .arg(offlineTrackingWindow.front().first)
                    .arg(offlineTrackingWindow.back().first)
                    .arg(trackingWindowFrames)
                    .arg(static_cast<int>(trackingPoints.size())));
                infoText->show();
            }
            if (verboseTrackingLogs) printOfflineTrackingDebug(result);
        } else {
            displayProcessingResult(result);
        }
    } else {
        // Plain offline point-cloud processing remains single-frame.
        displayProcessingResult(result);
    }

    // Every 50 offline frames, keep one progress line without flooding the terminal.
    if (offlineFrameCount % 50 == 0) {
        logMessage(QString("%1: 已处理 %2 帧").arg(modeName).arg(offlineFrameCount));
    }
}

void RadarGUI::displayProcessingResult(const ProcessingResult& result)
{
    for (auto* item : pointItems) {
        scene->removeItem(item);
        delete item;
    }
    pointItems.clear();

    // 添加新的点云（与 processUdpData 使用相同的坐标变换）。
    // 取消“全局点云”时，这里只清屏和刷新坐标轴，锁定目标点由 drawTrackingOverlay() 绘制。
    updateSceneRectFromRadarConfig(false);
    const bool showGlobalPointCloud = !ui->globalPointcloudCheck || ui->globalPointcloudCheck->isChecked();
    if (showGlobalPointCloud) {
        for (const auto& point : result.point_cloud) {
            QColor color = point.is_moving ? Qt::red : Qt::black;  // 移动目标红色，静止目标黑色
            QGraphicsEllipseItem* ellipse = scene->addEllipse(
                sceneXFromMeters(point.x) - 10,
                sceneYFromMeters(point.y) - 10,
                20, 20, QPen(color), QBrush(color));
            pointItems.push_back(ellipse);
        }
    }

    if (infoText) {
        infoText->setPlainText(QString("当前帧: %1\n点数: %2")
            .arg(result.frame_number)
            .arg(result.point_cloud.size()));
        infoText->show();
    }

    // 确保 sceneRect 与当前雷达配置保持一致。
    updateSceneRectFromRadarConfig(false);
}
void RadarGUI::on_pushButton_6_clicked()
{
    if(shutdown_||!rosRuntime_)return;
    if(isOfflineMode||isOfflineTrackingMode||isTestMode){logMessage("请切换到实时雷达模式后启动小车追踪。");return;}
    rosRuntime_->setCaptureSession(is_capturing&&collector?collector->session_id():0);
    rosRuntime_->startFollow();
}
