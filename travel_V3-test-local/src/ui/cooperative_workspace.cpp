#include "radar_gui.hpp"
#include "ui_radar_gui_v2.h"
#include "gui_icons.hpp"
#if RADAR_ENABLE_CAMERA
#include "camera_replay.hpp"
#endif
#include <QApplication>
#include <QScreen>
#include <QToolBar>
#include <QSplitter>
#include <QTabWidget>
#include <QScrollArea>
#include <QHeaderView>
#include <QMenuBar>
#include <QStatusBar>
#include <QFormLayout>
#include <QTimer>
#include <algorithm>

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
    ui->groupBox->setTitle(QString());
    ui->pointCloudLayout->setContentsMargins(0, 0, 0, 0);
    ui->groupBox->setMinimumSize(0, 0);
    ui->graphicsView->setMinimumSize(200, 120);
    ui->graphicsView->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    auto* cooperativePage = new QWidget(splitter);
    cooperativePage->setObjectName("cooperativePage");
    cooperativePage->setMinimumWidth(280);
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
    cooperativeRadarHost->addWidget(ui->groupBox, 1);
    ui->groupBox->show();
#if RADAR_ENABLE_CAMERA
    cameraAssist_->setCompactView(true);
    cooperativeCameraHost->addWidget(cameraAssist_, 1);
    cameraAssist_->show();
#else
    cooperativeCameraHost->addWidget(new QLabel("此构建未启用摄像头", sensorSplit));
#endif
    QTimer::singleShot(0, this, [this] { positionZoomButtons(); applyZoom(); });
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
    connect(cameraAction, &QAction::triggered, this, [pages, cameraSettingsIndex] {
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

