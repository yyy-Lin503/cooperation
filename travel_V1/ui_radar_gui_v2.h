/********************************************************************************
** Form generated from reading UI file 'radar_gui_v2.ui'
**
** Created by: Qt User Interface Compiler version 5.15.3
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_RADAR_GUI_V2_H
#define UI_RADAR_GUI_V2_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QGraphicsView>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_RadarGUIRedesignV2
{
public:
    QAction *action;
    QAction *action_3;
    QAction *action_5;
    QAction *action_2;
    QAction *action_4;
    QWidget *centralwidget;
    QVBoxLayout *rootLayout;
    QHBoxLayout *mainLayout;
    QFrame *navFrame;
    QVBoxLayout *navLayout;
    QLabel *logoLabel;
    QPushButton *connectButton;
    QPushButton *configureButton;
    QPushButton *pushButton_2;
    QPushButton *pushButton_6;
    QPushButton *pushButton_3;
    QPushButton *pushButton;
    QFrame *configPanel;
    QVBoxLayout *configPanelLayout;
    QGroupBox *serialGroup;
    QGridLayout *serialGrid;
    QLabel *label;
    QComboBox *serialPortCombo;
    QPushButton *refreshPortsButton;
    QGroupBox *configGroup;
    QGridLayout *configGrid;
    QLabel *label_2;
    QComboBox *configFileCombo;
    QGroupBox *cfarGroup;
    QGridLayout *cfarGrid;
    QLabel *labelNex;
    QLabel *labelAlpha;
    QLabel *labelPr;
    QDoubleSpinBox *spinAlpha;
    QSpinBox *spinNex;
    QSpinBox *spinK;
    QSpinBox *spinProtectR;
    QLabel *labelK;
    QLabel *labelPd;
    QSpinBox *spinProtectD;
    QPushButton *cfarSaveButton;
    QGroupBox *captureGroup;
    QVBoxLayout *captureLayout;
    QTextEdit *textEdit;
    QGroupBox *pointCloudGroup;
    QGridLayout *pointCloudGrid;
    QComboBox *pointCloudCombo;
    QLabel *pointCloudLabel;
    QPushButton *pushButton_4;
    QGroupBox *offlinePointCloudGroup;
    QGridLayout *offlineGrid;
    QLabel *offlineBinLabel;
    QComboBox *offlineBinCombo;
    QPushButton *offlineProcessButton;
    QGroupBox *offlinePointCloudGroup_2;
    QGridLayout *offlineGrid_2;
    QLabel *offlineBinLabel_2;
    QComboBox *offlineBinCombo_2;
    QPushButton *offlineProcessButton_2;
    QGroupBox *logGroup;
    QVBoxLayout *logGroupLayout;
    QTextEdit *logTextEdit;
    QGroupBox *groupBox;
    QVBoxLayout *pointCloudLayout;
    QGraphicsView *graphicsView;
    QFrame *taskPanel;
    QVBoxLayout *taskPanelLayout;
    QGroupBox *targetTrackingGroup;
    QVBoxLayout *targetTrackingLayout;
    QLabel *targetModeLabel;
    QComboBox *targetModeCombo;
    QLabel *targetRuleLabel;
    QComboBox *targetRuleCombo;
    QPushButton *autoSelectButton;
    QTableWidget *tableWidget;
    QTextEdit *lockedTargetInfoText;
    QGroupBox *backgroundModelGroup;
    QVBoxLayout *backgroundLayout;
    QHBoxLayout *backgroundButtonsLayout;
    QPushButton *backgroundStartButton;
    QPushButton *backgroundClearButton;
    QCheckBox *backgroundFilterCheck;
    QCheckBox *globalPointcloudCheck;
    QMenuBar *menubar;
    QMenu *menu;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *RadarGUIRedesignV2)
    {
        if (RadarGUIRedesignV2->objectName().isEmpty())
            RadarGUIRedesignV2->setObjectName(QString::fromUtf8("RadarGUIRedesignV2"));
        RadarGUIRedesignV2->resize(1583, 1112);
        RadarGUIRedesignV2->setStyleSheet(QString::fromUtf8("\n"
"background-color: rgb(198, 225, 239);\n"
""));
        action = new QAction(RadarGUIRedesignV2);
        action->setObjectName(QString::fromUtf8("action"));
        QFont font;
        action->setFont(font);
        action_3 = new QAction(RadarGUIRedesignV2);
        action_3->setObjectName(QString::fromUtf8("action_3"));
        action_5 = new QAction(RadarGUIRedesignV2);
        action_5->setObjectName(QString::fromUtf8("action_5"));
        action_2 = new QAction(RadarGUIRedesignV2);
        action_2->setObjectName(QString::fromUtf8("action_2"));
        action_4 = new QAction(RadarGUIRedesignV2);
        action_4->setObjectName(QString::fromUtf8("action_4"));
        centralwidget = new QWidget(RadarGUIRedesignV2);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        rootLayout = new QVBoxLayout(centralwidget);
        rootLayout->setSpacing(8);
        rootLayout->setObjectName(QString::fromUtf8("rootLayout"));
        rootLayout->setContentsMargins(8, 8, 8, 8);
        mainLayout = new QHBoxLayout();
        mainLayout->setSpacing(8);
        mainLayout->setObjectName(QString::fromUtf8("mainLayout"));
        navFrame = new QFrame(centralwidget);
        navFrame->setObjectName(QString::fromUtf8("navFrame"));
        navFrame->setMinimumSize(QSize(86, 0));
        navFrame->setMaximumSize(QSize(96, 16777215));
        navFrame->setStyleSheet(QString::fromUtf8(""));
        navFrame->setFrameShape(QFrame::StyledPanel);
        navLayout = new QVBoxLayout(navFrame);
        navLayout->setSpacing(12);
        navLayout->setObjectName(QString::fromUtf8("navLayout"));
        logoLabel = new QLabel(navFrame);
        logoLabel->setObjectName(QString::fromUtf8("logoLabel"));
        logoLabel->setMinimumSize(QSize(68, 68));
        logoLabel->setMaximumSize(QSize(68, 68));
        logoLabel->setPixmap(QPixmap(QString::fromUtf8("8.png")));
        logoLabel->setAlignment(Qt::AlignCenter);

        navLayout->addWidget(logoLabel);

        connectButton = new QPushButton(navFrame);
        connectButton->setObjectName(QString::fromUtf8("connectButton"));
        connectButton->setMinimumSize(QSize(74, 60));
        QFont font1;
        font1.setPointSize(11);
        font1.setBold(true);
        font1.setWeight(75);
        connectButton->setFont(font1);

        navLayout->addWidget(connectButton);

        configureButton = new QPushButton(navFrame);
        configureButton->setObjectName(QString::fromUtf8("configureButton"));
        configureButton->setMinimumSize(QSize(74, 60));
        configureButton->setFont(font1);

        navLayout->addWidget(configureButton);

        pushButton_2 = new QPushButton(navFrame);
        pushButton_2->setObjectName(QString::fromUtf8("pushButton_2"));
        pushButton_2->setMinimumSize(QSize(74, 60));
        pushButton_2->setFont(font1);

        navLayout->addWidget(pushButton_2);

        pushButton_6 = new QPushButton(navFrame);
        pushButton_6->setObjectName(QString::fromUtf8("pushButton_6"));
        pushButton_6->setMinimumSize(QSize(74, 60));
        pushButton_6->setFont(font1);

        navLayout->addWidget(pushButton_6);

        pushButton_3 = new QPushButton(navFrame);
        pushButton_3->setObjectName(QString::fromUtf8("pushButton_3"));
        pushButton_3->setMinimumSize(QSize(74, 60));
        pushButton_3->setFont(font1);

        navLayout->addWidget(pushButton_3);

        pushButton = new QPushButton(navFrame);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));
        pushButton->setMinimumSize(QSize(74, 60));
        pushButton->setFont(font1);

        navLayout->addWidget(pushButton);


        mainLayout->addWidget(navFrame);

        configPanel = new QFrame(centralwidget);
        configPanel->setObjectName(QString::fromUtf8("configPanel"));
        configPanel->setMinimumSize(QSize(390, 0));
        configPanel->setMaximumSize(QSize(460, 16777215));
        configPanel->setFrameShape(QFrame::StyledPanel);
        configPanelLayout = new QVBoxLayout(configPanel);
        configPanelLayout->setSpacing(8);
        configPanelLayout->setObjectName(QString::fromUtf8("configPanelLayout"));
        serialGroup = new QGroupBox(configPanel);
        serialGroup->setObjectName(QString::fromUtf8("serialGroup"));
        QFont font2;
        font2.setPointSize(10);
        font2.setBold(true);
        font2.setWeight(75);
        serialGroup->setFont(font2);
        serialGrid = new QGridLayout(serialGroup);
        serialGrid->setObjectName(QString::fromUtf8("serialGrid"));
        label = new QLabel(serialGroup);
        label->setObjectName(QString::fromUtf8("label"));
        QSizePolicy sizePolicy(QSizePolicy::Fixed, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(label->sizePolicy().hasHeightForWidth());
        label->setSizePolicy(sizePolicy);
        QFont font3;
        font3.setPointSize(10);
        font3.setBold(false);
        font3.setWeight(50);
        label->setFont(font3);

        serialGrid->addWidget(label, 0, 0, 1, 1);

        serialPortCombo = new QComboBox(serialGroup);
        serialPortCombo->setObjectName(QString::fromUtf8("serialPortCombo"));
        QSizePolicy sizePolicy1(QSizePolicy::Preferred, QSizePolicy::Fixed);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(serialPortCombo->sizePolicy().hasHeightForWidth());
        serialPortCombo->setSizePolicy(sizePolicy1);
        serialPortCombo->setFont(font3);

        serialGrid->addWidget(serialPortCombo, 0, 1, 1, 1);

        refreshPortsButton = new QPushButton(serialGroup);
        refreshPortsButton->setObjectName(QString::fromUtf8("refreshPortsButton"));
        QSizePolicy sizePolicy2(QSizePolicy::Maximum, QSizePolicy::Fixed);
        sizePolicy2.setHorizontalStretch(0);
        sizePolicy2.setVerticalStretch(0);
        sizePolicy2.setHeightForWidth(refreshPortsButton->sizePolicy().hasHeightForWidth());
        refreshPortsButton->setSizePolicy(sizePolicy2);

        serialGrid->addWidget(refreshPortsButton, 0, 2, 1, 1);


        configPanelLayout->addWidget(serialGroup);

        configGroup = new QGroupBox(configPanel);
        configGroup->setObjectName(QString::fromUtf8("configGroup"));
        configGroup->setFont(font2);
        configGrid = new QGridLayout(configGroup);
        configGrid->setObjectName(QString::fromUtf8("configGrid"));
        label_2 = new QLabel(configGroup);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        sizePolicy.setHeightForWidth(label_2->sizePolicy().hasHeightForWidth());
        label_2->setSizePolicy(sizePolicy);
        label_2->setFont(font3);

        configGrid->addWidget(label_2, 0, 0, 1, 1);

        configFileCombo = new QComboBox(configGroup);
        configFileCombo->setObjectName(QString::fromUtf8("configFileCombo"));
        configFileCombo->setFont(font3);

        configGrid->addWidget(configFileCombo, 0, 1, 1, 1);


        configPanelLayout->addWidget(configGroup);

        cfarGroup = new QGroupBox(configPanel);
        cfarGroup->setObjectName(QString::fromUtf8("cfarGroup"));
        cfarGroup->setFont(font2);
        cfarGrid = new QGridLayout(cfarGroup);
        cfarGrid->setObjectName(QString::fromUtf8("cfarGrid"));
        labelNex = new QLabel(cfarGroup);
        labelNex->setObjectName(QString::fromUtf8("labelNex"));
        labelNex->setFont(font3);
        labelNex->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);

        cfarGrid->addWidget(labelNex, 0, 0, 1, 1);

        labelAlpha = new QLabel(cfarGroup);
        labelAlpha->setObjectName(QString::fromUtf8("labelAlpha"));
        labelAlpha->setFont(font3);
        labelAlpha->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);

        cfarGrid->addWidget(labelAlpha, 1, 0, 1, 1);

        labelPr = new QLabel(cfarGroup);
        labelPr->setObjectName(QString::fromUtf8("labelPr"));
        labelPr->setFont(font3);

        cfarGrid->addWidget(labelPr, 2, 0, 1, 1);

        spinAlpha = new QDoubleSpinBox(cfarGroup);
        spinAlpha->setObjectName(QString::fromUtf8("spinAlpha"));
        spinAlpha->setFont(font3);
        spinAlpha->setStyleSheet(QString::fromUtf8("background-color: rgb(235, 241, 255);"));
        spinAlpha->setDecimals(2);
        spinAlpha->setValue(3.500000000000000);

        cfarGrid->addWidget(spinAlpha, 1, 1, 1, 1);

        spinNex = new QSpinBox(cfarGroup);
        spinNex->setObjectName(QString::fromUtf8("spinNex"));
        spinNex->setFont(font3);
        spinNex->setStyleSheet(QString::fromUtf8("background-color: rgb(235, 241, 255);"));
        spinNex->setValue(8);

        cfarGrid->addWidget(spinNex, 0, 1, 1, 1);

        spinK = new QSpinBox(cfarGroup);
        spinK->setObjectName(QString::fromUtf8("spinK"));
        spinK->setFont(font3);
        spinK->setStyleSheet(QString::fromUtf8("background-color: rgb(235, 241, 255);"));
        spinK->setValue(5);

        cfarGrid->addWidget(spinK, 0, 3, 1, 1);

        spinProtectR = new QSpinBox(cfarGroup);
        spinProtectR->setObjectName(QString::fromUtf8("spinProtectR"));
        spinProtectR->setFont(font3);
        spinProtectR->setStyleSheet(QString::fromUtf8("background-color: rgb(235, 241, 255);"));
        spinProtectR->setValue(5);

        cfarGrid->addWidget(spinProtectR, 2, 1, 1, 1);

        labelK = new QLabel(cfarGroup);
        labelK->setObjectName(QString::fromUtf8("labelK"));
        labelK->setFont(font3);
        labelK->setTextFormat(Qt::AutoText);
        labelK->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);

        cfarGrid->addWidget(labelK, 0, 2, 1, 1);

        labelPd = new QLabel(cfarGroup);
        labelPd->setObjectName(QString::fromUtf8("labelPd"));
        labelPd->setFont(font3);

        cfarGrid->addWidget(labelPd, 1, 2, 1, 1);

        spinProtectD = new QSpinBox(cfarGroup);
        spinProtectD->setObjectName(QString::fromUtf8("spinProtectD"));
        spinProtectD->setFont(font3);
        spinProtectD->setStyleSheet(QString::fromUtf8("background-color: rgb(235, 241, 255);"));
        spinProtectD->setValue(5);

        cfarGrid->addWidget(spinProtectD, 1, 3, 1, 1);

        cfarSaveButton = new QPushButton(cfarGroup);
        cfarSaveButton->setObjectName(QString::fromUtf8("cfarSaveButton"));

        cfarGrid->addWidget(cfarSaveButton, 2, 3, 1, 1);


        configPanelLayout->addWidget(cfarGroup);

        captureGroup = new QGroupBox(configPanel);
        captureGroup->setObjectName(QString::fromUtf8("captureGroup"));
        captureGroup->setFont(font2);
        captureLayout = new QVBoxLayout(captureGroup);
        captureLayout->setObjectName(QString::fromUtf8("captureLayout"));
        textEdit = new QTextEdit(captureGroup);
        textEdit->setObjectName(QString::fromUtf8("textEdit"));
        textEdit->setMinimumSize(QSize(0, 150));
        textEdit->setFont(font3);

        captureLayout->addWidget(textEdit);


        configPanelLayout->addWidget(captureGroup);

        pointCloudGroup = new QGroupBox(configPanel);
        pointCloudGroup->setObjectName(QString::fromUtf8("pointCloudGroup"));
        pointCloudGroup->setFont(font2);
        pointCloudGrid = new QGridLayout(pointCloudGroup);
        pointCloudGrid->setObjectName(QString::fromUtf8("pointCloudGrid"));
        pointCloudCombo = new QComboBox(pointCloudGroup);
        pointCloudCombo->setObjectName(QString::fromUtf8("pointCloudCombo"));
        pointCloudCombo->setFont(font3);

        pointCloudGrid->addWidget(pointCloudCombo, 0, 1, 1, 1);

        pointCloudLabel = new QLabel(pointCloudGroup);
        pointCloudLabel->setObjectName(QString::fromUtf8("pointCloudLabel"));
        sizePolicy.setHeightForWidth(pointCloudLabel->sizePolicy().hasHeightForWidth());
        pointCloudLabel->setSizePolicy(sizePolicy);
        pointCloudLabel->setFont(font3);

        pointCloudGrid->addWidget(pointCloudLabel, 0, 0, 1, 1);

        pushButton_4 = new QPushButton(pointCloudGroup);
        pushButton_4->setObjectName(QString::fromUtf8("pushButton_4"));
        sizePolicy2.setHeightForWidth(pushButton_4->sizePolicy().hasHeightForWidth());
        pushButton_4->setSizePolicy(sizePolicy2);

        pointCloudGrid->addWidget(pushButton_4, 0, 2, 1, 1);


        configPanelLayout->addWidget(pointCloudGroup);

        offlinePointCloudGroup = new QGroupBox(configPanel);
        offlinePointCloudGroup->setObjectName(QString::fromUtf8("offlinePointCloudGroup"));
        offlinePointCloudGroup->setFont(font2);
        offlineGrid = new QGridLayout(offlinePointCloudGroup);
        offlineGrid->setObjectName(QString::fromUtf8("offlineGrid"));
        offlineBinLabel = new QLabel(offlinePointCloudGroup);
        offlineBinLabel->setObjectName(QString::fromUtf8("offlineBinLabel"));
        sizePolicy.setHeightForWidth(offlineBinLabel->sizePolicy().hasHeightForWidth());
        offlineBinLabel->setSizePolicy(sizePolicy);
        offlineBinLabel->setFont(font3);

        offlineGrid->addWidget(offlineBinLabel, 0, 0, 1, 1);

        offlineBinCombo = new QComboBox(offlinePointCloudGroup);
        offlineBinCombo->setObjectName(QString::fromUtf8("offlineBinCombo"));
        offlineBinCombo->setFont(font3);

        offlineGrid->addWidget(offlineBinCombo, 0, 1, 1, 1);

        offlineProcessButton = new QPushButton(offlinePointCloudGroup);
        offlineProcessButton->setObjectName(QString::fromUtf8("offlineProcessButton"));
        sizePolicy2.setHeightForWidth(offlineProcessButton->sizePolicy().hasHeightForWidth());
        offlineProcessButton->setSizePolicy(sizePolicy2);

        offlineGrid->addWidget(offlineProcessButton, 0, 2, 1, 1);


        configPanelLayout->addWidget(offlinePointCloudGroup);

        offlinePointCloudGroup_2 = new QGroupBox(configPanel);
        offlinePointCloudGroup_2->setObjectName(QString::fromUtf8("offlinePointCloudGroup_2"));
        offlinePointCloudGroup_2->setFont(font2);
        offlineGrid_2 = new QGridLayout(offlinePointCloudGroup_2);
        offlineGrid_2->setObjectName(QString::fromUtf8("offlineGrid_2"));
        offlineBinLabel_2 = new QLabel(offlinePointCloudGroup_2);
        offlineBinLabel_2->setObjectName(QString::fromUtf8("offlineBinLabel_2"));
        sizePolicy.setHeightForWidth(offlineBinLabel_2->sizePolicy().hasHeightForWidth());
        offlineBinLabel_2->setSizePolicy(sizePolicy);
        offlineBinLabel_2->setFont(font3);

        offlineGrid_2->addWidget(offlineBinLabel_2, 0, 0, 1, 1);

        offlineBinCombo_2 = new QComboBox(offlinePointCloudGroup_2);
        offlineBinCombo_2->setObjectName(QString::fromUtf8("offlineBinCombo_2"));
        offlineBinCombo_2->setFont(font3);

        offlineGrid_2->addWidget(offlineBinCombo_2, 0, 1, 1, 1);

        offlineProcessButton_2 = new QPushButton(offlinePointCloudGroup_2);
        offlineProcessButton_2->setObjectName(QString::fromUtf8("offlineProcessButton_2"));
        sizePolicy2.setHeightForWidth(offlineProcessButton_2->sizePolicy().hasHeightForWidth());
        offlineProcessButton_2->setSizePolicy(sizePolicy2);

        offlineGrid_2->addWidget(offlineProcessButton_2, 0, 2, 1, 1);


        configPanelLayout->addWidget(offlinePointCloudGroup_2);

        logGroup = new QGroupBox(configPanel);
        logGroup->setObjectName(QString::fromUtf8("logGroup"));
        logGroup->setFont(font2);
        logGroupLayout = new QVBoxLayout(logGroup);
        logGroupLayout->setObjectName(QString::fromUtf8("logGroupLayout"));
        logTextEdit = new QTextEdit(logGroup);
        logTextEdit->setObjectName(QString::fromUtf8("logTextEdit"));
        logTextEdit->setMinimumSize(QSize(0, 210));
        logTextEdit->setFont(font3);

        logGroupLayout->addWidget(logTextEdit);


        configPanelLayout->addWidget(logGroup);


        mainLayout->addWidget(configPanel);

        groupBox = new QGroupBox(centralwidget);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setFont(font2);
        pointCloudLayout = new QVBoxLayout(groupBox);
        pointCloudLayout->setObjectName(QString::fromUtf8("pointCloudLayout"));
        graphicsView = new QGraphicsView(groupBox);
        graphicsView->setObjectName(QString::fromUtf8("graphicsView"));
        graphicsView->setMinimumSize(QSize(520, 520));
        graphicsView->setFont(font3);

        pointCloudLayout->addWidget(graphicsView);


        mainLayout->addWidget(groupBox);

        taskPanel = new QFrame(centralwidget);
        taskPanel->setObjectName(QString::fromUtf8("taskPanel"));
        taskPanel->setMinimumSize(QSize(300, 0));
        taskPanel->setMaximumSize(QSize(360, 16777215));
        taskPanel->setFrameShape(QFrame::StyledPanel);
        taskPanelLayout = new QVBoxLayout(taskPanel);
        taskPanelLayout->setSpacing(8);
        taskPanelLayout->setObjectName(QString::fromUtf8("taskPanelLayout"));
        targetTrackingGroup = new QGroupBox(taskPanel);
        targetTrackingGroup->setObjectName(QString::fromUtf8("targetTrackingGroup"));
        targetTrackingGroup->setFont(font2);
        targetTrackingLayout = new QVBoxLayout(targetTrackingGroup);
        targetTrackingLayout->setObjectName(QString::fromUtf8("targetTrackingLayout"));
        targetModeLabel = new QLabel(targetTrackingGroup);
        targetModeLabel->setObjectName(QString::fromUtf8("targetModeLabel"));

        targetTrackingLayout->addWidget(targetModeLabel);

        targetModeCombo = new QComboBox(targetTrackingGroup);
        targetModeCombo->addItem(QString());
        targetModeCombo->addItem(QString());
        targetModeCombo->setObjectName(QString::fromUtf8("targetModeCombo"));
        targetModeCombo->setFont(font3);

        targetTrackingLayout->addWidget(targetModeCombo);

        targetRuleLabel = new QLabel(targetTrackingGroup);
        targetRuleLabel->setObjectName(QString::fromUtf8("targetRuleLabel"));

        targetTrackingLayout->addWidget(targetRuleLabel);

        targetRuleCombo = new QComboBox(targetTrackingGroup);
        targetRuleCombo->addItem(QString());
        targetRuleCombo->addItem(QString());
        targetRuleCombo->addItem(QString());
        targetRuleCombo->setObjectName(QString::fromUtf8("targetRuleCombo"));
        targetRuleCombo->setFont(font3);

        targetTrackingLayout->addWidget(targetRuleCombo);

        autoSelectButton = new QPushButton(targetTrackingGroup);
        autoSelectButton->setObjectName(QString::fromUtf8("autoSelectButton"));

        targetTrackingLayout->addWidget(autoSelectButton);

        tableWidget = new QTableWidget(targetTrackingGroup);
        if (tableWidget->columnCount() < 4)
            tableWidget->setColumnCount(4);
        QTableWidgetItem *__qtablewidgetitem = new QTableWidgetItem();
        __qtablewidgetitem->setTextAlignment(Qt::AlignCenter);
        tableWidget->setHorizontalHeaderItem(0, __qtablewidgetitem);
        QTableWidgetItem *__qtablewidgetitem1 = new QTableWidgetItem();
        tableWidget->setHorizontalHeaderItem(1, __qtablewidgetitem1);
        QTableWidgetItem *__qtablewidgetitem2 = new QTableWidgetItem();
        tableWidget->setHorizontalHeaderItem(2, __qtablewidgetitem2);
        QTableWidgetItem *__qtablewidgetitem3 = new QTableWidgetItem();
        tableWidget->setHorizontalHeaderItem(3, __qtablewidgetitem3);
        if (tableWidget->rowCount() < 4)
            tableWidget->setRowCount(4);
        tableWidget->setObjectName(QString::fromUtf8("tableWidget"));
        QSizePolicy sizePolicy3(QSizePolicy::Fixed, QSizePolicy::Expanding);
        sizePolicy3.setHorizontalStretch(0);
        sizePolicy3.setVerticalStretch(0);
        sizePolicy3.setHeightForWidth(tableWidget->sizePolicy().hasHeightForWidth());
        tableWidget->setSizePolicy(sizePolicy3);
        tableWidget->setRowCount(4);
        tableWidget->horizontalHeader()->setCascadingSectionResizes(false);
        tableWidget->horizontalHeader()->setMinimumSectionSize(40);
        tableWidget->horizontalHeader()->setDefaultSectionSize(60);

        targetTrackingLayout->addWidget(tableWidget);

        lockedTargetInfoText = new QTextEdit(targetTrackingGroup);
        lockedTargetInfoText->setObjectName(QString::fromUtf8("lockedTargetInfoText"));
        lockedTargetInfoText->setMinimumSize(QSize(0, 82));
        lockedTargetInfoText->setFont(font3);

        targetTrackingLayout->addWidget(lockedTargetInfoText);


        taskPanelLayout->addWidget(targetTrackingGroup);

        backgroundModelGroup = new QGroupBox(taskPanel);
        backgroundModelGroup->setObjectName(QString::fromUtf8("backgroundModelGroup"));
        backgroundModelGroup->setFont(font2);
        backgroundLayout = new QVBoxLayout(backgroundModelGroup);
        backgroundLayout->setObjectName(QString::fromUtf8("backgroundLayout"));
        backgroundButtonsLayout = new QHBoxLayout();
        backgroundButtonsLayout->setObjectName(QString::fromUtf8("backgroundButtonsLayout"));
        backgroundStartButton = new QPushButton(backgroundModelGroup);
        backgroundStartButton->setObjectName(QString::fromUtf8("backgroundStartButton"));

        backgroundButtonsLayout->addWidget(backgroundStartButton);

        backgroundClearButton = new QPushButton(backgroundModelGroup);
        backgroundClearButton->setObjectName(QString::fromUtf8("backgroundClearButton"));

        backgroundButtonsLayout->addWidget(backgroundClearButton);


        backgroundLayout->addLayout(backgroundButtonsLayout);

        backgroundFilterCheck = new QCheckBox(backgroundModelGroup);
        backgroundFilterCheck->setObjectName(QString::fromUtf8("backgroundFilterCheck"));

        backgroundLayout->addWidget(backgroundFilterCheck);

        globalPointcloudCheck = new QCheckBox(backgroundModelGroup);
        globalPointcloudCheck->setObjectName(QString::fromUtf8("globalPointcloudCheck"));

        backgroundLayout->addWidget(globalPointcloudCheck);


        taskPanelLayout->addWidget(backgroundModelGroup);


        mainLayout->addWidget(taskPanel);

        mainLayout->setStretch(2, 1);

        rootLayout->addLayout(mainLayout);

        RadarGUIRedesignV2->setCentralWidget(centralwidget);
        menubar = new QMenuBar(RadarGUIRedesignV2);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 1583, 23));
        menu = new QMenu(menubar);
        menu->setObjectName(QString::fromUtf8("menu"));
        RadarGUIRedesignV2->setMenuBar(menubar);
        statusbar = new QStatusBar(RadarGUIRedesignV2);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        RadarGUIRedesignV2->setStatusBar(statusbar);

        menubar->addAction(menu->menuAction());
        menu->addAction(action);
        menu->addAction(action_3);
        menu->addAction(action_5);
        menu->addAction(action_4);

        retranslateUi(RadarGUIRedesignV2);

        QMetaObject::connectSlotsByName(RadarGUIRedesignV2);
    } // setupUi

    void retranslateUi(QMainWindow *RadarGUIRedesignV2)
    {
        RadarGUIRedesignV2->setWindowTitle(QCoreApplication::translate("RadarGUIRedesignV2", "\345\215\225\347\233\256\346\240\207\350\257\206\345\210\253\344\270\216\350\207\252\344\270\273\350\267\237\351\232\217\347\263\273\347\273\237", nullptr));
        action->setText(QCoreApplication::translate("RadarGUIRedesignV2", "\345\256\236\346\227\266\347\202\271\344\272\221", nullptr));
        action_3->setText(QCoreApplication::translate("RadarGUIRedesignV2", "\347\246\273\347\272\277\350\201\232\347\261\273", nullptr));
        action_5->setText(QCoreApplication::translate("RadarGUIRedesignV2", "\347\246\273\347\272\277\347\202\271\344\272\221", nullptr));
        action_2->setText(QCoreApplication::translate("RadarGUIRedesignV2", "\347\246\273\347\272\277", nullptr));
        action_4->setText(QCoreApplication::translate("RadarGUIRedesignV2", "\347\246\273\347\272\277\350\277\275\350\270\252", nullptr));
        logoLabel->setText(QString());
        connectButton->setText(QCoreApplication::translate("RadarGUIRedesignV2", "\350\277\236\346\216\245\351\233\267\350\276\276", nullptr));
        configureButton->setText(QCoreApplication::translate("RadarGUIRedesignV2", "\351\205\215\347\275\256\351\233\267\350\276\276", nullptr));
        pushButton_2->setText(QCoreApplication::translate("RadarGUIRedesignV2", "\345\274\200\345\247\213\351\207\207\351\233\206", nullptr));
        pushButton_6->setText(QCoreApplication::translate("RadarGUIRedesignV2", "\345\274\200\345\247\213\350\277\275\350\270\252", nullptr));
        pushButton_3->setText(QCoreApplication::translate("RadarGUIRedesignV2", "\345\201\234\346\255\242\351\207\207\351\233\206", nullptr));
        pushButton->setText(QCoreApplication::translate("RadarGUIRedesignV2", "\351\200\200\345\207\272", nullptr));
        serialGroup->setTitle(QCoreApplication::translate("RadarGUIRedesignV2", "\344\270\262\345\217\243\351\205\215\347\275\256", nullptr));
        label->setText(QCoreApplication::translate("RadarGUIRedesignV2", "\344\270\262\345\217\243:", nullptr));
        refreshPortsButton->setText(QCoreApplication::translate("RadarGUIRedesignV2", "\345\210\267\346\226\260", nullptr));
        configGroup->setTitle(QCoreApplication::translate("RadarGUIRedesignV2", "\351\233\267\350\276\276\351\205\215\347\275\256", nullptr));
        label_2->setText(QCoreApplication::translate("RadarGUIRedesignV2", "\351\205\215\347\275\256\346\226\207\344\273\266:", nullptr));
        cfarGroup->setTitle(QCoreApplication::translate("RadarGUIRedesignV2", "CFAR\345\217\202\346\225\260", nullptr));
        labelNex->setText(QCoreApplication::translate("RadarGUIRedesignV2", "N_ex:", nullptr));
        labelAlpha->setText(QCoreApplication::translate("RadarGUIRedesignV2", "alpha:", nullptr));
        labelPr->setText(QCoreApplication::translate("RadarGUIRedesignV2", "N_protect_r:", nullptr));
        labelK->setText(QCoreApplication::translate("RadarGUIRedesignV2", "K:", nullptr));
        labelPd->setText(QCoreApplication::translate("RadarGUIRedesignV2", "N_protect_d:", nullptr));
        cfarSaveButton->setText(QCoreApplication::translate("RadarGUIRedesignV2", "\344\277\235\345\255\230", nullptr));
        captureGroup->setTitle(QCoreApplication::translate("RadarGUIRedesignV2", "\345\275\223\345\211\215\347\212\266\346\200\201", nullptr));
        pointCloudGroup->setTitle(QCoreApplication::translate("RadarGUIRedesignV2", "\347\246\273\347\272\277\350\201\232\347\261\273", nullptr));
        pointCloudLabel->setText(QCoreApplication::translate("RadarGUIRedesignV2", "\347\202\271\344\272\221\346\226\207\344\273\266:", nullptr));
        pushButton_4->setText(QCoreApplication::translate("RadarGUIRedesignV2", "\345\274\200\345\247\213", nullptr));
        offlinePointCloudGroup->setTitle(QCoreApplication::translate("RadarGUIRedesignV2", "\347\246\273\347\272\277\347\202\271\344\272\221", nullptr));
        offlineBinLabel->setText(QCoreApplication::translate("RadarGUIRedesignV2", "bin\346\226\207\344\273\266:", nullptr));
        offlineProcessButton->setText(QCoreApplication::translate("RadarGUIRedesignV2", "\345\244\204\347\220\206", nullptr));
        offlinePointCloudGroup_2->setTitle(QCoreApplication::translate("RadarGUIRedesignV2", "\347\246\273\347\272\277\350\277\275\350\270\252", nullptr));
        offlineBinLabel_2->setText(QCoreApplication::translate("RadarGUIRedesignV2", "bin\346\226\207\344\273\266:", nullptr));
        offlineProcessButton_2->setText(QCoreApplication::translate("RadarGUIRedesignV2", "\350\277\275\350\270\252", nullptr));
        logGroup->setTitle(QCoreApplication::translate("RadarGUIRedesignV2", "\345\244\204\347\220\206\350\277\207\347\250\213", nullptr));
        groupBox->setTitle(QCoreApplication::translate("RadarGUIRedesignV2", "\346\257\253\347\261\263\346\263\242\347\202\271\344\272\221\350\247\206\345\233\276", nullptr));
        targetTrackingGroup->setTitle(QCoreApplication::translate("RadarGUIRedesignV2", "\347\233\256\346\240\207\350\257\206\345\210\253\344\270\216\350\267\237\350\270\252", nullptr));
        targetModeLabel->setText(QCoreApplication::translate("RadarGUIRedesignV2", "\351\200\211\346\213\251\346\226\271\345\274\217", nullptr));
        targetModeCombo->setItemText(0, QCoreApplication::translate("RadarGUIRedesignV2", "\350\207\252\345\212\250\351\200\211\346\213\251", nullptr));
        targetModeCombo->setItemText(1, QCoreApplication::translate("RadarGUIRedesignV2", "\344\272\272\345\267\245\347\202\271\345\207\273\345\200\231\351\200\211\347\233\256\346\240\207", nullptr));

        targetRuleLabel->setText(QCoreApplication::translate("RadarGUIRedesignV2", "\351\200\211\346\213\251\347\255\226\347\225\245", nullptr));
        targetRuleCombo->setItemText(0, QCoreApplication::translate("RadarGUIRedesignV2", "\350\267\235\347\246\273\346\234\200\350\277\221", nullptr));
        targetRuleCombo->setItemText(1, QCoreApplication::translate("RadarGUIRedesignV2", "\351\200\237\345\272\246\346\234\200\345\277\253", nullptr));
        targetRuleCombo->setItemText(2, QCoreApplication::translate("RadarGUIRedesignV2", "\350\277\220\345\212\250\347\202\271\344\274\230\345\205\210", nullptr));

        autoSelectButton->setText(QCoreApplication::translate("RadarGUIRedesignV2", "\346\211\247\350\241\214", nullptr));
        QTableWidgetItem *___qtablewidgetitem = tableWidget->horizontalHeaderItem(0);
        ___qtablewidgetitem->setText(QCoreApplication::translate("RadarGUIRedesignV2", "\351\200\211\346\213\251", nullptr));
        QTableWidgetItem *___qtablewidgetitem1 = tableWidget->horizontalHeaderItem(1);
        ___qtablewidgetitem1->setText(QCoreApplication::translate("RadarGUIRedesignV2", "\347\233\256\346\240\207ID", nullptr));
        QTableWidgetItem *___qtablewidgetitem2 = tableWidget->horizontalHeaderItem(2);
        ___qtablewidgetitem2->setText(QCoreApplication::translate("RadarGUIRedesignV2", "\350\267\235\347\246\273", nullptr));
        QTableWidgetItem *___qtablewidgetitem3 = tableWidget->horizontalHeaderItem(3);
        ___qtablewidgetitem3->setText(QCoreApplication::translate("RadarGUIRedesignV2", "\351\200\237\345\272\246", nullptr));
        lockedTargetInfoText->setPlaceholderText(QCoreApplication::translate("RadarGUIRedesignV2", "\345\275\223\345\211\215\346\234\252\351\224\201\345\256\232\347\233\256\346\240\207", nullptr));
        backgroundModelGroup->setTitle(QCoreApplication::translate("RadarGUIRedesignV2", "\350\203\214\346\231\257\345\273\272\346\250\241", nullptr));
        backgroundStartButton->setText(QCoreApplication::translate("RadarGUIRedesignV2", "\345\274\200\345\247\213\345\273\272\346\250\241", nullptr));
        backgroundClearButton->setText(QCoreApplication::translate("RadarGUIRedesignV2", "\346\270\205\351\231\244\350\203\214\346\231\257", nullptr));
        backgroundFilterCheck->setText(QCoreApplication::translate("RadarGUIRedesignV2", "\345\220\257\347\224\250\350\203\214\346\231\257\350\277\207\346\273\244", nullptr));
        globalPointcloudCheck->setText(QCoreApplication::translate("RadarGUIRedesignV2", "\345\205\250\345\261\200\347\202\271\344\272\221", nullptr));
        menu->setTitle(QCoreApplication::translate("RadarGUIRedesignV2", "\350\217\234\345\215\225\346\240\217", nullptr));
    } // retranslateUi

};

namespace Ui {
    class RadarGUIRedesignV2: public Ui_RadarGUIRedesignV2 {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_RADAR_GUI_V2_H
