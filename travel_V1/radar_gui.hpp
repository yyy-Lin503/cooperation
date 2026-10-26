#ifndef RADAR_GUI_HPP
#define RADAR_GUI_HPP

#include <QMainWindow>
#include <QTimer>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QTextEdit>
#include <QPushButton>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QLabel>
#include <QComboBox>
#include <QSpinBox>
#include <QLineEdit>
#include <QTextEdit>
#include <QGraphicsView>
#include <QGraphicsEllipseItem>
#include <QGraphicsTextItem>
#include <QDateTime>
#include <QUdpSocket>
#include <QTableWidget>
#include <vector>
#include <cstdint>
#include <map>
#include <string>
#include <thread>
#include <chrono>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <fstream>
#include <experimental/filesystem>
#include <deque>
#include <cuda_runtime.h>
#include <cstdlib>  
#include <QThread>  
#include "radar_config.hpp"
#include "realtime_capture.hpp"
#include "iwr6843param.hpp"
#include "signal_processing.hpp"
#include "cluster.hpp"
#include "target_tracking.hpp"
#include "chassis_velocity.hpp"
#include "radar_runtime_config.hpp"
#include "offline_odom_replay.hpp"
#include "radar_connection_config.hpp"
#include <QCloseEvent>
#include "recording_frame_index.hpp"
#include "cooperative_view.hpp"
#include "ros_runtime_panel.hpp"
#if RADAR_ENABLE_CAMERA
#include "camera_assist_panel.hpp"
#endif

namespace fs = std::experimental::filesystem;

QT_BEGIN_NAMESPACE
namespace Ui { class RadarGUIRedesignV2; }
QT_END_NAMESPACE

class RadarGUI : public QMainWindow
{
    Q_OBJECT

public:
    explicit RadarGUI(QWidget *parent = nullptr);
    ~RadarGUI();
    // 对外公开的日志追加接口，供辅助类实时写入日志
    void appendLog(const QString& message);

private slots:
    void on_connectButton_clicked();
    void on_configureButton_clicked();
    void handleStartCaptureClicked();
    void handleStopCaptureClicked();
    void handleExitClicked();
    void on_refreshPortsButton_clicked();
    void on_pushButton_4_clicked(); // 离线聚类开始/停止
    void on_pushButton_6_clicked(); // 🆕 注入：专门处理“开始追踪”按钮的点击
    void updatePointCloudDisplay();
    void processUdpData();
    void on_action_triggered();      // 原始点云 菜单项
    void on_action_3_triggered();    // 目标聚类 菜单项
    void on_action_5_triggered();    // 离线处理 菜单项
    void on_action_4_triggered();    // 离线追踪 菜单项
    // 动态创建的缩放按钮使用显式连接，避免触发 Qt 的自动连接查找。
    void handleZoomInClicked();
    void handleZoomOutClicked();

    // 目标聚类播放结束回调
    void onClusterPlaybackFinished();

    // 离线处理相关槽函数
    void on_offlineProcessButton_clicked();  // 处理/停止按钮
    void processOfflineFrame();              // 逐帧处理bin文件

private:
#ifdef RADAR_GUI_ENABLE_TEST_ACCESS
    friend struct RadarGUIIntegrationAccess;
#endif
    Ui::RadarGUIRedesignV2 *ui;

    void shutdownRuntime();
    bool writeTrackingMetadata(int frame, const QString& stopReason = QString());
    bool visualMotionAllowed() const;
    RosRuntimePanel* rosRuntime_ = nullptr;
    QPushButton* stopFollowButton_ = nullptr;
#if RADAR_ENABLE_CAMERA
    CameraAssistPanel* cameraAssist_ = nullptr;
    void syncCameraRadarSelection();
#endif
    bool shutdown_ = false;
    RadarConnectionConfig connection_;
    RadarFrameContext currentFrameContext;
    RecordingFrameIndex offlineFrameIndex;
    int lastRealtimeFrame_ = -1;
    CaptureStatus::State lastCaptureState_ = CaptureStatus::State::Stopped;
    double nextCaptureDiagnosticSec_ = 0;
    AWR2944SerialConfig* radar = nullptr;
    DCA1000Config* dca1000_cfg = nullptr;
    std::string config_file;
    std::map<std::string, double> radar_config;
    UdpListener* collector = nullptr;
    bool is_capturing = false;
    bool is_configuring = false; // 防止配置流程重入/重复触发
    std::queue<std::vector<char>> bin_data;

    QGraphicsScene* scene = nullptr;
    QTimer* updateTimer = nullptr;
    QUdpSocket* udpSocket = nullptr;

    // 点云显示相关
    std::vector<QGraphicsItem*> pointItems;
    QGraphicsTextItem* infoText;
    QDateTime lastModifiedTime;
    // 坐标轴元素（线、刻度、文字），单独管理，避免与点云混淆
    std::vector<QGraphicsItem*> axisItems;

    // 缩放控制
    double zoomFactor = 1.0;      // 当前缩放倍数
    const double zoomStep = 1.2;  // 每次缩放步长
    const double minZoom = 0.5;
    const double maxZoom = 3.0;
    double sceneYMaxMeters = 40.0; // 当前画布前向距离上限，默认 40m，配置后随雷达参数更新
    double sceneXMaxMeters = 12.0; // 当前画布横向距离上限，左右对称显示
    
    // 模式切换
    bool isTestMode = false;    // false: 原始点云(实时)；true: 目标聚类(读取文件)
    bool isOfflineMode = false; // true: 离线处理(读取bin文件)
    bool isOfflineTrackingMode = false; // true: 离线bin跟踪调试

    // 目标聚类模块
    Cluster* cluster = nullptr;
    TargetTracking::TargetTracker targetTracker;
    std::vector<PointCloud> latestTrackingPoints;

    // 实时点云聚类缓存（最近5帧）
    struct RealtimeFrameData {
        std::vector<TestPoint> points;
        ChassisVelocitySample chassis;
        ProcessingDiagnostics processing;
        std::string radar_timestamp;
        RadarFrameContext frame;
    };
    std::deque<int> realtimeFrameOrder;
    std::map<int, RealtimeFrameData> realtimeFrameBuffer;
    int trackingWindowFrames = RadarRuntimeConfig::kPointWindowFrames;
    bool verboseTrackingLogs = RadarRuntimeConfig::kDebugLogging;
    std::deque<std::pair<int, std::vector<PointCloud>>> offlineTrackingWindow;

    struct TrackingRuntimeStats {
        std::int64_t session_id = 0;
        std::string source;
        std::string input_path;
        std::uint64_t updates = 0;
        std::uint64_t locked_updates = 0;
        std::uint64_t measurement_hits = 0;
        std::uint64_t predicted_updates = 0;
        std::uint64_t stopped_updates = 0;
        std::uint64_t reacquire_updates = 0;
        std::uint64_t deadline_misses = 0;
        double points_sum = 0.0;
        double candidates_sum = 0.0;
        double clusters_sum = 0.0;
        double active_tracks_sum = 0.0;
        double measured_step_sq_sum = 0.0;
        std::uint64_t measured_step_count = 0;
        bool have_last_measurement = false;
        float last_measured_x = 0.0f;
        float last_measured_y = 0.0f;
        std::uint64_t last_written_updates = 0;
        std::uint64_t raw_track_births = 0;
        std::uint64_t exposed_id_births = 0;
        int max_display_id = 0;
        bool finalized = false;
        std::vector<double> tracker_times_ms;
    } trackingStats;

    struct TargetTableCacheEntry {
        TargetTracking::CandidateTarget candidate;
        int missed_updates = 0;
        bool current = false;
    };
    std::map<int, TargetTableCacheEntry> targetTableCache;
    // Keep a missing row in its numeric position for three seconds at 10 Hz.
    // The row is grey and disabled, preventing lower IDs from jumping upward
    // while the user is trying to select a person.
    static constexpr int kTargetTableHoldUpdates = 30;
    std::uint64_t previewTargetUid = 0;
    int previewTargetDisplayId = -1;
    // The checked person stays in this row. The main table keeps scrolling.
    QWidget* pinnedTargetRowHost = nullptr;
    QTableWidget* pinnedTargetTable = nullptr;
    QWidget* pinnedScrollSpacer = nullptr;

    // Short diagnostic trail for the currently locked target. Samples are
    // maintained in the current radar frame so a moving cart does not make an
    // old relative-coordinate trail appear to run in the wrong direction.
    struct LockedTrajectorySample {
        int frame = -1;
        int display_id = -1;
        std::uint64_t internal_uid = 0;
        std::uint64_t previous_internal_uid = 0;
        float filtered_x = 0.0f;
        float filtered_y = 0.0f;
        float measured_x = 0.0f;
        float measured_y = 0.0f;
        bool has_measurement = false;
        bool prediction_only = false;
        bool uid_changed = false;
    };
    std::deque<LockedTrajectorySample> lockedTrajectoryHistory;
    int lockedTrajectoryDisplayId = -1;
    std::uint64_t lockedTrajectoryLastUid = 0;
    int lockedTrajectoryLastFrame = -1;
    static constexpr std::size_t kLockedTrajectoryHistoryFrames = 50;

    ProcessingDiagnostics currentProcessingDiagnostics;
    ChassisVelocitySample currentTrackingChassis;
    bool currentTrackingOdomAgeFromReplay = false;
    double currentTrackingOdomAgeSec = 0.0;
    std::string currentRadarTimestamp;
    QString currentDiagnosticCsvPath;
    QString currentEventLogPath;
    std::ofstream trackingDiagnosticStream;
    std::ofstream trackingEventStream;

    // ===== 离线处理相关成员 =====
    QGroupBox* offlineGroup = nullptr;        // "离线处理"选项框
    QComboBox* offlineBinCombo = nullptr;     // bin文件下拉选择框
    QPushButton* offlineProcessButton = nullptr; // "处理"按钮
    QGroupBox* offlineTrackGroup = nullptr;   // "离线追踪"选项框
    QComboBox* offlineTrackCombo = nullptr;   // 追踪bin下拉选择框
    QPushButton* offlineTrackButton = nullptr; // "追踪"按钮
    QTableWidget* targetTable = nullptr;      // 目标信息表格
    QTimer* offlineTimer = nullptr;           // 逐帧处理定时器
    std::ifstream offlineBinFile;             // 当前打开的bin文件流
    int offlineFrameLength = 0;               // 每帧字节数
    int offlineFrameCount = 0;                // 已处理帧计数
    OfflineOdomReplay offlineOdomReplay;
    QString currentTrackTxtPath;              // 当前追踪坐标输出文件
    QString currentTrackCsvPath;              // 当前追踪坐标CSV输出文件
    QString currentControlRecordLine_;
    QString selectionHintText_;
    QLabel* selectionHintLabel_ = nullptr;
    QString trackingMetadataStopReason;       // 异步视觉刷新不能清除显式停车

    // 根据视图尺寸把“+/-”按钮放到右下角
    void positionZoomButtons();
    void applyZoom();
    // 绘制/刷新坐标轴（基于当前 sceneRect），包含主轴线、1m 主刻度与数字标签
    void drawAxes();
    void updateSceneRectFromRadarConfig(bool fitView = false);
    qreal sceneXFromMeters(float x) const;
    qreal sceneYFromMeters(float y) const;

    // 离线处理辅助方法
    void updateBinList();                                  // 刷新./bin目录下的bin文件列表
    void updateTrackBinList();                             // 刷新./bin目录下的追踪bin文件列表
    void initTargetTable();                                // 初始化目标表格(勾选列)
    void onTargetTableCellClicked(int row, int column);
    void updatePinnedTargetRowChrome();
    void ensureTargetTableRows(int requiredRows);          // 确保目标表格行数
    void displayProcessingResult(const ProcessingResult& result); // 在点云图上显示处理结果
    bool load_config_from_file(const std::string& path);   // 仅从配置文件解析参数，无需雷达硬件
    void updateTargetTrackingCandidates(const std::vector<PointCloud>& points, int frameNumber);
    void printOfflineTrackingDebug(const ProcessingResult& result);
    void drawTrackingOverlay();
    void updateLockedTrajectoryHistory(int frameNumber);
    void resetLockedTrajectoryHistory();
    void refreshTargetTable();
    void refreshLockedTargetInfo();
    void clearTargetPreview();
    void updateTargetSelectionControls();
    bool selectFollowTargetById(int candidateId);
    bool selectFollowTargetByUid(std::uint64_t internalUid);
    bool autoSelectFollowTarget(TargetTracking::AutoSelectRule rule);
    TargetTracking::AutoSelectRule currentAutoSelectRule() const;
    void resetRealtimeClusterBuffer();
    void syncBackgroundUiAfterTrackerReset();
    void resetTrackTxtOutput();
    void appendLockedTargetToTxt(int frameNumber);
    void setupWorkspaceLayout();
    void refreshCooperativeView();
    RadarCooperative::CooperativeView* cooperativeView_ = nullptr;
    QTimer* cooperativeTimer_ = nullptr;
    double realtimeFramePeriodSeconds() const;
    std::vector<TestPoint> compensatedRealtimeWindowPoints() const;
    void resetTrackingRuntimeStats();
    void recordTrackingRuntimeStats(std::size_t windowPointCount, double trackerMs,
                                    int frameNumber);
    void writeTrackingStatsSnapshot(bool finalSnapshot,
                                    const std::string& endReason = std::string());
    void finalizeTrackingSession(const std::string& endReason);
    bool ensureTrackingDiagnosticLogOpen();
    void appendTrackingDiagnosticRow(int frameNumber, std::size_t windowPointCount,
                                     double trackerMs);
    void closeTrackingDiagnosticLog();
    static std::string csvEscape(const std::string& value);
    std::string candidateSnapshotForLog() const;
    std::string clusterSnapshotForLog() const;
    std::vector<PointCloud> aggregateOfflineTrackingWindow() const;

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;
    void closeEvent(QCloseEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;
    void showEvent(QShowEvent* event) override;

    void logMessage(const QString& message);
    void updatePortList();
    void updateConfigList();
    std::vector<std::string> listSerialPorts();
    std::string selectSerialPort();
    std::string detectRadarPort();   
    void connect_radar();
    std::vector<std::string> list_config_files();
    void configure_radar();
    void start_capture();
    void select_config_file();
    void stop_capture();
    void exit_program();
};

#endif // RADAR_GUI_HPP
