#ifndef ROS_RUNTIME_PANEL_HPP
#define ROS_RUNTIME_PANEL_HPP
#include <QWidget>
#include <QProcess>
#include <QTimer>
#include <QElapsedTimer>
#include <QJsonObject>
#include <QLineEdit>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>

class RosRuntimePanel : public QWidget {
    Q_OBJECT
public:
    explicit RosRuntimePanel(const QString& projectRoot,QWidget* parent=nullptr);
    ~RosRuntimePanel();
    void setCaptureSession(quint64 session);
    void startFollow();
    void stopFollow();
    void shutdownRos();
    bool shutdownBlocking(int timeoutMs=14000);
    bool active() const;
    // Last helper state plus local lifecycle gates. This is not measured motion.
    QJsonObject displayState() const;
signals:
    void diagnostic(const QString&);
    void controlsChanged(bool canStart,bool canStop);
    void statusChanged(const QString&);
private:
#ifdef RADAR_GUI_ENABLE_TEST_ACCESS
    friend struct RosRuntimePanelTestAccess;
#endif
    QString root_,folder_,configPath_;
    QProcess process_,cleanup_;
    QTimer heartbeat_,startupTimeout_;
    QByteArray pending_;
    QJsonObject displayEvent_;
    QElapsedTimer displayEventAge_;
    QString session_="0";
    bool ready_=false,followRequested_=false,closing_=false,expectedExit_=false;
    bool stopping_=false,cleanupFailed_=false;
    bool exitFailed_=false;
    QLineEdit *setup_,*package_,*launch_,*node_,*odom_,*cmd_;
    QDoubleSpinBox *linear_,*angular_;
    QLabel *status_,*components_,*odomStatus_;
    QPushButton *shutdown_;
    QList<QWidget*> settings_;
    QJsonObject configuration() const;
    void saveConfiguration();
    void lockSettings(bool);
    void send(QJsonObject);
    void readEvents();
    void handleEvent(const QJsonObject&);
    void finished(int,QProcess::ExitStatus);
    void emergencyCleanup();
    void updateControls();
    void setStatus(const QString&);
    void clearDisplayState();
};
#endif
