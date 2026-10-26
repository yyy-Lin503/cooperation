#ifndef V3_CAMERA_ASSIST_PANEL_HPP
#define V3_CAMERA_ASSIST_PANEL_HPP
#include "camera_worker.hpp"
#include <QWidget>
#include <QComboBox>
#include <QLineEdit>
#include <QCheckBox>
#include <QLabel>
#include <QTimer>
#include <QJsonObject>

class CameraCanvas : public QWidget {
    Q_OBJECT
public:
    explicit CameraCanvas(QWidget* parent=nullptr);
    void setReplayMode(bool replay){replay_=replay;}
    void display(std::shared_ptr<const CameraAssist::CameraPacket>,std::uint64_t selected,
                 int radar_display_id=-1,std::uint64_t task_epoch=0);
signals:
    void personClicked(qulonglong id);
protected:
    void paintEvent(QPaintEvent*) override;
    void mousePressEvent(QMouseEvent*) override;
private:
    bool replay_=false;
    QRectF imageRect() const;
    std::shared_ptr<const CameraAssist::CameraPacket> packet_;
    std::uint64_t selected_=0;
    int radar_display_id_=-1;
    std::uint64_t task_epoch_=0;
};

class CameraAssistPanel : public QWidget {
    Q_OBJECT
public:
    explicit CameraAssistPanel(QWidget* parent=nullptr);
    ~CameraAssistPanel();
    void setRadarFrame(const CameraAssist::RadarFrame&);
    // Updates the existing radar lock; only a subsequent camera click binds it.
    void setLockedTarget(std::uint64_t uid,std::uint64_t session,std::uint64_t epoch,
                         int display_id);
    void resetRadar();
    void stopCamera();
    QWidget* takeControlsWidget(); // Keep the image large while settings live in the main sidebar.
    void setCompactView(bool compact);
    // Read-only display evidence; independent of the motion-protection checkbox.
    QJsonObject displayEvidence() const;
    void recordRadarSnapshot(const QJsonObject& event){worker_.recordEvidence(event);}
    CameraAssist::MotionDecision decision(std::uint64_t locked_uid,std::uint64_t epoch) const;
    QJsonObject metadata(std::uint64_t locked_uid,std::uint64_t epoch) const;
signals:
    void replayRequested();
    void evidenceChanged();
    void diagnostic(const QString& message);
private:
#ifdef RADAR_GUI_ENABLE_TEST_ACCESS
    friend struct CameraPanelTestAccess;
#endif
    CameraAssist::CameraWorker worker_;
    CameraAssist::RadarFrame radar_;
    CameraAssist::Binding binding_;
    std::shared_ptr<const CameraAssist::CameraPacket> shown_;
    std::uint64_t locked_uid_=0,locked_session_=0,locked_epoch_=0;
    std::uint64_t pending_camera_track_=0,pending_camera_session_=0;
    int locked_display_id_=-1;
    bool refreshing_=false;
    QString last_error_;
    CameraCanvas* canvas_;
    QWidget* controls_widget_;
    QComboBox* device_;
    QLineEdit* model_;
    QCheckBox *guard_,*cuda_,*record_;
    QLabel *status_,*binding_status_;
    QTimer timer_;
    void refreshDevices();
    void startCamera();
    void tick();
    void refresh();
    void acceptPacket(std::shared_ptr<const CameraAssist::CameraPacket>);
    void bindClickedPerson(std::uint64_t id);
    void clearBinding();
    CameraAssist::VisionFrame vision() const;
};
#endif
