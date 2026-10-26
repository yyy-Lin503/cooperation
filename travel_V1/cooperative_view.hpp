#ifndef RADAR_COOPERATIVE_VIEW_HPP
#define RADAR_COOPERATIVE_VIEW_HPP

#include <QElapsedTimer>
#include <QPointF>
#include <QString>
#include <QTimer>
#include <QVector>
#include <QWidget>

namespace RadarCooperative {

// All positions are radar-relative metres: +x right, +y forward.
// Camera evidence identifies the selected person; it does not supply depth.
struct Person {
    QString label;
    QPointF position;
};

struct SensorState {
    bool available = false;
    double ageSeconds = -1.0; // Age of the source frame when setSnapshot is called.
    double freshForSeconds = 0.5;
};

// Verified means that the manually bound visual track is still observed;
// it does not assert geometric radar/camera fusion or automatic identity matching.
enum class VisualEvidence { None, Bound, Verified, Mismatch };
enum class ObservationState { Unknown, Measured, Predicted, Lost };
enum class FollowState { Unknown, NotStarted, Requested, Running, Limited, Turning, Stopped, Reacquiring };

struct Snapshot {
    SensorState radar;
    SensorState camera;
    bool acquisitionActive = false;
    bool trackingEnabled = false;
    bool targetSelected = false;
    bool targetPositionValid = false;
    bool targetPersonConfirmed = false;
    ObservationState observation = ObservationState::Unknown;
    QString targetLabel;
    QPointF targetPosition;
    QVector<Person> otherPeople;
    VisualEvidence visualEvidence = VisualEvidence::None;
    // This describes the actual follow-session state supplied by the caller,
    // not just the tracking algorithm's recommendation to allow movement.
    FollowState followState = FollowState::Unknown;
    QString detail;
};

class CooperativeView final : public QWidget {
public:
    explicit CooperativeView(QWidget* parent = nullptr);
    void setSnapshot(const Snapshot& snapshot);
    const Snapshot& snapshot() const { return snapshot_; }
    QSize sizeHint() const override;
    QSize minimumSizeHint() const override;

protected:
    void paintEvent(QPaintEvent*) override;

private:
    bool isFresh(const SensorState& sensor) const;
    Snapshot snapshot_;
    QElapsedTimer received_;
    QTimer repaintTimer_;
};

} // namespace RadarCooperative
#endif
