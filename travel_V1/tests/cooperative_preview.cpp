// Offline layout-only preview. Synthetic data; never opens sensors or ROS.
// Build on a Qt5 development host:
// g++ -std=c++17 -fPIC cooperative_view.cpp tests/cooperative_preview.cpp \
//     $(pkg-config --cflags --libs Qt5Widgets) -o cooperative_preview
#include "../cooperative_view.hpp"
#include <QApplication>
#include <QDir>
#include <QFont>
#include <QTextStream>

int main(int argc, char** argv) {
    QApplication app(argc, argv);
    if (app.arguments().size() != 2) return 2;
    const QString output = app.arguments().at(1);
    if (!QDir().mkpath(output)) return 3;
    QFont font(QStringLiteral("Noto Sans CJK SC"));
    app.setFont(font);
    RadarCooperative::CooperativeView view;
    view.resize(350, 420);
    view.show();
    using namespace RadarCooperative;
    Snapshot base;
    base.acquisitionActive = true;
    base.trackingEnabled = true;
    base.radar = {true, 0.01, 0.5};
    base.camera = {true, 0.01, 0.5};
    base.targetSelected = true;
    base.targetPositionValid = true;
    base.observation = ObservationState::Measured;
    base.targetLabel = QStringLiteral("P1");
    base.targetPosition = QPointF(-0.35, 3.2);
    base.otherPeople = {{QStringLiteral("P2"), QPointF(1.7, 5.1)},
                        {QStringLiteral("P3"), QPointF(-2.3, 6.5)}};
    base.visualEvidence = VisualEvidence::Bound;
    base.followState = FollowState::NotStarted;
    base.detail = QStringLiteral("布局预览 · 人工示例数据");
    auto save = [&](const QString& name, const Snapshot& state) {
        view.setSnapshot(state);
        app.processEvents();
        return view.grab().save(QDir(output).filePath(name + QStringLiteral(".png")));
    };
    if (!save(QStringLiteral("observed"), base)) return 4;
    Snapshot predicted = base;
    predicted.observation = ObservationState::Predicted;
    predicted.followState = FollowState::Stopped;
    if (!save(QStringLiteral("predicted"), predicted)) return 4;
    Snapshot stale = base;
    stale.radar.ageSeconds = 2;
    stale.camera.ageSeconds = 2;
    if (!save(QStringLiteral("stale"), stale)) return 4;
    Snapshot lost = base;
    lost.observation = ObservationState::Lost;
    if (!save(QStringLiteral("lost"), lost)) return 4;
    Snapshot stopped = base;
    stopped.acquisitionActive = false; // Residual lock must not remain visible.
    if (!save(QStringLiteral("stopped"), stopped)) return 4;
    Snapshot empty;
    empty.detail = QStringLiteral("布局预览 · 无输入数据");
    if (!save(QStringLiteral("empty"), empty)) return 4;
    view.resize(260, 340);
    if (!save(QStringLiteral("compact"), base)) return 4;
    view.resize(350, 420);
    Snapshot outside = base;
    outside.targetPosition = QPointF(-9, 4);
    if (!save(QStringLiteral("outside"), outside)) return 4;
    QTextStream(stdout) << "Saved 8 synthetic layout previews in " << output << '\n';
    return 0;
}
