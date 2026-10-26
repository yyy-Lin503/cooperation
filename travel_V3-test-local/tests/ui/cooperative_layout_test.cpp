// Exercises the real GUI layout without starting capture, cameras, or ROS.
// Optional: QT_QPA_PLATFORM=offscreen ./cooperative_layout_test --screenshot /tmp/ui.png
#include "radar_gui.hpp"
#include "ui_radar_gui_v2.h"
#include <QAction>
#include <QApplication>
#include <QDir>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QFileInfo>
#include <QGraphicsEllipseItem>
#include <QGraphicsTextItem>
#include <QHostAddress>
#include <QMenuBar>
#include <QLinearGradient>
#include <QMouseEvent>
#include <QPainter>
#include <QPointer>
#include <QSignalBlocker>
#include <QSplitter>
#include <QSplitterHandle>
#include <QStyleFactory>
#include <QStringList>
#include <QTabWidget>
#include <QTemporaryDir>
#include <QThread>
#include <QUdpSocket>
#include <cmath>
#include <iostream>
#include <stdexcept>

static void check(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
}

static void settle(int milliseconds = 25) {
    QElapsedTimer elapsed;
    elapsed.start();
    do {
        QApplication::processEvents(QEventLoop::AllEvents, 5);
        QThread::msleep(1);
    } while (elapsed.elapsed() < milliseconds);
}

#if RADAR_ENABLE_CAMERA
struct CameraPanelTestAccess {
    static CameraCanvas* canvas(CameraAssistPanel& panel) { return panel.canvas_; }
    static bool idle(const CameraAssistPanel& panel) { return !panel.worker_.running(); }
    static void injectImage(CameraAssistPanel& panel) {
        check(!panel.worker_.running(), "Camera worker unexpectedly running");
        panel.timer_.stop();
        auto packet = std::make_shared<CameraAssist::CameraPacket>();
        packet->image = QImage(640, 360, QImage::Format_RGB888);
        QPainter painter(&packet->image);
        QLinearGradient sky(0, 0, 0, 360);
        sky.setColorAt(0, QColor(175, 207, 232));
        sky.setColorAt(1, QColor(238, 244, 248));
        painter.fillRect(packet->image.rect(), sky);
        painter.fillRect(QRect(0, 204, 640, 156), QColor(174, 187, 190));
        painter.setPen(QPen(QColor(207, 217, 219), 2));
        for (int y : {235, 278, 334}) painter.drawLine(0, y, 640, y);
        for (int x : {0, 160, 320, 480, 640}) painter.drawLine(320, 204, x, 360);
        auto person = [&](QPointF feet, qreal scale, QColor color) {
            painter.save();
            painter.translate(feet);
            painter.scale(scale, scale);
            painter.setPen(Qt::NoPen);
            painter.setBrush(color);
            painter.drawEllipse(QRectF(-18, -180, 36, 36));
            painter.drawRoundedRect(QRectF(-27, -139, 54, 85), 17, 17);
            painter.setPen(QPen(color, 16, Qt::SolidLine, Qt::RoundCap));
            painter.drawLine(QPointF(-14, -61), QPointF(-19, -5));
            painter.drawLine(QPointF(14, -61), QPointF(19, -5));
            painter.drawLine(QPointF(-24, -124), QPointF(-43, -77));
            painter.drawLine(QPointF(24, -124), QPointF(43, -77));
            painter.restore();
        };
        person(QPointF(302, 323), 1.0, QColor(48, 87, 136));
        person(QPointF(474, 257), 0.62, QColor(119, 137, 155));
        painter.fillRect(QRect(0, 0, 640, 44), QColor(24, 45, 66, 190));
        painter.setPen(Qt::white);
        QFont font = painter.font();
        font.setPixelSize(24);
        font.setBold(true);
        painter.setFont(font);
        painter.drawText(QRect(15, 0, 610, 44), Qt::AlignVCenter,
                         QStringLiteral("UI TEST / 模拟画面 · 非摄像头采集"));
        painter.end();
        auto& frame = packet->frame;
        frame.session = 10242030;
        frame.sequence = 1;
        frame.device = "synthetic-ui-test";
        frame.width = 640;
        frame.height = 360;
        frame.captured_monotonic = frame.completed_monotonic = RadarTime::monotonicSeconds();
        frame.captured_wall = RadarTime::wallSeconds();
        frame.people = {{{251, 133, 102, 198}, .98f, 7, true},
                        {{442, 142, 64, 121}, .91f, 8, true}};
        panel.canvas_->display(packet, 7, 1, 1);
    }
};
#endif

struct RadarGUIIntegrationAccess {
    static QWidget* radarGroup(RadarGUI& gui) { return gui.ui->groupBox; }
    static QGraphicsView* radarView(RadarGUI& gui) { return gui.ui->graphicsView; }
    static RadarCooperative::CooperativeView* cooperative(RadarGUI& gui) { return gui.cooperativeView_; }
#if RADAR_ENABLE_CAMERA
    static CameraAssistPanel* camera(RadarGUI& gui) { return gui.cameraAssist_; }
#endif
    static bool hardwareIdle(RadarGUI& gui) {
        return !gui.is_capturing && !gui.collector && !gui.radar && !gui.dca1000_cfg &&
               gui.rosRuntime_ && !gui.rosRuntime_->active()
#if RADAR_ENABLE_CAMERA
               && gui.cameraAssist_ && CameraPanelTestAccess::idle(*gui.cameraAssist_)
#endif
            ;
    }
    static bool socketClosed(RadarGUI& gui) {
        return gui.udpSocket && gui.udpSocket->state() == QAbstractSocket::UnconnectedState;
    }
    static void injectPreview(RadarGUI& gui) {
        check(hardwareIdle(gui), "Preview requires all hardware and ROS to remain idle");
        if (gui.cooperativeTimer_) gui.cooperativeTimer_->stop();
        if (gui.updateTimer) gui.updateTimer->stop();
        using namespace RadarCooperative;
        Snapshot snapshot;
        snapshot.acquisitionActive = true; // Display fixture only; GUI capture flag remains false.
        snapshot.trackingEnabled = true;
        snapshot.radar = {true, 0.0, .35};
        snapshot.camera = {true, 0.0, .5};
        snapshot.targetSelected = true;
        snapshot.targetPersonConfirmed = true;
        snapshot.targetPositionValid = true;
        snapshot.observation = ObservationState::Measured;
        snapshot.targetLabel = QStringLiteral("T1 · P1");
        snapshot.targetPosition = QPointF(-.35, 3.2);
        snapshot.otherPeople = {{QStringLiteral("P2"), QPointF(1.7, 5.1)},
                                {QStringLiteral("P3"), QPointF(-2.3, 6.5)}};
        snapshot.visualEvidence = VisualEvidence::Verified;
        snapshot.followState = FollowState::NotStarted;
        snapshot.detail = QStringLiteral("UI测试 · 全部为人工示例数据");
        gui.cooperativeView_->setSnapshot(snapshot);
#if RADAR_ENABLE_CAMERA
        CameraPanelTestAccess::injectImage(*gui.cameraAssist_);
#endif
        gui.sceneXMaxMeters = 5;
        gui.sceneYMaxMeters = 8;
        gui.scene->setSceneRect(-500, 0, 1000, 800);
        gui.drawAxes();
        const QVector<QPointF> centers{QPointF(-.35, 3.2), QPointF(1.7, 5.1), QPointF(-2.3, 6.5)};
        for (int target = 0; target < centers.size(); ++target) {
            const auto& center = centers[target];
            for (int n = 0; n < 36; ++n) {
                const float x = center.x() + .22 * std::sin(n * 2.399);
                const float y = center.y() + .28 * std::cos(n * 1.813);
                const QColor color = target == 0 ? QColor(37, 115, 236) : QColor(98, 121, 145);
                auto* point = gui.scene->addEllipse(gui.sceneXFromMeters(x) - 5,
                    gui.sceneYFromMeters(y) - 5, 10, 10, Qt::NoPen, color);
                gui.pointItems.push_back(point);
            }
            auto* label = gui.scene->addText(QStringLiteral("P%1").arg(target + 1));
            QFont font = label->font();
            font.setPixelSize(35);
            font.setBold(true);
            label->setFont(font);
            label->setDefaultTextColor(target == 0 ? QColor(37, 115, 236) : QColor(98, 121, 145));
            label->setPos(gui.sceneXFromMeters(center.x()) + 22, gui.sceneYFromMeters(center.y()) - 55);
            gui.pointItems.push_back(label);
        }
        if (gui.infoText) gui.infoText->setPlainText(QStringLiteral("模拟点云 · UI测试"));
        gui.applyZoom();
        gui.positionZoomButtons();
        const QSignalBlocker blocker(gui.ui->tableWidget);
        gui.ui->tableWidget->setRowCount(3);
        for (int row = 0; row < 3; ++row) {
            const QStringList values{row == 0 ? QStringLiteral("✓") : QString(),
                QStringLiteral("P%1").arg(row + 1),
                QString::number(std::hypot(centers[row].x(), centers[row].y()), 'f', 2),
                QStringLiteral("--")};
            for (int column = 0; column < 4; ++column) {
                auto* item = new QTableWidgetItem(values[column]);
                item->setTextAlignment(Qt::AlignCenter);
                if (row == 0) item->setBackground(QColor(222, 237, 255));
                gui.ui->tableWidget->setItem(row, column, item);
            }
        }
        gui.ui->lockedTargetInfoText->setPlainText(QStringLiteral(
            "UI测试：人工示例数据\n模拟锁定 P1；距离 3.22 m\n界面布局预览，不代表硬件实测\n实际采集、摄像头和 ROS 均未启动"));
        gui.setWindowTitle(QStringLiteral("UI测试 / 人工示例数据 · V1024-2158"));
        auto* label = gui.menuBar()->addAction(QStringLiteral("UI测试 · 人工示例数据"));
        label->setEnabled(false);
        check(hardwareIdle(gui), "Display fixture changed actual hardware state");
    }
};

static QRect inWindow(QWidget* widget, QWidget* window) {
    return QRect(widget->mapTo(window, QPoint(0, 0)), widget->size());
}

static void checkCombinedGeometry(RadarGUI& window) {
    auto* left = RadarGUIIntegrationAccess::cooperative(window);
    auto* cameraPane = window.findChild<QWidget*>("cooperativeCameraPane");
    auto* radarPane = window.findChild<QWidget*>("cooperativeRadarPane");
    auto* cloud = RadarGUIIntegrationAccess::radarView(window);
    check(left && cameraPane && radarPane && cloud, "Missing three-panel widgets");
    check(left->isVisible() && cameraPane->isVisible() && radarPane->isVisible() && cloud->isVisible(),
          "Combined page hid a required panel");
    const QRect l = inWindow(left, &window), c = inWindow(cameraPane, &window), r = inWindow(radarPane, &window);
    check(window.rect().contains(l) && window.rect().contains(c) && window.rect().contains(r),
          "Three-panel geometry exceeds the requested window");
    check(l.right() < c.left() && l.right() < r.left(), "Cooperative scene is not left of both sensor panes");
    check(c.bottom() < r.top(), "Camera is not above radar");
    check(std::abs(c.left() - r.left()) <= 2 && std::abs(c.width() - r.width()) <= 2,
          "Right sensor panels are not aligned");
    check(l.width() >= 245 && c.width() >= 195 && c.height() >= 135 && r.height() >= 135,
          "A panel collapsed below its readable size");
    auto* plus = cloud->viewport()->findChild<QPushButton*>("zoomInButton");
    auto* minus = cloud->viewport()->findChild<QPushButton*>("zoomOutButton");
    check(plus && minus && cloud->viewport()->rect().contains(plus->geometry()) &&
          cloud->viewport()->rect().contains(minus->geometry()), "Radar zoom buttons escaped their viewport");
}

static void dragHandle(QSplitter* splitter, int pixels) {
    check(splitter && splitter->count() == 2, "Missing two-child splitter");
    auto* handle = splitter->handle(1);
    check(handle && handle->isVisible(), "Splitter handle is hidden");
    const auto before = splitter->sizes();
    const QPoint local = handle->rect().center();
    const QPoint start = handle->mapToGlobal(local);
    const QPoint end = start + (splitter->orientation() == Qt::Horizontal ? QPoint(pixels, 0) : QPoint(0, pixels));
    QMouseEvent press(QEvent::MouseButtonPress, QPointF(local), QPointF(start),
                      Qt::LeftButton, Qt::LeftButton, Qt::NoModifier);
    QApplication::sendEvent(handle, &press);
    QMouseEvent move(QEvent::MouseMove, QPointF(handle->mapFromGlobal(end)), QPointF(end),
                     Qt::NoButton, Qt::LeftButton, Qt::NoModifier);
    QApplication::sendEvent(handle, &move);
    QMouseEvent release(QEvent::MouseButtonRelease, QPointF(handle->mapFromGlobal(end)), QPointF(end),
                        Qt::LeftButton, Qt::NoButton, Qt::NoModifier);
    QApplication::sendEvent(handle, &release);
    settle();
    if(splitter->sizes()==before)throw std::runtime_error(QString("Splitter %1 frozen at %2/%3; total %4; minima %5/%6")
        .arg(splitter->objectName()).arg(before[0]).arg(before[1]).arg(splitter->orientation()==Qt::Horizontal?splitter->width():splitter->height())
        .arg(splitter->widget(0)->minimumSizeHint().width()).arg(splitter->widget(1)->minimumSizeHint().width()).toStdString());
}

int main(int argc, char** argv) {
    QApplication app(argc, argv);
    app.setStyle(QStyleFactory::create("Fusion"));
    try {
#if !RADAR_ENABLE_CAMERA
        throw std::runtime_error("cooperative_layout_test requires RADAR_ENABLE_CAMERA=ON");
#else
        const auto arguments = app.arguments();
        check(arguments.size() == 1 || (arguments.size() == 3 && arguments[1] == "--screenshot"),
              "Usage: cooperative_layout_test [--screenshot PATH]");
        // Resolve screenshot path before changing cwd to the disposable directory.
        const QString screenshot = arguments.size() == 3 ? QFileInfo(arguments[2]).absoluteFilePath() : QString();
        QTemporaryDir directory;
        check(directory.isValid() && QDir::setCurrent(directory.path()), "Cannot create isolated test directory");
        QUdpSocket probe;
        check(probe.bind(QHostAddress(QHostAddress::LocalHost), static_cast<quint16>(0)), "Cannot allocate loopback UDP port");
        const quint16 port = probe.localPort();
        probe.close();
        qputenv("RADAR_RESULT_IP", "127.0.0.1");
        qputenv("RADAR_RESULT_PORT", QByteArray::number(port));
        qputenv("RADAR_HOST_IP", "127.0.0.1");
        qputenv("RADAR_FPGA_IP", "127.0.0.1");

        RadarGUI window;
        window.show();
        settle();
        check(!window.findChild<QTabWidget*>("sensorViewTabs"), "Standalone sensor tabs remain");
        check(window.findChild<QWidget*>("cooperativePage"), "Missing cooperative workspace");
        check(!window.findChild<QWidget*>("radarSinglePage") && !window.findChild<QWidget*>("cameraSinglePage"),
              "Standalone sensor pages remain");
        auto* controls = window.findChild<QTabWidget*>("workspaceControlTabs");
        check(controls && controls->count() >= 4, "Required controls are unreachable");
        QPointer<QWidget> radar = RadarGUIIntegrationAccess::radarGroup(window);
        QPointer<CameraAssistPanel> camera = RadarGUIIntegrationAccess::camera(window);
        check(camera && radar, "Missing shared camera or radar widget");
        QPointer<CameraCanvas> canvas = CameraPanelTestAccess::canvas(*camera);
        auto* horizontal = window.findChild<QSplitter*>("cooperativeSplitter");
        auto* vertical = window.findChild<QSplitter*>("cooperativeSensorSplitter");
        check(camera && canvas && radar && horizontal && vertical, "Missing shared sensor widgets");
        check(RadarGUIIntegrationAccess::hardwareIdle(window), "GUI construction started hardware or ROS");

        for (const QSize requested : {QSize(960, 573), QSize(1280, 720)}) {
            controls->setCurrentIndex(0);
            window.resize(requested);
            settle();
            check(window.size() == requested, "Layout minimum sizes expanded the requested window");
            checkCombinedGeometry(window);
            for (int round = 0; round < 3; ++round) {
                for (int index = 0; index < controls->count(); ++index) {
                    controls->setCurrentIndex(index);
                    settle();
                    check(radar && camera && canvas &&
                          RadarGUIIntegrationAccess::radarGroup(window) == radar &&
                          RadarGUIIntegrationAccess::camera(window) == camera &&
                          CameraPanelTestAccess::canvas(*camera) == canvas,
                          "Changing controls replaced shared sensor widgets");
                    check(window.findChildren<CameraAssistPanel*>().size() == 1 &&
                          window.findChildren<CameraCanvas*>().size() == 1,
                          "Changing controls duplicated camera panel or canvas");
                    check(radar->parentWidget()->objectName() == "cooperativeRadarPane" &&
                          camera->parentWidget()->objectName() == "cooperativeCameraPane",
                          "Controls moved sensors out of the cooperative workspace");
                    check(radar->isVisible() && camera->isVisible(), "A shared sensor became hidden");
                }
            }
            controls->setCurrentIndex(0);
            vertical->setSizes({vertical->height()/2,vertical->height()/2});
            settle();
            dragHandle(horizontal, 25);
            checkCombinedGeometry(window);
            dragHandle(vertical, 25);
            checkCombinedGeometry(window);
            dragHandle(horizontal, -25);
            dragHandle(vertical, -25);
            checkCombinedGeometry(window);
        }
        check(RadarGUIIntegrationAccess::hardwareIdle(window), "Layout interaction started hardware or ROS");
        if (!screenshot.isEmpty()) {
            controls->setCurrentIndex(0);
            window.resize(960, 573);
            horizontal->setSizes({350, 280});
            vertical->setSizes({220, 220});
            settle();
            RadarGUIIntegrationAccess::injectPreview(window);
            settle(5);
            checkCombinedGeometry(window);
            check(window.size() == QSize(960, 573), "Preview size differs from 960x573");
            check(QDir().mkpath(QFileInfo(screenshot).absolutePath()), "Cannot create screenshot output directory");
            check(window.grab().save(screenshot), "Cannot save synthetic UI screenshot");
        }
        window.close();
        settle();
        check(RadarGUIIntegrationAccess::socketClosed(window), "Closing test GUI left UDP socket bound");
        check(RadarGUIIntegrationAccess::hardwareIdle(window), "Closing test GUI did not leave hardware idle");
        std::cout << "PASS: 960x573/1280x720 three-pane geometry, shared widgets across all sidebar controls, "
                     "8 real splitter drags, isolated synthetic preview and shutdown; no hardware/ROS started\n";
        return 0;
#endif
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
