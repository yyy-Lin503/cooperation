#include "cooperative_view.hpp"

#include <QFontMetrics>
#include <QLinearGradient>
#include <QPainter>
#include <QPainterPath>
#include <QRadialGradient>
#include <QStringList>
#include <algorithm>
#include <cmath>

namespace RadarCooperative {
namespace {
const QColor ink(31, 49, 69);
const QColor muted(115, 132, 151);
const QColor blue(37, 115, 236);
const QColor amber(187, 123, 25);
const QColor red(189, 67, 71);

bool finitePoint(const QPointF& p) {
    return std::isfinite(p.x()) && std::isfinite(p.y());
}

void text(QPainter& p, const QRectF& box, const QString& value, int pixels,
          const QColor& color, bool bold = false, int align = Qt::AlignLeft | Qt::AlignVCenter) {
    QFont f = p.font();
    f.setPixelSize(pixels);
    f.setBold(bold);
    p.setFont(f);
    p.setPen(color);
    p.drawText(box, align, QFontMetrics(f).elidedText(value, Qt::ElideRight,
                                                  std::max(0, static_cast<int>(box.width()))));
}

void drawPerson(QPainter& p, const QPointF& feet, qreal scale, const QColor& color,
                bool selected, bool ghost) {
    p.save();
    p.translate(feet);
    p.scale(scale, scale);
    p.setPen(Qt::NoPen);
    p.setBrush(QColor(45, 65, 90, ghost ? 12 : 24));
    p.drawEllipse(QRectF(-16, -3, 32, 8));
    if (selected) {
        p.setBrush(QColor(color.red(), color.green(), color.blue(), ghost ? 10 : 22));
        p.setPen(QPen(QColor(color.red(), color.green(), color.blue(), 130), 1.5,
                      ghost ? Qt::DashLine : Qt::SolidLine));
        p.drawEllipse(QRectF(-24, -7, 48, 16));
    }
    p.setBrush(color);
    p.setPen(Qt::NoPen);
    p.drawEllipse(QRectF(-6.5, -49, 13, 13));
    p.drawRoundedRect(QRectF(-10, -33, 20, 23), 8, 8);
    p.setPen(QPen(color, 5.3, Qt::SolidLine, Qt::RoundCap));
    p.drawLine(QPointF(-6, -14), QPointF(-7, -2));
    p.drawLine(QPointF(6, -14), QPointF(7, -2));
    p.drawLine(QPointF(-9, -29), QPointF(-14, -17));
    p.drawLine(QPointF(9, -29), QPointF(14, -17));
    p.restore();
}

void drawCart(QPainter& p, const QPointF& position) {
    p.save();
    p.translate(position);
    p.setPen(Qt::NoPen);
    p.setBrush(QColor(40, 65, 100, 24));
    p.drawEllipse(QRectF(-29, 6, 58, 16));
    p.setBrush(QColor(79, 96, 115));
    p.drawRoundedRect(QRectF(-23, -12, 7, 27), 3, 3);
    p.drawRoundedRect(QRectF(16, -12, 7, 27), 3, 3);
    QLinearGradient body(0, -22, 0, 20);
    body.setColorAt(0, QColor(250, 253, 255));
    body.setColorAt(1, QColor(185, 202, 219));
    p.setBrush(body);
    p.setPen(QPen(QColor(156, 177, 199), 1));
    p.drawRoundedRect(QRectF(-18, -24, 36, 42), 9, 9);
    p.setPen(Qt::NoPen);
    p.setBrush(QColor(62, 89, 119));
    p.drawRoundedRect(QRectF(-11, -16, 22, 12), 4, 4);
    p.setBrush(blue);
    p.drawRoundedRect(QRectF(-11, 9, 22, 4), 2, 2);
    p.setPen(QPen(QColor(255, 255, 255, 220), 1.4));
    p.drawLine(QPointF(-4, 2), QPointF(0, -2));
    p.drawLine(QPointF(0, -2), QPointF(4, 2));
    p.restore();
}
} // namespace

CooperativeView::CooperativeView(QWidget* parent) : QWidget(parent) {
    setObjectName("cooperativeView");
    setAccessibleName(QStringLiteral("雷视协同实时跟踪视图"));
    setMinimumSize(minimumSizeHint());
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    repaintTimer_.setInterval(100);
    connect(&repaintTimer_, &QTimer::timeout, this, [this] {
        if (isVisible()) update(); // Age out a frozen source even without new snapshots.
    });
    repaintTimer_.start();
}

QSize CooperativeView::sizeHint() const { return QSize(420, 460); }
QSize CooperativeView::minimumSizeHint() const { return QSize(250, 320); }

void CooperativeView::setSnapshot(const Snapshot& snapshot) {
    snapshot_ = snapshot;
    received_.restart();
    update();
}

bool CooperativeView::isFresh(const SensorState& sensor) const {
    if (!sensor.available || !received_.isValid() || !std::isfinite(sensor.ageSeconds) ||
        !std::isfinite(sensor.freshForSeconds) || sensor.ageSeconds < 0 || sensor.freshForSeconds <= 0)
        return false;
    return sensor.ageSeconds + received_.elapsed() / 1000.0 <= sensor.freshForSeconds;
}

void CooperativeView::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    p.setRenderHint(QPainter::TextAntialiasing);
    QLinearGradient background(0, 0, 0, height());
    background.setColorAt(0, QColor(250, 252, 255));
    background.setColorAt(1, QColor(229, 237, 247));
    p.fillRect(rect(), background);

    const bool radarFresh = snapshot_.acquisitionActive && isFresh(snapshot_.radar);
    const bool cameraFresh = isFresh(snapshot_.camera);
    const bool selected = snapshot_.acquisitionActive && snapshot_.trackingEnabled && snapshot_.targetSelected;
    const bool positionValid = selected && snapshot_.targetPositionValid && finitePoint(snapshot_.targetPosition);
    const bool predicted = positionValid && snapshot_.observation == ObservationState::Predicted;
    const bool lost = selected && snapshot_.observation == ObservationState::Lost;
    const bool unknown = positionValid && snapshot_.observation == ObservationState::Unknown;
    const bool mismatch = cameraFresh && snapshot_.visualEvidence == VisualEvidence::Mismatch;
    const bool cooperative = radarFresh && cameraFresh &&
                             snapshot_.visualEvidence == VisualEvidence::Verified;
    const qreal margin = width() < 300 ? 12 : 16;
    const qreal contentWidth = width() - margin * 2;
    QString title, subtitle;
    QColor statusColor = muted;
    if (!snapshot_.acquisitionActive) {
        title = QStringLiteral("采集尚未启动或已停止");
        subtitle = QStringLiteral("开始采集后显示实时目标位置");
    } else if (!snapshot_.trackingEnabled) {
        title = QStringLiteral("等待开始追踪");
        subtitle = QStringLiteral("开启采集与追踪后显示目标");
    } else if (!radarFresh) {
        title = snapshot_.radar.available ? QStringLiteral("雷达数据已过期") : QStringLiteral("等待雷达数据");
        subtitle = positionValid ? QStringLiteral("仅保留上次位置，请检查数据连接") : QStringLiteral("暂无可用的实时目标位置");
        statusColor = snapshot_.radar.available ? red : muted;
    } else if (!selected) {
        title = QStringLiteral("等待选择跟踪目标");
        subtitle = QStringLiteral("锁定雷达目标后，点击画面人物绑定");
    } else if (lost) {
        title = QStringLiteral("目标丢失 · 等待重选");
        subtitle = positionValid ? QStringLiteral("显示上次位置，请重新确认目标") : QStringLiteral("请重新选择并确认跟踪目标");
        statusColor = red;
    } else if (!positionValid) {
        title = QStringLiteral("等待目标位置");
        subtitle = QStringLiteral("已选目标，尚无有效雷达坐标");
        statusColor = amber;
    } else if (mismatch) {
        title = QStringLiteral("相机绑定状态异常");
        subtitle = QStringLiteral("请重新核对手动绑定的人物");
        statusColor = red;
    } else if (predicted) {
        title = QStringLiteral("目标暂失 · 预测位置");
        subtitle = QStringLiteral("当前帧未观测到目标，等待重新确认");
        statusColor = amber;
    } else if (unknown) {
        title = QStringLiteral("目标状态待确认");
        subtitle = QStringLiteral("已有位置，但尚未确认当前观测状态");
        statusColor = amber;
    } else {
        statusColor = blue;
        switch (snapshot_.followState) {
        case FollowState::Running: title = QStringLiteral("随行控制已启动"); break;
        case FollowState::Limited: title = QStringLiteral("受限跟随"); statusColor = amber; break;
        case FollowState::Turning: title = QStringLiteral("仅允许转向"); statusColor = amber; break;
        case FollowState::Stopped: title = QStringLiteral("跟随已暂停"); statusColor = amber; break;
        case FollowState::Reacquiring: title = QStringLiteral("正在重新确认目标"); statusColor = amber; break;
        case FollowState::Requested: title = QStringLiteral("请求跟随 · 待反馈"); break;
        case FollowState::NotStarted: title = QStringLiteral("已锁定 · 跟随未启动"); break;
        case FollowState::Unknown: title = QStringLiteral("已锁定 · 跟随状态未知"); break;
        }
        const QString label = snapshot_.targetLabel.isEmpty() ? QStringLiteral("已选目标") : snapshot_.targetLabel;
        subtitle = cooperative ? label + QStringLiteral(" · 绑定人工关联人物当前可见") :
                   cameraFresh && snapshot_.visualEvidence == VisualEvidence::Bound
                       ? label + QStringLiteral(" · 已绑定相机人物") :
                   label + (cameraFresh ? QStringLiteral(" · 等待相机绑定") : QStringLiteral(" · 相机暂无实时画面"));
    }

    text(p, QRectF(margin, 10, contentWidth * 0.46, 22), QStringLiteral("实时跟踪"), 12, muted, true);
    const qreal chipWidth = 53;
    auto sensorChip = [&](qreal x, const QString& label, bool fresh) {
        p.setPen(Qt::NoPen);
        p.setBrush(fresh ? QColor(51, 164, 133) : QColor(165, 178, 194));
        p.drawEllipse(QPointF(x + 4, 21), 3, 3);
        text(p, QRectF(x + 12, 10, 40, 22), label, 11, muted);
    };
    sensorChip(width() - margin - 2 * chipWidth, QStringLiteral("雷达"), radarFresh);
    sensorChip(width() - margin - chipWidth, QStringLiteral("相机"), cameraFresh);

    const QRectF statusBox(margin, 39, contentWidth, 59);
    p.setPen(QPen(QColor(statusColor.red(), statusColor.green(), statusColor.blue(), 40), 1));
    p.setBrush(QColor(255, 255, 255, 210));
    p.drawRoundedRect(statusBox, 12, 12);
    p.setPen(Qt::NoPen);
    p.setBrush(statusColor);
    p.drawRoundedRect(QRectF(margin + 11, 52, 4, 31), 2, 2);
    text(p, QRectF(margin + 24, 46, contentWidth - 34, 25), title,
         width() < 300 ? 15 : 17, statusColor, true);
    text(p, QRectF(margin + 24, 72, contentWidth - 34, 18), subtitle, 10, muted);

    QString distance = QStringLiteral("—"), direction = QStringLiteral("—");
    if (positionValid) {
        distance = QStringLiteral("%1 m").arg(std::hypot(snapshot_.targetPosition.x(), snapshot_.targetPosition.y()), 0, 'f', 2);
        const double angle = std::atan2(snapshot_.targetPosition.x(), snapshot_.targetPosition.y()) * 180.0 / 3.14159265358979323846;
        direction = std::abs(angle) < 2.0 ? QStringLiteral("正前方") :
            QStringLiteral("%1 %2°").arg(angle < 0 ? QStringLiteral("左") : QStringLiteral("右")).arg(std::abs(angle), 0, 'f', 0);
    }
    const QString positionType = !positionValid ? QStringLiteral("暂无位置") : lost ? QStringLiteral("目标丢失") : !radarFresh
        ? QStringLiteral("上次位置") : predicted ? QStringLiteral("预测位置") : QStringLiteral("实时观测");
    const qreal columnWidth = contentWidth / 3;
    const QStringList labels{QStringLiteral("目标距离"), QStringLiteral("目标方位"), QStringLiteral("位置状态")};
    const QStringList values{distance, direction, positionType};
    for (int i = 0; i < 3; ++i) {
        const QRectF cell(margin + columnWidth * i, 108, columnWidth - 4, 43);
        text(p, QRectF(cell.x(), cell.y(), cell.width(), 15), labels[i], 10, muted);
        const QString value = i == 2 && unknown ? QStringLiteral("待确认") : values[i];
        text(p, QRectF(cell.x(), cell.y() + 17, cell.width(), 24), value,
             i == 2 ? 12 : 16, positionValid && (predicted || lost || unknown || !radarFresh) ? statusColor : ink, true);
    }

    const QRectF ground(margin, 160, contentWidth, std::max(60.0, height() - 194.0));
    const qreal centerX = width() * 0.5;
    // Leave room above the farthest person's feet for the silhouette and label;
    // keep the same projection for distance lines and people.
    const qreal horizonY = ground.top() + std::max<qreal>(52, std::min<qreal>(66, ground.height() * 0.48));
    const qreal baseY = ground.bottom() - 26;
    const double forwardRange = positionValid && snapshot_.targetPosition.y() > 0
        ? std::max(8.0, std::min(40.0, snapshot_.targetPosition.y() * 1.25)) : 8.0;
    const auto project = [&](const QPointF& point) {
        const double y = std::max(0.0, std::min(forwardRange, point.y()));
        const double fraction = (y / (y + 4.0)) / (forwardRange / (forwardRange + 4.0));
        const qreal pixelsPerMetre = contentWidth * 0.19 / (1.0 + y * 0.13);
        return QPointF(centerX + point.x() * pixelsPerMetre, baseY - (baseY - horizonY) * fraction);
    };

    p.save();
    p.setClipRect(ground.adjusted(-3, -3, 3, 10));
    QPainterPath floor;
    floor.moveTo(centerX - contentWidth * 0.12, horizonY);
    floor.lineTo(centerX + contentWidth * 0.12, horizonY);
    floor.lineTo(ground.right() + 35, ground.bottom() + 20);
    floor.lineTo(ground.left() - 35, ground.bottom() + 20);
    floor.closeSubpath();
    QLinearGradient floorFill(0, horizonY, 0, ground.bottom());
    floorFill.setColorAt(0, QColor(242, 247, 253, 20));
    floorFill.setColorAt(1, QColor(211, 224, 240, 160));
    p.fillPath(floor, floorFill);
    const int interval = forwardRange > 16 ? 5 : 2;
    for (int metres = interval; metres <= forwardRange; metres += interval) {
        const QPointF at = project(QPointF(0, metres));
        const qreal half = contentWidth * (0.49 - 0.35 * (baseY - at.y()) / std::max<qreal>(1, baseY - horizonY));
        p.setPen(QPen(QColor(159, 181, 205, 75), 1));
        p.drawLine(QPointF(centerX - half, at.y()), QPointF(centerX + half, at.y()));
        text(p, QRectF(centerX + half + 5, at.y() - 7, 35, 14), QStringLiteral("%1m").arg(metres), 9, QColor(145, 164, 184));
    }

    struct PositionedPerson { QPointF position; QString label; bool target; };
    QVector<PositionedPerson> people;
    if (radarFresh && snapshot_.trackingEnabled) {
        for (const auto& person : snapshot_.otherPeople) {
            if (finitePoint(person.position) && person.position.y() >= 0 && person.position.y() <= forwardRange)
                people.push_back({person.position, person.label, false});
        }
    }
    if (positionValid && snapshot_.trackingEnabled && snapshot_.targetPosition.y() >= 0)
        people.push_back({snapshot_.targetPosition, snapshot_.targetLabel, true});
    std::sort(people.begin(), people.end(), [](const PositionedPerson& a, const PositionedPerson& b) {
        return a.position.y() > b.position.y();
    });
    for (const auto& person : people) {
        QPointF feet = project(person.position);
        const bool outside = feet.x() < ground.left() + 24 || feet.x() > ground.right() - 24 || person.position.y() > forwardRange;
        feet.setX(std::max(ground.left() + 24, std::min(ground.right() - 24, feet.x())));
        const qreal compactScale = ground.height() < 155 ? 0.72 : 1.0;
        const qreal scale = compactScale * std::max(0.58, 1.0 - std::max(0.0, person.position.y()) * 0.025);
        if (!person.target && outside) continue;
        const QColor color = !person.target ? QColor(155, 171, 189) : !radarFresh ? QColor(151, 163, 181) :
                             lost || mismatch ? red : predicted || unknown ? amber : blue;
        if(person.target&&snapshot_.targetPersonConfirmed)
            drawPerson(p, feet, scale, color, true, !radarFresh || predicted || lost || unknown || !cooperative);
        else {
            p.setPen(QPen(color,2));p.setBrush(QColor(color.red(),color.green(),color.blue(),45));
            p.drawEllipse(feet,person.target?11:7,person.target?11:7);
            if(!person.target)text(p,QRectF(feet.x()+9,feet.y()-12,65,20),person.label,11,color,true);
        }
        if (person.target) {
            const QString label = (person.label.isEmpty() ? QStringLiteral("跟踪目标") : person.label) +
                (outside ? QStringLiteral(" · 超出示意范围") : lost ? QStringLiteral(" · 已丢失") : !radarFresh ? QStringLiteral(" · 上次") : predicted ? QStringLiteral(" · 预测") : unknown ? QStringLiteral(" · 待确认") : QString());
            const qreal tagCenter = std::max(ground.left() + 62, std::min(ground.right() - 62, feet.x()));
            const QRectF tag(tagCenter - 62, feet.y() - 50 * scale - 23, 124, 21);
            p.setPen(Qt::NoPen);
            p.setBrush(QColor(255, 255, 255, 220));
            p.drawRoundedRect(tag, 7, 7);
            text(p, tag.adjusted(4, 0, -4, 0), label, 11, color, true, Qt::AlignCenter);
        }
    }
    drawCart(p, QPointF(centerX, baseY + 9));
    if (!positionValid || !snapshot_.trackingEnabled) {
        text(p, QRectF(ground.left(), ground.top() + 26, ground.width(), 24),
             snapshot_.trackingEnabled ? QStringLiteral("在右侧锁定人员，即可查看相对位置") : QStringLiteral("等待实时跟踪数据"),
             11, muted, false, Qt::AlignCenter);
    } else if (snapshot_.targetPosition.y() < 0) {
        text(p, QRectF(ground.left(), ground.top() + 26, ground.width(), 24),
             QStringLiteral("目标位于小车后方"), 13, amber, true, Qt::AlignCenter);
    }
    p.restore();
    const QString footer = snapshot_.detail.isEmpty()
        ? QStringLiteral("相对位置示意 · 距离来自雷达") : snapshot_.detail;
    text(p, QRectF(margin, height() - 25, contentWidth, 18), footer, 10, muted, false, Qt::AlignCenter);
    setToolTip(title + QLatin1Char('\n') + subtitle + QLatin1Char('\n') +
               QStringLiteral("距离：%1　方位：%2　%3").arg(distance, direction, positionType));
}

} // namespace RadarCooperative
