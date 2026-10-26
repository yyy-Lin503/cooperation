#ifndef V3_GUI_ICONS_HPP
#define V3_GUI_ICONS_HPP

#include <QIcon>
#include <QPainter>
#include <QPolygonF>

namespace RadarUi {
enum class Icon { Connect, Configure, Start, Track, Stop, Exit, Radar, Camera, Person, Log };

// Draw bundled vector shapes at each size, without external files or icon themes.
inline QIcon icon(Icon kind)
{
    QIcon result;
    for (int size : {16, 20, 24, 32, 48, 64}) {
        QPixmap pixmap(size, size);
        pixmap.fill(Qt::transparent);
        QPainter p(&pixmap);
        p.setRenderHint(QPainter::Antialiasing);
        p.scale(size / 24.0, size / 24.0);
        const QColor blue(40, 87, 120), green(35, 120, 70), red(170, 55, 55);
        p.setPen(QPen(blue, 1.8, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
        switch (kind) {
        case Icon::Connect:
            p.drawRoundedRect(QRectF(3, 3, 7, 6), 1, 1);
            p.drawRoundedRect(QRectF(14, 15, 7, 6), 1, 1);
            p.drawLine(QPointF(6.5, 9), QPointF(6.5, 17));
            p.drawLine(QPointF(6.5, 17), QPointF(14, 17));
            p.drawLine(QPointF(17.5, 15), QPointF(17.5, 7));
            p.drawLine(QPointF(17.5, 7), QPointF(10, 7));
            break;
        case Icon::Configure:
            for (int x : {5, 12, 19}) p.drawLine(x, 3, x, 21);
            p.setBrush(QColor(225, 240, 248));
            p.drawRoundedRect(QRectF(2, 6, 6, 4), 1, 1);
            p.drawRoundedRect(QRectF(9, 14, 6, 4), 1, 1);
            p.drawRoundedRect(QRectF(16, 8, 6, 4), 1, 1);
            break;
        case Icon::Start: {
            p.setPen(Qt::NoPen); p.setBrush(green);
            QPolygonF triangle;
            triangle << QPointF(6, 3) << QPointF(21, 12) << QPointF(6, 21);
            p.drawPolygon(triangle);
            break;
        }
        case Icon::Track:
            p.drawEllipse(QPointF(12, 12), 7, 7);
            p.drawEllipse(QPointF(12, 12), 2, 2);
            p.drawLine(12, 1, 12, 6); p.drawLine(12, 18, 12, 23);
            p.drawLine(1, 12, 6, 12); p.drawLine(18, 12, 23, 12);
            break;
        case Icon::Stop:
            p.setPen(Qt::NoPen); p.setBrush(red);
            p.drawRoundedRect(QRectF(4, 4, 16, 16), 2, 2);
            break;
        case Icon::Exit:
            p.drawLine(11, 3, 4, 3); p.drawLine(4, 3, 4, 21); p.drawLine(4, 21, 11, 21);
            p.setPen(QPen(red, 2, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
            p.drawLine(9, 12, 21, 12); p.drawLine(16, 7, 21, 12); p.drawLine(16, 17, 21, 12);
            break;
        case Icon::Radar:
            p.drawEllipse(QPointF(12, 12), 9, 9);
            p.drawEllipse(QPointF(12, 12), 5, 5);
            p.drawLine(12, 12, 18, 5);
            p.setBrush(green); p.drawEllipse(QPointF(8, 10), 1.5, 1.5);
            break;
        case Icon::Camera:
            p.drawRoundedRect(QRectF(2, 6, 20, 15), 2, 2);
            p.drawLine(7, 6, 9, 3); p.drawLine(9, 3, 15, 3); p.drawLine(15, 3, 17, 6);
            p.drawEllipse(QPointF(12, 13.5), 4.5, 4.5);
            break;
        case Icon::Person:
            p.drawEllipse(QPointF(12, 6), 4, 4);
            p.drawArc(QRectF(4, 12, 16, 16), 0, 180 * 16);
            p.drawLine(4, 20, 20, 20);
            break;
        case Icon::Log:
            p.drawRoundedRect(QRectF(4, 2, 16, 20), 1, 1);
            for (int y : {7, 12, 17}) p.drawLine(8, y, 16, y);
            break;
        }
        p.end();
        result.addPixmap(pixmap);
    }
    return result;
}
}
#endif
