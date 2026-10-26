#include "cluster.hpp"
#include <QFile>
#include <QTextStream>
#include <QFont>
#include <QPen>
#include <QBrush>
#include <QMap>
#include <QVector>
#include <QDebug>
#include <limits>
#include <algorithm>
#include <cmath>

Cluster::Cluster(QGraphicsScene* scene, QObject* parent)
    : QObject(parent), scene_(scene)
{
    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &Cluster::advancePlayback);
}

Cluster::~Cluster()
{
    stopPlayback();
    clearRenderedItems();
}

bool Cluster::loadFile(const QString& filePath)
{
    frames_.clear();
    QFile f(filePath);
    if (!f.open(QIODevice::ReadOnly | QIODevice::Text))
        return false;

    QTextStream in(&f);
    std::map<int, std::vector<TestPoint>> framesMap;
    int minFrame = std::numeric_limits<int>::max();
    int maxFrame = std::numeric_limits<int>::min();

    while (!in.atEnd()) {
        const QString line = in.readLine();
        if (line.isEmpty()) continue;
        const QStringList parts = line.split(',');
        if (parts.size() >= 6) {
            bool okF = false, okX = false, okY = false, okM = false, okRv = false, okAv = false;
            int frame = parts[0].toInt(&okF);
            float x = parts[1].toFloat(&okX);
            float y = parts[2].toFloat(&okY);
            float rv = parts[3].toFloat(&okRv);
            float av = parts[4].toFloat(&okAv);
            int movingInt = parts[5].toInt(&okM);
            if (okF && okX && okY && okM) {
                TestPoint tp;
                tp.x = x;
                tp.y = y;
                tp.relVel = okRv ? rv : 0.0f;
                tp.absVel = okAv ? av : std::abs(rv);
                tp.moving = movingInt == 1;
                tp.sourceFrame = parts.size() >= 8
                    ? parts[7].trimmed().toInt() : frame;
                tp.rangeBin = parts.size() >= 9
                    ? parts[8].trimmed().toInt() : -1;
                tp.dopplerBin = parts.size() >= 10
                    ? parts[9].trimmed().toInt() : -1;
                tp.snr = parts.size() >= 11
                    ? parts[10].trimmed().toFloat() : 0.0f;
                tp.doaPeakConfidence = parts.size() >= 12
                    ? parts[11].trimmed().toFloat() : 0.0f;
                framesMap[frame].push_back(tp);
                if (frame < minFrame) minFrame = frame;
                if (frame > maxFrame) maxFrame = frame;
            }
        }
    }
    f.close();

    if (minFrame == std::numeric_limits<int>::max())
        return false;

    frames_.reserve(maxFrame - minFrame + 1);
    for (int fr = minFrame; fr <= maxFrame; ++fr) {
        auto it = framesMap.find(fr);
        if (it != framesMap.end())
            frames_.push_back(std::move(it->second));
        else
            frames_.push_back({});
    }
    return true;
}

void Cluster::renderFrame(int index)
{
    if (index < 0 || index + windowSize_ - 1 >= static_cast<int>(frames_.size()))
        return;

    // Aggregate the same configurable window used by realtime/offline tracking.
    std::vector<TestPoint> windowPts;
    windowPts.reserve(512);
    for (int k = 0; k < windowSize_; ++k) {
        const auto& fr = frames_[index + k];
        windowPts.insert(windowPts.end(), fr.begin(), fr.end());
    }

    renderWindow(windowPts, index + 1, index + windowSize_, true);
}

void Cluster::setWindowSize(int frames)
{
    windowSize_ = std::clamp(frames, 1, 10);
}

void Cluster::renderWindow(const std::vector<TestPoint>& windowPts,
                           int startFrameNumber,
                           int endFrameNumber,
                           bool showInfo, bool drawClusters)
{
    clearRenderedItems();

    const qreal scale = 100.0;
    const QRectF sceneRect = scene_->sceneRect();
    auto sceneX = [scale](double x) -> qreal {
        return static_cast<qreal>(x) * scale;
    };
    auto sceneY = [scale, sceneRect](double y) -> qreal {
        return sceneRect.bottom() - static_cast<qreal>(y) * scale;
    };

    for (const auto& pt : windowPts) {
        QColor color = pt.sourceFrame>=0&&pt.sourceFrame!=endFrameNumber ? QColor(175,185,195) : (pt.moving ? QColor(Qt::red) : QColor(Qt::black));
        auto* ellipse = scene_->addEllipse(sceneX(pt.x) - 6, sceneY(pt.y) - 6,
                                           12, 12, QPen(color), QBrush(color));
        renderedItems_.push_back(ellipse);
    }

    if(drawClusters) {
    // DBSCAN on moving targets. eps/minPts are distance-adaptive inside dbscan().
    std::vector<QPointF> movingXY;
    movingXY.reserve(windowPts.size());
    std::vector<float> movingAbsV;
    movingAbsV.reserve(windowPts.size());
    for (const auto& p : windowPts) {
        if (p.moving) {
            movingXY.emplace_back(p.x, p.y);
            movingAbsV.push_back(p.absVel);
        }
    }
    std::vector<int> labels = movingXY.empty() ? std::vector<int>() : dbscan(movingXY, 0.6, 6, true);

    QMap<int, QVector<int>> clusters;
    for (int i = 0; i < static_cast<int>(labels.size()); ++i)
        if (labels[i] >= 0)
            clusters[labels[i]].push_back(i);

    for (auto it = clusters.begin(); it != clusters.end(); ++it) {
        const auto& idxs = it.value();
        if (idxs.isEmpty()) continue;
        double sx = 0, sy = 0, sv = 0;
        for (int id : idxs) {
            sx += movingXY[id].x();
            sy += movingXY[id].y();
            sv += movingAbsV[id];
        }
        double cx = sx / idxs.size();
        double cy = sy / idxs.size();
        double vbar = sv / idxs.size();
        auto* label = scene_->addText(QString::number(vbar, 'f', 2) + " m/s", QFont("Arial", 50));
        label->setDefaultTextColor(Qt::red);
        label->setPos(sceneX(cx + 0.8), sceneY(cy) - 10);
        renderedItems_.push_back(label);
    }

    // DBSCAN on static targets. eps/minPts are distance-adaptive inside dbscan().
    std::vector<QPointF> staticXY;
    staticXY.reserve(windowPts.size());
    for (const auto& p : windowPts)
        if (!p.moving)
            staticXY.emplace_back(p.x, p.y);

    if (!staticXY.empty()) {
        std::vector<int> slabels = dbscan(staticXY, 1.0, 5, false);
        QMap<int, QVector<int>> sclusters;
        for (int i = 0; i < static_cast<int>(slabels.size()); ++i)
            if (slabels[i] >= 0)
                sclusters[slabels[i]].push_back(i);

        QPen cenPen(QColor(100, 100, 100));
        QBrush cenBrush(QColor(160, 160, 160, 160));
        for (auto it = sclusters.begin(); it != sclusters.end(); ++it) {
            const auto& idxs = it.value();
            if (idxs.isEmpty()) continue;
            double sx = 0, sy = 0;
            for (int id : idxs) {
                sx += staticXY[id].x();
                sy += staticXY[id].y();
            }
            double cx = sx / idxs.size();
            double cy = sy / idxs.size();
            auto* cen = scene_->addEllipse(sceneX(cx) - 4, sceneY(cy) - 4,
                                           8, 8, cenPen, cenBrush);
            renderedItems_.push_back(cen);
        }
    }

    } // Optional legacy overlays; GUI reuses the tracker cluster result.
    if (showInfo) {
        auto* infoItem = scene_->addText(
            QString("目标聚类 - 窗口: %1-%2\n点数(%3帧总计): %4")
                .arg(startFrameNumber)
                .arg(endFrameNumber)
                .arg(windowSize_)
                .arg(static_cast<int>(windowPts.size())),
            QFont("Arial", 35));
        infoItem->setDefaultTextColor(Qt::black);
        infoItem->setPos(10, 10);
        renderedItems_.push_back(infoItem);
    }
}

int Cluster::frameCount() const
{
    return static_cast<int>(frames_.size());
}

void Cluster::startPlayback(int intervalMs)
{
    if (frames_.size() < static_cast<std::size_t>(windowSize_) || timer_->isActive())
        return;
    frameIndex_ = 0;
    renderFrame(frameIndex_);
    timer_->start(intervalMs);
}

void Cluster::stopPlayback()
{
    timer_->stop();
    clearRenderedItems();
}

bool Cluster::isPlaying() const
{
    return timer_->isActive();
}

void Cluster::advancePlayback()
{
    if (frames_.size() < static_cast<std::size_t>(windowSize_))
        return;
    frameIndex_++;
    if (frameIndex_ + windowSize_ - 1 >= static_cast<int>(frames_.size())) {
        timer_->stop();
        emit playbackFinished();
        return;
    }
    renderFrame(frameIndex_);
}

// ---- DBSCAN (2D Euclidean distance, distance-adaptive eps/minPts) ----
std::vector<int> Cluster::dbscan(const std::vector<QPointF>& pts,
                                 double baseEps,
                                 int baseMinPts,
                                 bool movingCluster)
{
    const int n = static_cast<int>(pts.size());
    if (n == 0) return {};
    std::vector<int> labels(n, -99);

    auto pointRange = [&](int i) {
        return std::sqrt(pts[i].x() * pts[i].x() + pts[i].y() * pts[i].y());
    };

    auto adaptiveEps = [&](int i) {
        const double r = pointRange(i);
        if (movingCluster) {
            // Moving human points become sparser with range. Keep near-field
            // clusters tight, then relax gradually for far walking targets.
            return std::clamp(baseEps * 0.70 + 0.035 * r, 0.50, 0.95);
        }
        // Static clutter can be more spread out, but do not let far static
        // reflections absorb too much space.
        return std::clamp(baseEps * 0.65 + 0.035 * r, 0.80, 1.25);
    };

    auto adaptiveMinPts = [&](int i) {
        const double r = pointRange(i);
        int adaptive = baseMinPts;
        if (r < 4.0) {
            adaptive = baseMinPts + 1;
        } else if (r < 8.0) {
            adaptive = baseMinPts;
        } else if (r < 12.0) {
            adaptive = baseMinPts - 1;
        } else {
            adaptive = baseMinPts - 2;
        }

        const int lowerBound = movingCluster ? 3 : 3;
        return std::max(lowerBound, adaptive);
    };

    auto regionQuery = [&](int i) {
        std::vector<int> neigh;
        neigh.reserve(16);
        const double eps = adaptiveEps(i);
        const double eps2 = eps * eps;
        for (int j = 0; j < n; ++j) {
            double dx = pts[j].x() - pts[i].x();
            double dy = pts[j].y() - pts[i].y();
            if (dx * dx + dy * dy <= eps2)
                neigh.push_back(j);
        }
        return neigh;
    };

    int clusterId = 0;
    for (int i = 0; i < n; ++i) {
        if (labels[i] != -99) continue;
        auto neigh = regionQuery(i);
        if (static_cast<int>(neigh.size()) < adaptiveMinPts(i)) {
            labels[i] = -1;
            continue;
        }
        labels[i] = clusterId;
        for (size_t k = 0; k < neigh.size(); ++k) {
            int j = neigh[k];
            if (labels[j] == -1) labels[j] = clusterId;
            if (labels[j] != -99) continue;
            labels[j] = clusterId;
            auto neigh2 = regionQuery(j);
            if (static_cast<int>(neigh2.size()) >= adaptiveMinPts(j))
                neigh.insert(neigh.end(), neigh2.begin(), neigh2.end());
        }
        clusterId++;
    }
    return labels;
}

void Cluster::clearRenderedItems()
{
    for (auto* item : renderedItems_) {
        scene_->removeItem(item);
        delete item;
    }
    renderedItems_.clear();
}

#include "moc_cluster.cpp"
