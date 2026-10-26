#ifndef CLUSTER_HPP
#define CLUSTER_HPP

#include <QObject>
#include <QTimer>
#include <QGraphicsScene>
#include <QGraphicsItem>
#include <QGraphicsTextItem>
#include <QString>
#include <QPointF>
#include <vector>

struct TestPoint {
    float x = 0.0f;
    float y = 0.0f;
    float relVel = 0.0f;
    float absVel = 0.0f;
    bool moving = false;
    int sourceFrame = -1;
    int rangeBin = -1;
    int dopplerBin = -1;
    float snr = 0.0f;
    float doaPeakConfidence = 0.0f;
};

class Cluster : public QObject {
    Q_OBJECT

public:
    explicit Cluster(QGraphicsScene* scene, QObject* parent = nullptr);
    ~Cluster();

    bool loadFile(const QString& filePath);
    void renderFrame(int index);
    void renderWindow(const std::vector<TestPoint>& windowPoints,
                      int startFrameNumber,
                      int endFrameNumber,
                      bool showInfo = true, bool drawClusters = true);
    int frameCount() const;
    void setWindowSize(int frames);
    int windowSize() const { return windowSize_; }

    void startPlayback(int intervalMs = 100);
    void stopPlayback();
    bool isPlaying() const;

    const std::vector<std::vector<TestPoint>>& frames() const { return frames_; }

signals:
    void playbackFinished();

private slots:
    void advancePlayback();

private:
    QGraphicsScene* scene_;
    std::vector<QGraphicsItem*> renderedItems_;

    std::vector<std::vector<TestPoint>> frames_;
    int frameIndex_ = 0;
    int windowSize_ = 5;
    QTimer* timer_ = nullptr;

    std::vector<int> dbscan(const std::vector<QPointF>& pts,
                            double baseEps,
                            int baseMinPts,
                            bool movingCluster);
    void clearRenderedItems();
};

#endif // CLUSTER_HPP
