#ifndef ROUTINGWORKER_H
#define ROUTINGWORKER_H

#include <QObject>
#include <QString>
#include <cmath>

static double euclid(double x1,double y1,double x2,double y2){
    return ::hypot(x1 - x2, y1 - y2);
}

class RoutingWorker : public QObject
{
    Q_OBJECT

public:
    enum AlgorithmType {
        Dijkstra = 0,
        AStar     = 1
    };

    RoutingWorker(const QString &shapefile,
                  double sx, double sy,
                  double ex, double ey,
                  AlgorithmType algo);

public slots:
    void process();

signals:
    void stageUpdate(const QString &msg);
    /// Emits the single output path (shp;csv) when done
    void routingComplete(const QString &outputPaths);
    void finished();

public:
    QString        inputPath;
    double         startX, startY, endX, endY;
    AlgorithmType  algoType;

    static double euclid(double x1,double y1,double x2,double y2) {
        return std::hypot(x1-x2, y1-y2);
    }

    struct PtKey { double x,y; bool operator==(PtKey const& o) const {
        return x==o.x && y==o.y;
    }};

    struct Edge { int to; double cost; };
};

#endif // ROUTINGWORKER_H
