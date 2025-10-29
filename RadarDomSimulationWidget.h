#pragma once
#include <QWidget>
#include <QTimer>
#include <osgEarth/MapNode>
#include <osg/Geode>

#include <osg/Geode>
#include <osg/Geometry>
#include <osgEarth/SpatialReference>
#include <osgEarth/ElevationQuery>

namespace Ui {
class RadarDomSimulationWidget;
}

class RadarDomSimulationWidget : public QWidget
{
    Q_OBJECT

public:
    explicit RadarDomSimulationWidget(osgEarth::MapNode* mapNode, QWidget *parent = nullptr);
    ~RadarDomSimulationWidget();

signals:
    void simulationStarted();
    void simulationStopped();

    void closeWindowClicked();

private slots:
    void on_pushButton_select_clicked();
    void on_pushButton_update_clicked();
    void on_pushButton_start_clicked();
    void on_pushButton_stop_clicked();
    void on_pushButton_clear_clicked();
    void glowUpdate();

    void on_pushButton_clicked();

private:
    void createRadarDome();
    void removeRadarDome();
    void resetDefaults();

    Ui::RadarDomSimulationWidget *ui;
    osgEarth::MapNode* _mapNode;

    osg::ref_ptr<osg::Geode> _radarGeode;
    osg::ref_ptr<osgEarth::GeoTransform> xform{nullptr};

    osg::ref_ptr<osg::Node> model = nullptr;
    osg::ref_ptr<osgEarth::GeoTransform> geoXform= nullptr;

    QTimer _glowTimer;
    bool _glowIncreasing;
    float _glowValue;
    void add3DModelToMap();

private:
    osg::ref_ptr<osg::Geode> _losGeode;    // for line-of-sight visualization
    const osgEarth::SpatialReference* _geoSRS;
    bool computeLineOfSight(double observerLon, double observerLat, double observerAlt,
                            double targetLon, double targetLat, double targetAlt,
                            std::vector<osg::Vec3d>& visiblePoints,
                            std::vector<osg::Vec3d>& blockedPoints);
    void drawLineSegment(const std::vector<osg::Vec3d>& pts, osg::Vec4 color);
    void updateLOSVisualization(double obsLon, double obsLat, double obsAlt,
                                double tgtLon, double tgtLat, double tgtAlt);

};
