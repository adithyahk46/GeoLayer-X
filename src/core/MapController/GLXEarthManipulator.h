#ifndef GLXEARTHMANIPULATOR_H
#define GLXEARTHMANIPULATOR_H

#include "GeoPointMetaType.h"
#include <QObject>

#include <osgEarth/EarthManipulator>
#include <osgEarth/MapNode>
#include <osgEarth/Viewpoint>
#include <osgViewer/View>

#include "osgearthcore_global.h"

class OSGEARTHCORE_EXPORT GLXEarthManipulator : public QObject, public osgEarth::EarthManipulator
{
    Q_OBJECT

public:
    explicit GLXEarthManipulator(osgEarth::MapNode* mapNode, QObject* parent = nullptr);
    void disableMouseAction(bool state);
    void setMapNode(osgEarth::MapNode* mapNode);

    void zoomToFullExtent();
    void zoomIn(double factor = 0.8);
    void zoomOut(double factor = 1.25);
    void flyTo(const osgEarth::Viewpoint& viewpoint, double duration = 1.0);
    void flyTo(double lon, double lat, double altitude, double range,
               double heading = 0.0, double pitch = -90.0, double duration = 1.0);
    osgEarth::Viewpoint currentViewpoint() const;
    const osgEarth::GeoPoint& currentGeoPosition() const { return _currentGeoPos; }
    const osg::Vec3d& currentWorldPosition() const { return _currentWorldPos; }
    /**
     * Returns the world-space normal at the most recent mouse intersection.
     */
    const osg::Vec3d& currentWorldNormal() const { return _currentWorldNormal; }

signals:
    void defaultHandleEvent(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
    void mouseClickEvent(osgEarth::GeoPoint mapPoint);
    void mouseDoubleClickEvent(osgEarth::GeoPoint mapPoint);
    void mouseMoveEvent(osgEarth::GeoPoint mapPoint);
    /**
     * Notifies consumers that the pointer no longer intersects the map.
     */
    void mousePositionLostEvent();
    void mouseScrollEvent(osgEarth::GeoPoint mapPoint);

protected:
    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa) override;

private:
    void zoomBy(double factor);
    bool getGeoPointFromScreen(osgViewer::View* view, float x, float y);
    osg::ref_ptr<osgEarth::MapNode> _mapNode;
    osgEarth::GeoPoint _currentGeoPos;
    osg::Vec3d _currentWorldPos;
    osg::Vec3d _currentWorldNormal;
    bool _disableMouseAction = false;
};

#endif // GLXEARTHMANIPULATOR_H
