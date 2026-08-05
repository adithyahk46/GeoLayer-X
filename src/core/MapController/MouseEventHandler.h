#ifndef MOUSEEVENTHANDLER_H
#define MOUSEEVENTHANDLER_H

#include <iostream>
#include <QObject>

#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>
#include <osgEarth/MapNode>


#include "osgearthcore_global.h"

class OSGEARTHCORE_EXPORT MouseEventHandler :public QObject, public osgGA::GUIEventHandler
{
    Q_OBJECT

public:

    static MouseEventHandler* Instance(osgEarth::MapNode* mapNode = nullptr);

    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa) override;

    void disableMouseAction(bool state);



signals:
    void mouseClickEvent(osgEarth::GeoPoint mapPoint);
    void mouseDoubleClickEvent(osgEarth::GeoPoint mapPoint);
    void mouseMoveEvent(osgEarth::GeoPoint mapPoint);
    void mouseScrollEvent(osgEarth::GeoPoint mapPoint);


private:
    explicit MouseEventHandler(osgEarth::MapNode* mapNode = nullptr);


    double lon;
    double lat;
    double alt;

    osgViewer::View* view = nullptr;
    osg::ref_ptr<osgEarth::MapNode> _mapNode;

    bool _disableMouseAction = false;

    osgEarth::GeoPoint                                     _currentGeoPos;
    osg::Vec3d                                             _currentLocalPos;
    osg::Vec3d                                             _currentWorldPos;
    osgUtil::LineSegmentIntersector::Intersections         _intersections;
    osgUtil::LineSegmentIntersector::Intersection          _nearestIntesection;
    osg::ref_ptr<const osgEarth::SpatialReference>          srs_wgs84   = osgEarth::SpatialReference::get("wgs84");


    bool getLatLonFromScreen(osgViewer::View *view, float x, float y, double &lat, double &lon, double &height);
};

#endif // MOUSEEVENTHANDLER_H