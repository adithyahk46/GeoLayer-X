#ifndef DRAWCIRCLE_H
#define DRAWCIRCLE_H


#include "MouseEventHandler.h"
#include <QObject>

#include <osgEarth/FeatureNode>
#include <osgEarth/Geometry>
#include <osgEarth/Style>

#include <osgEarth/FeatureModelLayer>
#include <osgEarth/FeatureSource>
#include <osgEarth/FeatureCursor>
#include <osgEarth/Feature>
#include <osgEarth/OGRFeatureSource>

#include <osgEarth/GeoData>
#include <osgEarth/Geometry>
#include <osgEarth/MapNode>

using namespace osgEarth;

class DrawCircle : public QObject
{
    Q_OBJECT
public:
    explicit DrawCircle(osgEarth::MapNode* mapNode,MouseEventHandler* mouseControlle);

protected:
    ~DrawCircle();

signals:
    void DrawCircleFinished(osgEarth::FeatureNode* featureNode);

private slots:
    void mouseClickEvent(const double lon, const double lat,const double alt,const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
    void mouseDoubleClickEvent(const double lon, const double lat,const double alt,const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
    void mouseMoveEvent(const double lon, const double lat,const double alt,const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);

private:

     bool _isDrawing = false;
     bool _hasCenter =false;

     osgEarth::GeoPoint _center;

    osg::ref_ptr<osgEarth::MapNode> _mapNode;
    MouseEventHandler* _mouseControlle{nullptr};

    osg::ref_ptr<osgEarth::Polygon> _geom ;
    osg::ref_ptr<osgEarth::Geometry> circle ;
   osg::ref_ptr<osgEarth::Feature> feature ;
     osgEarth::Style circleStyle;
    osg::ref_ptr<osgEarth::FeatureNode> _previewCircleNode;
    osg::ref_ptr<osg::Group> _CirCleGroup;


};

#endif // DRAWCIRCLE_H
