#ifndef DRAWPOLYGONTOOL_H
#define DRAWPOLYGONTOOL_H

#include <osgEarth/MapNode>
#include <osgEarth/FeatureNode>
#include <osgEarth/Geometry>
#include <osgEarth/Style>
#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>

#include <osgEarth/FeatureModelLayer>
#include <osgEarth/FeatureSource>
#include <osgEarth/FeatureCursor>
#include <osgEarth/Feature>
#include <osgEarth/OGRFeatureSource>


#include <QObject>
#include "MouseEventHandler.h"


class DrawPolygonTool :public QObject
{
    Q_OBJECT

public:
    DrawPolygonTool(osgEarth::MapNode* mapNode,MouseEventHandler* mouseControlle);

protected:
    ~DrawPolygonTool();

signals:
    void DrawPolygonFinished(osgEarth::FeatureNode* featureNode);


private slots:
    void mouseClickEvent(const double lon, const double lat,const double alt,const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
    void mouseDoubleClickEvent(const double lon, const double lat,const double alt,const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
    void mouseMoveEvent(const double lon, const double lat,const double alt,const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
    // void mouseScrollEvent(const double lon, const double lat,const double alt,const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);

private:
    void finishPolygon();

    osgEarth::MapNode* _mapNode;
    MouseEventHandler* _mouseControlle = nullptr;

    osg::ref_ptr<osgEarth::Geometry> _geom;
    osgEarth::Style _polygonStyle;
    osg::ref_ptr<osgEarth::Feature> feature;
    osg::ref_ptr<osgEarth::FeatureNode> _previewNode;

    osg::ref_ptr<osg::Group> _polygonGroup;

    bool _isDrawing;
};

#endif
