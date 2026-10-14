#ifndef POLYGONDRAWTOOL_H
#define POLYGONDRAWTOOL_H

#include <QObject>

#include "DrawTool.h"

#include "MapController/GLXEarthManipulator.h"
#include <osgEarth/FeatureNode>
#include "osgearthcore_global.h"

class OSGEARTHCORE_EXPORT PolygonDrawTool :public DrawTool
{
    Q_OBJECT
public:
    explicit PolygonDrawTool(osgEarth::MapNode* mapNode, GLXEarthManipulator* manipulator, QObject* parent = nullptr);

    ~PolygonDrawTool();

private slots:
    void defaultHandleEvent(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);

signals:
    void polygonDrawn(osg::ref_ptr<osgEarth::Feature> feature);

private:
    osg::ref_ptr<osgEarth::MapNode> _mapNode;
    osg::ref_ptr<GLXEarthManipulator> _manipulator;

    osg::ref_ptr<osg::Group> _polygonGroup;
    osg::ref_ptr<osgEarth::Geometry> _currentGeometry;
    osg::ref_ptr<osgEarth::Feature> _currentFeature;
    osg::ref_ptr<osgEarth::FeatureNode> _currentFeatureNode;

    void resetCurrentDrawing();

};

#endif // POLYGONDRAWTOOL_H