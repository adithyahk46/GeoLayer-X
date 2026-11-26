#include "PolygonDrawTool.h"

#include <osgGA/GUIEventAdapter>

PolygonDrawTool::PolygonDrawTool(osgEarth::MapNode* mapNode, GLXEarthManipulator* manipulator, QObject* parent)
    : DrawTool(parent), _mapNode(mapNode), _manipulator(manipulator)
{

    _polygonGroup = new osg::Group();
    _polygonGroup->setName("PolygonDrawTool_Group");
    _mapNode->addChild(_polygonGroup);
    QObject::connect(_manipulator, &GLXEarthManipulator::defaultHandleEvent, this, &PolygonDrawTool::defaultHandleEvent);

    resetCurrentDrawing();
}

PolygonDrawTool::~PolygonDrawTool()
{
    if (_polygonGroup.valid() && _mapNode.valid())
        _mapNode->removeChild(_polygonGroup);

    QObject::disconnect(_manipulator, &GLXEarthManipulator::defaultHandleEvent, this, &PolygonDrawTool::defaultHandleEvent);
}

void PolygonDrawTool::defaultHandleEvent(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{

    if (ea.getEventType() == osgGA::GUIEventAdapter::PUSH && ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON)
    {
        _currentGeometry->push_back(_manipulator->currentWorldPosition());
        _currentFeatureNode->dirtyBound();

    }
}

void PolygonDrawTool::resetCurrentDrawing()
{
    _currentGeometry = nullptr;
    _currentFeature = nullptr;
    _currentFeatureNode = nullptr;

    _currentGeometry = new osgEarth::Polygon();
    _currentFeature = new osgEarth::Feature(_currentGeometry, _mapNode->getMap()->getSRS());
    _currentFeatureNode = new osgEarth::FeatureNode(_currentFeature, getStyle());
    _polygonGroup->addChild(_currentFeatureNode);
}