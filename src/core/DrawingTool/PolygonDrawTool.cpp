#include "PolygonDrawTool.h"

#include <osgGA/GUIEventAdapter>
#include <QDebug>

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
    // clearCurrentDrawing();

    if (_polygonGroup.valid())
    {
        // _mapNode->removeChild(_polygonGroup);
    }

    QObject::disconnect(_manipulator, &GLXEarthManipulator::defaultHandleEvent, this, &PolygonDrawTool::defaultHandleEvent);
}

void PolygonDrawTool::defaultHandleEvent(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{

    if (ea.getEventType() == osgGA::GUIEventAdapter::PUSH && ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON)
    {
        // Handle left mouse button press event
        // Start drawing polygon
        _currentGeometry->push_back(_manipulator->currentWorldPosition());
        _currentFeatureNode->dirtyBound();

    }
    else if (ea.getEventType() == osgGA::GUIEventAdapter::MOVE)
    {
        // Handle mouse move event
        // Update polygon drawing
    }
    else if (ea.getEventType() == osgGA::GUIEventAdapter::RELEASE && ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON)
    {
        // Handle left mouse button release event
        // Finish drawing polygon and emit signal
        // emit polygonDrawn(_currentFeature);
    }
}

// void PolygonDrawTool::finishDrawingPolygon()
// {
//     if (_currentFeature.valid())
//     {
//         // Finalize the polygon feature
//         // Add it to the map or perform any necessary actions
//         emit polygonDrawn(_currentFeature);
//     }

//     resetCurrentDrawing();
// }

void PolygonDrawTool::resetCurrentDrawing()
{
    _currentGeometry = nullptr;
    _currentFeature = nullptr;
    _currentFeatureNode = nullptr;

    _currentGeometry = new osgEarth::Polygon();
    _currentFeature = new osgEarth::Feature(_currentGeometry, _mapNode->getMap()->getSRS());
    _currentFeatureNode = new osgEarth::FeatureNode(_currentFeature, getStyle());
    _polygonGroup->addChild(_currentFeatureNode);

     Geometry* geom = new osgEarth::Polygon();
        geom->push_back( -160., -30. );
        geom->push_back(  150., -20. );
        geom->push_back(  160., -45. );
        geom->push_back( -150., -40. );

        Feature* feature = new Feature(geom, _mapNode->getMap()->getSRS());
        feature->geoInterp() = GEOINTERP_RHUMB_LINE;

        FeatureNode* gnode = new FeatureNode(feature, getStyle());
        _polygonGroup->addChild( gnode );
}


// void PolygonDrawTool::clearCurrentDrawing()
// {
//     // if (_currentFeatureNode.valid())
//     // {
//     //     _polygonGroup->removeChild(_currentFeatureNode);
//     //     _currentFeatureNode = nullptr;
//     // }
//     // _currentGeometry = nullptr;
//     // _currentFeature = nullptr;
// }