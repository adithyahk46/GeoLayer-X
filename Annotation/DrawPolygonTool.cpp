#include "DrawPolygonTool.h"

#include <osgEarth/FeatureNode>
#include <osgEarth/Feature>
#include <osgEarth/GeoTransform>
#include <osgEarth/SpatialReference>
#include <osg/Vec3d>
#include <QDebug>

using namespace osgEarth;


DrawPolygonTool::DrawPolygonTool(MapNode* mapNode,MouseEventHandler* mouseControlle)
    : _mapNode(mapNode),
      _mouseControlle(mouseControlle),
      _isDrawing(false)
{

    _mouseControlle->disableMouseAction(true);

    _isDrawing = true;
     _geom = new osgEarth::Polygon();

     feature = new osgEarth::Feature(_geom, _mapNode->getMapSRS());

    _polygonStyle.getOrCreate<PolygonSymbol>()->fill().mutable_value().color() = Color(Color::Red);
    _polygonStyle.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;
    _polygonStyle.getOrCreate<AltitudeSymbol>()->technique() = AltitudeSymbol::TECHNIQUE_DRAPE;
    _polygonStyle.getOrCreate<AltitudeSymbol>()->binding()= AltitudeSymbol::BINDING_VERTEX;

    // _polygonStyle.getOrCreate<LineSymbol>()->stroke()->color() = Color(Color::Red, 1.0);
    // _polygonStyle.getOrCreate<LineSymbol>()->stroke()->width() = Distance(2.0f, Units::PIXELS);
    // _polygonStyle.getOrCreate<LineSymbol>()->tessellationSize() = Distance(75000, Units::METERS);
    _polygonStyle.getOrCreate<RenderSymbol>()->depthOffset()->range() = Distance(1.0, Units::PIXELS);

     _previewNode = new FeatureNode(feature,_polygonStyle);

     _polygonGroup = new osg::Group();
     _mapNode->addChild(_polygonGroup);
    _polygonGroup->addChild( _previewNode );

    QObject::connect(_mouseControlle, &MouseEventHandler::mouseClickEvent,this, &DrawPolygonTool::mouseClickEvent);
    QObject::connect(_mouseControlle, &MouseEventHandler::mouseMoveEvent,this, &DrawPolygonTool::mouseMoveEvent);
    QObject::connect(_mouseControlle, &MouseEventHandler::mouseDoubleClickEvent,this, &DrawPolygonTool::mouseDoubleClickEvent);

}

DrawPolygonTool::~DrawPolygonTool()
{

}

void DrawPolygonTool::mouseClickEvent(const double lon, const double lat, const double alt, const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa)
{
    if(_isDrawing){
        // _geom->push_back(lon, lat);
        // _previewNode->dirty();

        osgEarth::GeoPoint mapPoint(_mapNode->getMapSRS(), lon, lat);
        _geom->push_back(mapPoint.vec3d());
        _previewNode->dirty();

    }

}

void DrawPolygonTool::mouseDoubleClickEvent(const double lon, const double lat, const double alt, const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa)
{

        finishPolygon();

}

void DrawPolygonTool::mouseMoveEvent(const double lon, const double lat, const double alt, const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa)
{
     if(_isDrawing){
         // _geom->push_back(lon, lat);
         // _previewNode->dirty();

         osgEarth::GeoPoint mapPoint(_mapNode->getMapSRS(), lon, lat);
         _geom->push_back(mapPoint.vec3d());
         _previewNode->dirty();

         _geom->pop_back();

     }
}

void DrawPolygonTool::finishPolygon()
{

    _geom->push_back(_geom->front());

    _previewNode->dirty();

     _mapNode->removeChild(_polygonGroup);

    _polygonGroup->removeChild(_previewNode);


    _isDrawing = 0;

    QObject::disconnect(_mouseControlle, &MouseEventHandler::mouseClickEvent,this, &DrawPolygonTool::mouseClickEvent);
    QObject::disconnect(_mouseControlle, &MouseEventHandler::mouseMoveEvent,this, &DrawPolygonTool::mouseMoveEvent);
    QObject::disconnect(_mouseControlle, &MouseEventHandler::mouseDoubleClickEvent,this, &DrawPolygonTool::mouseDoubleClickEvent);

    emit DrawPolygonFinished(_previewNode);

    _mouseControlle->disableMouseAction(false); // re-enable normal mouse



    this->deleteLater();

}
