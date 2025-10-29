#include "DrawCircle.h"

#include <osgEarth/CircleNode>

DrawCircle::DrawCircle(osgEarth::MapNode* mapNode,MouseEventHandler* mouseControlle)
    :_mapNode(mapNode)
    ,_mouseControlle(mouseControlle)
{

    _mouseControlle->disableMouseAction(true);

    _isDrawing = true;

     osgEarth::CircleNode* circle = new osgEarth::CircleNode();
     _geom = new osgEarth::Polygon();

     feature = new osgEarth::Feature(_geom, _mapNode->getMapSRS());

    circleStyle.getOrCreate<osgEarth::PolygonSymbol>()->fill().mutable_value().color() = osgEarth::Color(osgEarth::Color::Cyan, 0.5);
    circleStyle.getOrCreate<osgEarth::AltitudeSymbol>()->clamping() = osgEarth::AltitudeSymbol::CLAMP_TO_TERRAIN;

    circleStyle.getOrCreate<osgEarth::RenderSymbol>()->depthOffset()->range() = Distance(1.0, osgEarth::Units::KILOMETERS);

    _previewCircleNode = new FeatureNode(feature,circleStyle);

    _CirCleGroup = new osg::Group();
    _mapNode->addChild(_CirCleGroup);
    _CirCleGroup->addChild( _previewCircleNode );

    QObject::connect(_mouseControlle,&MouseEventHandler::mouseClickEvent,this, &DrawCircle::mouseClickEvent);
    QObject::connect(_mouseControlle,&MouseEventHandler::mouseMoveEvent,this, &DrawCircle::mouseMoveEvent);
    QObject::connect(_mouseControlle,&MouseEventHandler::mouseDoubleClickEvent,this, &DrawCircle::mouseDoubleClickEvent);


}
DrawCircle::~DrawCircle()
{

}

void DrawCircle::mouseClickEvent(const double lon, const double lat, const double alt,
                                 const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa)
{
    if (!_isDrawing)
        return;

    if (!_hasCenter)
    {
        // First click defines circle center
        _center = GeoPoint(_mapNode->getMapSRS(), lon, lat, 0.0, ALTMODE_ABSOLUTE);
        _hasCenter = true;
    }
}
#include <osgEarth/GeometryFactory>
#include <osgEarth/Units>

void DrawCircle::mouseMoveEvent(const double lon, const double lat, const double alt,
                                const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa)
{
    if (!_isDrawing || !_hasCenter)
        return;

    GeoPoint cursor(_mapNode->getMapSRS(), lon, lat, 0.0, ALTMODE_ABSOLUTE);

    // Calculate radius (in meters)
    double radiusMeters = _center.distanceTo(cursor);

    osgEarth::GeometryFactory gf(_mapNode->getMapSRS());

    // Build circle polygon
    circle =
        gf.createCircle(_center.vec3d(),
                        osgEarth::Linear(radiusMeters, osgEarth::Units::METERS),
                        64); // segments

     _geom->assign(circle->begin(), circle->end());
    feature->setGeometry(_geom.get());
    _previewCircleNode->dirty(); // refresh node
}


void DrawCircle::mouseDoubleClickEvent(const double lon, const double lat, const double alt,
                                       const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa)
{

    if (!_isDrawing || !_hasCenter)
        return;

    GeoPoint cursor(_mapNode->getMapSRS(), lon, lat, 0.0, ALTMODE_ABSOLUTE);

    // Calculate radius (in meters)
    double radiusMeters = _center.distanceTo(cursor);

    osgEarth::GeometryFactory gf(_mapNode->getMapSRS());

    // Build circle polygon
    circle =
        gf.createCircle(_center.vec3d(),
                        osgEarth::Linear(radiusMeters, osgEarth::Units::METERS),
                        64); // segments

    _geom->assign(circle->begin(), circle->end());
   feature->setGeometry(_geom.get());
   _previewCircleNode->dirty(); // refresh node

    emit DrawCircleFinished(_previewCircleNode);

    _mapNode->removeChild(_CirCleGroup);

    _CirCleGroup->removeChild(_previewCircleNode);

    _isDrawing = false;

    QObject::disconnect(_mouseControlle,&MouseEventHandler::mouseClickEvent,this, &DrawCircle::mouseClickEvent);
    QObject::disconnect(_mouseControlle,&MouseEventHandler::mouseMoveEvent,this, &DrawCircle::mouseMoveEvent);
    QObject::disconnect(_mouseControlle,&MouseEventHandler::mouseDoubleClickEvent,this, &DrawCircle::mouseDoubleClickEvent);

    _mouseControlle->disableMouseAction(false); // re-enable normal mouse

    this->deleteLater();

}
