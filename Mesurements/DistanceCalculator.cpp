#include "DistanceCalculator.h"
#include "ui_DistanceCalculator.h"

#include "MouseEventHandler.h"
#include <QDebug>

#include <osgEarth/GeoMath>
#include <osgEarth/LabelNode>
#include <osgEarth/AnnotationUtils>


MouseEventHandler* _mouseControlle= nullptr;

DistanceCalculator::DistanceCalculator(osgEarth::MapNode* mapNode) :
    _mapNode(mapNode)
  ,_isDrawing(false)
{

    _mouseControlle = MouseEventHandler::Instance();
    _mouseControlle->disableMouseAction(true);
    QObject::connect(_mouseControlle, &MouseEventHandler::mouseClickEvent,this, &DistanceCalculator::mouseClickEvent);
    QObject::connect(_mouseControlle, &MouseEventHandler::mouseMoveEvent,this, &DistanceCalculator::mouseMoveEvent);
    QObject::connect(_mouseControlle, &MouseEventHandler::mouseDoubleClickEvent,this, &DistanceCalculator::mouseDoubleClickEvent);

    _isDrawing =true;
    _geom = new osgEarth::LineString();
    feature = new osgEarth::Feature(_geom, _mapNode->getMapSRS());

    osgEarth::AltitudeSymbol* alt = _lineStyle.getOrCreate<osgEarth::AltitudeSymbol>();
    alt->clamping() = osgEarth::AltitudeSymbol::CLAMP_ABSOLUTE;
    alt->technique() = osgEarth::AltitudeSymbol::TECHNIQUE_DRAPE;

    // Configure render options to avoid Z-fighting
    osgEarth::RenderSymbol* render = _lineStyle.getOrCreate<osgEarth::RenderSymbol>();
    render->lighting() = false;
    render->depthOffset()->enabled() = true;
    render->depthOffset()->automatic() = true;

    // Configure line appearance
    osgEarth::LineSymbol* ls = _lineStyle.getOrCreate<osgEarth::LineSymbol>();
    ls->stroke()->color() = osgEarth::Color::Red;
    ls->stroke().mutable_value().width() = osgEarth::Distance(2.0, osgEarth::Units::PIXELS);
    ls->stroke()->widthUnits() = osgEarth::Units::PIXELS;
    ls->tessellation() = 150;

    _previewNode = new osgEarth::FeatureNode(feature, _lineStyle);

    _mapNode->addChild(_previewNode);


}

DistanceCalculator::~DistanceCalculator()
{
    delete ui;
}

void DistanceCalculator::mouseClickEvent(const double lon, const double lat, const double alt, const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa)
{

    if(_isDrawing){
        osgEarth::GeoPoint mapPoint(_mapNode->getMapSRS(), lon, lat, alt);
        _geom->push_back(mapPoint.vec3d());
        _previewNode->dirty();

    }
}

void DistanceCalculator::mouseDoubleClickEvent(const double lon, const double lat, const double alt, const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa)
{

    _geom->push_back(_geom->front());

    _previewNode->dirty();

     _mapNode->removeChild(_previewNode);
     _isDrawing= false;

     emit DrawPolygonFinished(_previewNode);


    QObject::disconnect(_mouseControlle, &MouseEventHandler::mouseClickEvent,this, &DistanceCalculator::mouseClickEvent);
    QObject::disconnect(_mouseControlle, &MouseEventHandler::mouseMoveEvent,this, &DistanceCalculator::mouseMoveEvent);
    QObject::disconnect(_mouseControlle, &MouseEventHandler::mouseDoubleClickEvent,this, &DistanceCalculator::mouseDoubleClickEvent);
    _mouseControlle->disableMouseAction(false);

}

void DistanceCalculator::mouseMoveEvent(const double lon, const double lat, const double alt, const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa)
{

    if(_isDrawing){

        osgEarth::GeoPoint mapPoint(_mapNode->getMapSRS(), lon, lat, alt);

        //here write a code to calculate distance b/w the last _geom point to current one and display using label
        // <label text="Tessellated line" lat="35" long="10">
        //     <style type="text/css">
        //         text-align:              center_bottom;
        //         text-geographic-course:  0;
        //     </style>
        // </label>

        _geom->push_back(mapPoint.vec3d());
        _previewNode->dirty();

        _geom->pop_back();


    }
}

