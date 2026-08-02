#include "MyMapCallback.h"
#include <QDebug>

#include <DataManager/LayerManagerWidget.h>
#include <osgEarth/TMS>
#include <osgearth/XYZ>

using namespace osgEarth;

MyMapCallback::MyMapCallback(osgEarth::EarthManipulator* manip)
    : _manip(manip)
{
}
void MyMapCallback::onLayerAdded(osgEarth::Layer* layer, unsigned index){
    OE_NOTICE << "Layer Added: "<< layer->getName() << std::endl;

    emit onMapLoaded(layer);
    if(_zoomToLayer) zoomToLayer(layer);
    LayerManagerWidget::getInstance()->addLayer(layer);

}

void MyMapCallback::zoomToLayer(osgEarth::Layer* layer)
{
    //do not zoom if the layer is xyz tiles or tms tiles
    if (dynamic_cast<osgEarth::TMSImageLayer*>(layer) || dynamic_cast<osgEarth::XYZImageLayer*>(layer)) {
        qDebug() << "Zooming to XYZ or TMS layers is not supported.";
        return;
    }

    const osgEarth::SpatialReference* wgs84 = osgEarth::SpatialReference::get("wgs84");

    osgEarth::GeoExtent layerExtent = layer->getExtent().transform(wgs84);
    if (!layerExtent.isValid())
    {
        qDebug() << "Error:layer extent is invalid.";
        return;
    }

    GeoPoint p1(layerExtent.getSRS(), layerExtent.xMin(), layerExtent.yMin(),0.0);
    GeoPoint p2(layerExtent.getSRS(), layerExtent.xMax(), layerExtent.yMax(), 0.0);

    GeoPoint center(layerExtent.getSRS(),layerExtent.getCentroid().x(),layerExtent.getCentroid().y(),0.0);

    // Optional: set viewpoint with calculated range
    Viewpoint vp("",center.x(),center.y(),0.0f,0.0f,-90.0f, p1.distanceTo(p2)<1000?1000:p1.distanceTo(p2) );
    _manip->setViewpoint(vp
                         ); // with 2-second transition
    // qDebug()<<"The Range Covered by the layer = "<<p1.distanceTo(p2);

}
