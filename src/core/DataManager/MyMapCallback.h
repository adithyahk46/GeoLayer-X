#ifndef MAPCALLBACK_H
#define MAPCALLBACK_H

#include <QObject>
#include <osgEarth/MapCallback>
#include <osgEarth/EarthManipulator>
#include "osgearthcore_global.h"


class OSGEARTHCORE_EXPORT MyMapCallback:  public QObject ,public osgEarth::MapCallback
{
    Q_OBJECT
public:
    MyMapCallback(osgEarth::EarthManipulator* manip);

    void onLayerAdded(osgEarth::Layer* layer, unsigned index) override;

    void enableZoomToLayer(bool state = false){_zoomToLayer = state;}

    void zoomToLayer(osgEarth::Layer *layer);
signals:
    void onMapLoaded(osgEarth::Layer* layer);

private:
    bool _zoomToLayer = false;

    osgEarth::EarthManipulator* _manip = nullptr;

};

#endif // MAPCALLBACK_H
