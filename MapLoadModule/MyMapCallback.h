#ifndef MYMAPCALLBACK_H
#define MYMAPCALLBACK_H

#include <QObject>
#include <osgEarth/MapCallback>
#include <osgEarth/EarthManipulator>


class MyMapCallback:  public QObject ,public osgEarth::MapCallback
{
    Q_OBJECT
public:
    MyMapCallback(osgEarth::EarthManipulator* manip,QObject* parent = nullptr);

    void onLayerAdded(osgEarth::Layer* layer, unsigned index) override;

    void enableZoomToLayer(bool state = false){_zoomToLayer = state;}

    void zoomToLayer(osgEarth::Layer *layer);
signals:
    void onMapLoaded(osgEarth::Layer* layer);

private:
    bool _zoomToLayer = false;

    osgEarth::EarthManipulator* _manip = nullptr;


};

#endif // MYMAPCALLBACK_H
