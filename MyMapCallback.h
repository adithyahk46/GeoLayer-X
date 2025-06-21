#ifndef MYMAPCALLBACK_H
#define MYMAPCALLBACK_H

#include <QObject>
#include <osgEarth/MapCallback>

class MyMapCallback:  public QObject ,public osgEarth::MapCallback
{
    Q_OBJECT
public:
    MyMapCallback(QObject* parent = nullptr);

    void onLayerAdded(osgEarth::Layer* layer, unsigned index) override;

signals:
    void onMapLoaded(osgEarth::Layer* layer);



};

#endif // MYMAPCALLBACK_H
