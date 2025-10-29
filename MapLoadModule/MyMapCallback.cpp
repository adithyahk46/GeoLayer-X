#include "MyMapCallback.h"
#include <QDebug>


MyMapCallback::MyMapCallback(QObject* parent)
    : QObject(parent)
{
}
void MyMapCallback::onLayerAdded(osgEarth::Layer* layer, unsigned index){
    // std::cout << "Layer added: " << layer->getName() << " at index " << index << std::endl;
    emit onMapLoaded(layer);
}
