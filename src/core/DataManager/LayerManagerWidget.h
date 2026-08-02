#ifndef LAYERMANAGERWIDGET_H
#define LAYERMANAGERWIDGET_H

#include <QDockWidget>
#include <QTreeWidget>



#include <osgEarth/MapNode>
#include <osgEarth/MapCallback>

#include "osgearthcore_global.h"

class OSGEARTHCORE_EXPORT LayerManagerWidget : public QDockWidget, public osgEarth::MapCallback
{
    Q_OBJECT
public:
    static LayerManagerWidget* getInstance(QWidget* parent = nullptr);

    void onLayerAdded(osgEarth::Layer* layer, unsigned index) override;

    void onLayerRemoved(osgEarth::Layer* layer, unsigned index) override;


    void addLayer(osgEarth::Layer* layer);

private:
    LayerManagerWidget(QWidget* parent = nullptr);
    static LayerManagerWidget* _layerManager;

    QTreeWidget* treeWidget = nullptr;
};

#endif // LAYERMANAGERWIDGET_H
