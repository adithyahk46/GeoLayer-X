#ifndef LAYERMANAGERWIDGET_H
#define LAYERMANAGERWIDGET_H

#include <QTreeWidget>

#include "DockWidgetPanels.h"

#include <osgEarth/MapNode>
#include <osgEarth/MapCallback>


class  LayerManagerWidget : public DockWidgetPanels, public osgEarth::MapCallback
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
