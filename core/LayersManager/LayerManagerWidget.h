#ifndef LAYERMANAGERWIDGET_H
#define LAYERMANAGERWIDGET_H

#include <QDockWidget>
#include "../core_global.h"

class GEOLAYERCORE_EXPORT LayerManagerWidget : public QDockWidget
{
    Q_OBJECT
public:
    static LayerManagerWidget* getInstance();

    void addLayer(osgEarth::Layer* layer);

private:
    LayerManagerWidget();
    static LayerManagerWidget* _layerManager;

    QTreeWidget* treeWidget = nullptr;
};

#endif // LAYERMANAGERWIDGET_H
