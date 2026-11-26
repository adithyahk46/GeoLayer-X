#ifndef LAYERMANAGERWIDGET_H
#define LAYERMANAGERWIDGET_H

#include <QListWidget>

#include "DockWidgetPanels.h"

#include <osgEarth/MapNode>
#include <osgEarth/MapCallback>


class LayerManagerWidget : public DockWidgetPanels
{
    Q_OBJECT
public:
    static LayerManagerWidget* getInstance(QWidget* parent = nullptr);
    static osgEarth::MapCallback* createMapCallback(LayerManagerWidget* target);

    void onLayerAdded(osgEarth::Layer* layer, unsigned index);

    void onLayerRemoved(osgEarth::Layer* layer, unsigned index);

    void onLayerMoved(osgEarth::Layer* layer, unsigned oldIndex, unsigned newIndex);

    void addLayer(osgEarth::Layer* layer);

protected:
    void setWindowTitle(const QString &title) override { QDockWidget::setWindowTitle(title); }
    void setObjectName(const QString &name) override { QDockWidget::setObjectName(name); }

private:
    LayerManagerWidget(QWidget* parent = nullptr);
    void onLayerOrderChanged();
    void syncLayers();
    static LayerManagerWidget* _layerManager;

    QListWidget* layerListWidget = nullptr;
};

#endif // LAYERMANAGERWIDGET_H
