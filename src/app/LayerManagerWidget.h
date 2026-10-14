#ifndef LAYERMANAGERWIDGET_H
#define LAYERMANAGERWIDGET_H

#include <QTreeWidget>

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


    void addLayer(osgEarth::Layer* layer);

protected:
    void setWindowTitle(const QString &title) override { QDockWidget::setWindowTitle(title); }
    void setObjectName(const QString &name) override { QDockWidget::setObjectName(name); }

private:
    LayerManagerWidget(QWidget* parent = nullptr);
    static LayerManagerWidget* _layerManager;

    QTreeWidget* treeWidget = nullptr;
};

#endif // LAYERMANAGERWIDGET_H
