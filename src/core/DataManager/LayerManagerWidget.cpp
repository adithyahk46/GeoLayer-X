#include "LayerManagerWidget.h"

#include <osgEarth/TMS>

#include "legends.h"
#include <osgviewerwidget.h>


LayerManagerWidget* LayerManagerWidget::_layerManager = nullptr;

LayerManagerWidget::LayerManagerWidget(QWidget* parent)
    : QDockWidget(parent)
{
    this->setWindowTitle("Layers");
    setObjectName("LayerManagerWidget");
    treeWidget= new QTreeWidget(this);
    this->setWidget(treeWidget);
}

LayerManagerWidget* LayerManagerWidget::getInstance(QWidget* parent){
    if(!_layerManager){
        _layerManager = new LayerManagerWidget(parent);
    }
    return _layerManager;
}

void LayerManagerWidget::addLayer(osgEarth::Layer *layer)
{
    //legend view setup
    {
    if (!layer) return;

    // Create the legend for this layer
    Legends* legend = new Legends(layer, OSGViewerWidget::instance()->getMapNode(), OSGViewerWidget::instance()->getManipulator());

    // Determine layer type and add to corresponding category in QTreeWidget
    QString category;

    if (dynamic_cast<osgEarth::ImageLayer*>(layer))
    {
        category = "Raster Layers";
    }
    else if (dynamic_cast<osgEarth::ElevationLayer*>(layer))
    {
        category = "Elevation Layers";
    }
    else if (dynamic_cast<osgEarth::FeatureModelLayer*>(layer))
    {
        category = "Vector Layers";
    }
    else if (dynamic_cast<osgEarth::XYZImageLayer*>(layer) || dynamic_cast<osgEarth::TMSImageLayer*>(layer))
    {
        category = "Tile Layers";
    }
    else if (dynamic_cast<osgEarth::AnnotationLayer*>(layer))
    {
        category = "Annotation Layers";
    }
    else
    {
        category = "Other Layers";
    }

    // Find or create category item in QTreeWidget
    QTreeWidgetItem* categoryItem = nullptr;
    QList<QTreeWidgetItem*> foundItems = treeWidget->findItems(category, Qt::MatchExactly);
    if (!foundItems.isEmpty())
    {
        categoryItem = foundItems.first();
    }
    else
    {
        categoryItem = new QTreeWidgetItem(treeWidget);
        categoryItem->setText(0, category);
        categoryItem->setFlags(Qt::ItemIsEnabled);
    }

    // Add new layer as a child item under the category
    QTreeWidgetItem* layerItem = new QTreeWidgetItem(categoryItem);
    //layerItem->setText(0, QString::fromStdString(layer->getName()));

    // Store pointer to legend widget for later retrieval (optional)
    treeWidget->setItemWidget(layerItem, 0, legend);

    QObject::connect(legend, &QObject::destroyed, this, [=]() {
        QTreeWidgetItem* parent = layerItem->parent();
        delete layerItem;
        if (parent && parent->childCount() == 0) {
            delete parent; // remove empty category
        }
    });

    // Expand category for visibility
     treeWidget->expandItem(categoryItem);

    }
}


void LayerManagerWidget::onLayerAdded(osgEarth::Layer* layer, unsigned index){
    OE_NOTICE << "Layer Added: "<< layer->getName() << std::endl;

    // if(_zoomToLayer) zoomToLayer(layer);
    addLayer(layer);
}

void LayerManagerWidget::onLayerRemoved(osgEarth::Layer* layer, unsigned index){
    OE_NOTICE << "Layer Removed: "<< layer->getName() << std::endl;

    // Find the corresponding QTreeWidgetItem and remove it
    QList<QTreeWidgetItem*> items = treeWidget->findItems(layer->getName().c_str(), Qt::MatchExactly | Qt::MatchRecursive);
    for (QTreeWidgetItem* item : items)
    {
        if (item->parent()) // Ensure it's a child item
        {
            QTreeWidgetItem* parent = item->parent();
            delete item; // Remove the layer item
            if (parent && parent->childCount() == 0) {
                delete parent; // remove empty category
            }
            break;
        }
    }
}