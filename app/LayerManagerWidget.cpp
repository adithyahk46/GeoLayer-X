#include "LayerManagerWidget.h"

#include "app/legends.h"
#include "app/app.h"


LayerManagerWidget* LayerManagerWidget::_layerManager = nullptr;

LayerManagerWidget::LayerManagerWidget()
{
    this->setWindowTitle("Layers");
    treeWidget= new QTreeWidget(this);
    this->setWidget(treeWidget);
    this->setAllowedAreas(Qt::AllDockWidgetAreas);
    this->setDockLocation(Qt::AllDockWidgetAreas);
}

LayerManagerWidget* LayerManagerWidget::getInstance(){
    if(!_layerManager){
        _layerManager = new LayerManagerWidget;
    }
    return _layerManager;
}

void LayerManagerWidget::addLayer(osgEarth::Layer *layer)
{
    //legend view setup
    {
    if (!layer) return;

    // Create the legend for this layer
    Legends* legend = new Legends(layer,App::getInstance()->getMapNode(),App::getInstance()->getManipulator());

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
    else if (dynamic_cast<osgEarth::XYZImageLayer*>(layer))
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


