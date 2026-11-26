#include "LayerManagerWidget.h"

#include "DataManager/legends.h"
#include <osgviewerwidget.h>
#include "App.h"

#include <QAbstractItemView>
#include <QApplication>
#include <QDropEvent>
#include <QHBoxLayout>
#include <QLabel>
#include <QMetaObject>
#include <QMouseEvent>
#include <QPointer>
#include <QVariant>

#include <functional>
#include <utility>

namespace
{
class LayerOrderListWidget final : public QListWidget
{
public:
    using QListWidget::QListWidget;

    void setOrderChangedCallback(std::function<void()> callback)
    {
        _orderChanged = std::move(callback);
    }

    void startItemDrag(QListWidgetItem* item)
    {
        setCurrentItem(item);
        startDrag(Qt::MoveAction);
    }

protected:
    void dropEvent(QDropEvent* event) override
    {
        QListWidget::dropEvent(event);
        if (event->isAccepted() && _orderChanged)
            _orderChanged();
    }

private:
    std::function<void()> _orderChanged;
};

class LayerDragHandle final : public QLabel
{
public:
    LayerDragHandle(LayerOrderListWidget* list, QListWidgetItem* item)
        : QLabel(QStringLiteral("::"), list),
          _list(list),
          _item(item)
    {
        setCursor(Qt::SizeVerCursor);
        setToolTip(QObject::tr("Drag to reorder layers"));
        setAlignment(Qt::AlignCenter);
    }

protected:
    void mousePressEvent(QMouseEvent* event) override
    {
        if (event->button() == Qt::LeftButton)
        {
            _dragStart = event->pos();
            _dragStarted = false;
            event->accept();
            return;
        }
        QLabel::mousePressEvent(event);
    }

    void mouseMoveEvent(QMouseEvent* event) override
    {
        if (!_dragStarted &&
            (event->buttons() & Qt::LeftButton) &&
            (event->pos() - _dragStart).manhattanLength() >= QApplication::startDragDistance())
        {
            _dragStarted = true;
            _list->startItemDrag(_item);
            event->accept();
            return;
        }
        QLabel::mouseMoveEvent(event);
    }

private:
    LayerOrderListWidget* _list;
    QListWidgetItem* _item;
    QPoint _dragStart;
    bool _dragStarted = false;
};

class LayerManagerMapCallback final : public osgEarth::MapCallback
{
public:
    explicit LayerManagerMapCallback(LayerManagerWidget* target)
        : _target(target)
    {
    }

    void onLayerAdded(osgEarth::Layer* layer, unsigned index) override
    {
        if (!_target || !layer)
            return;

        QPointer<LayerManagerWidget> target = _target;
        osg::ref_ptr<osgEarth::Layer> layerRef = layer;
        QMetaObject::invokeMethod(target.data(), [target, layerRef, index] {
            if (target)
                target->onLayerAdded(layerRef.get(), index);
        }, Qt::QueuedConnection);
    }

    void onLayerRemoved(osgEarth::Layer* layer, unsigned index) override
    {
        if (!_target || !layer)
            return;

        QPointer<LayerManagerWidget> target = _target;
        osg::ref_ptr<osgEarth::Layer> layerRef = layer;
        QMetaObject::invokeMethod(target.data(), [target, layerRef, index] {
            if (target)
                target->onLayerRemoved(layerRef.get(), index);
        }, Qt::QueuedConnection);
    }

    void onLayerMoved(osgEarth::Layer* layer, unsigned oldIndex, unsigned newIndex) override
    {
        if (!_target || !layer)
            return;

        QPointer<LayerManagerWidget> target = _target;
        osg::ref_ptr<osgEarth::Layer> layerRef = layer;
        QMetaObject::invokeMethod(target.data(), [target, layerRef, oldIndex, newIndex] {
            if (target)
                target->onLayerMoved(layerRef.get(), oldIndex, newIndex);
        }, Qt::QueuedConnection);
    }

private:
    QPointer<LayerManagerWidget> _target;
};
}

LayerManagerWidget* LayerManagerWidget::_layerManager = nullptr;

osgEarth::MapCallback* LayerManagerWidget::createMapCallback(LayerManagerWidget* target)
{
    return new LayerManagerMapCallback(target);
}

LayerManagerWidget::LayerManagerWidget(QWidget* parent)
    : DockWidgetPanels(parent)
{
    setWindowTitle("Layers");
    setObjectName("LayerManagerWidget");
    auto* list = new LayerOrderListWidget(this);
    layerListWidget = list;
    list->setDragEnabled(true);
    list->setAcceptDrops(true);
    list->setDropIndicatorShown(true);
    list->setDragDropMode(QAbstractItemView::InternalMove);
    list->setDefaultDropAction(Qt::MoveAction);
    list->setSelectionMode(QAbstractItemView::SingleSelection);
    list->setSortingEnabled(false);
    list->setOrderChangedCallback([this] { onLayerOrderChanged(); });
    setWidget(list);
}

LayerManagerWidget* LayerManagerWidget::getInstance(QWidget* parent){
    if(!_layerManager){
        _layerManager = new LayerManagerWidget(parent);
    }
    return _layerManager;
}

void LayerManagerWidget::addLayer(osgEarth::Layer *layer)
{
    if (layer)
        syncLayers();
}

void LayerManagerWidget::onLayerAdded(osgEarth::Layer* layer, unsigned index)
{
    if (!layer)
        return;
    OE_NOTICE << "Layer Added: " << layer->getName() << " at index " << index << std::endl;
    syncLayers();
}

void LayerManagerWidget::onLayerRemoved(osgEarth::Layer* layer, unsigned index)
{
    if (!layer)
        return;
    OE_NOTICE << "Layer Removed: " << layer->getName() << " at index " << index << std::endl;
    syncLayers();
}

void LayerManagerWidget::onLayerMoved(osgEarth::Layer* layer, unsigned oldIndex, unsigned newIndex)
{
    if (!layer)
        return;
    OE_NOTICE << "Layer Moved: " << layer->getName()
              << " from index " << oldIndex << " to " << newIndex << std::endl;
    syncLayers();
}

void LayerManagerWidget::onLayerOrderChanged()
{
    OSGViewerWidget* viewer = App::getThreedWindow();
    osgEarth::MapNode* mapNode = viewer ? viewer->getMapNode() : nullptr;
    osgEarth::Map* map = mapNode ? mapNode->getMap() : nullptr;
    if (!map || layerListWidget->count() != static_cast<int>(map->getNumLayers()))
        return;

    for (int row = 0; row < layerListWidget->count(); ++row)
    {
        const auto uid = static_cast<osgEarth::UID>(
            layerListWidget->item(row)->data(Qt::UserRole).toUInt());
        auto* layer = map->getLayerByUID(uid);
        if (layer && map->getIndexOfLayer(layer) != static_cast<unsigned>(row))
            map->moveLayer(layer, static_cast<unsigned>(row));
    }
}

void LayerManagerWidget::syncLayers()
{
    OSGViewerWidget* viewer = App::getThreedWindow();
    osgEarth::MapNode* mapNode = viewer ? viewer->getMapNode() : nullptr;
    osgEarth::Map* map = mapNode ? mapNode->getMap() : nullptr;
    if (!map)
        return;

    bool alreadyInMapOrder = layerListWidget->count() == static_cast<int>(map->getNumLayers());
    for (int row = 0; alreadyInMapOrder && row < layerListWidget->count(); ++row)
    {
        auto* layer = map->getLayerAt(static_cast<unsigned>(row));
        const auto itemLayer = static_cast<osgEarth::UID>(
            layerListWidget->item(row)->data(Qt::UserRole).toUInt());
        alreadyInMapOrder = layer && itemLayer == layer->getUID();
    }
    if (alreadyInMapOrder)
        return;

    layerListWidget->clear();
    for (unsigned index = 0; index < map->getNumLayers(); ++index)
    {
        osgEarth::Layer* layer = map->getLayerAt(index);
        if (!layer)
            continue;

        auto* item = new QListWidgetItem();
        item->setData(Qt::UserRole, static_cast<uint>(layer->getUID()));
        item->setFlags(
            Qt::ItemIsEnabled | Qt::ItemIsSelectable |
            Qt::ItemIsDragEnabled | Qt::ItemIsDropEnabled);
        layerListWidget->addItem(item);

        auto* row = new QWidget(layerListWidget);
        auto* rowLayout = new QHBoxLayout(row);
        rowLayout->setContentsMargins(2, 0, 2, 0);
        rowLayout->setSpacing(6);
        rowLayout->addWidget(new LayerDragHandle(
            static_cast<LayerOrderListWidget*>(layerListWidget), item));
        rowLayout->addWidget(new Legends(
            layer, mapNode, viewer->getManipulator(), row), 1);
        layerListWidget->setItemWidget(item, row);
        item->setSizeHint(row->sizeHint());
    }
}