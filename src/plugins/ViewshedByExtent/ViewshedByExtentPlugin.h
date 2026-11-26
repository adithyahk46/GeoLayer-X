#ifndef VIEWSHED_BY_EXTENT_PLUGIN_H
#define VIEWSHED_BY_EXTENT_PLUGIN_H

#include <memory>

#include <QColor>
#include <QPointer>
#include <PluginManager/PluginInterface.h>
#include <osg/Array>
#include <osg/PrimitiveSet>
#include <osg/Vec4>
#include <osgEarth/GeoData>

#include "ViewshedByExtentCore.h"
#include "MapController/GLXEarthManipulator.h"
#include "ui_ViewshedByExtentDialog.h"

class QDialog;
class ColorPickerCheckBox;
namespace osg {
class Geometry;
class Geode;
}

class ViewshedByExtentPlugin : public PluginInterface
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "com.geolayerx.plugins.PluginInterface/1.0" FILE "ViewshedByExtentPlugin.json")
    Q_INTERFACES(PluginInterface)

public:
    explicit ViewshedByExtentPlugin(QObject* parent = nullptr);
    ~ViewshedByExtentPlugin() override;

    QString pluginName() const override;
    void initialize() override;

    bool executeCommand(
        const QString& commandId,
        const QVariantMap& arguments,
        QVariantMap* result = nullptr,
        QString* errorMessage = nullptr) override;

private:
    void stopExtentSelection(bool resetButton = true);
    void stopLocationSelection(bool resetButton = true);
    void restoreMapNavigationIfIdle();
    void ensureExtentDrawing(osgEarth::MapNode* mapNode);
    void updateExtentRubberBand(const osgEarth::GeoExtent& extent);
    void removeExtentDrawing();
    bool readExtent(osgEarth::GeoExtent& extent, QString* errorMessage) const;
    bool readViewerLocation(const osgEarth::GeoExtent& extent,
                            osgEarth::GeoPoint& location,
                            QString* errorMessage) const;
    void createOrUpdateAnalysis();
    void updateViewerLocationFromFields();
    void updateVisibleAreaColor(const QColor& color);
    void updateInvisibleAreaColor(const QColor& color);
    void updateRubberBandColor();

    QDialog* _dialog = nullptr;
    Ui::ViewshedByExtentDialog ui;
    ColorPickerCheckBox* _visibleColorPicker = nullptr;
    ColorPickerCheckBox* _invisibleColorPicker = nullptr;
    QPointer<GLXEarthManipulator> _selectionManipulator;
    QMetaObject::Connection _extentSelectionConnection;
    QMetaObject::Connection _extentMoveConnection;
    QMetaObject::Connection _locationSelectionConnection;
    osg::ref_ptr<osgEarth::MapNode> _selectionMapNode;
    osg::ref_ptr<osg::Geode> _rubberBandGeode;
    osg::ref_ptr<osg::Geometry> _rubberBandGeometry;
    osg::ref_ptr<osg::Vec3Array> _rubberBandVertices;
    osg::ref_ptr<osg::Vec4Array> _rubberBandColors;
    osg::ref_ptr<osg::DrawArrays> _rubberBandDrawArrays;
    std::unique_ptr<ViewshedByExtentCore> _viewshedCore;
    osg::Vec4 _visibleAreaColor{159.0f / 255.0f, 1.0f, 61.0f / 255.0f, 0.8f};
    osg::Vec4 _invisibleAreaColor{1.0f, 87.0f / 255.0f, 61.0f / 255.0f, 0.8f};
    osgEarth::GeoPoint _firstExtentCorner;
    bool _hasFirstExtentCorner = false;
    double _rubberBandAltitude = 30.0;
};

#endif // VIEWSHED_BY_EXTENT_PLUGIN_H