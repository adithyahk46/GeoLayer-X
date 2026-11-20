#ifndef SYMBOLOGYEDITORDIALOG_H
#define SYMBOLOGYEDITORDIALOG_H

#include <QDialog>
#include <osg/ref_ptr>
#include <osgEarth/Geometry>

#include "colorpickercheckbox.h"

namespace osgEarth
{
class FeatureImageLayer;
class FeatureModelLayer;
class FeatureSource;
class Layer;
class MapNode;
class Style;
}

namespace Ui
{
class SymbologyEditorDialog;
}

class SymbologyEditorDialog : public QDialog
{
    Q_OBJECT

public:
    explicit SymbologyEditorDialog(osgEarth::FeatureModelLayer* featureModelLayer, QWidget* parent = nullptr);
    explicit SymbologyEditorDialog(
        osgEarth::FeatureImageLayer* featureImageLayer,
        QWidget* parent = nullptr,
        osgEarth::MapNode* mapNode = nullptr);
    ~SymbologyEditorDialog() override;

private:
    void setUpUi();
    void loadCurrentStyle();
    void populateStyle(const osgEarth::Style& style);
    void verifyGeometryType();
    void applyStyle();
    osgEarth::Geometry::Type selectedGeometryType() const;

    void createPointSymbol(osgEarth::Style& style) const;
    void createLineSymbol(osgEarth::Style& style) const;
    void createPolygonSymbol(osgEarth::Style& style) const;
    void createAltitudeSymbol(osgEarth::Style& style) const;

    Ui::SymbologyEditorDialog* ui = nullptr;
    osg::ref_ptr<osgEarth::Layer> _featureLayer;
    osgEarth::FeatureSource* _featureSource = nullptr;
    osgEarth::MapNode* _mapNode = nullptr;
    ColorPickerCheckBox* _pointFillColor = nullptr;
    ColorPickerCheckBox* _lineColor = nullptr;
    ColorPickerCheckBox* _polygonFillColor = nullptr;
};

#endif
