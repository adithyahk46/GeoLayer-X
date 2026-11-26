#include "SymbologyEditorDialog.h"
#include "ui_SymbologyEditorDialog.h"
#include "App.h"

#include <QMessageBox>

#include <osgEarth/AltitudeSymbol>
#include <osgEarth/FeatureImageLayer>
#include <osgEarth/FeatureModelLayer>
#include <osgEarth/FeatureSource>
#include <osgEarth/LineSymbol>
#include <osgEarth/MapNode>
#include <osgEarth/PointSymbol>
#include <osgEarth/PolygonSymbol>
#include <osgEarth/Style>
#include <osgEarth/StyleSheet>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/Units>

#include <vector>




namespace
{
osgEarth::Color toEarthColor(const QColor& color)
{
    return osgEarth::Color(color.redF(), color.greenF(), color.blueF(), color.alphaF());
}

QColor toQColor(const osgEarth::Color& color)
{
    return QColor::fromRgbF(color.r(), color.g(), color.b(), color.a());
}

const osgEarth::UnitsType& distanceUnit(const QString& text)
{
    if (text == QStringLiteral("PIXELS"))
        return osgEarth::Units::PIXELS;
    if (text == QStringLiteral("KILOMETERS"))
        return osgEarth::Units::KILOMETERS;
    return osgEarth::Units::METERS;
}

QString geometryTypeName(osgEarth::Geometry::Type type)
{
    switch (type)
    {
    case osgEarth::Geometry::TYPE_POINT:
    case osgEarth::Geometry::TYPE_POINTSET:
        return QStringLiteral("Point");
    case osgEarth::Geometry::TYPE_LINESTRING:
        return QStringLiteral("Line");
    case osgEarth::Geometry::TYPE_POLYGON:
    case osgEarth::Geometry::TYPE_RING:
    case osgEarth::Geometry::TYPE_TRIMESH:
        return QStringLiteral("Polygon");
    case osgEarth::Geometry::TYPE_MULTI:
        return QStringLiteral("Mixed");
    default:
        return QStringLiteral("Unknown");
    }
}
}

SymbologyEditorDialog::SymbologyEditorDialog(
    osgEarth::FeatureModelLayer* featureModelLayer,
    QWidget* parent)
    : QDialog(parent)
    , ui(new Ui::SymbologyEditorDialog)
    , _featureLayer(featureModelLayer)
    , _featureSource(featureModelLayer ? featureModelLayer->getFeatureSource() : nullptr)
{
    ui->setupUi(this);
    setUpUi();
    loadCurrentStyle();
}

SymbologyEditorDialog::SymbologyEditorDialog(
    osgEarth::FeatureImageLayer* featureImageLayer,
    QWidget* parent,
    osgEarth::MapNode* mapNode)
    : QDialog(parent)
    , ui(new Ui::SymbologyEditorDialog)
    , _featureLayer(featureImageLayer)
    , _featureSource(featureImageLayer ? featureImageLayer->getFeatureSource() : nullptr)
    , _mapNode(mapNode)
{
    ui->setupUi(this);
    setUpUi();
    loadCurrentStyle();
}

SymbologyEditorDialog::~SymbologyEditorDialog()
{
    delete ui;
}

void SymbologyEditorDialog::setUpUi()
{
    _pointFillColor = new ColorPickerCheckBox(QColor(Qt::red), this);
    ui->pointFillColorLayout->addWidget(_pointFillColor);
    _lineColor = new ColorPickerCheckBox(QColor(Qt::blue), this);
    ui->lineColorLayout->addWidget(_lineColor);
    _polygonFillColor = new ColorPickerCheckBox(QColor(Qt::green), this);
    ui->polygonFillColorLayout->addWidget(_polygonFillColor);

    ui->layerComboBox->clear();
    if (_featureLayer.valid())
        ui->layerComboBox->addItem(QString::fromStdString(_featureLayer->getName()));
    ui->layerComboBox->setEnabled(false);

    ui->pointSymbolGroupBox->setVisible(false);
    ui->pointSymbolGroupBox->setEnabled(false);
    ui->lineSymbolGroupBox->setVisible(false);
    ui->lineSymbolGroupBox->setEnabled(false);
    ui->polygonSymbolGroupBox->setVisible(false);
    ui->polygonSymbolGroupBox->setEnabled(false);

    connect(ui->enablePointSymbolCheckBox, &QCheckBox::toggled, this,
        [this](bool enabled) {
            ui->pointSymbolGroupBox->setEnabled(enabled);
            ui->pointSymbolGroupBox->setVisible(enabled);
        });
    connect(ui->enableLineSymbolCheckBox, &QCheckBox::toggled, this,
        [this](bool enabled) {
            ui->lineSymbolGroupBox->setEnabled(enabled);
            ui->lineSymbolGroupBox->setVisible(enabled);
        });
    connect(ui->enablePolygonSymbolCheckBox, &QCheckBox::toggled, this,
        [this](bool enabled) {
            ui->polygonSymbolGroupBox->setEnabled(enabled);
            ui->polygonSymbolGroupBox->setVisible(enabled);
        });
    connect(ui->applyButton, &QPushButton::clicked, this, &SymbologyEditorDialog::applyStyle);
    connect(ui->cancelButton, &QPushButton::clicked, this, [this]() {
        close();
    });

    verifyGeometryType();
}

void SymbologyEditorDialog::verifyGeometryType()
{
    const osgEarth::Geometry::Type type = selectedGeometryType();
    ui->geometryLabel->setText(QStringLiteral("Geometry type: %1").arg(geometryTypeName(type)));

    const bool isPoint = type == osgEarth::Geometry::TYPE_POINT ||
                         type == osgEarth::Geometry::TYPE_POINTSET;
    const bool isLine = type == osgEarth::Geometry::TYPE_LINESTRING;
    const bool isPolygon = type == osgEarth::Geometry::TYPE_POLYGON ||
                           type == osgEarth::Geometry::TYPE_RING ||
                           type == osgEarth::Geometry::TYPE_TRIMESH;
    const bool isMixed = type == osgEarth::Geometry::TYPE_MULTI;

    ui->enablePointSymbolCheckBox->setEnabled(isPoint || isMixed);
    ui->enableLineSymbolCheckBox->setEnabled(isLine || isPolygon || isMixed);
    ui->enablePolygonSymbolCheckBox->setEnabled(isPolygon || isMixed);
    if (!isPoint && !isMixed)
        ui->enablePointSymbolCheckBox->setChecked(false);
    if (!isLine && !isPolygon && !isMixed)
        ui->enableLineSymbolCheckBox->setChecked(false);
    if (!isPolygon && !isMixed)
        ui->enablePolygonSymbolCheckBox->setChecked(false);
}

osgEarth::Geometry::Type SymbologyEditorDialog::selectedGeometryType() const
{
    if (!_featureSource)
        return osgEarth::Geometry::TYPE_UNKNOWN;

    const osgEarth::Geometry::Type type = _featureSource->getGeometryType();
    if (type != osgEarth::Geometry::TYPE_UNKNOWN)
        return type;

    osg::ref_ptr<osgEarth::FeatureCursor> cursor = _featureSource->createFeatureCursor();
    if (cursor.valid() && cursor->hasMore())
    {
        osg::ref_ptr<osgEarth::Feature> feature = cursor->nextFeature();
        if (feature.valid() && feature->getGeometry())
            return feature->getGeometry()->getType();
    }
    return osgEarth::Geometry::TYPE_UNKNOWN;
}

void SymbologyEditorDialog::loadCurrentStyle()
{
    osgEarth::StyleSheet* styleSheet = nullptr;
    if (auto* featureModelLayer = dynamic_cast<osgEarth::FeatureModelLayer*>(_featureLayer.get()))
        styleSheet = featureModelLayer->getStyleSheet();
    else if (auto* featureImageLayer = dynamic_cast<osgEarth::FeatureImageLayer*>(_featureLayer.get()))
        styleSheet = featureImageLayer->getStyleSheet();

    if (styleSheet)
    {
        const osgEarth::Style* style = styleSheet->getDefaultStyle();
        if (style)
            populateStyle(*style);
    }
}

void SymbologyEditorDialog::populateStyle(const osgEarth::Style& style)
{
    if (const auto* point = style.get<osgEarth::PointSymbol>())
    {
        ui->enablePointSymbolCheckBox->setChecked(true);
        if (point->size().isSet())
            ui->pointSizeSpinBox->setValue(point->size().get());
        if (point->fill().isSet())
            _pointFillColor->setColor(toQColor(point->fill()->color()));
        ui->pointSmoothingCheckBox->setChecked(point->smooth().isSet() && point->smooth().get());
    }

    if (const auto* line = style.get<osgEarth::LineSymbol>())
    {
        ui->enableLineSymbolCheckBox->setChecked(true);
        if (line->stroke().isSet())
        {
            const auto& width = line->stroke()->width().get();
            if (width.expression().empty())
                ui->lineWidthSpinBox->setValue(
                    width.literal().as(osgEarth::Units::PIXELS));
            _lineColor->setColor(toQColor(line->stroke()->color()));
        }
        if (line->tessellationSize().isSet())
            ui->lineTessellationSizeSpinBox->setValue(
                line->tessellationSize().get().as(osgEarth::Units::METERS));
    }

    if (const auto* polygon = style.get<osgEarth::PolygonSymbol>())
    {
        ui->enablePolygonSymbolCheckBox->setChecked(true);
        if (polygon->fill().isSet())
            _polygonFillColor->setColor(toQColor(polygon->fill()->color()));
        ui->polygonOutlineCheckBox->setChecked(polygon->outline().get());
    }

    if (const auto* altitude = style.get<osgEarth::AltitudeSymbol>())
    {
        switch (altitude->clamping().get())
        {
        case osgEarth::AltitudeSymbol::CLAMP_NONE:
            ui->altitudeClampingComboBox->setCurrentText(QStringLiteral("CLAMP_NONE"));
            break;
        case osgEarth::AltitudeSymbol::CLAMP_ABSOLUTE:
            ui->altitudeClampingComboBox->setCurrentText(QStringLiteral("CLAMP_ABSOLUTE"));
            break;
        case osgEarth::AltitudeSymbol::CLAMP_RELATIVE_TO_TERRAIN:
            ui->altitudeClampingComboBox->setCurrentText(QStringLiteral("CLAMP_RELATIVE_TO_TERRAIN"));
            break;
        default:
            ui->altitudeClampingComboBox->setCurrentText(QStringLiteral("CLAMP_TO_TERRAIN"));
            break;
        }
        switch (altitude->technique().get())
        {
        case osgEarth::AltitudeSymbol::TECHNIQUE_GPU:
            ui->altitudeTechniqueComboBox->setCurrentText(QStringLiteral("TECHNIQUE_GPU"));
            break;
        case osgEarth::AltitudeSymbol::TECHNIQUE_SCENE:
            ui->altitudeTechniqueComboBox->setCurrentText(QStringLiteral("TECHNIQUE_SCENE"));
            break;
        case osgEarth::AltitudeSymbol::TECHNIQUE_DRAPE:
            ui->altitudeTechniqueComboBox->setCurrentText(QStringLiteral("TECHNIQUE_DRAPE"));
            break;
        default:
            ui->altitudeTechniqueComboBox->setCurrentText(QStringLiteral("TECHNIQUE_MAP"));
            break;
        }
        switch (altitude->binding().get())
        {
        case osgEarth::AltitudeSymbol::BINDING_CENTROID:
            ui->altitudeBindingComboBox->setCurrentText(QStringLiteral("BINDING_CENTROID"));
            break;
        case osgEarth::AltitudeSymbol::BINDING_ENDPOINT:
            ui->altitudeBindingComboBox->setCurrentText(QStringLiteral("BINDING_ENDPOINT"));
            break;
        default:
            ui->altitudeBindingComboBox->setCurrentText(QStringLiteral("BINDING_VERTEX"));
            break;
        }
    }
}

void SymbologyEditorDialog::createPointSymbol(osgEarth::Style& style) const
{
    auto* symbol = style.getOrCreate<osgEarth::PointSymbol>();
    symbol->size() = static_cast<float>(ui->pointSizeSpinBox->value());
    symbol->fill()->color() = toEarthColor(_pointFillColor->color());
    symbol->smooth() = ui->pointSmoothingCheckBox->isChecked();
}

void SymbologyEditorDialog::createLineSymbol(osgEarth::Style& style) const
{
    auto* symbol = style.getOrCreate<osgEarth::LineSymbol>();
    symbol->stroke()->color() = toEarthColor(_lineColor->color());
    symbol->stroke()->width() = osgEarth::Distance(
        ui->lineWidthSpinBox->value(), distanceUnit(ui->lineWidthUnitsComboBox->currentText()));
    symbol->tessellationSize() = osgEarth::Distance(
        ui->lineTessellationSizeSpinBox->value(),
        distanceUnit(ui->lineTessellationUnitsComboBox->currentText()));
}

void SymbologyEditorDialog::createPolygonSymbol(osgEarth::Style& style) const
{
    auto* polygon = style.getOrCreate<osgEarth::PolygonSymbol>();
    polygon->fill()->color() = toEarthColor(_polygonFillColor->color());
    polygon->outline() = ui->polygonOutlineCheckBox->isChecked();
}

void SymbologyEditorDialog::createAltitudeSymbol(osgEarth::Style& style) const
{
    auto* altitude = style.getOrCreate<osgEarth::AltitudeSymbol>();
    const QString clamping = ui->altitudeClampingComboBox->currentText();
    if (clamping == QStringLiteral("CLAMP_NONE"))
        altitude->clamping() = osgEarth::AltitudeSymbol::CLAMP_NONE;
    else if (clamping == QStringLiteral("CLAMP_ABSOLUTE"))
        altitude->clamping() = osgEarth::AltitudeSymbol::CLAMP_ABSOLUTE;
    else if (clamping == QStringLiteral("CLAMP_RELATIVE_TO_TERRAIN"))
        altitude->clamping() = osgEarth::AltitudeSymbol::CLAMP_RELATIVE_TO_TERRAIN;
    else
        altitude->clamping() = osgEarth::AltitudeSymbol::CLAMP_TO_TERRAIN;

    const QString technique = ui->altitudeTechniqueComboBox->currentText();
    if (technique == QStringLiteral("TECHNIQUE_GPU"))
        altitude->technique() = osgEarth::AltitudeSymbol::TECHNIQUE_GPU;
    else if (technique == QStringLiteral("TECHNIQUE_SCENE"))
        altitude->technique() = osgEarth::AltitudeSymbol::TECHNIQUE_SCENE;
    else if (technique == QStringLiteral("TECHNIQUE_DRAPE"))
        altitude->technique() = osgEarth::AltitudeSymbol::TECHNIQUE_DRAPE;
    else
        altitude->technique() = osgEarth::AltitudeSymbol::TECHNIQUE_MAP;

    const QString binding = ui->altitudeBindingComboBox->currentText();
    if (binding == QStringLiteral("BINDING_CENTROID"))
        altitude->binding() = osgEarth::AltitudeSymbol::BINDING_CENTROID;
    else if (binding == QStringLiteral("BINDING_ENDPOINT"))
        altitude->binding() = osgEarth::AltitudeSymbol::BINDING_ENDPOINT;
    else
        altitude->binding() = osgEarth::AltitudeSymbol::BINDING_VERTEX;

}

void SymbologyEditorDialog::applyStyle()
{
    if (!_featureSource || !_featureLayer.valid())
    {
        QMessageBox::warning(this, tr("No feature source"),
            tr("The feature layer does not have a valid feature source."));
        return;
    }

    const osgEarth::Style* currentStyle = nullptr;
    if (auto* featureModelLayer = dynamic_cast<osgEarth::FeatureModelLayer*>(_featureLayer.get()))
    {
        if (featureModelLayer->getStyleSheet())
            currentStyle = featureModelLayer->getStyleSheet()->getDefaultStyle();
    }
    else if (auto* featureImageLayer = dynamic_cast<osgEarth::FeatureImageLayer*>(_featureLayer.get()))
    {
        if (featureImageLayer->getStyleSheet())
            currentStyle = featureImageLayer->getStyleSheet()->getDefaultStyle();
    }

    osgEarth::Style style;
    if (currentStyle)
        style = *currentStyle;

    if (ui->enablePointSymbolCheckBox->isChecked())
        createPointSymbol(style);
    else
        style.remove<osgEarth::PointSymbol>();

    const bool polygonOutline = ui->enablePolygonSymbolCheckBox->isChecked() &&
                                ui->polygonOutlineCheckBox->isChecked();
    if (ui->enableLineSymbolCheckBox->isChecked() || polygonOutline)
        createLineSymbol(style);
    else
        style.remove<osgEarth::LineSymbol>();

    if (ui->enablePolygonSymbolCheckBox->isChecked())
        createPolygonSymbol(style);
    else
        style.remove<osgEarth::PolygonSymbol>();

    createAltitudeSymbol(style);

    osg::ref_ptr<osgEarth::StyleSheet> styleSheet = new osgEarth::StyleSheet();
    styleSheet->addStyle(style);

    if (auto* featureModelLayer = dynamic_cast<osgEarth::FeatureModelLayer*>(_featureLayer.get()))
    {
        featureModelLayer->setStyleSheet(styleSheet.get());
        if (_mapNode && _mapNode->getTerrainEngine())
        {
            const std::vector<const osgEarth::Layer*> layers{ featureModelLayer };
            _mapNode->getTerrainEngine()->invalidateRegion(layers, osgEarth::GeoExtent::INVALID);
        }

    }
    else if (auto* featureImageLayer = dynamic_cast<osgEarth::FeatureImageLayer*>(_featureLayer.get()))
    {
        featureImageLayer->setCachePolicy(osgEarth::CachePolicy::NO_CACHE);
        featureImageLayer->setStyleSheet(styleSheet.get());
        featureImageLayer->dirty();

        if (_mapNode && _mapNode->getTerrainEngine())
        {
            const std::vector<const osgEarth::Layer*> layers{ featureImageLayer };
            _mapNode->getTerrainEngine()->invalidateRegion(layers, osgEarth::GeoExtent::INVALID);
        }
    }

     OSGViewerWidget* viewer = App::getThreedWindow();
        if (!viewer || viewer->getMapNode() != _mapNode)
            viewer = App::get2DWindow();
        if (viewer && viewer->getMapNode() == _mapNode)
            viewer->update();
}
