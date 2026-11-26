#include "Extrusion3D.h"
#include "ui_Extrusion3D.h"

#include <QColorDialog>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QMessageBox>
#include <QShowEvent>
#include <QSignalBlocker>

#include <osgEarth/AltitudeSymbol>
#include <osgEarth/ExtrusionSymbol>
#include <osgEarth/FeatureImageLayer>
#include <osgEarth/FeatureModelLayer>
#include <osgEarth/LineSymbol>
#include <osgEarth/Map>
#include <osgEarth/PolygonSymbol>
#include <osgEarth/FeatureDisplayLayout>
#include <osgEarth/StyleSheet>

#include <vector>

namespace
{
    constexpr int PendingSourceRole = Qt::UserRole + 1;

    QColor toQColor(const osgEarth::Color& color)
    {
        return QColor::fromRgbF(color[0], color[1], color[2], color[3]);
    }

    osgEarth::Color toEarthColor(const QColor& color)
    {
        return osgEarth::Color(color.redF(), color.greenF(), color.blueF(), color.alphaF());
    }

    osgEarth::FeatureSource* featureSourceFor(osgEarth::Layer* layer)
    {
        if (auto* featureModelLayer = dynamic_cast<osgEarth::FeatureModelLayer*>(layer))
            return featureModelLayer->getFeatureSource();
        if (auto* featureImageLayer = dynamic_cast<osgEarth::FeatureImageLayer*>(layer))
            return featureImageLayer->getFeatureSource();
        return nullptr;
    }
}

Extrusion3D::Extrusion3D(osgEarth::MapNode *mapNode, QWidget *parent)
    : QDialog(parent),
      ui(new Ui::Extrusion3D),
      _mapNode(mapNode),
      _fillColor(217, 164, 65),
      _outlineColor(45, 45, 45)
{
    ui->setupUi(this);
    setWindowTitle(tr("3D Extrusion"));

    ui->extrusionDirectionComboBox->setItemText(0, tr("Up"));
    ui->extrusionDirectionComboBox->setItemText(1, tr("Down"));
    ui->extrusionHeightAttributeComboBox->setEnabled(false);
    ui->doubleSpinBox_outlineWidth->setRange(0.1, 100.0);
    ui->doubleSpinBox_outlineWidth->setValue(1.0);
    ui->checkBox_enableOutLine->setChecked(false);
    ui->pushButton_OutLineColor->setEnabled(false);
    ui->doubleSpinBox_outlineWidth->setEnabled(false);

    updateColorButton(ui->pushButton_fillColor, _fillColor);
    updateColorButton(ui->pushButton_OutLineColor, _outlineColor);

    connect(ui->extrusionFixedHeightRadioButton, &QRadioButton::toggled,
            this, &Extrusion3D::updateHeightControls);
    connect(ui->extrusionFieldHeightRadioButton, &QRadioButton::toggled,
            this, &Extrusion3D::updateHeightControls);
    connect(ui->checkBox_enableOutLine, &QCheckBox::toggled, this, [this](bool enabled) {
        ui->pushButton_OutLineColor->setEnabled(enabled);
        ui->doubleSpinBox_outlineWidth->setEnabled(enabled);
    });
    connect(ui->comboBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this](int) {
                if (ui->comboBox->currentData(PendingSourceRole).toBool())
                {
                    if (_pendingFeatureSource)
                        loadAttributeFields(_pendingFeatureSource.get());
                }
                else
                {
                    auto *map = _mapNode.valid() ? _mapNode->getMap() : nullptr;
                    const auto uid = ui->comboBox->currentData().toULongLong();
                    osgEarth::Layer *layer = map && uid
                        ? map->getLayerByUID(static_cast<osgEarth::UID>(uid))
                        : nullptr;
                    loadAttributeFields(featureSourceFor(layer));
                    _fillColor = QColor(217, 164, 65);
                    _outlineColor = QColor(45, 45, 45);
                    ui->checkBox_enableOutLine->setChecked(false);
                    ui->doubleSpinBox_outlineWidth->setValue(1.0);
                    ui->extrusionFixedHeightRadioButton->setChecked(true);
                    ui->extrusionFixedHeightSpinBox->setValue(10.0);
                    ui->extrusionDirectionComboBox->setCurrentIndex(0);

                    osgEarth::StyleSheet *styleSheet = nullptr;
                    if (auto *featureModel = dynamic_cast<osgEarth::FeatureModelLayer*>(layer))
                        styleSheet = featureModel->getStyleSheet();
                    else if (auto *featureImage = dynamic_cast<osgEarth::FeatureImageLayer*>(layer))
                        styleSheet = featureImage->getStyleSheet();
                    if (styleSheet)
                    {
                        const osgEarth::Style *style = styleSheet->getDefaultStyle();
                        if (style)
                        {
                            if (const auto *polygon = style->get<osgEarth::PolygonSymbol>())
                            {
                                if (polygon->fill().isSet())
                                    _fillColor = toQColor(polygon->fill()->color());
                                ui->checkBox_enableOutLine->setChecked(polygon->outline().get());
                            }
                            if (const auto *line = style->get<osgEarth::LineSymbol>())
                            {
                                if (line->stroke().isSet())
                                {
                                    _outlineColor = toQColor(line->stroke()->color());
                                    ui->checkBox_enableOutLine->setChecked(true);
                                    if (line->stroke()->width().isSet())
                                    {
                                        const auto& width = line->stroke()->width().get();
                                        if (width.expression().empty())
                                            ui->doubleSpinBox_outlineWidth->setValue(
                                                width.literal().as(osgEarth::Units::PIXELS));
                                    }
                                }
                            }
                            if (const auto *extrusion = style->get<osgEarth::ExtrusionSymbol>())
                            {
                                if (extrusion->height().isSet())
                                {
                                    const auto& height = extrusion->height().get();
                                    if (height.expression().empty())
                                    {
                                        ui->extrusionFixedHeightRadioButton->setChecked(true);
                                        ui->extrusionFixedHeightSpinBox->setValue(
                                            height.literal().as(osgEarth::Units::METERS));
                                    }
                                    else
                                    {
                                        QString field = QString::fromStdString(height.expression());
                                        if (field.startsWith(QLatin1Char('[')) &&
                                            field.endsWith(QLatin1Char(']')))
                                            field = field.mid(1, field.size() - 2);
                                        const int fieldIndex =
                                            ui->extrusionHeightAttributeComboBox->findText(field);
                                        if (fieldIndex >= 0)
                                        {
                                            ui->extrusionFieldHeightRadioButton->setChecked(true);
                                            ui->extrusionHeightAttributeComboBox->setCurrentIndex(fieldIndex);
                                        }
                                    }
                                }
                                ui->extrusionDirectionComboBox->setCurrentIndex(
                                    extrusion->direction().get() ==
                                            osgEarth::ExtrusionSymbol::DIRECTION_DOWN
                                        ? 1 : 0);
                            }
                        }
                    }
                    updateColorButton(ui->pushButton_fillColor, _fillColor);
                    updateColorButton(ui->pushButton_OutLineColor, _outlineColor);
                }
                updateHeightControls();
            });

    connect(ui->pushButton, &QPushButton::clicked, this, [this] {
        const QString path = QFileDialog::getOpenFileName(
            this, tr("Select vector dataset"), QString(),
            tr("Vector datasets (*.shp *.geojson *.json *.gpkg *.kml);;All files (*)"));
        if (path.isEmpty())
            return;

        osg::ref_ptr<osgEarth::OGRFeatureSource> source = new osgEarth::OGRFeatureSource();
        source->setURL(QFile::encodeName(path).toStdString());
        source->setName(QFileInfo(path).completeBaseName().toStdString());
        const osgEarth::Status status = source->open();
        if (status.isError())
        {
            QMessageBox::warning(this, tr("Cannot open vector dataset"),
                                 QString::fromStdString(status.message()));
            return;
        }

        _pendingFeatureSource = source;
        _pendingFilePath = path;
        _fillColor = QColor(217, 164, 65);
        _outlineColor = QColor(45, 45, 45);
        ui->checkBox_enableOutLine->setChecked(false);
        ui->doubleSpinBox_outlineWidth->setValue(1.0);
        ui->extrusionFixedHeightRadioButton->setChecked(true);
        ui->extrusionFixedHeightSpinBox->setValue(10.0);
        ui->extrusionDirectionComboBox->setCurrentIndex(0);
        updateColorButton(ui->pushButton_fillColor, _fillColor);
        updateColorButton(ui->pushButton_OutLineColor, _outlineColor);
        const QSignalBlocker blocker(ui->comboBox);
        const int row = ui->comboBox->count();
        ui->comboBox->addItem(QFileInfo(path).fileName(), QVariant());
        ui->comboBox->setItemData(row, true, PendingSourceRole);
        ui->comboBox->setCurrentIndex(row);
        loadAttributeFields(source.get());
        updateHeightControls();
    });

    connect(ui->pushButton_fillColor, &QPushButton::clicked, this, [this] {
        const QColor color = QColorDialog::getColor(_fillColor, this, tr("Fill color"),
                                                    QColorDialog::ShowAlphaChannel);
        if (color.isValid())
        {
            _fillColor = color;
            updateColorButton(ui->pushButton_fillColor, _fillColor);
        }
    });
    connect(ui->pushButton_OutLineColor, &QPushButton::clicked, this, [this] {
        const QColor color = QColorDialog::getColor(_outlineColor, this, tr("Outline color"),
                                                    QColorDialog::ShowAlphaChannel);
        if (color.isValid())
        {
            _outlineColor = color;
            updateColorButton(ui->pushButton_OutLineColor, _outlineColor);
        }
    });
    connect(ui->pushButton_2, &QPushButton::clicked, this, &Extrusion3D::applyChanges);

    refreshFeatureLayers();
}

Extrusion3D::~Extrusion3D()
{
    delete ui;
}

void Extrusion3D::showEvent(QShowEvent *event)
{
    QDialog::showEvent(event);
    refreshFeatureLayers();
}

void Extrusion3D::refreshFeatureLayers(osgEarth::UID selectedLayerUID)
{
    auto *map = _mapNode.valid() ? _mapNode->getMap() : nullptr;
    {
        const QSignalBlocker blocker(ui->comboBox);
        const QVariant previous = ui->comboBox->currentData();
        const bool previousWasPending =
            ui->comboBox->currentData(PendingSourceRole).toBool();
        ui->comboBox->clear();
        ui->comboBox->addItem(tr("Select Feature Layer"), QVariant());

        if (map)
        {
            std::vector<osg::ref_ptr<osgEarth::FeatureModelLayer>> featureModelLayers;
            std::vector<osg::ref_ptr<osgEarth::FeatureImageLayer>> featureImageLayers;
            map->getOpenLayers(featureModelLayers);
            map->getOpenLayers(featureImageLayers);
            for (const auto& layer : featureModelLayers)
            {
                if (layer.valid())
                    ui->comboBox->addItem(QString::fromStdString(layer->getName()),
                                          QVariant::fromValue<qulonglong>(layer->getUID()));
            }
            for (const auto& layer : featureImageLayers)
            {
                if (layer.valid())
                    ui->comboBox->addItem(
                        tr("%1 (Feature image source; creates 3D layer)")
                            .arg(QString::fromStdString(layer->getName())),
                        QVariant::fromValue<qulonglong>(layer->getUID()));
            }
        }

        if (_pendingFeatureSource && !_pendingFilePath.isEmpty())
        {
            const int row = ui->comboBox->count();
            ui->comboBox->addItem(QFileInfo(_pendingFilePath).fileName(), QVariant());
            ui->comboBox->setItemData(row, true, PendingSourceRole);
            ui->comboBox->setCurrentIndex(row);
            loadAttributeFields(_pendingFeatureSource.get());
        }
        else
        {
            int selectedIndex = selectedLayerUID
                ? ui->comboBox->findData(
                      QVariant::fromValue<qulonglong>(selectedLayerUID))
                : ui->comboBox->findData(previous);
            if (selectedIndex <= 0 && !previousWasPending && ui->comboBox->count() > 1)
                selectedIndex = 1;
            ui->comboBox->setCurrentIndex(selectedIndex > 0 ? selectedIndex : 0);
            if (ui->comboBox->currentIndex() > 0)
            {
                auto *layer = map->getLayerByUID<osgEarth::FeatureModelLayer>(
                    static_cast<osgEarth::UID>(ui->comboBox->currentData().toULongLong()));
                loadAttributeFields(layer ? layer->getFeatureSource() : nullptr);
            }
            else
            {
                ui->extrusionHeightAttributeComboBox->clear();
            }
        }
        updateHeightControls();
    }

    const int selectedIndex = ui->comboBox->currentIndex();
    if (selectedIndex > 0)
    {
        ui->comboBox->setCurrentIndex(0);
        ui->comboBox->setCurrentIndex(selectedIndex);
    }
}

void Extrusion3D::updateHeightControls()
{
    const bool fieldMode = ui->extrusionFieldHeightRadioButton->isChecked();
    ui->extrusionFixedHeightSpinBox->setEnabled(!fieldMode);
    ui->extrusionHeightAttributeComboBox->setEnabled(
        fieldMode && ui->extrusionHeightAttributeComboBox->count() > 0);
}

void Extrusion3D::updateColorButton(QPushButton *button, const QColor& color)
{
    button->setText(color.name(QColor::HexArgb));
    button->setStyleSheet(QStringLiteral(
        "QPushButton { background-color: %1; color: %2; }")
        .arg(color.name(QColor::HexArgb),
             color.lightness() < 128 ? QStringLiteral("white") : QStringLiteral("black")));
}

bool Extrusion3D::loadAttributeFields(osgEarth::FeatureSource *source)
{
    ui->extrusionHeightAttributeComboBox->clear();
    if (!source)
        return false;
    if (!source->isOpen())
    {
        const osgEarth::Status status = source->open();
        if (status.isError())
        {
            QMessageBox::warning(this, tr("Cannot read feature attributes"),
                                 QString::fromStdString(status.message()));
            return false;
        }
    }

    for (const auto& attribute : source->getSchema())
    {
        if (attribute.second == osgEarth::ATTRTYPE_DOUBLE ||
            attribute.second == osgEarth::ATTRTYPE_INT)
            ui->extrusionHeightAttributeComboBox->addItem(
                QString::fromStdString(attribute.first));
    }
    return ui->extrusionHeightAttributeComboBox->count() > 0;
}

void Extrusion3D::applyChanges()
{
    if (!_mapNode.valid() || !_mapNode->getMap())
    {
        QMessageBox::warning(this, tr("No map"), tr("Open a map before applying extrusion."));
        return;
    }

    osgEarth::Map *map = _mapNode->getMap();
    const int row = ui->comboBox->currentIndex();
    const bool pendingSource = row >= 0 &&
        ui->comboBox->currentData(PendingSourceRole).toBool();

    osg::ref_ptr<osgEarth::FeatureModelLayer> layer;
    osgEarth::FeatureSource *source = nullptr;
    QString layerName;
    if (pendingSource)
    {
        source = _pendingFeatureSource.get();
        layerName = QFileInfo(_pendingFilePath).completeBaseName();
    }
    else
    {
        const auto uid = ui->comboBox->currentData().toULongLong();
        osgEarth::Layer *selectedLayer = uid
            ? map->getLayerByUID(static_cast<osgEarth::UID>(uid))
            : nullptr;
        if (auto *featureModel = dynamic_cast<osgEarth::FeatureModelLayer*>(selectedLayer))
        {
            layer = featureModel;
            source = featureModel->getFeatureSource();
            layerName = QString::fromStdString(featureModel->getName());
        }
        else if (auto *featureImage =
                     dynamic_cast<osgEarth::FeatureImageLayer*>(selectedLayer))
        {
            source = featureImage->getFeatureSource();
            layerName = QString::fromStdString(featureImage->getName());
        }
    }

    if (!source)
    {
        QMessageBox::warning(this, tr("No feature source"),
                             tr("Select an opened feature layer or browse for a vector dataset."));
        return;
    }

    if (ui->extrusionFieldHeightRadioButton->isChecked() &&
        ui->extrusionHeightAttributeComboBox->currentText().trimmed().isEmpty())
    {
        QMessageBox::warning(this, tr("Height field required"),
                             tr("Select a numeric field or choose Fixed Height."));
        return;
    }

    osgEarth::StyleSheet *styleSheet = layer ? layer->getStyleSheet() : nullptr;
    osg::ref_ptr<osgEarth::StyleSheet> newStyleSheet;
    if (!styleSheet)
    {
        newStyleSheet = new osgEarth::StyleSheet();
        styleSheet = newStyleSheet.get();
    }

    osgEarth::Style *style = styleSheet->getDefaultStyle();
    if (!style)
    {
        osgEarth::Style defaultStyle;
        defaultStyle.setName("default");
        styleSheet->addStyle(defaultStyle);
        style = styleSheet->getDefaultStyle();
    }

    osgEarth::ExtrusionSymbol* extrusion = style->getOrCreate<osgEarth::ExtrusionSymbol>();
    if (ui->extrusionFieldHeightRadioButton->isChecked())
    {
        const QString field = ui->extrusionHeightAttributeComboBox->currentText().trimmed();
        extrusion->height() =
            osgEarth::Expression<osgEarth::Distance>(
                QStringLiteral("[%1]").arg(field).toStdString());
    }
    else
    {
        extrusion->height() = osgEarth::Distance(
            ui->extrusionFixedHeightSpinBox->value(), osgEarth::Units::METERS);
    }
    extrusion->direction() = ui->extrusionDirectionComboBox->currentIndex() == 1
        ? osgEarth::ExtrusionSymbol::DIRECTION_DOWN
        : osgEarth::ExtrusionSymbol::DIRECTION_UP;
    extrusion->flatten() = true;

    auto *polygon = style->getOrCreate<osgEarth::PolygonSymbol>();
    polygon->fill()->color() = toEarthColor(_fillColor);
    polygon->outline() = ui->checkBox_enableOutLine->isChecked();

    if (ui->checkBox_enableOutLine->isChecked())
    {
        osgEarth::LineSymbol *line = style->getOrCreate<osgEarth::LineSymbol>();
        line->stroke()->color() = toEarthColor(_outlineColor);
        line->stroke()->width() =
            osgEarth::Distance(ui->doubleSpinBox_outlineWidth->value(),
                               osgEarth::Units::PIXELS);
    }
    else
    {
        style->remove<osgEarth::LineSymbol>();
    }

    auto *altitude = style->getOrCreate<osgEarth::AltitudeSymbol>();
    altitude->clamping() = osgEarth::AltitudeSymbol::CLAMP_TO_TERRAIN;
    altitude->binding() = osgEarth::AltitudeSymbol::BINDING_VERTEX;

    if (!layer)
    {
        layer = new osgEarth::FeatureModelLayer();
        layer->setName((layerName + QStringLiteral(" (Extruded)")).toStdString());
        layer->setFeatureSource(source);
        layer->setStyleSheet(styleSheet);
        osgEarth::FeatureDisplayLayout layout;
        layout.tileSize() = 500;
        layer->setLayout(layout);
        layer->setMaxVisibleRange(20000.0);
        map->addLayer(layer.get());
    }
    else if (newStyleSheet.valid())
    {
        layer->setStyleSheet(styleSheet);
    }
    else
    {
        layer->dirty();
    }

    _pendingFeatureSource = nullptr;
    _pendingFilePath.clear();
    refreshFeatureLayers(layer->getUID());
    QMessageBox::information(this, tr("Extrusion applied"),
                             tr("The extrusion settings were applied to %1.")
                                 .arg(QString::fromStdString(layer->getName())));
}


