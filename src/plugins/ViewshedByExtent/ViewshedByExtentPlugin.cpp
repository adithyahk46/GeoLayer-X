#include "ViewshedByExtentPlugin.h"

#include <App.h>
#include <QDialog>
#include <QDoubleSpinBox>
#include <QLineEdit>
#include <QMainWindow>
#include <QMessageBox>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSpinBox>

#include <algorithm>
#include <array>
#include <cmath>

#include <osg/LineWidth>
#include <osg/StateSet>

#include "colorpickercheckbox.h"
#include "osgviewerwidget.h"

namespace
{
bool makeExtentFromCorners(
    const osgEarth::GeoPoint& firstPoint,
    const osgEarth::GeoPoint& secondPoint,
    osgEarth::GeoExtent& extent)
{
    const osgEarth::SpatialReference* geographicSrs =
        osgEarth::SpatialReference::get("epsg:4326");
    if (!geographicSrs)
        return false;

    osgEarth::GeoPoint first;
    osgEarth::GeoPoint second;
    firstPoint.transform(geographicSrs, first);
    secondPoint.transform(geographicSrs, second);
    if (!first.isValid() || !second.isValid())
        return false;

    const double west = std::min(first.x(), second.x());
    const double east = std::max(first.x(), second.x());
    const double south = std::min(first.y(), second.y());
    const double north = std::max(first.y(), second.y());
    if (east <= west || north <= south)
        return false;

    extent = osgEarth::GeoExtent(geographicSrs, west, south, east, north);
    return extent.isValid();
}
}

ViewshedByExtentPlugin::ViewshedByExtentPlugin(QObject* parent)
    : PluginInterface(parent)
{
}

ViewshedByExtentPlugin::~ViewshedByExtentPlugin()
{
    stopExtentSelection(false);
    stopLocationSelection(false);
    _viewshedCore.reset();
    removeExtentDrawing();
    delete _dialog;
    _dialog = nullptr;
}

void ViewshedByExtentPlugin::restoreMapNavigationIfIdle()
{
    if (!_extentSelectionConnection && !_locationSelectionConnection && _selectionManipulator)
    {
        _selectionManipulator->disableMouseAction(false);
        _selectionManipulator.clear();
    }
}

void ViewshedByExtentPlugin::stopExtentSelection(bool resetButton)
{
    QObject::disconnect(_extentSelectionConnection);
    _extentSelectionConnection = {};
    QObject::disconnect(_extentMoveConnection);
    _extentMoveConnection = {};
    _hasFirstExtentCorner = false;
    _firstExtentCorner = osgEarth::GeoPoint();

    if (resetButton && _dialog)
        ui.pushButton_selectExtent->setText(QStringLiteral("Select Extent"));

    restoreMapNavigationIfIdle();
}

void ViewshedByExtentPlugin::stopLocationSelection(bool resetButton)
{
    QObject::disconnect(_locationSelectionConnection);
    _locationSelectionConnection = {};

    if (resetButton && _dialog)
        ui.pb_pickLocation->setText(QStringLiteral("Pick Location"));

    restoreMapNavigationIfIdle();
}

void ViewshedByExtentPlugin::ensureExtentDrawing(osgEarth::MapNode* mapNode)
{
    if (!mapNode)
        return;

    if (_selectionMapNode == mapNode && _rubberBandGeode.valid())
        return;

    removeExtentDrawing();
    _selectionMapNode = mapNode;

    _rubberBandVertices = new osg::Vec3Array;
    _rubberBandGeometry = new osg::Geometry;
    _rubberBandGeometry->setDataVariance(osg::Object::DYNAMIC);
    _rubberBandGeometry->setVertexArray(_rubberBandVertices);
    _rubberBandDrawArrays = new osg::DrawArrays(GL_LINE_LOOP, 0, 0);
    _rubberBandGeometry->addPrimitiveSet(_rubberBandDrawArrays);

    _rubberBandColors = new osg::Vec4Array;
    _rubberBandColors->push_back(osg::Vec4(1.0f, 0.85f, 0.1f, 0.8f));
    _rubberBandGeometry->setColorArray(_rubberBandColors, osg::Array::BIND_OVERALL);

    _rubberBandGeode = new osg::Geode;
    _rubberBandGeode->addDrawable(_rubberBandGeometry);
    osg::StateSet* stateSet = _rubberBandGeode->getOrCreateStateSet();
    stateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    stateSet->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);
    stateSet->setMode(GL_BLEND, osg::StateAttribute::ON);
    stateSet->setAttributeAndModes(new osg::LineWidth(2.0f), osg::StateAttribute::ON);
    stateSet->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
}

void ViewshedByExtentPlugin::updateExtentRubberBand(const osgEarth::GeoExtent& extent)
{
    if (!_selectionMapNode.valid() || !_rubberBandGeode.valid() ||
        !_rubberBandVertices.valid() || !extent.isValid())
        return;

    osgEarth::GeoExtent geographicExtent;
    const osgEarth::SpatialReference* geographicSrs =
        osgEarth::SpatialReference::get("epsg:4326");
    if (!geographicSrs || !extent.transform(geographicSrs, geographicExtent) ||
        !geographicExtent.isValid())
        return;

    constexpr int segmentsPerEdge = 32;
    const std::array<std::pair<double, double>, 4> corners{{
        {geographicExtent.west(), geographicExtent.south()},
        {geographicExtent.east(), geographicExtent.south()},
        {geographicExtent.east(), geographicExtent.north()},
        {geographicExtent.west(), geographicExtent.north()}
    }};

    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
    vertices->reserve(segmentsPerEdge * 4);
    for (std::size_t edge = 0; edge < corners.size(); ++edge)
    {
        const auto& start = corners[edge];
        const auto& end = corners[(edge + 1) % corners.size()];
        for (int segment = 0; segment < segmentsPerEdge; ++segment)
        {
            const double fraction = static_cast<double>(segment) / segmentsPerEdge;
            osgEarth::GeoPoint point(
                geographicSrs,
                start.first + (end.first - start.first) * fraction,
                start.second + (end.second - start.second) * fraction,
                _rubberBandAltitude,
                osgEarth::ALTMODE_ABSOLUTE);
            osg::Vec3d worldPoint;
            if (!point.toWorld(worldPoint))
                return;

            vertices->push_back(osg::Vec3(
                static_cast<float>(worldPoint.x()),
                static_cast<float>(worldPoint.y()),
                static_cast<float>(worldPoint.z())));
        }
    }

    if (!_rubberBandGeode->getNumParents())
        _selectionMapNode->addChild(_rubberBandGeode);
    _rubberBandVertices->swap(*vertices);
    _rubberBandDrawArrays->setCount(static_cast<GLsizei>(_rubberBandVertices->size()));
    _rubberBandVertices->dirty();
    _rubberBandGeometry->dirtyBound();
    updateRubberBandColor();
}

void ViewshedByExtentPlugin::removeExtentDrawing()
{
    if (_selectionMapNode.valid() && _rubberBandGeode.valid())
        _selectionMapNode->removeChild(_rubberBandGeode);

    _rubberBandColors = nullptr;
    _rubberBandVertices = nullptr;
    _rubberBandDrawArrays = nullptr;
    _rubberBandGeometry = nullptr;
    _rubberBandGeode = nullptr;
    _selectionMapNode = nullptr;
}

bool ViewshedByExtentPlugin::readExtent(
    osgEarth::GeoExtent& extent,
    QString* errorMessage) const
{
    bool topLatOk = false;
    bool topLonOk = false;
    bool bottomLatOk = false;
    bool bottomLonOk = false;
    const double topLat = ui.lineEdit_topLeftLat->text().toDouble(&topLatOk);
    const double leftLon = ui.lineEdit_topLeftLon->text().toDouble(&topLonOk);
    const double bottomLat = ui.lineEdit_bottomRightLat->text().toDouble(&bottomLatOk);
    const double rightLon = ui.lineEdit_bottomRightLon->text().toDouble(&bottomLonOk);

    if (!topLatOk || !topLonOk || !bottomLatOk || !bottomLonOk ||
        !std::isfinite(topLat) || !std::isfinite(leftLon) ||
        !std::isfinite(bottomLat) || !std::isfinite(rightLon) ||
        topLat < -90.0 || topLat > 90.0 || bottomLat < -90.0 || bottomLat > 90.0 ||
        leftLon < -180.0 || leftLon > 180.0 || rightLon < -180.0 || rightLon > 180.0 ||
        topLat <= bottomLat || rightLon <= leftLon)
    {
        if (errorMessage)
            *errorMessage = QStringLiteral("Enter a valid extent with top-left and bottom-right coordinates.");
        return false;
    }

    const osgEarth::SpatialReference* geographicSrs =
        osgEarth::SpatialReference::get("epsg:4326");
    if (!geographicSrs)
    {
        if (errorMessage)
            *errorMessage = QStringLiteral("The geographic coordinate system is not available.");
        return false;
    }

    extent = osgEarth::GeoExtent(geographicSrs, leftLon, bottomLat, rightLon, topLat);
    return extent.isValid();
}

bool ViewshedByExtentPlugin::readViewerLocation(
    const osgEarth::GeoExtent& extent,
    osgEarth::GeoPoint& location,
    QString* errorMessage) const
{
    const osgEarth::SpatialReference* geographicSrs =
        osgEarth::SpatialReference::get("epsg:4326");
    if (!geographicSrs)
    {
        if (errorMessage)
            *errorMessage = QStringLiteral("The geographic coordinate system is not available.");
        return false;
    }

    location = osgEarth::GeoPoint(
        geographicSrs,
        ui.sb_longitude->value(),
        ui.sb_latitude->value(),
        ui.sb_altitude->value(),
        osgEarth::ALTMODE_ABSOLUTE);
    if (!extent.contains(location))
    {
        if (errorMessage)
            *errorMessage = QStringLiteral("The viewer location must be inside the selected extent.");
        return false;
    }
    return true;
}

void ViewshedByExtentPlugin::createOrUpdateAnalysis()
{
    osgEarth::GeoExtent extent;
    osgEarth::GeoPoint viewerLocation;
    QString errorMessage;
    if (!readExtent(extent, &errorMessage) ||
        !readViewerLocation(extent, viewerLocation, &errorMessage))
    {
        QMessageBox::warning(_dialog, QStringLiteral("Viewshed by Extent"), errorMessage);
        return;
    }

    OSGViewerWidget* mapView = App::getThreedWindow();
    osgEarth::MapNode* mapNode = mapView ? mapView->getMapNode() : nullptr;
    if (!mapNode)
    {
        QMessageBox::warning(_dialog, QStringLiteral("Viewshed by Extent"),
                             QStringLiteral("The 3D map view is not available."));
        return;
    }

    ensureExtentDrawing(mapNode);
    updateExtentRubberBand(extent);

    if (_viewshedCore)
    {
        if (!_viewshedCore->setExtent(extent) ||
            !_viewshedCore->setViewerLocation(viewerLocation))
        {
            QMessageBox::warning(_dialog, QStringLiteral("Viewshed by Extent"),
                                 QStringLiteral("Could not update the viewshed with the selected extent and viewer location."));
            return;
        }
    }
    else
    {
        std::unique_ptr<ViewshedByExtentCore> core =
            std::make_unique<ViewshedByExtentCore>(mapNode, extent, viewerLocation);
        std::string coreError;
        if (!core->buildModel(&coreError))
        {
            QMessageBox::warning(
                _dialog, QStringLiteral("Viewshed by Extent"),
                QString::fromStdString(coreError));
            return;
        }
        _viewshedCore = std::move(core);
    }

    _viewshedCore->setVisibleAreaColor(_visibleAreaColor);
    _viewshedCore->setInvisibleAreaColor(_invisibleAreaColor);
}

void ViewshedByExtentPlugin::updateViewerLocationFromFields()
{
    if (!_viewshedCore)
        return;

    osgEarth::GeoExtent extent;
    osgEarth::GeoPoint viewerLocation;
    if (readExtent(extent, nullptr) && readViewerLocation(extent, viewerLocation, nullptr))
        _viewshedCore->setViewerLocation(viewerLocation);
}

void ViewshedByExtentPlugin::updateVisibleAreaColor(const QColor& color)
{
    _visibleAreaColor.set(
        color.redF(), color.greenF(), color.blueF(),
        static_cast<float>(ui.sb_visibleAreaOpacity->value()) / 100.0f);
    if (_viewshedCore)
        _viewshedCore->setVisibleAreaColor(_visibleAreaColor);
}

void ViewshedByExtentPlugin::updateInvisibleAreaColor(const QColor& color)
{
    _invisibleAreaColor.set(
        color.redF(), color.greenF(), color.blueF(),
        static_cast<float>(ui.sb_hiddenAreaOpacity->value()) / 100.0f);
    if (_viewshedCore)
        _viewshedCore->setInvisibleAreaColor(_invisibleAreaColor);
}

void ViewshedByExtentPlugin::updateRubberBandColor()
{
    if (_rubberBandColors && !_rubberBandColors->empty())
    {
        const float alpha =
            static_cast<float>(ui.sb_boundarylineOpacity->value()) / 100.0f;
        _rubberBandColors->front().set(1.0f, 0.85f, 0.1f, alpha);
        _rubberBandColors->dirty();
    }
}

QString ViewshedByExtentPlugin::pluginName() const
{
    return QStringLiteral("Viewshed by Extent");
}

void ViewshedByExtentPlugin::initialize()
{
}

bool ViewshedByExtentPlugin::executeCommand(
    const QString& commandId,
    const QVariantMap& arguments,
    QVariantMap* result,
    QString* errorMessage)
{
    Q_UNUSED(arguments)

    if (commandId.compare(QStringLiteral("showDialog"), Qt::CaseInsensitive) != 0)
    {
        if (errorMessage)
            *errorMessage = QStringLiteral("Unsupported command: %1").arg(commandId);
        return false;
    }

    QMainWindow* parent = App::getMainwindow();
    if (!parent)
    {
        if (errorMessage)
            *errorMessage = QStringLiteral("Main window is not available.");
        return false;
    }

    if (!_dialog)
    {
        _dialog = new QDialog(parent);
        ui.setupUi(_dialog);
        _dialog->setAttribute(Qt::WA_DeleteOnClose, true);
        _visibleColorPicker = new ColorPickerCheckBox(QColor("#9FFF3D"), _dialog);
        _invisibleColorPicker = new ColorPickerCheckBox(QColor("#FF573D"), _dialog);
        ui.vl_visibleAreaColor->addWidget(_visibleColorPicker);
        ui.vl_hiddenAreaColor->addWidget(_invisibleColorPicker);
        updateVisibleAreaColor(_visibleColorPicker->color());
        updateInvisibleAreaColor(_invisibleColorPicker->color());

        QObject::connect(_visibleColorPicker, &ColorPickerCheckBox::colorChanged,
                         this, &ViewshedByExtentPlugin::updateVisibleAreaColor);
        QObject::connect(_invisibleColorPicker, &ColorPickerCheckBox::colorChanged,
                         this, &ViewshedByExtentPlugin::updateInvisibleAreaColor);
        QObject::connect(ui.sb_visibleAreaOpacity, &QSpinBox::valueChanged, this,
                         [this](int value) {
                             Q_UNUSED(value)
                             updateVisibleAreaColor(_visibleColorPicker->color());
                         });
        QObject::connect(ui.sb_hiddenAreaOpacity, &QSpinBox::valueChanged, this,
                         [this](int value) {
                             Q_UNUSED(value)
                             updateInvisibleAreaColor(_invisibleColorPicker->color());
                         });
        QObject::connect(ui.sb_boundarylineOpacity, &QSpinBox::valueChanged, this,
                         [this](int value) {
                             Q_UNUSED(value)
                             updateRubberBandColor();
                         });

        QObject::connect(ui.pushButton_selectExtent, &QPushButton::clicked, this, [this]() {
            if (_extentSelectionConnection || _locationSelectionConnection)
            {
                stopExtentSelection();
                stopLocationSelection();
                return;
            }

            OSGViewerWidget* mapView = App::getThreedWindow();
            osgEarth::MapNode* mapNode = mapView ? mapView->getMapNode() : nullptr;
            GLXEarthManipulator* manipulator = mapView ? mapView->getManipulator() : nullptr;
            if (!mapNode || !manipulator)
            {
                QMessageBox::warning(_dialog, QStringLiteral("Viewshed by Extent"),
                                     QStringLiteral("The 3D map view is not available."));
                return;
            }

            ensureExtentDrawing(mapNode);
            _selectionManipulator = manipulator;
            _hasFirstExtentCorner = false;
            manipulator->disableMouseAction(true);
            ui.pushButton_selectExtent->setText(QStringLiteral("Click first corner"));

            _extentSelectionConnection = QObject::connect(
                manipulator, &GLXEarthManipulator::mouseClickEvent, this,
                [this](const osgEarth::GeoPoint& mapPoint) {
                    if (!_hasFirstExtentCorner)
                    {
                        _firstExtentCorner = mapPoint;
                        _hasFirstExtentCorner = true;
                        _rubberBandAltitude = std::max(mapPoint.z(), 20.0) + 10.0;
                        ui.pushButton_selectExtent->setText(QStringLiteral("Click opposite corner"));
                        _extentMoveConnection = QObject::connect(
                            _selectionManipulator, &GLXEarthManipulator::mouseMoveEvent, this,
                            [this](const osgEarth::GeoPoint& currentCorner) {
                                osgEarth::GeoExtent extent;
                                if (makeExtentFromCorners(_firstExtentCorner, currentCorner, extent))
                                {
                                    _rubberBandAltitude =
                                        std::max({ _firstExtentCorner.z(), currentCorner.z(), 20.0 }) + 10.0;
                                    updateExtentRubberBand(extent);
                                }
                            });
                        return;
                    }

                    osgEarth::GeoExtent extent;
                    if (!makeExtentFromCorners(_firstExtentCorner, mapPoint, extent))
                    {
                        QMessageBox::warning(
                            _dialog, QStringLiteral("Viewshed by Extent"),
                            QStringLiteral("Select two distinct corners for the extent."));
                        return;
                    }
                    _rubberBandAltitude =
                        std::max({ _firstExtentCorner.z(), mapPoint.z(), 20.0 }) + 10.0;

                    const osgEarth::SpatialReference* geographicSrs =
                        osgEarth::SpatialReference::get("epsg:4326");
                    if (!geographicSrs)
                    {
                        QMessageBox::warning(
                            _dialog, QStringLiteral("Viewshed by Extent"),
                            QStringLiteral("The geographic coordinate system is not available."));
                        return;
                    }
                    ui.lineEdit_topLeftLat->setText(
                        QString::number(extent.north(), 'f', 6));
                    ui.lineEdit_topLeftLon->setText(
                        QString::number(extent.west(), 'f', 6));
                    ui.lineEdit_bottomRightLat->setText(
                        QString::number(extent.south(), 'f', 6));
                    ui.lineEdit_bottomRightLon->setText(
                        QString::number(extent.east(), 'f', 6));
                    updateExtentRubberBand(extent);

                    if (_viewshedCore)
                    {
                        osgEarth::GeoPoint viewerLocation;
                        if (_viewshedCore->setExtent(extent) &&
                            readViewerLocation(extent, viewerLocation, nullptr))
                        {
                            _viewshedCore->setViewerLocation(viewerLocation);
                        }
                    }
                    stopExtentSelection();
                });
        });

        QObject::connect(ui.pb_pickLocation, &QPushButton::clicked, this, [this]() {
            if (_locationSelectionConnection || _extentSelectionConnection)
            {
                stopLocationSelection();
                stopExtentSelection();
                return;
            }

            OSGViewerWidget* mapView = App::getThreedWindow();
            GLXEarthManipulator* manipulator = mapView ? mapView->getManipulator() : nullptr;
            if (!manipulator)
            {
                QMessageBox::warning(_dialog, QStringLiteral("Viewshed by Extent"),
                                     QStringLiteral("The 3D map view is not available."));
                return;
            }

            _selectionManipulator = manipulator;
            manipulator->disableMouseAction(true);
            ui.pb_pickLocation->setText(QStringLiteral("Click viewer location"));
            _locationSelectionConnection = QObject::connect(
                manipulator, &GLXEarthManipulator::mouseClickEvent, this,
                [this](const osgEarth::GeoPoint& mapPoint) {
                    const osgEarth::SpatialReference* geographicSrs =
                        osgEarth::SpatialReference::get("epsg:4326");
                    osgEarth::GeoPoint location;
                    if (!geographicSrs || !mapPoint.transform(geographicSrs, location) ||
                        !location.isValid())
                    {
                        QMessageBox::warning(
                            _dialog, QStringLiteral("Viewshed by Extent"),
                            QStringLiteral("Could not read the selected viewer location."));
                        return;
                    }

                    osgEarth::GeoExtent extent;
                    if (readExtent(extent, nullptr) && !extent.contains(location))
                    {
                        QMessageBox::warning(
                            _dialog, QStringLiteral("Viewshed by Extent"),
                            QStringLiteral("The viewer location must be inside the selected extent."));
                        return;
                    }

                    {
                        const QSignalBlocker blockLongitude(ui.sb_longitude);
                        const QSignalBlocker blockLatitude(ui.sb_latitude);
                        const QSignalBlocker blockAltitude(ui.sb_altitude);
                        ui.sb_longitude->setValue(location.x());
                        ui.sb_latitude->setValue(location.y());
                        ui.sb_altitude->setValue(location.z());
                    }
                    updateViewerLocationFromFields();
                    stopLocationSelection();
                });
        });

        QObject::connect(ui.pb_createORupdate, &QPushButton::clicked,
                         this, &ViewshedByExtentPlugin::createOrUpdateAnalysis);
        QObject::connect(ui.pushButton_removeNode, &QPushButton::clicked, this, [this]() {
            stopExtentSelection();
            stopLocationSelection();
            _viewshedCore.reset();
            removeExtentDrawing();
            ui.lineEdit_topLeftLat->clear();
            ui.lineEdit_topLeftLon->clear();
            ui.lineEdit_bottomRightLat->clear();
            ui.lineEdit_bottomRightLon->clear();
        });
        const auto viewerPositionChanged = [this](double value) {
            Q_UNUSED(value)
            updateViewerLocationFromFields();
        };
        QObject::connect(ui.sb_longitude, &QDoubleSpinBox::valueChanged,
                         this, viewerPositionChanged);
        QObject::connect(ui.sb_latitude, &QDoubleSpinBox::valueChanged,
                         this, viewerPositionChanged);
        QObject::connect(ui.sb_altitude, &QDoubleSpinBox::valueChanged,
                         this, viewerPositionChanged);
        QObject::connect(_dialog, &QDialog::finished, this, [this]() {
            stopExtentSelection();
            stopLocationSelection();
        });
        QObject::connect(_dialog, &QObject::destroyed, this, [this]() {
            stopExtentSelection(false);
            stopLocationSelection(false);
            _dialog = nullptr;
            _visibleColorPicker = nullptr;
            _invisibleColorPicker = nullptr;
        });
    }

    _dialog->adjustSize();
    _dialog->show();
    _dialog->raise();
    _dialog->activateWindow();

    if (result)
        (*result)[QStringLiteral("dialogVisible")] = true;
    return true;
}
