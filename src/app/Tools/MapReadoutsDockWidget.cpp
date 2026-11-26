#include "MapReadoutsDockWidget.h"

#include "App.h"
#include "MapController/GLXEarthManipulator.h"
#include "osgviewerwidget.h"

#include <QFormLayout>
#include <QGridLayout>
#include <QFrame>
#include <QFont>
#include <QGroupBox>
#include <QHideEvent>
#include <QLabel>
#include <QScrollArea>
#include <QShowEvent>
#include <QTimer>
#include <QVBoxLayout>

#include <osg/Camera>
#include <osgEarth/Map>
#include <osgEarth/SpatialReference>
#include <osgEarth/Units>
#include <osgEarth/Viewpoint>
#include <osgViewer/Viewer>

#include <cmath>

namespace
{
/**
 * Formats a finite numeric value and appends its display unit.
 */
QString formattedValue(double value, int precision, const QString& suffix)
{
    if (!std::isfinite(value))
        return QStringLiteral("--");

    return QStringLiteral("%1%2").arg(value, 0, 'f', precision).arg(suffix);
}
}

MapReadoutsDockWidget::MapReadoutsDockWidget(QWidget* parent)
    : DockWidgetPanels(parent)
{
    setWindowTitle(tr("Map Readouts"));
    setObjectName(QStringLiteral("MapReadoutsDock"));
    buildInterface();

    _updateTimer = new QTimer(this);
    _updateTimer->setInterval(250);
    connect(_updateTimer, &QTimer::timeout,
            this, &MapReadoutsDockWidget::updateReadouts);

    if (GLXEarthManipulator* manipulator = currentManipulator())
    {
        connect(manipulator, &GLXEarthManipulator::mouseMoveEvent,
                this, &MapReadoutsDockWidget::updateMouseReadouts);
        connect(manipulator, &GLXEarthManipulator::mousePositionLostEvent,
                this, &MapReadoutsDockWidget::clearMouseReadouts);
    }

    updateReadouts();
}

MapReadoutsDockWidget::~MapReadoutsDockWidget() = default;

void MapReadoutsDockWidget::setWindowTitle(const QString& title)
{
    QDockWidget::setWindowTitle(title);
}

void MapReadoutsDockWidget::setObjectName(const QString& name)
{
    QDockWidget::setObjectName(name);
}

void MapReadoutsDockWidget::showEvent(QShowEvent* event)
{
    DockWidgetPanels::showEvent(event);
    updateReadouts();
    _updateTimer->start();
}

void MapReadoutsDockWidget::hideEvent(QHideEvent* event)
{
    _updateTimer->stop();
    DockWidgetPanels::hideEvent(event);
}

void MapReadoutsDockWidget::buildInterface()
{
    auto* content = new QWidget;
    auto* layout = new QVBoxLayout(content);
    layout->setContentsMargins(5, 5, 5, 5);
    layout->setSpacing(4);

    const auto makePropertyLabel = [](const QString& text) {
        auto* label = new QLabel(text);
        QFont font = label->font();
        font.setBold(true);
        label->setFont(font);
        return label;
    };

    const auto makeValueLabel = [] {
        auto* value = new QLabel(QStringLiteral("--"));
        value->setWordWrap(true);
        value->setMinimumWidth(0);
        value->setTextInteractionFlags(Qt::TextSelectableByMouse);
        return value;
    };

    const auto addCoordinateGrid = [&makePropertyLabel, &makeValueLabel](
        QGridLayout* grid,
        std::array<QLabel*, 3>& geographicValues,
        std::array<QLabel*, 3>& localValues) {
        grid->setContentsMargins(7, 9, 7, 6);
        grid->setHorizontalSpacing(6);
        grid->setVerticalSpacing(2);
        for (int column = 0; column < 3; ++column)
            grid->setColumnStretch(column, 1);

        grid->addWidget(makePropertyLabel(QObject::tr("Geographic")), 0, 0, 1, 3);
        grid->addWidget(makePropertyLabel(QObject::tr("Lat")), 1, 0);
        grid->addWidget(makePropertyLabel(QObject::tr("Lon")), 1, 1);
        grid->addWidget(makePropertyLabel(QObject::tr("Alt")), 1, 2);
        for (int column = 0; column < 3; ++column)
        {
            geographicValues[static_cast<std::size_t>(column)] = makeValueLabel();
            grid->addWidget(geographicValues[static_cast<std::size_t>(column)], 2, column);
        }

        grid->addWidget(makePropertyLabel(QObject::tr("OSG World")), 3, 0, 1, 3);
        grid->addWidget(makePropertyLabel(QStringLiteral("X")), 4, 0);
        grid->addWidget(makePropertyLabel(QStringLiteral("Y")), 4, 1);
        grid->addWidget(makePropertyLabel(QStringLiteral("Z")), 4, 2);
        for (int column = 0; column < 3; ++column)
        {
            localValues[static_cast<std::size_t>(column)] = makeValueLabel();
            grid->addWidget(localValues[static_cast<std::size_t>(column)], 5, column);
        }
    };

    auto* viewpointGroup = new QGroupBox(tr("Viewpoint"), content);
    auto* viewpointLayout = new QGridLayout(viewpointGroup);
    addCoordinateGrid(viewpointLayout, _viewGeoValues, _viewLocalValues);
    layout->addWidget(viewpointGroup);

    auto* cameraGroup = new QGroupBox(tr("Camera"), content);
    auto* cameraLayout = new QGridLayout(cameraGroup);
    cameraLayout->setContentsMargins(7, 9, 7, 6);
    cameraLayout->setHorizontalSpacing(8);
    cameraLayout->setVerticalSpacing(3);
    const auto addStackedValue = [&makePropertyLabel, &makeValueLabel](
                                     QGridLayout* grid, int row, int column,
                                     const QString& property) {
        auto* value = makeValueLabel();
        grid->addWidget(makePropertyLabel(property), row * 2, column);
        grid->addWidget(value, row * 2 + 1, column);
        return value;
    };
    cameraLayout->setColumnStretch(0, 1);
    cameraLayout->setColumnStretch(1, 1);
    _headingValue = addStackedValue(cameraLayout, 0, 0, tr("Heading"));
    _pitchValue = addStackedValue(cameraLayout, 0, 1, tr("Pitch"));
    _heightValue = addStackedValue(cameraLayout, 1, 0, tr("Height"));
    _rangeValue = addStackedValue(cameraLayout, 1, 1, tr("Range"));
    layout->addWidget(cameraGroup);

    auto* mouseGroup = new QGroupBox(tr("Mouse position"), content);
    auto* mouseLayout = new QGridLayout(mouseGroup);
    addCoordinateGrid(mouseLayout, _mouseGeoValues, _mouseLocalValues);
    layout->addWidget(mouseGroup);

    auto* referenceGroup = new QGroupBox(tr("Spatial reference"), content);
    auto* referenceLayout = new QFormLayout(referenceGroup);
    referenceLayout->setContentsMargins(7, 9, 7, 6);
    referenceLayout->setHorizontalSpacing(8);
    referenceLayout->setVerticalSpacing(3);
    _srsValue = makeValueLabel();
    _projectionValue = makeValueLabel();
    _datumValue = makeValueLabel();
    referenceLayout->addRow(makePropertyLabel(tr("SRS")), _srsValue);
    referenceLayout->addRow(makePropertyLabel(tr("Projection")), _projectionValue);
    referenceLayout->addRow(makePropertyLabel(tr("Datum")), _datumValue);
    layout->addWidget(referenceGroup);

    auto* scaleGroup = new QGroupBox(tr("Map scale"), content);
    auto* scaleLayout = new QGridLayout(scaleGroup);
    scaleLayout->setContentsMargins(7, 9, 7, 6);
    scaleLayout->setHorizontalSpacing(8);
    scaleLayout->setVerticalSpacing(3);
    scaleLayout->setColumnStretch(0, 1);
    scaleLayout->setColumnStretch(1, 1);
    _scaleValue = addStackedValue(scaleLayout, 0, 0, tr("Scale"));
    _resolutionValue = addStackedValue(scaleLayout, 0, 1, tr("Resolution"));
    _viewModeValue = addStackedValue(scaleLayout, 1, 0, tr("View mode"));
    layout->addWidget(scaleGroup);

    auto* terrainGroup = new QGroupBox(tr("Terrain"), content);
    auto* terrainLayout = new QGridLayout(terrainGroup);
    terrainLayout->setContentsMargins(7, 9, 7, 6);
    terrainLayout->setHorizontalSpacing(10);
    terrainLayout->setVerticalSpacing(2);
    const auto addTerrainValue = [terrainLayout, &makePropertyLabel, &makeValueLabel](
                                     int column, const QString& property) {
        terrainLayout->addWidget(makePropertyLabel(property), 0, column);
        auto* value = makeValueLabel();
        terrainLayout->addWidget(value, 1, column);
        return value;
    };
    _terrainElevationValue = addTerrainValue(0, tr("Elevation"));
    _slopeValue = addTerrainValue(1, tr("Slope"));
    _aspectValue = addTerrainValue(2, tr("Aspect"));
    layout->addWidget(terrainGroup);

    auto* scrollArea = new QScrollArea(this);
    scrollArea->setFrameShape(QFrame::NoFrame);
    scrollArea->setWidgetResizable(true);
    scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scrollArea->setWidget(content);
    setWidget(scrollArea);
    setMinimumWidth(220);
}

void MapReadoutsDockWidget::setCoordinateValues(
    const osgEarth::GeoPoint& point,
    const std::array<QLabel*, 3>& geographicValues,
    const std::array<QLabel*, 3>& localValues)
{
    osgEarth::MapNode* mapNode = currentMapNode();
    const osgEarth::SpatialReference* mapSRS = mapNode ? mapNode->getMapSRS() : nullptr;
    if (!point.isValid() || !mapSRS)
    {
        for (QLabel* label : geographicValues)
            label->setText(QStringLiteral("--"));
        for (QLabel* label : localValues)
            label->setText(QStringLiteral("--"));
        return;
    }

    osgEarth::GeoPoint geographicPoint;
    if (point.transform(mapSRS->getGeographicSRS(), geographicPoint))
    {
        geographicValues[0]->setText(formattedValue(
            geographicPoint.y(), 6, QStringLiteral("°")));
        geographicValues[1]->setText(formattedValue(
            geographicPoint.x(), 6, QStringLiteral("°")));
        geographicValues[2]->setText(formattedValue(
            geographicPoint.z(), 2, QStringLiteral(" m")));
    }
    else
    {
        for (QLabel* label : geographicValues)
            label->setText(QStringLiteral("--"));
    }

    osg::Vec3d worldPoint;
    if (point.toWorld(worldPoint, mapNode->getTerrain()))
    {
        localValues[0]->setText(formattedValue(worldPoint.x(), 2, QStringLiteral(" m")));
        localValues[1]->setText(formattedValue(worldPoint.y(), 2, QStringLiteral(" m")));
        localValues[2]->setText(formattedValue(worldPoint.z(), 2, QStringLiteral(" m")));
    }
    else
    {
        for (QLabel* label : localValues)
            label->setText(QStringLiteral("--"));
    }
}

osgEarth::MapNode* MapReadoutsDockWidget::currentMapNode() const
{
    OSGViewerWidget* widget = App::getThreedWindow();
    osgViewer::Viewer* viewer = widget ? widget->getViewer() : nullptr;
    osg::Node* scene = viewer ? viewer->getSceneData() : nullptr;
    osgEarth::MapNode* mapNode = scene ? osgEarth::MapNode::findMapNode(scene) : nullptr;
    return mapNode ? mapNode : (widget ? widget->getMapNode() : nullptr);
}

GLXEarthManipulator* MapReadoutsDockWidget::currentManipulator() const
{
    OSGViewerWidget* widget = App::getThreedWindow();
    return widget ? widget->getManipulator() : nullptr;
}

void MapReadoutsDockWidget::updateMouseReadouts(osgEarth::GeoPoint mapPoint)
{
    setCoordinateValues(mapPoint, _mouseGeoValues, _mouseLocalValues);

    _terrainElevationValue->setText(formattedValue(mapPoint.z(), 2, QStringLiteral(" m")));
    _slopeValue->setText(QStringLiteral("--"));
    _aspectValue->setText(QStringLiteral("--"));

    osgEarth::MapNode* mapNode = currentMapNode();
    GLXEarthManipulator* manipulator = currentManipulator();
    const osgEarth::SpatialReference* mapSRS = mapNode ? mapNode->getMapSRS() : nullptr;
    if (!mapSRS || !manipulator)
        return;

    osg::Matrixd localToWorld;
    if (!mapSRS->createLocalToWorld(mapPoint.vec3d(), localToWorld))
        return;

    osg::Vec3d normal = manipulator->currentWorldNormal();
    if (normal.length2() < 1e-12)
        return;
    normal.normalize();

    osg::Vec3d east = osg::Matrixd::transform3x3(osg::Vec3d(1.0, 0.0, 0.0), localToWorld);
    osg::Vec3d north = osg::Matrixd::transform3x3(osg::Vec3d(0.0, 1.0, 0.0), localToWorld);
    osg::Vec3d up = osg::Matrixd::transform3x3(osg::Vec3d(0.0, 0.0, 1.0), localToWorld);
    east.normalize();
    north.normalize();
    up.normalize();

    double eastComponent = normal * east;
    double northComponent = normal * north;
    double upComponent = normal * up;
    if (upComponent < 0.0)
    {
        eastComponent = -eastComponent;
        northComponent = -northComponent;
        upComponent = -upComponent;
    }

    const double slope = std::atan2(
        std::sqrt(eastComponent * eastComponent + northComponent * northComponent),
        upComponent) * 180.0 / osg::PI;
    _slopeValue->setText(formattedValue(slope, 1, QStringLiteral("°")));

    if (std::hypot(eastComponent, northComponent) > 1e-8)
    {
        double aspect = std::atan2(-eastComponent, -northComponent) * 180.0 / osg::PI;
        if (aspect < 0.0)
            aspect += 360.0;
        _aspectValue->setText(formattedValue(aspect, 1, QStringLiteral("°")));
    }
}

void MapReadoutsDockWidget::clearMouseReadouts()
{
    for (QLabel* label : _mouseGeoValues)
        label->setText(QStringLiteral("--"));
    for (QLabel* label : _mouseLocalValues)
        label->setText(QStringLiteral("--"));
    _terrainElevationValue->setText(QStringLiteral("--"));
    _slopeValue->setText(QStringLiteral("--"));
    _aspectValue->setText(QStringLiteral("--"));
}

void MapReadoutsDockWidget::updateReadouts()
{
    osgEarth::MapNode* mapNode = currentMapNode();
    GLXEarthManipulator* manipulator = currentManipulator();
    if (!mapNode || !manipulator)
        return;

    const osgEarth::SpatialReference* mapSRS = mapNode->getMapSRS();
    if (mapSRS)
    {
        _srsValue->setText(QString::fromStdString(mapSRS->getName()));
        QString projection;
        if (mapSRS->isGeographic())
            projection = tr("Geographic");
        else if (mapSRS->isProjected())
            projection = tr("Projected");
        else if (mapSRS->isGeocentric())
            projection = tr("Geocentric");
        else
            projection = tr("Other");

        const std::string& initString = mapSRS->getHorizInitString();
        if (!initString.empty())
        {
            QString definition = QString::fromStdString(initString);
            if (definition.size() > 48)
                definition = definition.left(45) + QStringLiteral("...");
            projection += QStringLiteral(" (%1)").arg(definition);
        }
        _projectionValue->setText(projection);

        const std::string& datumName = mapSRS->getDatumName();
        _datumValue->setText(datumName.empty()
                                 ? tr("Unknown")
                                 : QString::fromStdString(datumName));
    }

    const osgEarth::Viewpoint viewpoint = manipulator->currentViewpoint();
    if (viewpoint.focalPoint().isSet())
        setCoordinateValues(*viewpoint.focalPoint(), _viewGeoValues, _viewLocalValues);
    else
    {
        for (QLabel* label : _viewGeoValues)
            label->setText(QStringLiteral("--"));
        for (QLabel* label : _viewLocalValues)
            label->setText(QStringLiteral("--"));
    }

    double rangeMeters = 0.0;
    double pitchDegrees = 0.0;
    if (viewpoint.range().isSet())
    {
        rangeMeters = viewpoint.range()->as(osgEarth::Units::METERS);
        _rangeValue->setText(formattedValue(rangeMeters, 1, QStringLiteral(" m")));
    }
    else
    {
        _rangeValue->setText(QStringLiteral("--"));
    }

    if (viewpoint.pitch().isSet())
    {
        pitchDegrees = viewpoint.pitch()->as(osgEarth::Units::DEGREES);
        _pitchValue->setText(formattedValue(pitchDegrees, 1, QStringLiteral("°")));
    }
    else
    {
        _pitchValue->setText(QStringLiteral("--"));
    }

    if (viewpoint.heading().isSet())
        _headingValue->setText(formattedValue(
            viewpoint.heading()->as(osgEarth::Units::DEGREES), 1, QStringLiteral("°")));
    else
        _headingValue->setText(QStringLiteral("--"));

    const double heightMeters = rangeMeters *
        std::sin(std::abs(pitchDegrees) * osg::PI / 180.0);
    _heightValue->setText(viewpoint.range().isSet() && viewpoint.pitch().isSet()
                              ? formattedValue(heightMeters, 1, QStringLiteral(" m"))
                              : QStringLiteral("--"));

    _viewModeValue->setText(mapNode->isGeocentric() ? tr("3D") : tr("2D"));
    _scaleValue->setText(QStringLiteral("--"));
    _resolutionValue->setText(QStringLiteral("--"));

    OSGViewerWidget* widget = App::getThreedWindow();
    osgViewer::Viewer* viewer = widget ? widget->getViewer() : nullptr;
    osg::Camera* camera = viewer ? viewer->getCamera() : nullptr;
    osg::Viewport* viewport = camera ? camera->getViewport() : nullptr;
    double fieldOfView = 0.0;
    double aspectRatio = 0.0;
    double nearPlane = 0.0;
    double farPlane = 0.0;
    if (rangeMeters > 0.0 && viewpoint.pitch().isSet() && camera && viewport &&
        viewport->height() > 0.0 &&
        camera->getProjectionMatrixAsPerspective(fieldOfView, aspectRatio, nearPlane, farPlane))
    {
        const double groundResolution =
            heightMeters * 2.0 * std::tan(fieldOfView * osg::PI / 360.0) /
            viewport->height();
        const double dpi = widget->logicalDpiX() * widget->devicePixelRatioF();
        const double scaleDenominator = groundResolution * dpi / 0.0254;
        if (std::isfinite(groundResolution) && groundResolution > 0.0 &&
            std::isfinite(scaleDenominator) && scaleDenominator > 0.0)
        {
            _resolutionValue->setText(
                formattedValue(groundResolution, 3, QStringLiteral(" m/px")));
            _scaleValue->setText(
                QStringLiteral("1:%1").arg(static_cast<qulonglong>(std::llround(scaleDenominator))));
        }
    }
}