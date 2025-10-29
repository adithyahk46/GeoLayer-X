#include "RadarDomSimulationWidget.h"
#include "ui_RadarDomSimulationWidget.h"
#include <osgEarth/GeoTransform>
#include <osgEarth/SpatialReference>
#include <osg/ShapeDrawable>
#include <osg/StateSet>
#include <QDebug>

RadarDomSimulationWidget::RadarDomSimulationWidget(osgEarth::MapNode* mapNode, QWidget *parent)
    : QWidget(parent),
      ui(new Ui::RadarDomSimulationWidget),
      _mapNode(mapNode),
      _glowIncreasing(true),
      _glowValue(0.4f),
       _geoSRS(osgEarth::SpatialReference::get("wgs84"))
{
    ui->setupUi(this);
    connect(&_glowTimer, &QTimer::timeout, this, &RadarDomSimulationWidget::glowUpdate);
}

RadarDomSimulationWidget::~RadarDomSimulationWidget()
{
    removeRadarDome();
    delete ui;
}

#include <MouseEventHandler.h>
void RadarDomSimulationWidget::on_pushButton_select_clicked()
{
    auto handler = MouseEventHandler::Instance();

    // First, make sure no previous connection lingers
    QObject::disconnect(handler, &MouseEventHandler::mouseClickEvent, this, nullptr);

    // Now connect and auto-disconnect after one click
    QObject::connect(handler, &MouseEventHandler::mouseClickEvent, this,
        [=](const double lon, const double lat, const double alt,
            const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
    {
        ui->lineEdit_lon->setText(QString::number(lon, 'f', 6));
        ui->lineEdit_lat->setText(QString::number(lat, 'f', 6));
        ui->lineEdit_alt->setText(QString::number(alt, 'f', 2));

        // Disconnect after one successful capture
        QObject::disconnect(handler, &MouseEventHandler::mouseClickEvent, this, nullptr);
    });
}

void RadarDomSimulationWidget::on_pushButton_update_clicked()
{
    // create geode for LOS lines
    _losGeode = new osg::Geode();
    _mapNode->addChild(_losGeode.get());

    removeRadarDome();
    createRadarDome();
}

void RadarDomSimulationWidget::on_pushButton_start_clicked()
{
    if (!_radarGeode.valid()) createRadarDome();
    _glowTimer.start(100);
    emit simulationStarted();
}

void RadarDomSimulationWidget::on_pushButton_stop_clicked()
{
    _glowTimer.stop();
    emit simulationStopped();
}

void RadarDomSimulationWidget::on_pushButton_clear_clicked()
{
    removeRadarDome();
    resetDefaults();
}

void RadarDomSimulationWidget::createRadarDome()
{
    if (!_mapNode) return;

    double lon = ui->lineEdit_lon->text().toDouble();
    double lat = ui->lineEdit_lat->text().toDouble();
    double range = ui->lineEdit_range->text().toDouble() * 1000.0;

    osg::ref_ptr<osg::Sphere> sphere = new osg::Sphere(osg::Vec3(), range);
    osg::ref_ptr<osg::ShapeDrawable> drawable = new osg::ShapeDrawable(sphere);
    drawable->setColor(osg::Vec4(0.0f, 1.0f, 0.0f, _glowValue));

    _radarGeode = new osg::Geode();
    _radarGeode->addDrawable(drawable);
    _radarGeode->getOrCreateStateSet()->setMode(GL_BLEND, osg::StateAttribute::ON);
    _radarGeode->getOrCreateStateSet()->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

    const osgEarth::SpatialReference* srs = osgEarth::SpatialReference::get("wgs84");
    xform = new osgEarth::GeoTransform();
    xform->setPosition(osgEarth::GeoPoint(srs, lon, lat, 0.0, osgEarth::ALTMODE_RELATIVE));
    xform->addChild(_radarGeode);
    _mapNode->addChild(xform);

    add3DModelToMap();

}

#include <osgDB/ReadFile>
#include <osg/MatrixTransform>
#include <osgEarth/GeoTransform>
#include <osgEarth/MapNode>
#include <osgEarth/ModelLayer>
#include <osgEarth/SpatialReference>

void RadarDomSimulationWidget::add3DModelToMap()
{
    if (!_mapNode) return;

    // --- Position from UI ---
    double lon = ui->lineEdit_lon->text().toDouble();
    double lat = ui->lineEdit_lat->text().toDouble();
    double alt = ui->lineEdit_alt->text().toDouble();

    // --- Load 3D model ---
    QString modelpath = QCoreApplication::applicationDirPath() + "/Data/3DModels/radar_tower/radr_tower.obj";

    model = osgDB::readNodeFile(modelpath.toStdString());
    if (!model)
    {
        qWarning("Failed to load 3D model!");
        return;
    }

    // --- Create a single transform for rotation + scaling ---
    osg::ref_ptr<osg::MatrixTransform> modelXform = new osg::MatrixTransform();

    double scaleFactor = 10.0;   // scale factor
    double headingDeg  = 45.0;   // rotation in degrees

    // Combine scale + rotation into one matrix
    osg::Matrix transformMatrix =
        osg::Matrix::rotate(osg::DegreesToRadians(headingDeg), osg::Z_AXIS) *
        osg::Matrix::scale(scaleFactor, scaleFactor, scaleFactor);

    modelXform->setMatrix(transformMatrix);
    modelXform->addChild(model);

    // --- Geographic placement using GeoTransform ---
    const osgEarth::SpatialReference* srs = osgEarth::SpatialReference::get("wgs84");
    geoXform = new osgEarth::GeoTransform();
    geoXform->setPosition(osgEarth::GeoPoint(srs, lon, lat, alt, osgEarth::ALTMODE_RELATIVE));
    geoXform->addChild(modelXform);

    // --- Add to the map ---
    _mapNode->addChild(geoXform);

    // osg::ref_ptr<osgEarth::ModelLayer> modelLayer = new osgEarth::ModelLayer();
    // modelLayer->setNode(geoXform);

    // // Position the model
    // osgEarth::GeoPoint location(
    //     mapNode->getMapSRS(),
    //     77.59, 12.97);

    // modelLayer->setLocation(location);

    // // Add to the map
    // mapNode->getMap()->addLayer(modelLayer);

    // // Optionally store references
    // _modelNode = model;
    // _modelXform = modelXform;
    // _geoXform = geoXform;
}


void RadarDomSimulationWidget::removeRadarDome()
{
    if (_mapNode && _radarGeode.valid())
    {
        _mapNode->removeChild(xform);
        _mapNode->removeChild(geoXform);
        xform = nullptr;
    }
}

void RadarDomSimulationWidget::resetDefaults()
{

    ui->lineEdit_lon->setText("75.000000");
    ui->lineEdit_lat->setText("22.000000");
    ui->lineEdit_alt->setText("100.0");
    ui->lineEdit_az->setText("0.0");
    ui->lineEdit_el->setText("15.0");
    ui->lineEdit_freq->setText("9.4");
    ui->lineEdit_range->setText("50.0");
    ui->lineEdit_bw->setText("2.0");
    ui->lineEdit_rspeed->setText("15.0");
    ui->lineEdit_power->setText("100.0");
    ui->lineEdit_gain->setText("30.0");
}

void RadarDomSimulationWidget::glowUpdate()
{
    if (!_radarGeode.valid()) return;
    _glowValue += (_glowIncreasing ? 0.05f : -0.05f);
    if (_glowValue > 1.0f) { _glowValue = 1.0f; _glowIncreasing = false; }
    if (_glowValue < 0.3f) { _glowValue = 0.3f; _glowIncreasing = true; }

    osg::ShapeDrawable* sd = dynamic_cast<osg::ShapeDrawable*>(_radarGeode->getDrawable(0));
    if (sd) sd->setColor(osg::Vec4(0.0f, 1.0f, 0.0f, _glowValue));
}

void RadarDomSimulationWidget::on_pushButton_clicked()
{

    emit closeWindowClicked();
}

bool RadarDomSimulationWidget::computeLineOfSight(double observerLon, double observerLat, double observerAlt,
                                                  double targetLon, double targetLat, double targetAlt,
                                                  std::vector<osg::Vec3d>& visiblePts,
                                                  std::vector<osg::Vec3d>& blockedPts)
{
    // Simplified: sample N points along the line from observer→target, check terrain elevation at each.
    // const int N = 100;
    // visiblePts.clear();
    // blockedPts.clear();

    // osg::Vec3d obs = _geoSRS->transform(osg::Vec3d(observerLon, observerLat, observerAlt),
    //                                    _geoSRS);
    // osg::Vec3d tgt = _geoSRS->transform(osg::Vec3d(targetLon, targetLat, targetAlt),
    //                                    _geoSRS);

    // for(int i=0;i<=N;++i)
    // {
    //     double t = double(i) / double(N);
    //     osg::Vec3d pos = obs + (tgt - obs) * t;
    //     // convert pos back to lat/lon/height
    //     osg::Vec3d geo = _geoSRS->transform(pos, _geoSRS);
    //     double lon = geo.x(), lat = geo.y(), height = geo.z();
    //     // Query terrain elevation at (lon, lat)
    //     double terrainElev = 0.0;
    //     osgEarth::ElevationQuery::Result result;
    //     osgEarth::ElevationQuery eq(*_mapNode->getMap(), lon, lat);
    //     result = eq.run();
    //     if(result.success())
    //         terrainElev = result.elevation();

    //     // Compute height of line at this fraction: linear interpolation
    //     double lineHeight = observerAlt + t * (targetAlt - observerAlt);

    //     if(terrainElev > lineHeight)
    //     {
    //         // blocked at this point
    //         blockedPts.push_back(pos);
    //         // remain points go into blocked
    //         for(int j=i+1;j<=N;++j)
    //         {
    //             double tt = double(j)/double(N);
    //             osg::Vec3d pos2 = obs + tt * (tgt - obs);
    //             blockedPts.push_back(pos2);
    //         }
    //         return false;
    //     }
    //     else
    //     {
    //         visiblePts.push_back(pos);
    //     }
    // }

    return true; // full visible
}

void RadarDomSimulationWidget::drawLineSegment(const std::vector<osg::Vec3d>& pts, osg::Vec4 color)
{
    if(pts.empty()) return;

    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry();
    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array();
    for(auto& p : pts) vertices->push_back(p);
    geom->setVertexArray(vertices.get());
    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINE_STRIP, 0, vertices->size()));

    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array();
    colors->push_back(color);
    geom->setColorArray(colors.get(), osg::Array::BIND_OVERALL);

    osg::ref_ptr<osg::StateSet> ss = geom->getOrCreateStateSet();
    ss->setMode(GL_BLEND, osg::StateAttribute::ON);
    ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);

    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    geode->addDrawable(geom.get());
    _losGeode->addChild(geode.get());
}

void RadarDomSimulationWidget::updateLOSVisualization(double obsLon, double obsLat, double obsAlt,
                                                     double tgtLon, double tgtLat, double tgtAlt)
{
    _losGeode->removeChildren(0, _losGeode->getNumChildren());

    std::vector<osg::Vec3d> visiblePts, blockedPts;
    bool fullVis = computeLineOfSight(obsLon, obsLat, obsAlt, tgtLon, tgtLat, tgtAlt, visiblePts, blockedPts);

    drawLineSegment(visiblePts, osg::Vec4(0.0f,1.0f,0.0f,1.0f));
    if(!fullVis)
    {
        drawLineSegment(blockedPts, osg::Vec4(1.0f,0.0f,0.0f,1.0f));
    }
}
