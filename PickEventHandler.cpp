#include "PickEventHandler.h"
#include <iostream>

#include <QHBoxLayout>

PickEventHandler::PickEventHandler(QStatusBar *StatusBar, osgEarth::MapNode *mapNode)
    : _statusBar(StatusBar), _mapNode(mapNode)
{
    QWidget* container = new QWidget;
    QHBoxLayout* layout = new QHBoxLayout(container);
    layout->setContentsMargins(8, 0, 8, 0); // Remove margins for status bar fit
    layout->addStretch();

    // _mapNode->get

    // Create and add latitude input
    layout->addWidget(new QLabel("Lat:"));
    _lat = new QLineEdit;
    _lat->setReadOnly(true);
    //_lat->setFixedWidth(100);
    layout->addWidget(_lat);

    // Create and add longitude input
    layout->addWidget(new QLabel("Long:"));
    _long = new QLineEdit;
    _long->setReadOnly(true);
   // _long->setFixedWidth(100);
    layout->addWidget(_long);

    // Create and add height input
    layout->addWidget(new QLabel("Hei:"));
    _height = new QLineEdit;
    _height->setReadOnly(true);
    //_height->setFixedWidth(100);
    layout->addWidget(_height);

    // Create and add CRS combo box
    layout->addWidget(new QLabel("CRS:"));
    _CRS = new QComboBox;
    _CRS->addItem("WGS84(EPSG:4326)");
    _CRS->addItem("Web Mercator(EPSG:3857");
    layout->addWidget(_CRS);

    // Add the container to the status bar
    _statusBar->addPermanentWidget(container);
}


bool PickEventHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
    if (!_mapNode  || ea.getEventType() != osgGA::GUIEventAdapter::MOVE)
        return false;

    osgViewer::View* view = dynamic_cast<osgViewer::View*>(&aa);
    if (!view)
        return false;

    double lat, lon, hei;
    if (getLatLonFromScreen(view, ea.getX(), ea.getY(), lat, lon, hei))
    {
        QString coords = QString("Lat: %1, Lon: %2").arg(lat, 0, 'f', 6).arg(lon, 0, 'f', 6);
        //_label->setText(coords); // Update QLabel
    }

    return false;
}

bool PickEventHandler::getLatLonFromScreen(osgViewer::View* view, float x, float y, double& lat, double& lon, double& height)
{
    osgUtil::LineSegmentIntersector::Intersections intersections;
    if (view->computeIntersections(x, y, intersections))
    {
        auto hit = *intersections.begin();
        osg::Vec3d world = hit.getWorldIntersectPoint();

        osgEarth::GeoPoint mapPoint;
        mapPoint.fromWorld(_mapNode->getMapSRS(), world);

        const osgEarth::SpatialReference* srs4326 = osgEarth::SpatialReference::get("epsg:4326");
        const osgEarth::SpatialReference* srs3857 = osgEarth::SpatialReference::get("epsg:3857");

        osgEarth::GeoPoint transformed;
        if ( _CRS->currentIndex() == 0) {
            mapPoint.transform(srs4326, transformed);
        } else {
            mapPoint.transform(srs3857, transformed);
        }

        lon = transformed.x();
        lat = transformed.y();
        height = transformed.z();

        QString latUnit = (transformed.getSRS()->isGeographic()) ? "°" : "m";
        QString lonUnit = (transformed.getSRS()->isGeographic()) ? "°" : "m";

        QString latText = QString("%1 %2").arg(lat, 0, 'f', 4).arg(latUnit);
        QString lonText = QString("%1 %2").arg(lon, 0, 'f', 4).arg(lonUnit);
        QString heightText = QString("%1 m").arg(height, 0, 'f', 2);

        // Update the UI fields
        _lat->setText(latText);
        _long->setText(lonText);
        _height->setText(heightText);

        return true;
    }
    return false;
}
