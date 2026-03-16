#include "StatusBarHandler.h"

#include "app/MouseEventHandler.h"
#include "app/app.h"

#include <osgGA/GUIEventHandler>


StatusBarHandler* StatusBarHandler::_statusbar = nullptr;

StatusBarHandler *StatusBarHandler::getInstance()
{
    if(!_statusbar){
        _statusbar = new StatusBarHandler;
    }

    return _statusbar;
}

void StatusBarHandler::CreatMapReaders()
{
    //status bar initialization/ setup
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
            layout->addWidget(new QLabel("Lon:"));
            _long = new QLineEdit;
            _long->setReadOnly(true);
           // _long->setFixedWidth(100);
            layout->addWidget(_long);

            // Create and add height input
            layout->addWidget(new QLabel("alt:"));
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
            _sbWidget->addPermanentWidget(container);

            QObject::connect(MouseEventHandler::Instance(),&MouseEventHandler::mouseMoveEvent, this ,[=](const double lon, const double lat,const double alt,const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa){
                osgUtil::LineSegmentIntersector::Intersections intersections;
                if (App::getInstance()->getOsgViewer()->computeIntersections(ea.getX(), ea.getY(), intersections))
                {
                    auto hit = *intersections.begin();
                    osg::Vec3d world = hit.getWorldIntersectPoint();

                    osgEarth::GeoPoint mapPoint;
                    mapPoint.fromWorld(App::getInstance()->getMapNode()->getMapSRS(), world);

                    const osgEarth::SpatialReference* srs4326 = osgEarth::SpatialReference::get("epsg:4326");
                    const osgEarth::SpatialReference* srs3857 = osgEarth::SpatialReference::get("epsg:3857");

                    osgEarth::GeoPoint transformed;
                    if ( _CRS->currentIndex() == 0) {
                        mapPoint.transform(srs4326, transformed);
                    } else {
                        mapPoint.transform(srs3857, transformed);
                    }

                    QString latUnit = (transformed.getSRS()->isGeographic()) ? "°" : "m";
                    QString lonUnit = (transformed.getSRS()->isGeographic()) ? "°" : "m";

                    QString latText = QString("%1 %2").arg(transformed.y(), 0, 'f', 4).arg(latUnit);
                    QString lonText = QString("%1 %2").arg(transformed.x(), 0, 'f', 4).arg(lonUnit);
                    QString heightText = QString("%1 m").arg(transformed.z(), 0, 'f', 2);

                    // Update the UI fields
                    _lat->setText(latText);
                    _long->setText(lonText);
                    _height->setText(heightText);
                }
            } );
        }
}

StatusBarHandler::StatusBarHandler()
{

}
