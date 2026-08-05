#include "StatusbarHandlerWidget.h"
#include "ui_StatusbarHandlerWidget.h"

StatusbarHandlerWidget::StatusbarHandlerWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::StatusbarHandlerWidget)
{
    ui->setupUi(this);
}

StatusbarHandlerWidget::~StatusbarHandlerWidget()
{
    delete ui;
}

void StatusbarHandlerWidget::updateGeoCoordinates(osgEarth::GeoPoint mapPoint)
{
    const osgEarth::SpatialReference* srs4326 = osgEarth::SpatialReference::get("epsg:4326");
    const osgEarth::SpatialReference* srs3857 = osgEarth::SpatialReference::get("epsg:3857");

    osgEarth::GeoPoint transformed;
    if ( ui->comboBox->currentIndex() == 0) {
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
    ui->lat->setText(latText);
    ui->lon->setText(lonText);
    ui->alt->setText(heightText);
}
