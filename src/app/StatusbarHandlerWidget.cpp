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
    if (!mapPoint.isValid())
    {
        clearGeoCoordinates();
        return;
    }

    const osgEarth::SpatialReference* srs4326 = osgEarth::SpatialReference::get("epsg:4326");
    const osgEarth::SpatialReference* srs3857 = osgEarth::SpatialReference::get("epsg:3857");

    osgEarth::GeoPoint transformed;
    const osgEarth::SpatialReference* targetSRS =
        ui->comboBox->currentIndex() == 0 ? srs4326 : srs3857;
    if (!targetSRS || !mapPoint.transform(targetSRS, transformed)
        || !transformed.isValid() || !transformed.getSRS())
    {
        clearGeoCoordinates();
        return;
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

void StatusbarHandlerWidget::clearGeoCoordinates()
{
    ui->lat->setText(QStringLiteral("--"));
    ui->lon->setText(QStringLiteral("--"));
    ui->alt->setText(QStringLiteral("--"));
}
