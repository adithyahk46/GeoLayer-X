#include "VolumeCalculatorWidget.h"
#include "ui_VolumeCalculatorWidget.h"
#include "MouseEventHandler.h"
#include <osgEarth/GeoTransform>
// #include <osgEarthUtil/GeoInterpolator>

VolumeCalculatorWidget::VolumeCalculatorWidget(QWidget* parent)
    : QWidget(parent), ui(new Ui::VolumeCalculatorWidget)
{
    ui->setupUi(this);
    _mouseController = MouseEventHandler::Instance();

    connect(ui->pushButton_selectRegion, &QPushButton::clicked, this, &VolumeCalculatorWidget::onSelectRegionClicked);
    connect(ui->pushButton_updatePolygon, &QPushButton::clicked, this, &VolumeCalculatorWidget::onUpdatePolygonClicked);
    connect(ui->pushButton_calculateVolume, &QPushButton::clicked, this, &VolumeCalculatorWidget::onCalculateVolumeClicked);
}

VolumeCalculatorWidget::~VolumeCalculatorWidget(){
    delete ui;
}

void VolumeCalculatorWidget::onSelectRegionClicked()
{
    ui->tableWidget_points->setRowCount(0);
    connect(_mouseController, &MouseEventHandler::mouseClickEvent, this, &VolumeCalculatorWidget::onMouseClickEvent);
}

void VolumeCalculatorWidget::onMouseClickEvent(double lon, double lat, double alt, const osgGA::GUIEventAdapter&, osgGA::GUIActionAdapter&)
{
    int row = ui->tableWidget_points->rowCount();
    ui->tableWidget_points->insertRow(row);
    ui->tableWidget_points->setItem(row, 0, new QTableWidgetItem(QString::number(lon, 'f', 6)));
    ui->tableWidget_points->setItem(row, 1, new QTableWidgetItem(QString::number(lat, 'f', 6)));
    ui->tableWidget_points->setItem(row, 2, new QTableWidgetItem(QString::number(alt, 'f', 2)));
}

void VolumeCalculatorWidget::onUpdatePolygonClicked()
{
    // Create and display polygon from selected points (clamped to terrain)
    // Example: use osgEarth::GeoPoint + osg::Geometry
}

void VolumeCalculatorWidget::onCalculateVolumeClicked()
{
    // Basic conceptual volume computation example
    // For a terrain polygon, integrate elevation differences over area.
    // Here just a placeholder:
    double volume = 0.0;
    for (int i = 0; i < ui->tableWidget_points->rowCount(); ++i)
    {
        double alt = ui->tableWidget_points->item(i, 2)->text().toDouble();
        volume += alt; // placeholder
    }
    ui->lineEdit_volume->setText(QString::number(volume, 'f', 2));
}
