#include "RoutingWidget.h"
#include "ui_routingwidget.h"
#include <QFileDialog>
#include <QMessageBox>
#include <QThread>
#include <QFileInfo>
#include <gdal_priv.h>
#include <ogrsf_frmts.h>

#include "RoutingWorker.h"
// #include "MapLoad/maploadmodule.h"
// #include <FileSystemModel/filesystemmodeldialog.h>
// #include <App/NotificationBox.h>
#include <QDateTime>
#include <QDebug>
#include <QFileDialog>




RoutingWidget::RoutingWidget(osgEarth::MapNode* mapNode,MouseEventHandler* mouseControlle)
    :_mapnode(mapNode)
    ,_mouseControlle(mouseControlle)
    ,ui(new Ui::RoutingWidget)
{
    ui->setupUi(this);

    connect(ui->browseButton, &QPushButton::clicked, this, &RoutingWidget::selectShapefile);
    connect(ui->computeButton, &QPushButton::clicked, this, &RoutingWidget::computePath);
    connect(ui->resetButton, &QPushButton::clicked, this, &RoutingWidget::resetFields);
}

RoutingWidget::~RoutingWidget()
{
    delete ui;
}

void RoutingWidget::selectShapefile()
{

    QString Filter = "vector Files (*.shp );;All Files (*.*)";

    QString FilePaths = QFileDialog::getOpenFileName(this,"Open Files","",Filter);

    validateShapefile(FilePaths);
}

void RoutingWidget::validateShapefile(const QString &path)
{
    ui->logTextEdit->clear();
    ui->logTextEdit->appendPlainText("Opening shapefile: " + path);

    GDALAllRegister();
    GDALDataset *ds = (GDALDataset*)GDALOpenEx(
                path.toStdString().c_str(), GDAL_OF_VECTOR, nullptr, nullptr, nullptr);
    if (!ds) {
        QMessageBox::critical(this, "Error", "Failed to open shapefile.");
        ui->logTextEdit->appendPlainText("ERROR: could not open file.");
        return;
    }

    OGRLayer *layer = ds->GetLayer(0);
    if (!layer || wkbFlatten(layer->GetGeomType()) != wkbLineString) {
        QMessageBox::critical(this, "Error", "Selected shapefile is not Line geometry.");
        ui->logTextEdit->appendPlainText("ERROR: wrong geometry type.");
        GDALClose(ds);
        return;
    }

    shapefilePath = path;
    ui->lineEdit_inputFile->setText(path);
    ui->logTextEdit->appendPlainText("Shapefile validated.");
    GDALClose(ds);
}

void RoutingWidget::computePath()
{
    if (shapefilePath.isEmpty()) {
        QMessageBox::warning(this, "Warning", "Please select a valid shapefile first.");
        return;
    }


    double sx = ui->le_fromLon->text().toDouble();
    double sy = ui->le_fromLat->text().toDouble();
    double ex = ui->le_toLon->text().toDouble();
    double ey = ui->le_toLat->text().toDouble();

    runRoutingInThread(sx, sy, ex, ey);
}

void RoutingWidget::runRoutingInThread(double sx, double sy, double ex, double ey)
{
    ui->computeButton->setEnabled(false);
    ui->logTextEdit->appendPlainText("\n"+QDateTime::currentDateTime().toString("hh:mm:ss:zzz - ")+"Starting routing thread...");

    QThread *thread = new QThread;

    RoutingWorker *worker = nullptr;
    if(ui->radioButton_Dijkstras->isChecked())
    {
        worker = new RoutingWorker(shapefilePath, sx, sy, ex, ey,RoutingWorker::Dijkstra);
    }
    else if(ui->radioButton_AStar->isChecked())
    {
        worker = new RoutingWorker(shapefilePath, sx, sy, ex, ey,RoutingWorker::AStar);
    }
    worker->moveToThread(thread);

    connect(thread, &QThread::started, worker, &RoutingWorker::process);
    connect(worker, &RoutingWorker::stageUpdate, this, &RoutingWidget::appendLog);
    connect(worker, &RoutingWorker::routingComplete, this, &RoutingWidget::onRoutingFinished);
    connect(worker, &RoutingWorker::finished, thread, &QThread::quit);
    connect(worker, &RoutingWorker::finished, worker, &RoutingWorker::deleteLater);
    connect(thread, &QThread::finished, thread, &QThread::deleteLater);

    if(!prevOP.isEmpty())
    {
        // Wrap in a QFileInfo
        QFileInfo fi(prevOP);

        // 1) Get just the filename without its extension:
        QString nameOnly = fi.completeBaseName();
        // — for "route_dijkstra.shp" this yields "route_dijkstra"

        // 2) If you want the full absolute path *up to* that basename:
        QString dir      = fi.absolutePath();                   // "/home/user/data"
        QString fullBase = dir + QDir::separator() + nameOnly;  // "/home/user/data/route_dijkstra"

        // 3) If you only wanted the name (no path), use nameOnly directly:
        qDebug() << "Base name:" << nameOnly;

        // CGLegend *legend = m_mapView->legend();
        // CGLegendEntry *le = legend->item(nameOnly);
        // if (le)
        // {
        //     le->setVisible(false);
        //     m_mapView->repaintMapForce(m_mapView->currentViewExtent());
        // }
    }

    thread->start();
}

void RoutingWidget::appendLog(const QString &msg)
{
    ui->logTextEdit->appendPlainText(QDateTime::currentDateTime().toString("hh:mm:ss:zzz - ")+msg);
}

#include <MapLoadModule/LoadLayersOnMap.h>
void RoutingWidget::onRoutingFinished(const QString &outPath)
{
    ui->computeButton->setEnabled(true);
    ui->logTextEdit->appendPlainText("Routing complete. Output: " + outPath);

    QStringList list = outPath.split(";");
    if(list.count()>1)
    {
        LoadLayersOnMap::getInstance(_mapnode)->LoadVectors(_mapnode,list.at(0),"Shortest Path");

        prevOP = list.at(0);
    }

    QMessageBox::information(this, "Done", "Saved to:" + outPath);
}


void RoutingWidget::resetFields()
{
    shapefilePath.clear();
    ui->logTextEdit->clear();
    ui->le_fromLon->clear();
    ui->le_fromLat->clear();
    ui->le_toLon->clear();
    ui->le_toLat->clear();

}

void RoutingWidget::on_pushButton_clicked()
{
    QObject::connect(_mouseControlle, &MouseEventHandler::mouseClickEvent, this, [=](const double lon, const double lat, const double alt, const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa)
    {
        ui->le_fromLon->setText(QString::number(lon, 'f', 6));  // 6 decimal places
        ui->le_fromLat->setText(QString::number(lat, 'f', 6));
        QObject::disconnect(_mouseControlle,nullptr,this,nullptr);

    });

}


void RoutingWidget::on_pushButton_2_clicked()
{
    QObject::connect(_mouseControlle, &MouseEventHandler::mouseClickEvent, this, [=](const double lon, const double lat, const double alt, const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa)
    {
        ui->le_toLon->setText(QString::number(lon, 'f', 6));  // 6 decimal places
        ui->le_toLat->setText(QString::number(lat, 'f', 6));
        QObject::disconnect(_mouseControlle,nullptr,this,nullptr);

    });

}

