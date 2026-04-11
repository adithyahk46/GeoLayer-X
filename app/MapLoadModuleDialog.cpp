#include "MapLoadModuleDialog.h"
#include "ui_MapLoadModuleDialog.h"


#include "pluggins/MapLoadModule.h"

MapLoadModuleDialog::MapLoadModuleDialog(osgEarth::MapNode* mapNode, QWidget* parent )
    :QDialog(parent)
    ,ui(new Ui::MapLoadModuleDialog)
    ,_mapNode(mapNode)

{
    ui->setupUi(this);
    setWindowTitle("Open Layers");
    setWindowIcon(QIcon(":/new/prefix1/icons/Layers/layers.png"));

  _initConnections();

  adjustSize();
}

void MapLoadModuleDialog::_initConnections()
{

    ui->le_rasterLabelName->setEnabled(!ui->cb_raster->isChecked());
    ui->le_rasterLabelName->setReadOnly(ui->cb_raster->isChecked());

    ui->le_vectorLayerName->setEnabled(!ui->cb_vectorLayerName->isChecked());
    ui->le_vectorLayerName->setReadOnly(ui->cb_vectorLayerName->isChecked());

    ui->le_eleLayerName->setEnabled(!ui->cb_eleLayerName->isChecked());
    ui->le_eleLayerName->setReadOnly(ui->cb_eleLayerName->isChecked());

    ui->le_sceneLayerName->setEnabled(!ui->cb_scenelayerName->isChecked());
    ui->le_sceneLayerName->setReadOnly(ui->cb_scenelayerName->isChecked());


    QObject::connect(ui->fromServer_radioButton,&QRadioButton::toggled,this,[=](){
        if(ui->fromServer_radioButton->isChecked()){
            ui->xyzUrlLabel->setText("Enter Server Url :");
            ui->pb_xyzUrl->hide();
            ui->le_xyzUrl->hide();
            ui->cb_ServerUrl->show();
            ui->cb_xyzLayerName->setChecked(false);
            ui->cb_xyzLayerName->setCheckable(false);
        }
    });
    QObject::connect(ui->fromLocal_radioButton,&QRadioButton::toggled,this,[=](){
        if(ui->fromLocal_radioButton->isChecked()){
            ui->xyzUrlLabel->setText("Select Directory : ");
            ui->pb_xyzUrl->show();
            ui->le_xyzUrl->show();
            ui->cb_ServerUrl->hide();

        }
    });
    ui->fromServer_radioButton->setChecked(true);
    ui->le_xyzLayername->setEnabled(!ui->cb_xyzLayerName->isChecked());
    ui->le_xyzLayername->setReadOnly(ui->cb_xyzLayerName->isChecked());

    QObject::connect(ui->pb_rasterUrl,&QPushButton::clicked,this,[=](){
        QString filePath = OpenFiles(FileType::RASTER);
            if (filePath.isEmpty())
                return;
            QFileInfo finfo(filePath);
            QString MapName = finfo.baseName().toStdString().c_str();
            QString fileExt = finfo.suffix().toStdString().c_str();  // Get the file extension

            ui->le_rasterUrl->setText(filePath);
            ui->le_rasterLabelName->setText(QString(MapName+"."+fileExt));
    });
    QObject::connect(ui->pb_vectorUrl,&QPushButton::clicked,this,[=](){
        QString filePath = OpenFiles(FileType::VECTOR);
            if (filePath.isEmpty())
                return;
            QFileInfo finfo(filePath);
            QString MapName = finfo.baseName().toStdString().c_str();
            QString fileExt = finfo.suffix().toStdString().c_str();  // Get the file extension

            ui->le_vectorUrl->setText(filePath);
            ui->le_vectorLayerName->setText(QString(MapName+"."+fileExt));
    });

    QObject::connect(ui->pb_eleUrl,&QPushButton::clicked,this,[=](){
        QString filePath = OpenFiles(FileType::ELEVATION);
            if (filePath.isEmpty())
                return;
            QFileInfo finfo(filePath);
            QString MapName = finfo.baseName().toStdString().c_str();
            QString fileExt = finfo.suffix().toStdString().c_str();  // Get the file extension

            ui->le_eleUrl->setText(filePath);
            ui->le_eleLayerName->setText(QString(MapName+fileExt));
    });

    QObject::connect(ui->pb_sceneUrl,&QPushButton::clicked,this,[=](){
        QString filePath = OpenFiles(FileType::VECTOR);
            if (filePath.isEmpty())
                return;
            QFileInfo finfo(filePath);
            QString MapName = finfo.baseName().toStdString().c_str();
            QString fileExt = finfo.suffix().toStdString().c_str();  // Get the file extension

            ui->le_sceanUrl->setText(filePath);
            ui->le_sceneLayerName->setText(QString(MapName+fileExt));
            getAttributes(filePath);
    });

    QObject::connect(ui->pb_sceneXml,&QPushButton::clicked,this,[=](){
        QString filePath = OpenFiles(FileType::TEXTURE);
            if (filePath.isEmpty())
                return;
            ui->le_sceneXml->setText(filePath);
    });

    QObject::connect(ui->pb_xyzUrl, &QPushButton::clicked, this, [=]() {
        // Let user pick a directory
        // QString FilePaths = QFileDialog::get(this,"Open Files","",Filter);

        QString folderPath = QFileDialog::getExistingDirectory(this, "Select XYZ Tile Folder");

        if (folderPath.isEmpty())
            return;
        // Example: convert local folder to file URL
        QUrl url = QUrl::fromLocalFile(folderPath );
        // Extract folder name
        QFileInfo finfo(folderPath);
        QString mapName = finfo.fileName();  // Just folder name
        // Set values to the UI
        ui->le_xyzUrl->setText(url.toString());
        ui->le_xyzLayername->setText(mapName);
    });



    QObject::connect(ui->cb_raster, &QCheckBox::toggled,this,[=](bool state){
            ui->le_rasterLabelName->setEnabled(!ui->cb_raster->isChecked());
            ui->le_rasterLabelName->setReadOnly(ui->cb_raster->isChecked());
    } );

    QObject::connect(ui->cb_vectorLayerName, &QCheckBox::toggled,this,[=](bool state){
            ui->le_vectorLayerName->setEnabled(!ui->cb_vectorLayerName->isChecked());
            ui->le_vectorLayerName->setReadOnly(ui->cb_vectorLayerName->isChecked());
    } );

    QObject::connect(ui->cb_eleLayerName, &QCheckBox::toggled,this,[=](bool state){
            ui->le_eleLayerName->setEnabled(!ui->cb_eleLayerName->isChecked());
            ui->le_eleLayerName->setReadOnly(ui->cb_eleLayerName->isChecked());
    } );

    QObject::connect(ui->cb_scenelayerName, &QCheckBox::toggled,this,[=](bool state){
            ui->le_eleLayerName->setEnabled(!ui->cb_scenelayerName->isChecked());
            ui->le_eleLayerName->setReadOnly(ui->cb_scenelayerName->isChecked());
    } );

  }


void MapLoadModuleDialog::getAttributes(QString FilePath){
 // OGR Shapefile support
 // Register all GDAL drivers
    GDALAllRegister();

    // Open the shapefile
    GDALDataset* dataset = static_cast<GDALDataset*>(
        GDALOpenEx(FilePath.toStdString().c_str(), GDAL_OF_VECTOR, nullptr, nullptr, nullptr)
    );

    if (!dataset) {
        qWarning("Failed to open shapefile.");
        return;
    }

    // Get the first layer
    OGRLayer* layer = dataset->GetLayer(0);
    if (!layer) {
        qWarning("Failed to get layer.");
        GDALClose(dataset);
        return;
    }

    // Get the layer definition (schema)
    OGRFeatureDefn* defn = layer->GetLayerDefn();
    if (!defn) {
        qWarning("Failed to get layer definition.");
        GDALClose(dataset);
        return;
    }

    // Clear the combo box
    ui->cb_heightAttribute->clear();

    // Loop through all fields and add their names to the combo box
    for (int i = 0; i < defn->GetFieldCount(); ++i) {
        OGRFieldDefn* fieldDefn = defn->GetFieldDefn(i);
        if (fieldDefn) {
            QString fieldName = QString::fromUtf8(fieldDefn->GetNameRef());
            ui->cb_heightAttribute->addItem(fieldName);
        }
    }

    // Clean up
    GDALClose(dataset);

}


MapLoadModuleDialog::~MapLoadModuleDialog()
{
    delete ui;

}

QString MapLoadModuleDialog::OpenFiles(FileType Type )
{
    QString Filter ="";
    switch(Type)
    {
    case FileType::RASTER:
        Filter = "Raster Files (*.tif *.tiff *.img *.asc *.bil);;All Files (*.*)";
        break;
    case FileType::VECTOR:
        Filter = "vector Files (*.shp );;All Files (*.*)";
        break;
    case FileType::ELEVATION:
        Filter = "Elevation Files (*.tif *.tiff *.dt2 *dt1 *dt0);;All Files (*.*)";
        break;
    case FileType::TEXTURE:
        Filter = "Texture Files(*.xml);; All Files (*.*)";
        break;
    case FileType::MODEL3D:
        Filter = "3D Model File (*.obj)";
    case FileType::ALLFILES:
        Filter = "All Files (*.*)";
        break;
    }

    QString FilePaths = QFileDialog::getOpenFileName(this,"Open Files","",Filter);

    return FilePaths;

}

void MapLoadModuleDialog::on_pb_openMap_clicked()
{
    if(ui->tabWidget->currentIndex() == 0)
    {
        if (ui->le_rasterUrl->text().isEmpty() )
            return;

            MapLoadModule::LoadRasters(_mapNode,ui->le_rasterUrl->text(), ui->le_rasterLabelName->text());

    }

    else if(ui->tabWidget->currentIndex() == 1)
    {
        if (ui->le_vectorUrl->text().isEmpty())
            return;

        // Run LoadVectors in a separate thread
           QtConcurrent::run([=]() {

               QMetaObject::invokeMethod(this, [=]() {

                   MapLoadModule::LoadVectors(_mapNode, ui->le_vectorUrl->text(), ui->le_vectorLayerName->text());

                  }, Qt::QueuedConnection);
           });
    }

    else if(ui->tabWidget->currentIndex() == 2){

        if (ui->le_eleUrl->text().isEmpty() || !_mapNode)
            return;

        MapLoadModule::LoadElevation(_mapNode,ui->le_eleUrl->text() , ui->le_eleLayerName->text());
    }

    else if(ui->tabWidget->currentIndex() == 3){
        if(ui->le_sceanUrl->text().isEmpty() || ui->le_sceneXml->text().isEmpty())
            return;

        if(ui->le_sceneLayerName->text().isEmpty()) return;

       // if(ui->le_heightAttribute->text().isEmpty()) return;

        QString LayerName = ui->le_sceneLayerName->text();
        QString heightText = "["+ui->cb_heightAttribute->currentText()+"]";

        MapLoadModule::Load3Dbuildings(_mapNode, ui->le_sceanUrl->text().trimmed(), heightText, LayerName, "" );

    }

    else if(ui->tabWidget->currentIndex() == 4)
        {
            qDebug() << "entered XYZ tab";

            if(ui->le_xyzLayername->text().isEmpty() || ui->le_xyzUrl->text().isEmpty()) {
                ui->warning_label->setText("⚠️ Warning: Please enter url or Layer Name");
                return;
            }
            QString LayerUrl;
            QString LayerName;

            if(ui->fromServer_radioButton->isChecked()) {

                LayerUrl = ui->cb_ServerUrl->currentText();

                if (!LayerUrl.contains("{z}") || !LayerUrl.contains("{x}") || !LayerUrl.contains("{y}")) {
                    ui->warning_label->setText("⚠️ Warning: URL must contain {z}/{x}/{y} placeholders");
                    return;
                }

            }
            else if(ui->fromLocal_radioButton->isChecked()) {

                QString inputUrl = ui->le_xyzUrl->text().trimmed();

                QFileInfo dirInfo(inputUrl);

                if (!dirInfo.isDir()) {
                    ui->warning_label->setText("⚠️ Warning: Invalid directory path");
                    return;
                }
                LayerName = dirInfo.completeBaseName().isEmpty() ? dirInfo.fileName() : dirInfo.completeBaseName();
                ui->le_xyzLayername->setText(LayerName);

                #ifdef _WIN32
                    // Remove "file:///" prefix (which is 8 characters long) on Windows
                    if (inputUrl.startsWith("file:///"))
                        inputUrl.remove(0, 8);
                    inputUrl += "/{z}/{x}/{y}.png";

                #else
                    // On Linux/macOS, "file://" is usually the prefix (7 characters)
                    if (inputUrl.startsWith("file://"))
                        inputUrl.remove(0, 7);
                    nputUrl += "{z}/{x}/{y}.png";
                #endif
                LayerUrl = inputUrl;

            }

            ui->warning_label->clear();
            MapLoadModule::LoadXYZTiles(_mapNode,LayerUrl, ui->le_xyzLayername->text());
        }

    else if(ui->tabWidget->currentIndex() == 5)
    {

    }
}


