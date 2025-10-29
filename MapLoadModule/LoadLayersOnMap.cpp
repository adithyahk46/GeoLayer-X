#include "LoadLayersOnMap.h"
#include "ui_LoadLayersOnMap.h"

#include <osgEarth/MapNode>
#include <osgEarth/EarthManipulator>
#include <osgEarth/Style>
#include <osgEarth/Map>
#include <osgEarth/FeatureModelLayer>
#include <osgEarth/ElevationQuery>
#include <osgEarth/FeatureSource>
#include <osgEarth/FeatureCursor>
#include <osgEarth/Feature>
#include <osgEarth/OGRFeatureSource>
#include <osgEarth/ElevationLayer>
#include <osgEarth/GDAL>
#include <osgEarth/GeoData>
#include <osgEarth/Expression>
#include <osgEarth/ResourceLibrary>

#include <osgEarth/Query>

#include "gdal_alg_priv.h"
#include <ogrsf_frmts.h>
#include "gdal.h"
#include "gdal_priv.h"
#include "ogr_feature.h"

#include <QStringList>
#include <QFileDialog>
#include <QMessageBox>
#include <QDebug>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QUrl>
#include <QTimer>
#include <QtConcurrent>
#include <QTcpServer>
#include "XYZServerManager.h"

LoadLayersOnMap::LoadLayersOnMap(QWidget* parent, osgEarth::MapNode* mapNode , osgEarth::EarthManipulator* ManIp)
    :QDialog(parent)
    ,ui(new Ui::MapLoadModule)
    ,_mapNode(mapNode)
    ,_manip(ManIp)

{
    ui->setupUi(this);
    setWindowTitle("Open Layers");
    setWindowIcon(QIcon(":/new/prefix1/icons/Layers/layers.png"));

  _initConnections();
}

void LoadLayersOnMap::_initConnections()
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


void LoadLayersOnMap::getAttributes(QString FilePath){
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

LoadLayersOnMap* LoadLayersOnMap::getInstance(osgEarth::MapNode* mapNode) {
    static LoadLayersOnMap instance;
    if (mapNode) {
        instance._mapNode = mapNode;
    }
    return &instance;
}

// static LoadLayersOnMap& getInstance(osgEarth::MapNode* mapNode) {
//         static LoadLayersOnMap instance;
//         instance._mapNode = mapNode;  // Optional: may want to guard or restrict setting this multiple times
//         return instance;
//     }

LoadLayersOnMap::~LoadLayersOnMap()
{
        for (auto it = m_servers.begin(); it != m_servers.end(); ++it) {
            ServerInfo& info = it.value();
            if (info.process->state() == QProcess::Running) {
                info.process->terminate();
                if (!info.process->waitForFinished(1000)) {
                    info.process->kill();
                }
            }
            info.process->deleteLater();
        }
        m_servers.clear();

    delete ui;

}

QString LoadLayersOnMap::OpenFiles(FileType Type )
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

void LoadLayersOnMap::LoadRasters(osgEarth::MapNode* mapNode, QString FilePath,  QString LayerName )
{

    QFileInfo finfo(FilePath);
    QString MapName = finfo.baseName().toStdString().c_str();
    QString fileExt = finfo.suffix().toStdString().c_str();  // Get the file extension


    osgEarth::GDALImageLayer* imageLayer = new osgEarth::GDALImageLayer();
    if(LayerName.isEmpty()){
        imageLayer->setName(QString(MapName+"."+fileExt).toStdString().c_str());
    }
    else{
        imageLayer->setName(LayerName.toStdString());
    }
    imageLayer->setURL(FilePath.toStdString());
    mapNode->getMap()->addLayer(imageLayer);
}

void LoadLayersOnMap::LoadVectors(osgEarth::MapNode* mapNode, QString FilePath , QString LayerName)
{

    QFileInfo finfo(FilePath);
    QString MapName = finfo.baseName().toStdString().c_str();
    QString fileExt = finfo.suffix().toStdString().c_str();

    osgEarth::OGRFeatureSource* features = new osgEarth::OGRFeatureSource();
    features->setURL(FilePath.toStdString().c_str());

    features->open();  // Open the feature source

    qDebug()<<"Loading ShapeFile = "<<FilePath;

    osgEarth::Style style;

    if (features)
    {
        osgEarth::Geometry::Type type = features->getGeometryType();

        QString geomTypeStr;

        switch (type)
        {
            case osgEarth::Geometry::TYPE_UNKNOWN:
            {
                geomTypeStr = "UNKNOWN";
            }
            case osgEarth::Geometry::TYPE_POINT:
            {
                geomTypeStr = "POINT";

                osgEarth::PointSymbol* pointSymbol = style.getOrCreateSymbol<osgEarth::PointSymbol>();
                style.getOrCreate<osgEarth::AltitudeSymbol>()->clamping() = osgEarth::AltitudeSymbol::CLAMP_TO_TERRAIN;

                pointSymbol->fill() =  osgEarth::Color::Red;
                pointSymbol->size() = 10.0f;
                pointSymbol->smooth() = 10.0f;
                break;
            }
            case osgEarth::Geometry::TYPE_POINTSET:
            {
                geomTypeStr = "MULTIPOINT";

                osgEarth::PointSymbol* pointSymbol = style.getOrCreateSymbol<osgEarth::PointSymbol>();
                style.getOrCreate<osgEarth::AltitudeSymbol>()->clamping() = osgEarth::AltitudeSymbol::CLAMP_TO_TERRAIN;

                pointSymbol->fill() =  osgEarth::Color::Red;
                pointSymbol->size() = 10.0f;
                pointSymbol->smooth() = 10.0f;
                break;
            }
            case osgEarth::Geometry::TYPE_LINESTRING:
            {
                geomTypeStr = "LINESTRING";

                // Configure altitude behavior (clamp to terrain)
                osgEarth::AltitudeSymbol* alt = style.getOrCreate<osgEarth::AltitudeSymbol>();
                alt->clamping() = osgEarth::AltitudeSymbol::CLAMP_TO_TERRAIN;
                alt->technique() = osgEarth::AltitudeSymbol::TECHNIQUE_DRAPE;

                // Configure render options to avoid Z-fighting
                osgEarth::RenderSymbol* render = style.getOrCreate<osgEarth::RenderSymbol>();
                render->lighting() = false;
                render->depthOffset()->enabled() = true;
                render->depthOffset()->automatic() = true;

                // Configure line appearance
                osgEarth::LineSymbol* ls = style.getOrCreate<osgEarth::LineSymbol>();
                ls->stroke()->color() = osgEarth::Color::Yellow;
                ls->stroke().mutable_value().width() = osgEarth::Distance(2.0, osgEarth::Units::PIXELS);
                ls->stroke()->widthUnits() = osgEarth::Units::PIXELS;
                ls->tessellation() = 150;
                break;
            }
            case osgEarth::Geometry::TYPE_RING:
            {

                osgEarth::LineSymbol* lineSymbol = style.getOrCreateSymbol<osgEarth::LineSymbol>();

                // Attribute-driven styling
                lineSymbol->stroke().mutable_value().color() = osgEarth::Color::Black;
                lineSymbol->stroke().mutable_value().width() = osgEarth::Distance(1.0,osgEarth::Units::PIXELS);
                lineSymbol->tessellationSize() = osgEarth::Distance(100, osgEarth::Units::KILOMETERS);

                auto* render = style.getOrCreateSymbol<osgEarth::RenderSymbol>();
                render->depthOffset()->range() = osgEarth::Distance(100.0, osgEarth::Units::METERS);
                break;
            }
            case osgEarth::Geometry::TYPE_POLYGON:
            {
                geomTypeStr = "POLYGON";

                style.setName("building Style");

                // Extrude the shapes into 3D buildings.
                // osgEarth::ExtrusionSymbol* extrusion = style.getOrCreate<osgEarth::ExtrusionSymbol>();
                // QString heightText = "[""]";
                // // extrusion->setHeightExpression(NumericExpression(heightText));
                // extrusion->heightExpression() = osgEarth::NumericExpression(heightText.toStdString().c_str());
                // extrusion->flatten() = false;
                // extrusion->wallStyleName() = "building-wall";
                // extrusion->roofStyleName() = "building-roof";

                osgEarth::PolygonSymbol* poly = style.getOrCreate<osgEarth::PolygonSymbol>();
                poly->fill()->color() = osgEarth::Color::White;

                // Clamp the buildings to the terrain.
                osgEarth::AltitudeSymbol* alt = style.getOrCreate<osgEarth::AltitudeSymbol>();
                alt->clamping() = alt->CLAMP_TO_TERRAIN;
                alt->technique() = alt->TECHNIQUE_DRAPE;
                alt->binding() = alt->BINDING_VERTEX;

                auto* render = style.getOrCreate<osgEarth::RenderSymbol>();
                render->depthOffset()->range() = osgEarth::Distance(100.0, osgEarth::Units::PIXELS);

                break;
            }
            case osgEarth::Geometry::TYPE_MULTI:
            {
                geomTypeStr = "MULTI";

                osgEarth::LineSymbol* ls = style.getOrCreateSymbol<osgEarth::LineSymbol>();
                ls->stroke().mutable_value().color() = osgEarth::Color::Yellow;
                ls->stroke().mutable_value().width() = osgEarth::Distance(2.0, osgEarth::Units::PIXELS);

                osgEarth::AltitudeSymbol* alt = style.getOrCreate<osgEarth::AltitudeSymbol>();
                alt->clamping() = alt->CLAMP_TO_TERRAIN;
                alt->technique() = alt->TECHNIQUE_DRAPE;
                break;
            }
            case osgEarth::Geometry::TYPE_TRIMESH:
            {
                geomTypeStr = "TYPE_TRIMESH"; break;
            }
            default:
            {
                geomTypeStr = "OTHER"; break;
            }
        }

        qDebug() << "Geometry Type of ShapeFile =" << geomTypeStr;
    }

    {
    osgEarth::StyleSheet* styleSheet = new osgEarth::StyleSheet();
    styleSheet->addStyle(style);

    osgEarth::FeatureModelLayer* featureModelLayer = new osgEarth::FeatureModelLayer();

    if(LayerName.isEmpty())
    {
        features->setName(QString(MapName+"."+fileExt).toStdString().c_str());
        featureModelLayer->setName(QString(MapName+"."+fileExt).toStdString().c_str());
    }
    else
    {
        features->setName(LayerName.toStdString().c_str());
        featureModelLayer->setName(LayerName.toStdString().c_str());
    }

    osgEarth::FeatureDisplayLayout layout;
    layout.tileSize() = 5500;

    featureModelLayer->setFeatureSource(features);
    featureModelLayer->setStyleSheet(styleSheet);
    // featureModelLayer->setMaxVisibleRange(10000.0);
    // featureModelLayer->setLayout(layout);
    mapNode->getMap()->addLayer(featureModelLayer);
    }

}

void LoadLayersOnMap:: LoadElevation(osgEarth::MapNode* mapNode,QString FilePath, QString LayerName)
{
    QFileInfo finfo(FilePath);
    QString MapName = finfo.baseName().toStdString().c_str();
    QString fileExt = finfo.suffix().toStdString().c_str();  // Get the file extension

    osgEarth::GDALElevationLayer* elevationLayer = new osgEarth::GDALElevationLayer();
    elevationLayer->setName(QString(MapName+"."+fileExt).toStdString().c_str());

    LayerName.isEmpty()?elevationLayer->setName(QString(MapName+"."+fileExt).toStdString().c_str())
                                   :elevationLayer->setName(LayerName.toStdString().c_str());

    elevationLayer->setURL(FilePath.toStdString().c_str());
    elevationLayer->setCachePolicy(osgEarth::CachePolicy::USAGE_READ_WRITE);

    mapNode->getMap()->addLayer(elevationLayer);
}

void LoadLayersOnMap::LoadXYZTiles(osgEarth::MapNode* mapNode, QString url , QString LayerName){

    // QString url = "http://[abc].tile.openstreetmap.org/{z}/{x}/{y}.png";
    //              // QString url = "http://172.22.22.213:8000/{z}/{x}/{y}.png";
    // QString url = "http://[a].tile.openstreetmap.org/{z}/{x}/{y}.png";

     osgEarth::XYZImageLayer* remoteTiles = new osgEarth::XYZImageLayer();
     remoteTiles->setURL(url.toStdString());
     remoteTiles->setName(LayerName.toStdString().c_str());
     remoteTiles->setProfile(osgEarth::Profile::create(osgEarth::Profile::SPHERICAL_MERCATOR));
     remoteTiles->setOpacity(1.0f);
     // osm->setMinLevel(8);
     // osm->setMaxLevel(21);

     // if (ui->fromLocal_radioButton->isChecked()) {
     //     connect(remoteTiles, &osgEarth::XYZImageLayer::removedFromMap,
     //             this, [url]() {
     //                 XYZServerManager::instance().releaseServer(url);
     //             });
     // }

     mapNode->getMap()->addLayer(remoteTiles);
}

void LoadLayersOnMap::on_pb_openMap_clicked()
{
    if(ui->tabWidget->currentIndex() == 0)
    {
        if (ui->le_rasterUrl->text().isEmpty() )
            return;

            LoadRasters(_mapNode,ui->le_rasterUrl->text(), ui->le_rasterLabelName->text());

    }

    else if(ui->tabWidget->currentIndex() == 1)
    {
        if (ui->le_vectorUrl->text().isEmpty())
            return;

        // Run LoadVectors in a separate thread
           QtConcurrent::run([=]() {
               LoadVectors(_mapNode,ui->le_vectorUrl->text(),ui->le_vectorLayerName->text());
           });
    }

    else if(ui->tabWidget->currentIndex() == 2){

        if (ui->le_eleUrl->text().isEmpty() || !_mapNode)
            return;

        LoadElevation(_mapNode,ui->le_eleUrl->text() , ui->le_eleLayerName->text());
    }

    else if(ui->tabWidget->currentIndex() == 3){
        if(ui->le_sceanUrl->text().isEmpty() || ui->le_sceneXml->text().isEmpty())
            return;

        if(ui->le_sceneLayerName->text().isEmpty()) return;

       // if(ui->le_heightAttribute->text().isEmpty()) return;

        QFileInfo finfo(ui->le_sceanUrl->text());

        QString MapName = finfo.baseName().toStdString().c_str();
        QString fileExt = finfo.suffix().toStdString().c_str();  // Get the file extension

        osgEarth::OGRFeatureSource* feature = new osgEarth::OGRFeatureSource();
        feature->setURL(ui->le_sceanUrl->text().toStdString());

        if(feature){
            osgEarth::Style buildingStyle;
            buildingStyle.setName("building Style");

            // Extrude the shapes into 3D buildings.
            osgEarth::ExtrusionSymbol* extrusion = buildingStyle.getOrCreate<osgEarth::ExtrusionSymbol>();
            QString heightText = "["+ui->cb_heightAttribute->currentText()+"]";
            // extrusion->setHeightExpression(NumericExpression(heightText));
            extrusion->heightExpression() = osgEarth::NumericExpression(heightText.toStdString().c_str());
            extrusion->flatten() = false;
            extrusion->wallStyleName() = "building-wall";
            extrusion->roofStyleName() = "building-roof";

            osgEarth::PolygonSymbol* poly = buildingStyle.getOrCreate<osgEarth::PolygonSymbol>();
            poly->fill()->color() = osgEarth::Color::White;

            // Clamp the buildings to the terrain.
            osgEarth::AltitudeSymbol* alt = buildingStyle.getOrCreate<osgEarth::AltitudeSymbol>();
            alt->clamping() = alt->CLAMP_TO_TERRAIN;
            alt->binding() = alt->BINDING_VERTEX;

            auto* render = buildingStyle.getOrCreate<osgEarth::RenderSymbol>();
            render->depthOffset()->range() = osgEarth::Distance(100.0, osgEarth::Units::PIXELS);

            osgEarth::Style wallStyle;
            wallStyle.setName("building-wall");
            osgEarth::SkinSymbol* wallSkin = wallStyle.getOrCreate<osgEarth::SkinSymbol>();
            wallSkin->library() = "us_resources";
            wallSkin->addTag("building");
            wallSkin->randomSeed() = 1;

            // a style for the rooftop textures:
            osgEarth::Style roofStyle;
            roofStyle.setName("building-roof");
            osgEarth::SkinSymbol* roofSkin = roofStyle.getOrCreate<osgEarth::SkinSymbol>();
            roofSkin->library() = "us_resources";
            roofSkin->addTag("rooftop");
            roofSkin->randomSeed() = 1;
            roofSkin->isTiled() = true;

            // assemble a stylesheet and add our styles to it:
            osgEarth::StyleSheet* styleSheet = new osgEarth::StyleSheet();
            styleSheet->addStyle(buildingStyle);
            styleSheet->addStyle(wallStyle);
            styleSheet->addStyle(roofStyle);

            // load a resource library that contains the building textures.
            osgEarth::ResourceLibrary* reslib = new osgEarth::ResourceLibrary("us_resources", ui->le_sceneXml->text().toStdString().c_str());
            styleSheet->addResourceLibrary(reslib);

            // set up a paging layout for incremental loading. The tile size factor and
            // the visibility range combine to determine the tile size, such that
            // tile radius = max range / tile size factor.
            osgEarth::FeatureDisplayLayout layout;
            layout.tileSize() = 5500;

            osgEarth::FeatureModelLayer* buildingsLayer = new osgEarth::FeatureModelLayer();

            buildingsLayer->setName("Buildings");
            buildingsLayer->setFeatureSource(feature);
            buildingsLayer->setStyleSheet(styleSheet);
            buildingsLayer->setLayout(layout);
            // buildingsLayer->setMaxVisibleRange(10000.0);

            if(ui->cb_scenelayerName->isChecked()){
                feature->setName(QString(MapName+"."+fileExt).toStdString().c_str());
                buildingsLayer->setName(QString(MapName+"."+fileExt).toStdString().c_str());
            }
            else{
                feature->setName(QString(MapName+"."+fileExt).toStdString().c_str());
                buildingsLayer->setName(ui->le_sceneLayerName->text().toStdString().c_str());
            }

            _mapNode->getMap()->addLayer(buildingsLayer);
        }
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
            LoadXYZTiles(_mapNode,LayerUrl, ui->le_xyzLayername->text());
        }

    else if(ui->tabWidget->currentIndex() == 5)
    {

    }
}


