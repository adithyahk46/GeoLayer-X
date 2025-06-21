#include "MapLoadModule.h"
#include "ui_MapLoadModule.h"

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

MapLoadModule::MapLoadModule(osgEarth::MapNode* mapNode , osgEarth::EarthManipulator* ManIp)
    // :QDialog(parent)
    :ui(new Ui::MapLoadModule)
    ,_mapNode(mapNode)
    ,_manip(ManIp)

{
    ui->setupUi(this);
    setWindowTitle("Open Layers");
    setWindowIcon(QIcon(":/new/prefix1/icons/Layers/layers.png"));

  _initConnections();
}

void MapLoadModule::_initConnections()
{

    ui->le_rasterLabelName->setEnabled(!ui->cb_raster->isChecked());
    ui->le_rasterLabelName->setReadOnly(ui->cb_raster->isChecked());

    ui->le_vectorLayerName->setEnabled(!ui->cb_vectorLayerName->isChecked());
    ui->le_vectorLayerName->setReadOnly(ui->cb_vectorLayerName->isChecked());

    ui->le_eleLayerName->setEnabled(!ui->cb_eleLayerName->isChecked());
    ui->le_eleLayerName->setReadOnly(ui->cb_eleLayerName->isChecked());

    ui->le_sceneLayerName->setEnabled(!ui->cb_scenelayerName->isChecked());
    ui->le_sceneLayerName->setReadOnly(ui->cb_scenelayerName->isChecked());

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


void MapLoadModule::getAttributes(QString FilePath){
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

MapLoadModule* MapLoadModule::getInstance(osgEarth::MapNode* mapNode){
    _mapNode = mapNode;

    return this;
}

// static MapLoadModule& getInstance(osgEarth::MapNode* mapNode) {
//         static MapLoadModule instance;
//         instance._mapNode = mapNode;  // Optional: may want to guard or restrict setting this multiple times
//         return instance;
//     }

MapLoadModule::~MapLoadModule()
{
    delete ui;
}

QString MapLoadModule::OpenFiles(FileType Type )
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
    case FileType::ALLFILES:
        Filter = "All Files (*.*)";
        break;
    }

    QString FilePaths = QFileDialog::getOpenFileName(this,"Open Files","",Filter);

    return FilePaths;

}

void MapLoadModule::LoadRasters(QString FilePath,  QString LayerName )
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
    _mapNode->getMap()->addLayer(imageLayer);
}

void MapLoadModule::LoadVectors(QString FilePath , QString LayerName)
{

    QFileInfo finfo(FilePath);
    QString MapName = finfo.baseName().toStdString().c_str();
    QString fileExt = finfo.suffix().toStdString().c_str();

    osgEarth::OGRFeatureSource* features = new osgEarth::OGRFeatureSource();
    features->setURL(FilePath.toStdString().c_str());

    features->open();  // Open the feature source

    if (features)
    {
        osgEarth::Geometry::Type type = features->getGeometryType();

        qDebug() << "Geometry type = " << QString::fromStdString(osgEarth::Geometry::toString(type));
        osgEarth::Style style;

        switch (type)
        {
            case osgEarth::Geometry::TYPE_POINTSET:
            {
            qDebug()<<"step p ";

                osgEarth::PointSymbol* pointSymbol = style.getOrCreateSymbol<osgEarth::PointSymbol>();
                pointSymbol->fill() =  osgEarth::Color::Red;
                pointSymbol->size() = 10.0f;
                pointSymbol->smooth() = 10.0f;
                break;
            }
            case osgEarth::Geometry::TYPE_LINESTRING:
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
            style.setName("Polygon Style");


                osgEarth::PolygonSymbol* polygonSymbol = style.getOrCreate<osgEarth::PolygonSymbol>();
                polygonSymbol->fill()->color() = osgEarth::Color::White;

                // osgEarth::LineSymbol* lineSymbol = style.getOrCreate<osgEarth::LineSymbol>();

                // // Attribute-driven styling
                // lineSymbol->stroke().mutable_value().color() = osgEarth::Color::Black;
                // lineSymbol->stroke().mutable_value().width() = osgEarth::Distance(5.0,osgEarth::Units::PIXELS);
                // lineSymbol->tessellationSize() = osgEarth::Distance(100, osgEarth::Units::KILOMETERS);

                // polygonSymbol->fill() = osgEarth::Color::Cyan;
                // auto* render = style.getOrCreate<osgEarth::RenderSymbol>();
                // render->depthOffset()->range() = osgEarth::Distance(100.0, osgEarth::Units::METERS);

                break;
            }

            case osgEarth::Geometry::TYPE_MULTI:
            {
                // For multi-geometries, you might want to recursively check contained geometry types
                std::cout << "MultiGeometry detected: handle recursively if needed.\n";
                break;
            }

            default:
            {
                std::cout << "Unknown or unsupported geometry type.\n";
                break;
            }
        }

        osgEarth::StyleSheet* styleSheet = new osgEarth::StyleSheet();
        styleSheet->addStyle(style);

        osgEarth::FeatureDisplayLayout layout;
        layout.tileSize() = 500;

        // Apply the style to a new feature model layer
        osgEarth::FeatureModelLayer* featureModelLayer = new osgEarth::FeatureModelLayer();

        if(LayerName.isEmpty()){
            features->setName(QString(MapName+"."+fileExt).toStdString().c_str());
            featureModelLayer->setName(QString(MapName+"."+fileExt).toStdString().c_str());
        }
        else{
            features->setName(LayerName.toStdString().c_str());
            featureModelLayer->setName(LayerName.toStdString().c_str());
        }

        featureModelLayer->setFeatureSource(features);
        featureModelLayer->setStyleSheet(styleSheet);
        featureModelLayer->setLayout(layout);
        featureModelLayer->setMaxVisibleRange(55340.0);

        _mapNode->getMap()->addLayer(featureModelLayer);

    }
}

void MapLoadModule::LoadElevation(QString FilePath, QString LayerName)
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

    _mapNode->getMap()->addLayer(elevationLayer);
}

void MapLoadModule::on_pb_openMap_clicked()
{
    if(ui->tabWidget->currentIndex() == 0)
    {
        if (ui->le_rasterUrl->text().isEmpty() )
            return;

            LoadRasters(ui->le_rasterUrl->text(), ui->le_rasterLabelName->text());

    }

    else if(ui->tabWidget->currentIndex() == 1)
    {
        if (ui->le_vectorUrl->text().isEmpty())
            return;

        LoadVectors(ui->le_vectorUrl->text(),ui->le_vectorLayerName->text());

    }

    else if(ui->tabWidget->currentIndex() == 2){

        if (ui->le_eleUrl->text().isEmpty() || !_mapNode)
            return;

        LoadElevation(ui->le_eleUrl->text() , ui->le_eleLayerName->text());
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


            // osg::ref_ptr<osgEarth::FeatureCursor> cursor;
            // try {
            //     osgEarth::Query query;

            //     // Use the correct method 'limit()' as defined in the osgEarth header.
            //     // This is the only change required to fix the error.
            //     query.limit() = 1;

            //     // NOTE: The header also shows no 'setFilterSpatial'. To not filter
            //     // spatially, you simply don't set a bounds or tilekey, so that
            //     // line should be removed.

            //     cursor = feature->createFeatureCursor(query);

            // } catch (const std::exception& e) {
            //     QMessageBox::critical(this, "Error", QString("Failed to create feature cursor: %1").arg(e.what()));
            //     return;
            // }

            // if (cursor.valid() && cursor->hasMore()) {
            //     osg::ref_ptr<osgEarth::Feature> firstFeature = cursor->nextFeature();
            //     if (firstFeature.valid()) {
            //         QString heightAttributeName = ui->le_heightAttribute->text();

            //         // Check if the attribute exists
            //         if (!firstFeature->hasAttr(heightAttributeName.toStdString())) { // CORRECTED LINE 1
            //             qDebug() << "Attribute '" << heightAttributeName << "' NOT found in source. Available attributes:";
            //             // CORRECTED LOOP for iterating over AttributeTable (vector_map)
            //             for (const auto& pair : firstFeature->getAttrs()) { // CORRECTED LINE 2 (direct iteration)
            //                 qDebug() << " - " << QString::fromStdString(pair.first);
            //             }
            //             QMessageBox::warning(this, "Attribute Not Found", "The specified attribute was not found.");
            //             return;
            //         } else {
            //             qDebug() << "Attribute '" << heightAttributeName << "' found.";
            //             // Success! You can now proceed.
            //         }
            //     }
            // } else {
            //     QMessageBox::warning(this, "Warning", "Feature source is empty or could not be read.");
            //     return;
            // }


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
            layout.tileSize() = 500;

            osgEarth::FeatureModelLayer* buildingsLayer = new osgEarth::FeatureModelLayer();

            buildingsLayer->setName("Buildings");
            buildingsLayer->setFeatureSource(feature);
            buildingsLayer->setStyleSheet(styleSheet);
            buildingsLayer->setLayout(layout);
            buildingsLayer->setMaxVisibleRange(10000.0);

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


}

