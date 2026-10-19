#include "MapLoadModule.h"

#include <QFileInfo>

#include <osgDB/ReadFile>

#include <osgEarth/GDAL>
#include <osgEarth/OGRFeatureSource>
#include <osgEarth/FeatureModelLayer>
#include <osgEarth/XYZ>
#include <osgEarth/StyleSheet>

#include "osgviewerwidget.h"
#include "App.h"


MapLoadModule::MapLoadModule()
{

}

bool MapLoadModule::LoadEarthFile(const QString& FilePath)
{
    //clear the data from the existing mapNode and load the new .earth file

    osg::ref_ptr<osg::Node> node = osgDB::readRefNodeFile(FilePath.toStdString());
    if (node.valid())
    {
        osg::ref_ptr<osgEarth::MapNode> new_mapNode = osgEarth::MapNode::findMapNode(node);
        if (new_mapNode)
        {
            osgEarth::MapNode* currentMapNode = App::getThreedWindow()->getMapNode();
            if (!currentMapNode || !currentMapNode->getMap() || !new_mapNode->getMap())
            {
                OE_WARN << "Failed to load .earth file: invalid source or destination map" << std::endl;
                return false;
            }

            // Replace the current map's layer stack with the layers from the loaded earth map.
            osgEarth::LayerVector newLayers;
            new_mapNode->getMap()->getLayers(newLayers);

            //open each layer in the new map to ensure they are ready for rendering
            for (osgEarth::Layer* layer : newLayers)
            {
                if (layer)
                {
                    layer->open();
                    currentMapNode->getMap()->addLayer(layer);

                }
            }

            // currentMap->beginUpdate();
            // currentMap->clear();
            // currentMap->endUpdate();

            // App::getThreedWindow()->getRoot()->removeChild(App::getThreedWindow()->getMapNode());
            // App::getThreedWindow()->setMapNode(new_mapNode.get());
            // new_mapNode->open();
            // App::getThreedWindow()->getRoot()->addChild(new_mapNode);
            // App::getThreedWindow()->getRoot()->dirtyBound();
            // App::getThreedWindow()->getViewer()->realize();
        }
    }
    else{
        // Handle error: failed to load the .earth file
        OE_WARN<<"Failed to load .earth file: " <<  FilePath.toStdString().c_str() ;
    }
    return true;
}

void MapLoadModule::LoadRasters(osgEarth::MapNode* mapNode, QString FilePath,  QString LayerName )
{
    QFileInfo finfo(FilePath);
    QString MapName = finfo.baseName().toStdString().c_str();
    QString fileExt = finfo.suffix().toStdString().c_str();  // Get the file extension


    osg::ref_ptr<osgEarth::GDALImageLayer> imageLayer = new osgEarth::GDALImageLayer();
    if(LayerName.isEmpty()){
        imageLayer->setName(QString(MapName+"."+fileExt).toStdString().c_str());
    }
    else{
        imageLayer->setName(LayerName.toStdString());
    }
    imageLayer->setURL(FilePath.toStdString());
    mapNode->getMap()->addLayer(imageLayer.get());
}

void MapLoadModule::LoadVectors(osgEarth::MapNode* mapNode, QString FilePath , QString LayerName)
{
    if (!mapNode || !mapNode->getMap())
    {
        OE_WARN << "LoadVectors failed: invalid map node" << std::endl;
        return;
    }

    QFileInfo finfo(FilePath);
    QString mapName = finfo.baseName();
    QString fileExt = finfo.suffix();
    QString displayName = LayerName.isEmpty() ? (mapName + "." + fileExt) : LayerName;

    osg::ref_ptr<osgEarth::OGRFeatureSource> features = new osgEarth::OGRFeatureSource();
    features->setURL(FilePath.toStdString());
    features->setName(displayName.toStdString());

    qDebug() << "Loading ShapeFile =" << FilePath;

    const auto openStatus = features->open();
    if (openStatus.isError())
    {
        OE_WARN << "Failed to open vector source: " << FilePath.toStdString()
                << " : " << openStatus.message() << std::endl;
        return;
    }

    osgEarth::Style style;
    QString geomTypeStr = "UNKNOWN";

    const osgEarth::Geometry::Type type = features->getGeometryType();
    switch (type)
    {
        case osgEarth::Geometry::TYPE_UNKNOWN:
        {
            geomTypeStr = "UNKNOWN";

            osgEarth::LineSymbol* ls = style.getOrCreateSymbol<osgEarth::LineSymbol>();
            ls->stroke().mutable_value().color() = osgEarth::Color::Yellow;
            ls->stroke().mutable_value().width() = osgEarth::Distance(2.0, osgEarth::Units::PIXELS);
            break;
        }
        case osgEarth::Geometry::TYPE_POINT:
        case osgEarth::Geometry::TYPE_POINTSET:
        {
            geomTypeStr = (type == osgEarth::Geometry::TYPE_POINT) ? "POINT" : "MULTIPOINT";

            osgEarth::PointSymbol* pointSymbol = style.getOrCreateSymbol<osgEarth::PointSymbol>();
            style.getOrCreate<osgEarth::AltitudeSymbol>()->clamping() = osgEarth::AltitudeSymbol::CLAMP_TO_TERRAIN;

            pointSymbol->fill() = osgEarth::Color::Red;
            pointSymbol->size() = 10.0f;
            pointSymbol->smooth() = 10.0f;
            break;
        }
        case osgEarth::Geometry::TYPE_LINESTRING:
        case osgEarth::Geometry::TYPE_RING:
        {
            geomTypeStr = (type == osgEarth::Geometry::TYPE_LINESTRING) ? "LINESTRING" : "RING";

            osgEarth::LineSymbol* ls = style.getOrCreateSymbol<osgEarth::LineSymbol>();
            ls->stroke().mutable_value().color() = osgEarth::Color::Yellow;
            ls->stroke().mutable_value().width() = osgEarth::Distance(2.0f, osgEarth::Units::PIXELS);
            ls->tessellationSize() = osgEarth::Distance(100, osgEarth::Units::KILOMETERS);

            osgEarth::AltitudeSymbol* alt = style.getOrCreate<osgEarth::AltitudeSymbol>();
            alt->clamping() = osgEarth::AltitudeSymbol::CLAMP_TO_TERRAIN;
            alt->technique() = osgEarth::AltitudeSymbol::TECHNIQUE_DRAPE;

            osgEarth::RenderSymbol* render = style.getOrCreate<osgEarth::RenderSymbol>();
            render->lighting() = false;
            render->depthOffset()->enabled() = true;
            render->depthOffset().mutable_value().automatic() = true;
            break;
        }
        case osgEarth::Geometry::TYPE_POLYGON:
        {
            geomTypeStr = "POLYGON";

            osgEarth::PolygonSymbol* poly = style.getOrCreate<osgEarth::PolygonSymbol>();
            poly->fill()->color() = osgEarth::Color::White;

            osgEarth::AltitudeSymbol* alt = style.getOrCreate<osgEarth::AltitudeSymbol>();
            alt->clamping() = osgEarth::AltitudeSymbol::CLAMP_TO_TERRAIN;
            alt->binding() = osgEarth::AltitudeSymbol::BINDING_VERTEX;

            osgEarth::RenderSymbol* render = style.getOrCreate<osgEarth::RenderSymbol>();
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
            alt->clamping() = osgEarth::AltitudeSymbol::CLAMP_TO_TERRAIN;
            alt->technique() = osgEarth::AltitudeSymbol::TECHNIQUE_DRAPE;
            break;
        }
        case osgEarth::Geometry::TYPE_TRIMESH:
        {
            geomTypeStr = "TRIMESH";
            break;
        }
        default:
        {
            geomTypeStr = "OTHER";
            break;
        }
    }

    qDebug() << "Geometry Type of ShapeFile =" << geomTypeStr;

    osg::ref_ptr<osgEarth::StyleSheet> styleSheet = new osgEarth::StyleSheet();
    styleSheet->addStyle(style);

    osg::ref_ptr<osgEarth::FeatureModelLayer> featureModelLayer = new osgEarth::FeatureModelLayer();
    featureModelLayer->setName(displayName.toStdString());
    featureModelLayer->setFeatureSource(features);
    featureModelLayer->setStyleSheet(styleSheet);

    osgEarth::FeatureDisplayLayout layout;
    layout.tileSize() = 4500;
    // featureModelLayer->setLayout(layout);

    mapNode->getMap()->addLayer(featureModelLayer);

    if (featureModelLayer->getStatus().isError())
    {
        OE_WARN << featureModelLayer->getName() << " : " << featureModelLayer->getStatus().toString() << std::endl;
    }

}

void MapLoadModule::LoadElevation(osgEarth::MapNode* mapNode,QString FilePath, QString LayerName)
{
    QFileInfo finfo(FilePath);
    QString MapName = finfo.baseName().toStdString().c_str();
    QString fileExt = finfo.suffix().toStdString().c_str();  // Get the file extension

    osg::ref_ptr<osgEarth::GDALElevationLayer> elevationLayer = new osgEarth::GDALElevationLayer();
    elevationLayer->setName(QString(MapName+"."+fileExt).toStdString().c_str());

    LayerName.isEmpty()?elevationLayer->setName(QString(MapName+"."+fileExt).toStdString().c_str())
                                   :elevationLayer->setName(LayerName.toStdString().c_str());

    elevationLayer->setURL(FilePath.toStdString().c_str());
    elevationLayer->setCachePolicy(osgEarth::CachePolicy::USAGE_READ_WRITE);

    mapNode->getMap()->addLayer(elevationLayer.get());
}

void MapLoadModule::LoadXYZTiles(osgEarth::MapNode* mapNode, QString url , QString LayerName){

    // QString url = "http://[abc].tile.openstreetmap.org/{z}/{x}/{y}.png";
    //              // QString url = "http://172.22.22.213:8000/{z}/{x}/{y}.png";
    // QString url = "http://[a].tile.openstreetmap.org/{z}/{x}/{y}.png";

     osg::ref_ptr<osgEarth::XYZImageLayer> remoteTiles = new osgEarth::XYZImageLayer();
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

     mapNode->getMap()->addLayer(remoteTiles.get());
}

void MapLoadModule::LoadTMSLayer(osgEarth::MapNode* mapNode, QString url, QString LayerName){

    osg::ref_ptr<osgEarth::TMSImageLayer> tmsLayer = new osgEarth::TMSImageLayer();
    tmsLayer->setURL(url.toStdString());
    tmsLayer->setName(LayerName.toStdString().c_str());
    // tmsLayer->setProfile(osgEarth::Profile::create(osgEarth::Profile::SPHERICAL_MERCATOR));
    // tmsLayer->setOpacity(1.0f);

    mapNode->getMap()->addLayer(tmsLayer.get());
}

void MapLoadModule::Load3Dbuildings(osgEarth::MapNode* mapNode, QString FilePath, QString heightAttr, QString LayerName, QString TexResPath ){

    osg::ref_ptr<osgEarth::OGRFeatureSource> feature = new osgEarth::OGRFeatureSource();
    feature->setURL(FilePath.toStdString());

    if(feature){
        osgEarth::Style buildingStyle;
        buildingStyle.setName("building Style");

        // Extrude the shapes into 3D buildings.
        // osg::ref_ptr<osgEarth::ExtrusionSymbol> extrusion = buildingStyle.getOrCreate<osgEarth::ExtrusionSymbol>();
        // // extrusion->setHeightExpression(NumericExpression(heightAttr));
        // // extrusion->heightExpression() = osgEarth::NumericExpression(heightAttr.toStdString().c_str());
        // extrusion->flatten() = false;
        // extrusion->wallStyleName() = "building-wall";
        // extrusion->roofStyleName() = "building-roof";

        osg::ref_ptr<osgEarth::PolygonSymbol> poly = buildingStyle.getOrCreate<osgEarth::PolygonSymbol>();
        poly->fill()->color() = osgEarth::Color::White;

        // Clamp the buildings to the terrain.
        osg::ref_ptr<osgEarth::AltitudeSymbol> alt = buildingStyle.getOrCreate<osgEarth::AltitudeSymbol>();
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
        osg::ref_ptr<osgEarth::StyleSheet> styleSheet = new osgEarth::StyleSheet();
        styleSheet->addStyle(buildingStyle);
        styleSheet->addStyle(wallStyle);
        styleSheet->addStyle(roofStyle);

        // load a resource library that contains the building textures.
        osg::ref_ptr<osgEarth::ResourceLibrary> reslib = new osgEarth::ResourceLibrary("resources", TexResPath.toStdString());
        styleSheet->addResourceLibrary(reslib);

        // set up a paging layout for incremental loading. The tile size factor and
        // the visibility range combine to determine the tile size, such that
        // tile radius = max range / tile size factor.
        osgEarth::FeatureDisplayLayout layout;
        layout.tileSize() = 5500;

        osg::ref_ptr<osgEarth::FeatureModelLayer> buildingsLayer = new osgEarth::FeatureModelLayer();

        buildingsLayer->setName("Buildings");
        buildingsLayer->setFeatureSource(feature);
        buildingsLayer->setStyleSheet(styleSheet);
        // buildingsLayer->setLayout(layout);
        // buildingsLayer->setMaxVisibleRange(10000.0);

        feature->setName(LayerName.toStdString());
        buildingsLayer->setName(LayerName.toStdString());


        mapNode->getMap()->addLayer(buildingsLayer);
    }
}
