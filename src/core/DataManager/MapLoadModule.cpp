#include "MapLoadModule.h"

#include <QFile>
#include <QFileInfo>
#include <QXmlStreamReader>

#include <osgDB/ReadFile>

#include <osgEarth/GDAL>
#include <osgEarth/OGRFeatureSource>
#include <osgEarth/FeatureModelLayer>
#include <osgEarth/XYZ>
#include <osgEarth/StyleSheet>
#include <osgEarth/FeatureImageLayer>

#include "osgviewerwidget.h"
#include "App.h"

namespace
{
// Keeps generating a different color for each feature layer loaded by LoadVectors.
static unsigned sNextFeatureColor = 0;

osgEarth::Color getNextFeatureColor()
{
    const unsigned index = sNextFeatureColor++;
    const float hue = static_cast<float>((index * 137 + 20) % 360) / 360.0f;
    const float saturation = 0.72f;
    const float value = 0.95f;

    const float sector = hue * 6.0f;
    const unsigned sectorIndex = static_cast<unsigned>(sector) % 6;
    const float fraction = sector - static_cast<float>(static_cast<unsigned>(sector));
    const float p = value * (1.0f - saturation);
    const float q = value * (1.0f - saturation * fraction);
    const float t = value * (1.0f - saturation * (1.0f - fraction));

    float r = 0.0f, g = 0.0f, b = 0.0f;
    switch (sectorIndex)
    {
        case 0: r = value; g = t; b = p; break;
        case 1: r = q; g = value; b = p; break;
        case 2: r = p; g = value; b = t; break;
        case 3: r = p; g = q; b = value; break;
        case 4: r = t; g = p; b = value; break;
        default: r = value; g = p; b = q; break;
    }

    return osgEarth::Color(r, g, b, 0.75f);
}
}

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

    if (const osgEarth::FeatureProfile* profile = features->getFeatureProfile())
    {
        const osgEarth::GeoExtent& extent = features->getExtent();
        const osgEarth::SpatialReference* srs = extent.getSRS();
        qDebug() << "Vector profile SRS:" << (srs ? QString::fromStdString(srs->getName()) : QStringLiteral("unknown"))
                 << "geographic:" << (srs ? srs->isGeographic() : false)
                 << "extent:" << extent.xMin() << extent.yMin() << extent.xMax() << extent.yMax();
    }

    const osgEarth::Color featureColor = getNextFeatureColor();
    qDebug() << "New feature color:" << QString::fromStdString(featureColor.toHTML());

    osgEarth::Style style;
    QString geomTypeStr = "UNKNOWN";

    const osgEarth::Geometry::Type type = features->getGeometryType();
    switch (type)
    {
        case osgEarth::Geometry::TYPE_UNKNOWN:
        {
            geomTypeStr = "UNKNOWN";

            osgEarth::LineSymbol* ls = style.getOrCreateSymbol<osgEarth::LineSymbol>();
            ls->stroke().mutable_value().color() = featureColor;
            ls->stroke().mutable_value().width() = osgEarth::Distance(2.0, osgEarth::Units::PIXELS);
            break;
        }
        case osgEarth::Geometry::TYPE_POINT:
        case osgEarth::Geometry::TYPE_POINTSET:
        {
            geomTypeStr = (type == osgEarth::Geometry::TYPE_POINT) ? "POINT" : "MULTIPOINT";

            osgEarth::PointSymbol* pointSymbol = style.getOrCreateSymbol<osgEarth::PointSymbol>();
            style.getOrCreate<osgEarth::AltitudeSymbol>()->clamping() = osgEarth::AltitudeSymbol::CLAMP_TO_TERRAIN;

            pointSymbol->fill() = featureColor;
            pointSymbol->size() = 10.0f;
            pointSymbol->smooth() = 10.0f;
            break;
        }
        case osgEarth::Geometry::TYPE_LINESTRING:
        case osgEarth::Geometry::TYPE_RING:
        {
            geomTypeStr = (type == osgEarth::Geometry::TYPE_LINESTRING) ? "LINESTRING" : "RING";

            osgEarth::LineSymbol* ls = style.getOrCreateSymbol<osgEarth::LineSymbol>();
            ls->stroke().mutable_value().color() = featureColor;
            ls->stroke().mutable_value().width() = osgEarth::Distance(2.0f, osgEarth::Units::PIXELS);
            ls->tessellationSize() = osgEarth::Distance(100, osgEarth::Units::METERS);

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
            poly->fill().mutable_value().color() = featureColor;

            // osgEarth::LineSymbol* outline = style.getOrCreate<osgEarth::LineSymbol>();
            // outline->stroke().mutable_value().color() = osgEarth::Color::Yellow;
            // outline->stroke().mutable_value().width() = osgEarth::Distance(1.5, osgEarth::Units::PIXELS);

            osgEarth::AltitudeSymbol* alt = style.getOrCreate<osgEarth::AltitudeSymbol>();
            alt->clamping() = osgEarth::AltitudeSymbol::CLAMP_TO_TERRAIN;
            alt->technique() = osgEarth::AltitudeSymbol::TECHNIQUE_DRAPE;
            alt->binding() = osgEarth::AltitudeSymbol::BINDING_CENTROID;
            alt->verticalOffset() = osgEarth::Distance(0.5, osgEarth::Units::METERS);

            osgEarth::RenderSymbol* render = style.getOrCreate<osgEarth::RenderSymbol>();
            render->lighting() = false;
            render->depthOffset()->enabled() = true;
            render->depthOffset().mutable_value().automatic() = true;
            break;
        }
        case osgEarth::Geometry::TYPE_MULTI:
        {
            geomTypeStr = "MULTI-POLYGON/MULTI-GEOMETRY";

            osgEarth::PolygonSymbol* poly = style.getOrCreate<osgEarth::PolygonSymbol>();
            poly->fill().mutable_value().color() = featureColor;

            // osgEarth::LineSymbol* outline = style.getOrCreate<osgEarth::LineSymbol>();
            // outline->stroke().mutable_value().color() = osgEarth::Color::Yellow;
            // outline->stroke().mutable_value().width() = osgEarth::Distance(1.5, osgEarth::Units::PIXELS);

            
            osgEarth::LineSymbol* ls = style.getOrCreateSymbol<osgEarth::LineSymbol>();
            ls->stroke().mutable_value().color() = featureColor;
            ls->stroke().mutable_value().width() = osgEarth::Distance(2.0f, osgEarth::Units::PIXELS);
            ls->tessellationSize() = osgEarth::Distance(100, osgEarth::Units::METERS);

            osgEarth::AltitudeSymbol* alt = style.getOrCreate<osgEarth::AltitudeSymbol>();
            alt->clamping() = osgEarth::AltitudeSymbol::CLAMP_TO_TERRAIN;
            alt->technique() = osgEarth::AltitudeSymbol::TECHNIQUE_DRAPE;
            alt->binding() = osgEarth::AltitudeSymbol::BINDING_CENTROID;
            alt->verticalOffset() = osgEarth::Distance(0.5, osgEarth::Units::METERS);

            osgEarth::RenderSymbol* render = style.getOrCreate<osgEarth::RenderSymbol>();
            render->lighting() = false;
            render->depthOffset()->enabled() = true;
            render->depthOffset().mutable_value().automatic() = true;
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

    // FeatureImageLayer is the rasterized-vector option from osgearth_features.
    // Do not let it infer its profile from a geographic source's SRS alone: a
    // geographic projected-unbounded SRS cannot provide a Profile without bounds.
    // Explicitly inherit the map profile so all image tiles use the same SRS.
    osg::ref_ptr<osgEarth::FeatureImageLayer> featureImageLayer = new osgEarth::FeatureImageLayer();
    featureImageLayer->setName(displayName.toStdString());
    featureImageLayer->setFeatureSource(features);
    featureImageLayer->setStyleSheet(styleSheet);
    featureImageLayer->options().backgroundColor() = osgEarth::Color::Transparent;

    const osgEarth::Profile* mapProfile = mapNode->getMap()->getProfile();
    if (!mapProfile)
        mapProfile = osgEarth::Profile::create(osgEarth::Profile::GLOBAL_GEODETIC);
    if (mapProfile)
        featureImageLayer->options().profile() = mapProfile->toProfileOptions();

    if (mapProfile && mapProfile->getSRS())
        qDebug() << "FeatureImage profile SRS:" << QString::fromStdString(mapProfile->getSRS()->getName());

    mapNode->getMap()->addLayer(featureImageLayer);

    if (featureImageLayer->getStatus().isError())
    {
        OE_WARN << featureImageLayer->getName() << " : " << featureImageLayer->getStatus().toString() << std::endl;
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

void MapLoadModule::Load3Dbuildings(osgEarth::MapNode* mapNode, QString FilePath, QString heightAttr, QString LayerName, QString TexResPath)
{
    if (!mapNode || FilePath.trimmed().isEmpty())
        return;

    osgEarth::OGRFeatureSource* data = new osgEarth::OGRFeatureSource();
    data->setName(LayerName.toStdString());
    data->setURL(FilePath.toStdString());

    osgEarth::Style buildingStyle;
    buildingStyle.setName("default");

    osgEarth::ExtrusionSymbol* extrusion = buildingStyle.getOrCreate<osgEarth::ExtrusionSymbol>();

    QString fieldName = heightAttr.trimmed();
    if (fieldName.startsWith('[') && fieldName.endsWith(']'))
        fieldName = fieldName.mid(1, fieldName.size() - 2).trimmed();

    bool isNumeric = false;
    const double height = fieldName.toDouble(&isNumeric);
    if (isNumeric)
        extrusion->height() = osgEarth::Distance(height, osgEarth::Units::METERS);
    else if (!fieldName.isEmpty())
        extrusion->height() = osgEarth::Expression<osgEarth::Distance>(
            QString("[%1]").arg(fieldName).toStdString());
    else
        extrusion->height() = osgEarth::Distance(10.0, osgEarth::Units::METERS);

    extrusion->flatten() = true;
    extrusion->wallStyleName() = "building-wall";
    extrusion->roofStyleName() = "building-roof";

    osgEarth::PolygonSymbol* poly = buildingStyle.getOrCreate<osgEarth::PolygonSymbol>();
    poly->fill()->color() = osgEarth::Color::White;

    osgEarth::AltitudeSymbol* altitude = buildingStyle.getOrCreate<osgEarth::AltitudeSymbol>();
    altitude->clamping() = osgEarth::AltitudeSymbol::CLAMP_TO_TERRAIN;
    altitude->binding() = osgEarth::AltitudeSymbol::BINDING_VERTEX;

    osgEarth::Style wallStyle;
    wallStyle.setName("building-wall");
    osgEarth::SkinSymbol* wallSkin = wallStyle.getOrCreate<osgEarth::SkinSymbol>();
    wallSkin->library() = "us_resources";
    wallSkin->addTag("building");
    wallSkin->randomSeed() = 1;

    osgEarth::Style roofStyle;
    roofStyle.setName("building-roof");
    osgEarth::SkinSymbol* roofSkin = roofStyle.getOrCreate<osgEarth::SkinSymbol>();
    roofSkin->library() = "us_resources";
    roofSkin->addTag("rooftop");
    roofSkin->randomSeed() = 1;
    roofSkin->isTiled() = true;

    osgEarth::StyleSheet* styleSheet = new osgEarth::StyleSheet();
    styleSheet->addStyle(buildingStyle);
    styleSheet->addStyle(wallStyle);
    styleSheet->addStyle(roofStyle);

    if (!TexResPath.trimmed().isEmpty() && QFileInfo::exists(TexResPath))
    {
        osgEarth::ResourceLibrary* resourceLibrary =
            new osgEarth::ResourceLibrary("us_resources", TexResPath.toStdString());
        styleSheet->addResourceLibrary(resourceLibrary);
    }

    osgEarth::FeatureDisplayLayout layout;
    layout.tileSize() = 500;

    osgEarth::FeatureModelLayer* layer = new osgEarth::FeatureModelLayer();
    layer->setName(LayerName.toStdString());
    layer->setFeatureSource(data);
    layer->setStyleSheet(styleSheet);
    layer->setLayout(layout);
    layer->setMaxVisibleRange(20000.0);

    mapNode->getMap()->addLayer(layer);
}
