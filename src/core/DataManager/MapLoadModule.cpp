#include "MapLoadModule.h"

#include <QFile>
#include <QFileInfo>
#include <QXmlStreamReader>

#include <algorithm>
#include <osgDB/ReadFile>

#include <osgEarth/GDAL>
#include <osgEarth/OGRFeatureSource>
#include <osgEarth/FeatureModelLayer>
#include <osgEarth/XYZ>
#include <osgEarth/StyleSheet>
#include <osgEarth/FeatureImageLayer>
#include <osgEarth/WMS>

#ifdef OSGEARTH_HAVE_CESIUM_NODEKIT
#include <osgEarthCesium/CesiumIon>
#include <osgEarthCesium/CesiumLayer>
#include <osgEarthCesium/Settings>
#include <osgEarth/CesiumIon>
#endif

#include "osgviewerwidget.h"
#include "App.h"


// inline CSS, for use with the --script option
const char* styles_css =
R"(
    p {
        altitude-clamping: terrain-drape;
        render-backface-culling: false;
    }
    p1: p{ fill: #ff3f3f9f; }
    p2: p{ fill: #3fff3f9f; }
    p3: p{ fill: #3f3fff9f; }
    p4: p{ fill: #ff3fff9f; }
    p5: p{ fill: #ffff3f9f; }
)";

// JavaScript style selector, for use with the --script option
const char* script_source =
R"(
    function getStyleClass()
    {
        // Exclude any countries beginning with the letter A: 
        if ( feature.properties.name.charAt(0) === 'A' )
            return null;
                        
        // If it starts with the letter C, return an inline style:
        if ( feature.properties.name.charAt(0) == 'C' )
            return '{ _fill: #ffc838; stroke: #8f8838; extrusion-height: 250000; }';
                        
        // Otherwise, return a named style based on some calculations:
        var pop = parseFloat(feature.properties.pop);
        if      ( pop <= 14045470 )  return "p1";
        else if ( pop <= 43410900 )  return "p2";
        else if ( pop <= 97228750 )  return "p3";
        else if ( pop <= 258833000 ) return "p4";
        else                         return "p5";
    }
)";


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

bool MapLoadModule::LoadEarthFile(const QString& FilePath)
{
    osg::ref_ptr<osg::Node> node = osgDB::readRefNodeFile(FilePath.toStdString());
    if (!node.valid()) {
        OE_WARN << "Failed to load .earth file: " << FilePath.toStdString() << std::endl;
        return false;
    }

    osg::ref_ptr<osgEarth::MapNode> newMapNode = osgEarth::MapNode::findMapNode(node);
    OSGViewerWidget* viewer = App::getThreedWindow();
    osgEarth::MapNode* currentMapNode = viewer ? viewer->getMapNode() : nullptr;
    if (!newMapNode || !newMapNode->getMap()
        || !currentMapNode || !currentMapNode->getMap()) {
        OE_WARN << "Failed to load .earth file: invalid source or destination map" << std::endl;
        return false;
    }

    osgEarth::LayerVector newLayers;
    newMapNode->getMap()->getLayers(newLayers);

    osgEarth::Map* currentMap = currentMapNode->getMap();
    currentMap->clear();

    for (osgEarth::Layer* layer : newLayers) {
        if (layer) {
            layer->open();
            currentMap->addLayer(layer);
        }
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

    const auto openStatus = features->open();
    if (openStatus.isError())
    {
        OE_WARN << "Failed to open vector source: " << FilePath.toStdString()
                << " : " << openStatus.message() << std::endl;
        return;
    }

    const osgEarth::Color featureColor = getNextFeatureColor();
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
            ls->tessellationSize() = osgEarth::Distance(100, osgEarth::Units::KILOMETERS);

            osgEarth::AltitudeSymbol* alt = style.getOrCreate<osgEarth::AltitudeSymbol>();
            alt->clamping() = osgEarth::AltitudeSymbol::CLAMP_TO_TERRAIN;
            alt->technique() = osgEarth::AltitudeSymbol::TECHNIQUE_GPU;

            osgEarth::RenderSymbol* render = style.getOrCreate<osgEarth::RenderSymbol>();
            render->depthOffset().mutable_value().automatic() = true;
            break;
        }
        case osgEarth::Geometry::TYPE_POLYGON:
        {
            geomTypeStr = "POLYGON";

            osgEarth::PolygonSymbol* poly = style.getOrCreate<osgEarth::PolygonSymbol>();
            poly->fill().mutable_value().color() = featureColor;

            osgEarth::LineSymbol* ls = style.getOrCreateSymbol<osgEarth::LineSymbol>();
            ls->stroke().mutable_value().color() = osgEarth::Color::Black;
            ls->stroke().mutable_value().width() = osgEarth::Distance(0.5f, osgEarth::Units::PIXELS);
            ls->tessellationSize() = osgEarth::Distance(100, osgEarth::Units::KILOMETERS);

            osgEarth::AltitudeSymbol* alt = style.getOrCreate<osgEarth::AltitudeSymbol>();
            alt->clamping() = osgEarth::AltitudeSymbol::CLAMP_TO_TERRAIN;
            alt->technique() = osgEarth::AltitudeSymbol::TECHNIQUE_GPU;

            osgEarth::RenderSymbol* render = style.getOrCreate<osgEarth::RenderSymbol>();
            render->depthOffset().mutable_value().automatic() = true;
            break;

        }
        case osgEarth::Geometry::TYPE_MULTI:
        {
            geomTypeStr = "MULTI-POLYGON/MULTI-GEOMETRY";

            osgEarth::PolygonSymbol* poly = style.getOrCreate<osgEarth::PolygonSymbol>();
            poly->fill().mutable_value().color() = featureColor;
            
            osgEarth::LineSymbol* ls = style.getOrCreateSymbol<osgEarth::LineSymbol>();
            ls->stroke().mutable_value().color() = osgEarth::Color::Black;
            ls->stroke().mutable_value().width() = osgEarth::Distance(0.5f, osgEarth::Units::PIXELS);
            ls->tessellationSize() = osgEarth::Distance(100, osgEarth::Units::METERS);

            osgEarth::AltitudeSymbol* alt = style.getOrCreate<osgEarth::AltitudeSymbol>();
            alt->clamping() = osgEarth::AltitudeSymbol::CLAMP_TO_TERRAIN;
            alt->technique() = osgEarth::AltitudeSymbol::TECHNIQUE_GPU;
            alt->verticalOffset() = osgEarth::Distance(0.5, osgEarth::Units::METERS);

            osgEarth::RenderSymbol* render = style.getOrCreate<osgEarth::RenderSymbol>();
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

    osg::ref_ptr<osgEarth::StyleSheet> styleSheet = new osgEarth::StyleSheet();
    styleSheet->addStyle(style);

    osg::ref_ptr<osgEarth::FeatureImageLayer> featureImageLayer = new osgEarth::FeatureImageLayer();
    featureImageLayer->setName(displayName.toStdString());
    featureImageLayer->setFeatureSource(features);
    featureImageLayer->setStyleSheet(styleSheet);

    const osgEarth::Profile* mapProfile = mapNode->getMap()->getProfile();
    if (!mapProfile)
        mapProfile = osgEarth::Profile::create(osgEarth::Profile::GLOBAL_GEODETIC);
    if (mapProfile)
        featureImageLayer->options().profile() = mapProfile->toProfileOptions();

    mapNode->getMap()->addLayer(featureImageLayer.get());

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

     osg::ref_ptr<osgEarth::XYZImageLayer> remoteTiles = new osgEarth::XYZImageLayer();
     remoteTiles->setURL(url.toStdString());
     remoteTiles->setName(LayerName.toStdString().c_str());
     remoteTiles->setProfile(osgEarth::Profile::create(osgEarth::Profile::SPHERICAL_MERCATOR));
     remoteTiles->setOpacity(1.0f);
     mapNode->getMap()->addLayer(remoteTiles.get());
}

void MapLoadModule::LoadTMSLayer(osgEarth::MapNode* mapNode,
                                 QString url,
                                 QString LayerName,
                                 bool treatAsElevation)
{
    if (!mapNode || !mapNode->getMap()) {
        OE_WARN << "LoadTMSLayer failed: invalid map node" << std::endl;
        return;
    }

    url = url.trimmed();
    LayerName = LayerName.trimmed();
    if (url.isEmpty()) {
        OE_WARN << "LoadTMSLayer failed: URL is empty" << std::endl;
        return;
    }

    if (treatAsElevation) {
        osg::ref_ptr<osgEarth::TMSElevationLayer> tmsLayer = new osgEarth::TMSElevationLayer();
        tmsLayer->setURL(url.toStdString());
        if (!LayerName.isEmpty())
            tmsLayer->setName(LayerName.toStdString());
        mapNode->getMap()->addLayer(tmsLayer.get());
    }
    else {
        osg::ref_ptr<osgEarth::TMSImageLayer> tmsLayer = new osgEarth::TMSImageLayer();
        tmsLayer->setURL(url.toStdString());
        if (!LayerName.isEmpty())
            tmsLayer->setName(LayerName.toStdString());
        mapNode->getMap()->addLayer(tmsLayer.get());
    }
}

void MapLoadModule::LoadWMSLayer(osgEarth::MapNode* mapNode,
                                 QString url,
                                 QString wmsLayerName,
                                 QString LayerName)
{
    if (!mapNode || !mapNode->getMap()) {
        OE_WARN << "LoadWMSLayer failed: invalid map node" << std::endl;
        return;
    }

    url = url.trimmed();
    wmsLayerName = wmsLayerName.trimmed();
    LayerName = LayerName.trimmed();
    if (url.isEmpty() || wmsLayerName.isEmpty()) {
        OE_WARN << "LoadWMSLayer failed: URL and WMS layer name are required" << std::endl;
        return;
    }

    osg::ref_ptr<osgEarth::WMSImageLayer> wmsLayer = new osgEarth::WMSImageLayer();
    wmsLayer->setURL(url.toStdString());
    wmsLayer->setLayers(wmsLayerName.toStdString());
    wmsLayer->setName((LayerName.isEmpty() ? wmsLayerName : LayerName).toStdString());
    mapNode->getMap()->addLayer(wmsLayer.get());
}

void MapLoadModule::Load3Dbuildings(osgEarth::MapNode* mapNode, QString FilePath, QString heightAttr, QString LayerName, QString TexResPath)
{
    if (!mapNode || FilePath.trimmed().isEmpty())
        return;

    osg::ref_ptr<osgEarth::OGRFeatureSource> data = new osgEarth::OGRFeatureSource();
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

    osg::ref_ptr<osgEarth::StyleSheet> styleSheet = new osgEarth::StyleSheet();
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

    osg::ref_ptr<osgEarth::FeatureModelLayer> layer = new osgEarth::FeatureModelLayer();
    layer->setName(LayerName.toStdString());
    layer->setFeatureSource(data);
    layer->setStyleSheet(styleSheet);
    layer->setLayout(layout);
    layer->setMaxVisibleRange(20000.0);

    mapNode->getMap()->addLayer(layer);
}

#ifdef OSGEARTH_HAVE_CESIUM_NODEKIT
void MapLoadModule::LoadCesiumLayer(osgEarth::MapNode* mapNode,
                                     const QString& accessToken,
                                     CesiumLayerType type,
                                     unsigned int assetId,
                                     const QString& LayerName)
{
    if (!mapNode || !mapNode->getMap())
    {
        OE_WARN << "LoadCesiumLayer failed: invalid map node" << std::endl;
        return;
    }

    switch (type)
    {
        case CesiumLayerType::Ion3DTiles:
        {
            osg::ref_ptr<osgEarth::Cesium::CesiumNative3DTilesLayer> layer =
                new osgEarth::Cesium::CesiumNative3DTilesLayer();
            layer->setAssetId(assetId);
            layer->setName(LayerName.toStdString());
            // layer->setToken(token);
            mapNode->getMap()->addLayer(layer.get());

            break;
        }
        case CesiumLayerType::IonImagery:
        {
            osg::ref_ptr<osgEarth::CesiumIonImageLayer> layer =
                new osgEarth::CesiumIonImageLayer();
            layer->setAssetId(osgEarth::Stringify() << assetId);
            layer->setName(LayerName.toStdString());
            mapNode->getMap()->addLayer(layer.get());
            break;
        }
        case CesiumLayerType::IonTerrain:
        {
            osg::ref_ptr<osgEarth::CesiumIonTerrainMeshLayer> layer =
                new osgEarth::CesiumIonTerrainMeshLayer();
            layer->setAssetId(osgEarth::Stringify() << assetId);
            layer->setName(LayerName.toStdString());
            // layer->setToken(token);
            mapNode->getMap()->addLayer(layer.get());
            break;
        }
        default:
            OE_WARN << "Unsupported Cesium layer type." << std::endl;
            return;
    }

}
#endif