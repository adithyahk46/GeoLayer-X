#include "MapLoadModule.h"


MapLoadModule::MapLoadModule()
{

}


void MapLoadModule::LoadRasters(osgEarth::MapNode* mapNode, QString FilePath,  QString LayerName )
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

void MapLoadModule::LoadVectors(osgEarth::MapNode* mapNode, QString FilePath , QString LayerName)
{

    QFileInfo finfo(FilePath);
    QString MapName = finfo.baseName().toStdString().c_str();
    QString fileExt = finfo.suffix().toStdString().c_str();

    osg::ref_ptr<osgEarth::OGRFeatureSource> features = new osgEarth::OGRFeatureSource();
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

                osg::ref_ptr<osgEarth::PolygonSymbol> poly = style.getOrCreate<osgEarth::PolygonSymbol>();
                poly->fill()->color() = osgEarth::Color::White;

                // Clamp the buildings to the terrain.
                osg::ref_ptr<osgEarth::AltitudeSymbol> alt = style.getOrCreate<osgEarth::AltitudeSymbol>();
                alt->clamping() = alt->CLAMP_TO_TERRAIN;
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
    osg::ref_ptr<osgEarth::StyleSheet> styleSheet = new osgEarth::StyleSheet();
    styleSheet->addStyle(style);

    osg::ref_ptr<osgEarth::FeatureModelLayer> featureModelLayer = new osgEarth::FeatureModelLayer();

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
    layout.tileSize() = 4500;

    featureModelLayer->setFeatureSource(features);
    featureModelLayer->setStyleSheet(styleSheet);
    // featureModelLayer->setMaxVisibleRange(10000.0);
    // featureModelLayer->setLayout(layout);
    mapNode->getMap()->addLayer(featureModelLayer);
    }

}

void MapLoadModule::LoadElevation(osgEarth::MapNode* mapNode,QString FilePath, QString LayerName)
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

void MapLoadModule::LoadXYZTiles(osgEarth::MapNode* mapNode, QString url , QString LayerName){

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

void MapLoadModule::Load3Dbuildings(osgEarth::MapNode* mapNode, QString FilePath, QString heightText, QString LayerName, QString TexResPath ){

    osg::ref_ptr<osgEarth::OGRFeatureSource> feature = new osgEarth::OGRFeatureSource();
    feature->setURL(FilePath.toStdString());

    if(feature){
        osgEarth::Style buildingStyle;
        buildingStyle.setName("building Style");

        // Extrude the shapes into 3D buildings.
        osg::ref_ptr<osgEarth::ExtrusionSymbol> extrusion = buildingStyle.getOrCreate<osgEarth::ExtrusionSymbol>();
        // extrusion->setHeightExpression(NumericExpression(heightText));
        // extrusion->heightExpression() = osgEarth::NumericExpression(heightText.toStdString().c_str());
        extrusion->flatten() = false;
        extrusion->wallStyleName() = "building-wall";
        extrusion->roofStyleName() = "building-roof";

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
        buildingsLayer->setLayout(layout);
        // buildingsLayer->setMaxVisibleRange(10000.0);

        feature->setName(LayerName.toStdString());
        buildingsLayer->setName(LayerName.toStdString());


        mapNode->getMap()->addLayer(buildingsLayer);
    }
}
