#include "legends.h"




#include <QMenu>
#include <QMessageBox>
#include <QDebug>

#include <osgEarth/ExtrusionSymbol>
#include <osgEarth/Expression>
#include <osgEarth/PolygonSymbol>
#include <osgEarth/Color>
#include <osgEarth/AltitudeSymbol>
#include <osgEarth/Skins>
#include <osgEarth/StyleSheet>
#include <osgEarth/ResourceLibrary>

#include <osgEarth/Notify>


using namespace osgEarth;


template class Legends<osgEarth::FeatureModelLayer>;
template class Legends<osgEarth::GDALElevationLayer>;
template class Legends<osgEarth::GDALImageLayer>;

template <typename T>

#define RESOURCE_LIB_URL "D:/OSG1/osgEarth/data/resources/textures_us/catalog.xml"


Legends<T>::Legends(T* mapLayer, MapNode* mapNode, EarthManipulator* manip, QWidget* parent)
    : LegendBase(parent), m_mapLayer(mapLayer)
    ,_manip(manip)
    ,_mapNode(mapNode)
{

    if (m_mapLayer){
        setText(QString::fromStdString(m_mapLayer->getName()));
        qDebug()<<QString::fromStdString(m_mapLayer->getName());
    }
    else
        setText("Unnamed Layer");

    setChecked(true);

    connect(this, &QCheckBox::toggled, this, [this]() {
        if (m_mapLayer) m_mapLayer->setVisible(isChecked());
    });

    // Context menu setup
    menu.addAction("Zoom to Layer", [=]() {
        const osgEarth::SpatialReference* wgs84 = osgEarth::SpatialReference::get("wgs84");

        osgEarth::GeoExtent layerExtent = mapLayer->getExtent().transform(wgs84);
        if (!layerExtent.isValid())
        {
            qDebug() << "Error: Border layer extent is invalid.";
            return;
        }

        GeoPoint p1(layerExtent.getSRS(), layerExtent.xMin(), layerExtent.yMin(),0.0);
        GeoPoint p2(layerExtent.getSRS(), layerExtent.xMax(), layerExtent.yMax(), 0.0);

        GeoPoint center(layerExtent.getSRS(),layerExtent.getCentroid().x(),layerExtent.getCentroid().y(),0.0);

        // qDebug()<<layerExtent.getCentroid().x()<<" " <<layerExtent.getCentroid().y() <<center.x()<<" " <<center.y() << p1.distanceTo(p2) ;
        // Optional: set viewpoint with calculated range
        Viewpoint vp("",center.x(),center.y(),0.0f,0.0f,-90.0f, p1.distanceTo(p2)<1000?1000:p1.distanceTo(p2) );
        manip->setViewpoint(vp,1.0); // 1.5 seconds animation


    });
    menu.addAction("Remove Layer", [=]() {
        _mapNode->getMap()->removeLayer(mapLayer);
        QMessageBox::information(this, "Success", "Layer removed successfully!");

        this->deleteLater();
    });

    menu.addAction("Move to Top", [=]() {
    _mapNode->getMap()->moveLayer(m_mapLayer,_mapNode->getMap()->getNumLayers() - 1);
    });

    menu.addAction("Move to Bottom", [=]() {
        _mapNode->getMap()->moveLayer(m_mapLayer,1);

    });

    menu.addAction("Apply 3d Style",[=](){

        Style buildingStyle;
        buildingStyle.setName("default");

        // Extrude the shapes into 3D buildings.
        ExtrusionSymbol* extrusion = buildingStyle.getOrCreate<ExtrusionSymbol>();
        extrusion->heightExpression() = NumericExpression("10 * max([floor], 1)");
        extrusion->flatten() = true;
        extrusion->wallStyleName() = "building-wall";
        extrusion->roofStyleName() = "building-roof";

        PolygonSymbol* poly = buildingStyle.getOrCreate<PolygonSymbol>();
        poly->fill()->color() = Color::White;

        // Clamp the buildings to the terrain.
        AltitudeSymbol* alt = buildingStyle.getOrCreate<AltitudeSymbol>();
        alt->clamping() = alt->CLAMP_TO_TERRAIN;
        alt->binding() = alt->BINDING_VERTEX;

        // a style for the wall textures:
        Style wallStyle;
        wallStyle.setName("building-wall");
        SkinSymbol* wallSkin = wallStyle.getOrCreate<SkinSymbol>();
        wallSkin->library() = "us_resources";
        wallSkin->addTag("building");
        wallSkin->randomSeed() = 1;

        // a style for the rooftop textures:
        Style roofStyle;
        roofStyle.setName("building-roof");
        SkinSymbol* roofSkin = roofStyle.getOrCreate<SkinSymbol>();
        roofSkin->library() = "us_resources";
        roofSkin->addTag("rooftop");
        roofSkin->randomSeed() = 1;
        roofSkin->isTiled() = true;

        // assemble a stylesheet and add our styles to it:
        StyleSheet* styleSheet = new StyleSheet();
        styleSheet->addStyle(buildingStyle);
        styleSheet->addStyle(wallStyle);
        styleSheet->addStyle(roofStyle);

        // load a resource library that contains the building textures.
        ResourceLibrary* reslib = new ResourceLibrary("us_resources", RESOURCE_LIB_URL);
        styleSheet->addResourceLibrary(reslib);

        // set up a paging layout for incremental loading. The tile size factor and
        // the visibility range combine to determine the tile size, such that
        // tile radius = max range / tile size factor.
        FeatureDisplayLayout layout;
        layout.tileSize() = 500;

        _mapNode->getMap()->removeLayer(m_mapLayer);

        if (m_mapLayer->getTypeName() == "class osgEarth::FeatureModelLayer"){
            qDebug()<<"enter";
            _mapNode->getMap()->removeLayer(m_mapLayer);

            FeatureModelLayer* fml = dynamic_cast<FeatureModelLayer*>(m_mapLayer);
                if (fml) {
                    qDebug()<<"enter 2";

            fml->setStyleSheet(styleSheet);
            fml->setLayout(layout);
            fml->setMaxVisibleRange(20000.0);

            _mapNode->getMap()->addLayer(fml);

                }

        }


    });


}


template <typename T>
void Legends<T>::mousePressEvent(QMouseEvent *event)
{
    if (event->button() == Qt::RightButton)
    {
        menu.exec(this->mapToGlobal(event->pos()));
    }
    else
    {
        QCheckBox::mousePressEvent(event);  // Preserve default behavior
    }
}
// Explicit template instantiations
template class Legends<osgEarth::FeatureModelLayer>;
template class Legends<osgEarth::GDALElevationLayer>;
template class Legends<osgEarth::GDALImageLayer>;

