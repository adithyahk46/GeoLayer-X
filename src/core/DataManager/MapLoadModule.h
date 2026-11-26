#ifndef MAPLOADMODULE_H
#define MAPLOADMODULE_H

#include <QString>
#include "osgearthcore_global.h"

#include <osgEarth/MapNode>
#include <osgEarth/TMS>


class OSGEARTHCORE_EXPORT MapLoadModule
{
public:
    struct LoadedVectorLayerInfo
    {
        QString layerName;
        QString filePath;
    };

    enum class CesiumLayerType
    {
        IonImagery,
        Ion3DTiles,
        IonTerrain
    };

    static bool LoadEarthFile(const QString& FilePath);

    static void LoadRasters(osgEarth::MapNode* mapNode, QString FilePath, QString LayerName ="");

    static void LoadVectors(osgEarth::MapNode* mapNode, QString FilePath, QString LayerName = "");

    static void LoadElevation (osgEarth::MapNode* mapNode, QString FilePath, QString LayerName = "" );

    static void LoadXYZTiles(osgEarth::MapNode* mapNode, QString url, QString LayerName  = "XYZ Tiles");

    static void LoadTMSLayer(osgEarth::MapNode* mapNode,
                             QString url,
                             QString LayerName = "TMS Layer",
                             bool treatAsElevation = false);

    static void LoadWMSLayer(osgEarth::MapNode* mapNode,
                             QString url,
                             QString wmsLayerName,
                             QString LayerName = "");
    
    static void Load3Dbuildings(osgEarth::MapNode *mapNode, QString FilePath, QString heightText, QString LayerName= " 3d scene", QString TexResPath =" ");
    
#ifdef OSGEARTH_HAVE_CESIUM_NODEKIT
    static void LoadCesiumLayer(osgEarth::MapNode* mapNode,
                                 const QString& accessToken,
                                 CesiumLayerType type,
                                 unsigned int assetId,
                                 const QString& LayerName = "Cesium Layer");
#endif

};

#endif // MAPLOADMODULE_H
