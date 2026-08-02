#ifndef MAPLOADMODULE_H
#define MAPLOADMODULE_H

#include <QString>
#include "osgearthcore_global.h"

#include <osgEarth/MapNode>


class OSGEARTHCORE_EXPORT MapLoadModule
{
public:
    struct LoadedVectorLayerInfo
    {
        QString layerName;
        QString filePath;
    };

    MapLoadModule();

    static void LoadEarthFile(QString FilePath);

    static void LoadRasters(osgEarth::MapNode* mapNode, QString FilePath, QString LayerName ="");

    static void LoadVectors(osgEarth::MapNode* mapNode, QString FilePath, QString LayerName = "");

    static void LoadElevation (osgEarth::MapNode* mapNode, QString FilePath, QString LayerName = "" );

    static void LoadXYZTiles(osgEarth::MapNode* mapNode, QString url, QString LayerName  = "XYZ Tiles");

    static void LoadTMSLayer(osgEarth::MapNode* mapNode, QString url, QString LayerName = "TMS Layer");
    
    static void Load3Dbuildings(osgEarth::MapNode *mapNode, QString FilePath, QString heightText, QString LayerName= " 3d scene", QString TexResPath =" ");


};

#endif // MAPLOADMODULE_H
