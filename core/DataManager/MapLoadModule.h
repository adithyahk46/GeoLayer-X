#ifndef MAPLOADMODULE_H
#define MAPLOADMODULE_H

#include <QString>
#include <QVector>
#include "../core_global.h"

namespace osgEarth {
class MapNode;
}


class GEOLAYERCORE_EXPORT MapLoadModule
{
public:
    struct LoadedVectorLayerInfo
    {
        QString layerName;
        QString filePath;
    };

    MapLoadModule();

    static void LoadRasters(osgEarth::MapNode* mapNode, QString FilePath, QString LayerName ="");

    static void LoadVectors(osgEarth::MapNode* mapNode, QString FilePath, QString LayerName = "");

    static void LoadElevation (osgEarth::MapNode* mapNode, QString FilePath, QString LayerName = "" );

    static void LoadXYZTiles(osgEarth::MapNode* mapNode, QString url, QString LayerName  = "XYZ Tiles");

    static void Load3Dbuildings(osgEarth::MapNode *mapNode, QString FilePath, QString heightText, QString LayerName= " 3d scene", QString TexResPath =" ");

    static QVector<LoadedVectorLayerInfo> loadedVectorLayers();

private:
    static QVector<LoadedVectorLayerInfo> s_loadedVectorLayers;
};

#endif // MAPLOADMODULE_H
