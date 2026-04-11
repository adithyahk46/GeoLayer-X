#ifndef MAPLOADMODULE_H
#define MAPLOADMODULE_H


class MapLoadModule
{
public:
    MapLoadModule();

    static void LoadRasters(osgEarth::MapNode* mapNode, QString FilePath, QString LayerName ="");

    static void LoadVectors(osgEarth::MapNode* mapNode, QString FilePath, QString LayerName = "");

    static void LoadElevation (osgEarth::MapNode* mapNode, QString FilePath, QString LayerName = "" );

    static void LoadXYZTiles(osgEarth::MapNode* mapNode, QString url, QString LayerName  = "XYZ Tiles");

    static void Load3Dbuildings(osgEarth::MapNode *mapNode, QString FilePath, QString heightText, QString LayerName= " 3d scene", QString TexResPath =" ");
};

#endif // MAPLOADMODULE_H
