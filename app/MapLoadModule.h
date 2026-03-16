#ifndef LOADLAYERSONMAP_H
#define LOADLAYERSONMAP_H

#include <QDialog>
#include "ui_MapLoadModule.h"

#include <osgEarth/MapNode>
#include <osgEarth/Map>
#include <osgEarth/EarthManipulator>
#include <osgEarth/Layer>

#include <osgEarth/ImageLayer>
#include <osgEarth/XYZ>

class MapLoadModule : public QDialog
{
    Q_OBJECT
public:
    explicit MapLoadModule(osgEarth::MapNode* mapNode = nullptr,QWidget* parent = nullptr);

    ~MapLoadModule();

    static void LoadRasters(osgEarth::MapNode* mapNode, QString FilePath, QString LayerName ="");
    static void LoadVectors(osgEarth::MapNode* mapNode, QString FilePath, QString LayerName = "");
    static void LoadElevation (osgEarth::MapNode* mapNode, QString FilePath, QString LayerName = "" );
    static void LoadXYZTiles(osgEarth::MapNode* mapNode, QString url, QString LayerName  = "XYZ Tiles");

private slots:
    void on_pb_openMap_clicked();

private:
    enum class FileType
    {
        RASTER,
        VECTOR,
        ELEVATION,
        MODEL3D,
        TEXTURE,
        ALLFILES
    };

private:
    Ui::MapLoadModule* ui;

    osgEarth::MapNode* _mapNode{nullptr};
    QString OpenFiles(FileType Type);
    void _initConnections();
    void getAttributes(QString FilePath);


};

#endif // LOADLAYERSONMAP_H
