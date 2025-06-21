#ifndef MAPLOADMODULE_H
#define MAPLOADMODULE_H

#include <QDialog>
#include "ui_MapLoadModule.h"

#include <osgEarth/MapNode>
#include <osgEarth/Map>
#include <osgEarth/EarthManipulator>
#include <osgEarth/Layer>

class MapLoadModule : public QDialog
{
    Q_OBJECT
public:
    explicit MapLoadModule(osgEarth::MapNode* mapNode = nullptr, osgEarth::EarthManipulator* ManIp = nullptr);
    ~MapLoadModule();

    void LoadRasters(QString FilePath, QString LayerName ="");
    void LoadVectors(QString FilePath, QString LayerName = "");
    MapLoadModule *getInstance(osgEarth::MapNode *mapNode);
    void LoadElevation(QString FilePath, QString LayerName);
private slots:
    void on_pb_openMap_clicked();

private:
    enum class FileType
    {
        RASTER,
        VECTOR,
        ELEVATION,
        TEXTURE,
        ALLFILES
    };

private:
    Ui::MapLoadModule* ui;
    osgEarth::MapNode* _mapNode{nullptr};
    osgEarth::EarthManipulator* _manip{nullptr};
    QString OpenFiles(FileType Type);
    void _initConnections();
    void getAttributes(QString FilePath);
};

#endif // MAPLOADMODULE_H
