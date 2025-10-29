#ifndef LOADLAYERSONMAP_H
#define LOADLAYERSONMAP_H

#include <QDialog>
#include "ui_LoadLayersOnMap.h"

#include <osgEarth/MapNode>
#include <osgEarth/Map>
#include <osgEarth/EarthManipulator>
#include <osgEarth/Layer>

#include <osgEarth/ImageLayer>
#include <osgEarth/XYZ>
#include <QProcess>

class LoadLayersOnMap : public QDialog
{
    Q_OBJECT
public:
    explicit LoadLayersOnMap(QWidget* parent = nullptr, osgEarth::MapNode* mapNode = nullptr, osgEarth::EarthManipulator* ManIp = nullptr);
    ~LoadLayersOnMap();

    void LoadRasters(osgEarth::MapNode* mapNode, QString FilePath, QString LayerName ="");
    void LoadVectors(osgEarth::MapNode* mapNode, QString FilePath, QString LayerName = "");
    static LoadLayersOnMap* getInstance(osgEarth::MapNode *mapNode);
    void LoadElevation (osgEarth::MapNode* mapNode, QString FilePath, QString LayerName );
    void LoadXYZTiles(osgEarth::MapNode* mapNode, QString url, QString LayerName  = "XYZ Tiles");

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
    osgEarth::EarthManipulator* _manip{nullptr};
    QString OpenFiles(FileType Type);
    void _initConnections();
    void getAttributes(QString FilePath);

    struct ServerInfo {
            int port;
            QProcess* process;
            int refCount;
        };

    QMap<QString, ServerInfo> m_servers;

};

#endif // LOADLAYERSONMAP_H
