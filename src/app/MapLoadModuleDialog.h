#ifndef LOADLAYERSONMAP_H
#define LOADLAYERSONMAP_H

#include <QDialog>
#include "ui_MapLoadModuleDialog.h"

#include <osgEarth/MapNode>

class MapLoadModuleDialog : public QDialog
{
    Q_OBJECT
public:
    explicit MapLoadModuleDialog(QWidget* parent = nullptr);

    ~MapLoadModuleDialog();

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
    Ui::MapLoadModuleDialog* ui;

    QString OpenFiles(FileType Type);
    void _initConnections();
    void getAttributes(QString FilePath);


};

#endif // LOADLAYERSONMAP_H
