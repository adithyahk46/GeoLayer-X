#ifndef LOADLAYERSONMAP_H
#define LOADLAYERSONMAP_H

#include <QDialog>
#include "ui_MapLoadModuleDialog.h"

#include <osgEarth/MapNode>

class QListWidget;
class QStackedWidget;

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
    QListWidget* _tabSelector = nullptr;
    QStackedWidget* _tabContent = nullptr;

    QString OpenFiles(FileType Type);
    void _initConnections();
    void getAttributes(QString FilePath);
    void setupHorizontalTabs();


};

#endif // LOADLAYERSONMAP_H
