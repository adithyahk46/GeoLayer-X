#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <PluginManager/PluginManager.h>

#include "osgviewerwidget.h"

#include "MapLoadModuleDialog.h"
#include "LayerManagerWidget.h"
#include "StatusbarHandlerWidget.h"

#include "Tools/EnvironmentGUI.h"
#include "Tools/ViewpointsGUI.h"
#include "Tools/MapReadoutsDockWidget.h"


class QCloseEvent;
class QString;
class PluginsDockWidget;
class Extrusion3D;
class ProjectWelcomeScreen;
class ProjectManager;
class QSplitter;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void initAll();
    // This function will show the welcome screen to the user
    void showWelcomeScreen();

    void setApplicationTheme(bool dark);

    QMenu* dockPanelsMenu() const;

    void restoreAllSettings();
private slots:
    void osgViewerSetup();

    void createShapeFile();
    void setMapProjection(bool flat);

private:
    void closeEvent(QCloseEvent* event) override;
    void handleProjectTransition(const QString& action, const QString& filePath);

    Ui::MainWindow *ui;

    OSGViewerWidget* osgwidget3D= nullptr;
#if GLX_BUILD_2D_MAP
    OSGViewerWidget* osgwidget2D = nullptr;
    QSplitter* mapViewSplitter = nullptr;
#endif
    StatusbarHandlerWidget* mStatusBarWidget = nullptr;
    osgEarth::MapCallback* _layerManagerCallback = nullptr;
    PluginManager* _pluginManager = nullptr;
    LayerManagerWidget* layerManager = nullptr;
    PluginsDockWidget* pluginsDockWidget = nullptr;
    ProjectManager* m_projectManager = nullptr;
    ProjectWelcomeScreen* m_ProjectWelcomeScreen = nullptr;
    bool m_allowApplicationClose = false;
    bool m_flatMapProjection = false;
     EnvironmentGUI* envGui = nullptr;
     ViewpointsGUI* viewpointsGui = nullptr;
     MapReadoutsDockWidget* mapReadoutsDock = nullptr;
     Extrusion3D* extrusion3D = nullptr;

    void initCore();
    void initSignalsAndSlots();
    void initPlugins();
};

#endif // MAINWINDOW_H
