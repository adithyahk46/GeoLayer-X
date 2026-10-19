#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QList>
#include <memory>

#include <PluginManager/PluginManager.h>

#include "MapController/GLXEarthManipulator.h"

#include "osgviewerwidget.h"

#include "MapLoadModuleDialog.h"
#include "GLXProjectsDialog.h"
#include "LayerManagerWidget.h"
#include "StatusbarHandlerWidget.h"

class QActionGroup;
class QAction;
class PluginsDockWidget;
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
    void showWelcomeScreen();

    void setApplicationTheme(bool dark);

    void saveAllSettings();

    QMenu* dockPanelsMenu() const;


    void restoreAllSettings();
private slots:
    void osgViewerSetup();

private:
    void keyPressEvent(QKeyEvent* event) override;

    Ui::MainWindow *ui;
    std::ofstream *_log;

    OSGViewerWidget* osgwidget3D= nullptr;
#if GLX_BUILD_2D_MAP
    OSGViewerWidget* osgwidget2D = nullptr;
    QSplitter* mapViewSplitter = nullptr;
#endif
    StatusbarHandlerWidget* mStatusBarWidget = nullptr;
    osg::ref_ptr<GLXEarthManipulator> _earthManipulator = nullptr;
    osgEarth::MapCallback* _layerManagerCallback = nullptr;
    PluginManager* _pluginManager = nullptr;
    LayerManagerWidget* layerManager = nullptr;
    PluginsDockWidget* pluginsDockWidget = nullptr;
    QActionGroup* m_drawingActionGroup = nullptr;
    QList<QAction*> _pluginActions;

//declare private functions here
    void initCore();
    void initSignalsAndSlots();
    void initLogs();
    void initPlugins();
};

#endif // MAINWINDOW_H
