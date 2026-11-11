#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QList>
#include <memory>

#include <PluginManager/PluginManager.h>

#include "MapController/GLXEarthManipulator.h"

#include "osgviewerwidget.h"

#include "MapLoadModuleDialog.h"
#include "LayerManagerWidget.h"
#include "StatusbarHandlerWidget.h"

class QActionGroup;
class QAction;
class QCloseEvent;
class QString;
class PluginsDockWidget;
class ProjectWelcomeScreen;
class ProjectManager;
class QSplitter;
class QUrl;

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

    void saveAllSettings();

    QMenu* dockPanelsMenu() const;

    void restoreAllSettings();
private slots:
    void osgViewerSetup();

    void createShapeFile();

private:
    void closeEvent(QCloseEvent* event) override;
    void handleProjectTransition(const QString& action, const QString& filePath);

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
    ProjectManager* m_projectManager = nullptr;
    ProjectWelcomeScreen* m_ProjectWelcomeScreen = nullptr;
    bool m_allowApplicationClose = false;
    QActionGroup* m_drawingActionGroup = nullptr;
    QList<QAction*> _pluginActions;

//declare private functions here
    void initCore();
    void initSignalsAndSlots();
    void initLogs();
    void initPlugins();
};

#endif // MAINWINDOW_H
