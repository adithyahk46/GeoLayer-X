#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <iostream>
#include <string>
#include <fstream>
using namespace std;

#include <osg/Notify>
#include <osgDB/FileUtils>
#include <osgEarth/Registry>
#include <osgEarth/Profile>


#include <QActionGroup>
#include <QCloseEvent>
#include <QFileDialog>
#include <QKeyEvent>
#include <QMessageBox>
#include <QSplitter>
#include <QSignalBlocker>
#include <QToolButton>

#include <optional>
#include <utility>

#include "App.h"
#include "plugins/Extrusion3D/Extrusion3D.h"
#include "SettingsManager/SettingsManager.h"
#include "DrawingTool/PolygonDrawTool.h"
#include "PluginsDockWidget.h"
#include "qgis/GLXNewVectorLayerDialog.h"
#include "ProjectManager/ProjectManager.h"
#include "project/ProjectWelcomeScreen.h"



MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    setCorner(Qt::BottomLeftCorner, Qt::LeftDockWidgetArea);
    setCorner(Qt::BottomRightCorner, Qt::RightDockWidgetArea);

}

MainWindow::~MainWindow()
{
    if (_pluginManager)
    {
        for (const PluginManager::PluginInfo& plugin : _pluginManager->plugins())
        {
            if (plugin.loaded)
            {
                QString errorMessage;
                if (!_pluginManager->setEnabled(plugin.id, false, &errorMessage))
                {
                    qWarning() << "Failed to unload plugin" << plugin.id << ":" << errorMessage;
                }
            }
        }
    }

    if (_layerManagerCallback && osgwidget3D && osgwidget3D->getMapNode())
    {
        osgwidget3D->getMapNode()->getMap()->removeMapCallback(_layerManagerCallback);
        _layerManagerCallback = nullptr;
    }

    saveAllSettings();

    SettingsManager::instance()
           .saveMainWindowState(this);

       SettingsManager::instance().sync();
       delete ui;

}

void MainWindow::saveAllSettings()
{
}

void MainWindow::restoreAllSettings()
{
    auto& settings = SettingsManager::instance();
    const bool dark = settings.value("theme", "light").toString() == "dark";
    const QSignalBlocker blocker(ui->actionTheame);
    ui->actionTheame->setChecked(dark);
    setApplicationTheme(dark);

}

void MainWindow::initAll()
{
    // initLogs();

    initCore();

    initSignalsAndSlots();

    initPlugins();

    SettingsManager::instance().restoreMainWindowState(this);

    restoreAllSettings();
}

void MainWindow::showWelcomeScreen()
{
    if (m_ProjectWelcomeScreen)
        m_ProjectWelcomeScreen->showWelcomeScreen();
}

void MainWindow::closeEvent(QCloseEvent* event)
{
    if (m_allowApplicationClose) {
        event->accept();
        QCoreApplication::quit();
    } else {
        event->ignore();
        if (m_projectManager)
            m_projectManager->requestApplicationExit();
    }
}

QMenu *MainWindow::dockPanelsMenu() const { return ui->menuDockPanels;}

void MainWindow::osgViewerSetup()
{
    App::getInstance()->setMainwindow(this);
}

void MainWindow::createShapeFile()
{
    GLXNewVectorLayerDialog dialog(this);
    dialog.exec();
}

void MainWindow::setMapProjection(bool flat)
{
    const auto restoreActionState = [this]() {
        const QSignalBlocker blocker(ui->actionToggleMapProjection);
        ui->actionToggleMapProjection->setChecked(m_flatMapProjection);
    };

    if (flat == m_flatMapProjection)
        return;

    if (!osgwidget3D || !osgwidget3D->getMapNode()
        || !osgwidget3D->getMapNode()->getMap() || !osgwidget3D->getManipulator())
    {
        QMessageBox::warning(this, tr("Map Projection"),
                             tr("The map view is not available."));
        restoreActionState();
        return;
    }

    osg::ref_ptr<osgEarth::MapNode> currentMapNode = osgwidget3D->getMapNode();
    osgEarth::Map* map = currentMapNode->getMap();
    osg::ref_ptr<const osgEarth::Profile> previousProfile = map->getProfile();
    const osgEarth::Profile* targetProfile = osgEarth::Profile::create(
        flat ? osgEarth::Profile::SPHERICAL_MERCATOR
             : osgEarth::Profile::GLOBAL_GEODETIC);
    if (!targetProfile || !targetProfile->isOK())
    {
        QMessageBox::warning(this, tr("Map Projection"),
                             tr("Could not create the requested map projection."));
        restoreActionState();
        return;
    }

    const osgEarth::Viewpoint viewpoint = osgwidget3D->getManipulator()->getViewpoint();
    osg::ref_ptr<osgEarth::MapNode> projectedMapNode = new osgEarth::MapNode(map);
    projectedMapNode->externalConfig() = currentMapNode->externalConfig();

    map->setProfile(targetProfile);
    if (!projectedMapNode->open() || !projectedMapNode->getTerrainEngine())
    {
        map->setProfile(previousProfile.get());
        QMessageBox::warning(this, tr("Map Projection"),
                             tr("Could not initialize the requested map projection."));
        restoreActionState();
        return;
    }

    m_projectManager->setMapNode(projectedMapNode.get());
    osgwidget3D->setMapNode(projectedMapNode.get());
    osgwidget3D->getManipulator()->setViewpoint(viewpoint, 0.5);
    m_flatMapProjection = flat;
}

void MainWindow::initLogs()
{
}

void MainWindow::initCore()
{

    layerManager = LayerManagerWidget::getInstance(this);
    addDockWidget(Qt::RightDockWidgetArea, layerManager);
    //create central layout and central StackedWidget and insert osgwidget3D into it

    osgwidget3D = new OSGViewerWidget(this);
    App::setThreedWindow(osgwidget3D);


    _layerManagerCallback = LayerManagerWidget::createMapCallback(layerManager);
    osgwidget3D->getMapNode()->getMap()->addMapCallback(_layerManagerCallback);

    setCentralWidget(osgwidget3D);

    QObject::connect(osgwidget3D, &OSGViewerWidget::setupCompleted,this, &MainWindow::osgViewerSetup);

    mStatusBarWidget = new StatusbarHandlerWidget();
    statusBar()->addPermanentWidget(mStatusBarWidget);
    statusBar()->setMinimumHeight(20);
    statusBar()->layout()->setContentsMargins(0, 0, 0, 0);
    QObject::connect(osgwidget3D->getManipulator(), &GLXEarthManipulator::mouseMoveEvent, mStatusBarWidget, &StatusbarHandlerWidget::updateGeoCoordinates);

    m_projectManager = new ProjectManager(osgwidget3D->getMapNode(), this);
    m_ProjectWelcomeScreen = new ProjectWelcomeScreen(m_projectManager, this);
    connect(m_projectManager, &ProjectManager::projectOpened, this, [this] {
        setWindowTitle(tr("GeoLayer-X - %1").arg(m_projectManager->currentProjectName()));
    });
    connect(m_projectManager, &ProjectManager::projectClosed, this, [this] {
        setWindowTitle(QStringLiteral("GeoLayer-X"));
    });
    connect(m_projectManager, &ProjectManager::projectTransitionRequested,
            this, &MainWindow::handleProjectTransition);
    connect(m_projectManager, &ProjectManager::saveAsRequested, this, [this] {
        const QString path = QFileDialog::getSaveFileName(
            this, tr("Save GeoLayer-X Project As"),
            m_projectManager->currentProjectPath(),
            tr("osgEarth Project Files (*.earth)"));
        if (!path.isEmpty())
            m_projectManager->saveProjectAs(path);
    });
    connect(m_projectManager, &ProjectManager::openEarthFileRequested, this, [this] {
        const QString path = QFileDialog::getOpenFileName(
            this, tr("Open Earth File"), QString(),
            tr("Earth Files (*.earth);;All files (*)"));
        if (!path.isEmpty())
            m_projectManager->openEarthFile(path);
    });

    if (!viewpointsGui)
    {
        viewpointsGui = new ViewpointsGUI(this);
        addDockWidget(Qt::RightDockWidgetArea, viewpointsGui);
        // viewpointsGui->setFloating(true);
        // viewpointsGui->hide();
    }

    if (!mapReadoutsDock)
    {
        mapReadoutsDock = new MapReadoutsDockWidget(this);
        addDockWidget(Qt::RightDockWidgetArea, mapReadoutsDock);
    }

}

void MainWindow::initSignalsAndSlots()
{
    connect(ui->actionExit, &QAction::triggered, this, &QWidget::close);

    connect(ui->actionToggleMapProjection, &QAction::toggled,
            this, &MainWindow::setMapProjection);

    QObject::connect(ui->actionTheame, &QAction::toggled, this, [=](bool dark){
        setApplicationTheme(dark);
    });

    QObject::connect(ui->actionFullScreen, &QAction::triggered, this, [this](){
        if(isFullScreen()){showMaximized();}
        else{showFullScreen(); }
    });

    QObject::connect(ui->actionLoad_layers, &QAction::triggered, this, [this](){
        //clear the memeory when dialog is closed
        MapLoadModuleDialog* dialog = new MapLoadModuleDialog(this);
        QObject::connect(dialog, &QDialog::finished, dialog, [&dialog](){
            // dialog->deleteLater();
            dialog = nullptr;
        });
        dialog->show();

    });


    /*setup mapInteraction toolbar
     * This setup tells to which manipulator apply the changes, so user can have the control using mapInteraction buttons
     * add manipulatorButton to  ui->toolbarMapIntraction at the first place
     * manipulatorButton text shows which manipulators receive the changes
     */
    {
        const auto applyToCheckedManipulators = [this](auto operation) {
            operation(osgwidget3D->getManipulator());
        };

        QObject::connect(ui->mActionZoomIn, &QAction::triggered, this, [applyToCheckedManipulators]() {
            applyToCheckedManipulators([](GLXEarthManipulator *manipulator) {
                manipulator->zoomIn();
            });
        });

        QObject::connect(ui->mActionZoomOut, &QAction::triggered, this, [applyToCheckedManipulators]() {
            applyToCheckedManipulators([](GLXEarthManipulator *manipulator) {
                manipulator->zoomOut();
            });
        });

        QObject::connect(ui->mActionZoomToFullExtent, &QAction::triggered, this, [applyToCheckedManipulators]() {
            applyToCheckedManipulators([](GLXEarthManipulator *manipulator) {
                manipulator->zoomToFullExtent();
            });
        });

        QObject::connect(ui->mActionZoomToArea, &QAction::triggered, this, [applyToCheckedManipulators]() {
            //allow user to select a rectangle on the map and zoom to that area
            applyToCheckedManipulators([](GLXEarthManipulator *manipulator) {
                // manipulator->zoomToExtent(1.5);
            });
        });
    }

    connect(ui->actionNew_project, &QAction::triggered,
            m_projectManager, &ProjectManager::requestNewProject);
    connect(ui->actionOpen_Project, &QAction::triggered,
            m_projectManager, &ProjectManager::requestOpenProject);
    connect(ui->actionSave_Project, &QAction::triggered,
            m_projectManager, &ProjectManager::saveProject);
    connect(ui->actionSave_as, &QAction::triggered,
            m_projectManager, &ProjectManager::requestSaveAs);
    connect(ui->actionClose_Project, &QAction::triggered,
            m_projectManager, &ProjectManager::requestCloseProject);
    connect(ui->actionOpenEarthFile, &QAction::triggered,
            m_projectManager, &ProjectManager::requestOpenEarthFile);


    //setup Tools menubar actions
    connect(ui->actionSky, &QAction::triggered, this, [this] {
        if(envGui == nullptr)
        {
            envGui = new EnvironmentGUI(this);
                addDockWidget(Qt::RightDockWidgetArea, envGui);
                envGui->setFloating(true);
                envGui->hide();
        }
              
        if (envGui->isVisible()) {
            envGui->hide();
            return;
        }

        envGui->show();
        envGui->raise();
        envGui->activateWindow();
    });

    connect(ui->actionViewpoints, &QAction::triggered, this, [this] {

        if (viewpointsGui->isVisible()) {
            viewpointsGui->hide();
            return;
        }

        viewpointsGui->show();
        viewpointsGui->raise();
        viewpointsGui->activateWindow();
    });

    connect(ui->actionExtrusion3D, &QAction::triggered, this, [this] {
        if (!osgwidget3D || !osgwidget3D->getMapNode())
        {
            QMessageBox::warning(this, tr("3D Extrusion"),
                                 tr("Open a map before using 3D extrusion."));
            return;
        }
        if (!extrusion3D)
            extrusion3D = new Extrusion3D(osgwidget3D->getMapNode(), this);
        extrusion3D->show();
        extrusion3D->raise();
        extrusion3D->activateWindow();
    });

    m_drawingActionGroup = new QActionGroup(this);
    m_drawingActionGroup->setExclusive(true);

    m_drawingActionGroup->addAction(ui->actionDrawCircle);
    m_drawingActionGroup->addAction(ui->actionDrawPolygon);
    m_drawingActionGroup->addAction(ui->actionDrawPolyLine);
    m_drawingActionGroup->addAction(ui->actionDrawRectangle);
    m_drawingActionGroup->addAction(ui->actionAddPoints);


    QObject::connect(ui->actionDrawCircle, &QAction::triggered, this, [this](bool checked) {

    });

    QObject::connect(ui->actionDrawPolygon, &QAction::triggered, this, [this](bool checked) {

    });

    QObject::connect(ui->actionDrawPolyLine, &QAction::triggered, this, [this](bool checked) {

    });

    QObject::connect(ui->actionDrawRectangle, &QAction::triggered, this, [this](bool checked) {

    });

    QObject::connect(ui->actionAddPoints, &QAction::triggered, this, [this](bool checked) {

    });

    QObject::connect(m_drawingActionGroup, &QActionGroup::triggered, this, [this](QAction*) {

    });

    QObject::connect(ui->actionNew_Shapefile_Layer, &QAction::triggered, this, &MainWindow::createShapeFile);


}

void MainWindow::handleProjectTransition(const QString& action,
                                         const QString& filePath)
{
    QString projectFilePath = filePath;
    if (action == QStringLiteral("openFile")) {
        projectFilePath = QFileDialog::getOpenFileName(
            this, tr("Open GeoLayer-X Project"), QString(),
            tr("osgEarth Project Files (*.earth);;All files (*)"));
        if (projectFilePath.isEmpty())
            return;
    }

    if (m_projectManager->hasOpenProject()) {
        const QMessageBox::StandardButton choice = QMessageBox::question(
            this, tr("Current project"),
            tr("Save changes to %1 before continuing?")
                .arg(m_projectManager->currentProjectName()),
            QMessageBox::Save | QMessageBox::Discard | QMessageBox::Cancel,
            QMessageBox::Save);
        if (choice != QMessageBox::Save && choice != QMessageBox::Discard)
            return;
        if (choice == QMessageBox::Save && !m_projectManager->saveProject())
            return;
        m_projectManager->closeProject();
    }

    if (action == QStringLiteral("openFile")
        || action == QStringLiteral("openRecent")) {
        if (m_projectManager->openProject(projectFilePath))
            m_ProjectWelcomeScreen->completeProjectOpen();
        else
            showWelcomeScreen();
    } else if (action == QStringLiteral("exit")) {
        m_allowApplicationClose = true;
        close();
    } else if (action == QStringLiteral("new")
               || action == QStringLiteral("close")) {
        showWelcomeScreen();
    }
}


void MainWindow::initPlugins()
{
    pluginsDockWidget = new PluginsDockWidget(this);
    addDockWidget(Qt::LeftDockWidgetArea, pluginsDockWidget);

    _pluginManager = PluginManager::instance();
    pluginsDockWidget->setUpPluginManager(_pluginManager);

    connect(_pluginManager, &PluginManager::pluginsChanged, pluginsDockWidget, &PluginsDockWidget::refreshPluginsList);
    connect(_pluginManager, &PluginManager::pluginError, this, [](const QString& pluginId, const QString& message) {
        qWarning() << "Plugin" << pluginId << "failed:" << message;
    });


    _pluginManager->scanPlugins();

    connect(ui->actionManage_Plugins, &QAction::triggered, this, [this] {
        _pluginManager->showManagerDialog(this);
    });
}

void MainWindow::setApplicationTheme(bool dark)
{
    QPalette palette;

    if(dark){
        palette.setColor(QPalette::Window,          QColor("#2e2e34"));
       palette.setColor(QPalette::WindowText,      QColor("#ffffff"));

       palette.setColor(QPalette::Base,            QColor("#2d2d2d"));
       palette.setColor(QPalette::AlternateBase,   QColor("#001a68"));

       palette.setColor(QPalette::Text,            QColor("#ffffff"));
       palette.setColor(QPalette::PlaceholderText, QColor("#80ffffff"));

       palette.setColor(QPalette::Button,          QColor("#3c3c3c"));
       palette.setColor(QPalette::ButtonText,      QColor("#ffffff"));

       palette.setColor(QPalette::Highlight,       QColor("#0078d4"));
       palette.setColor(QPalette::HighlightedText, QColor("#ffffff"));

       palette.setColor(QPalette::Link,            QColor("#99ebff"));
       palette.setColor(QPalette::LinkVisited,     QColor("#4cc2ff"));

       palette.setColor(QPalette::ToolTipBase,     QColor("#3c3c3c"));
       palette.setColor(QPalette::ToolTipText,     QColor("#d4d4d4"));
    }
    else {
        palette.setColor(QPalette::Window,          QColor("#f5f5f5"));
        palette.setColor(QPalette::WindowText,      QColor("#202020"));

        palette.setColor(QPalette::Base,            QColor("#ffffff"));
        palette.setColor(QPalette::AlternateBase,   QColor("#f0f0f0"));

        palette.setColor(QPalette::Text,            QColor("#202020"));
        palette.setColor(QPalette::PlaceholderText, QColor("#80000000"));

        palette.setColor(QPalette::Button,          QColor("#e5e5e5"));
        palette.setColor(QPalette::ButtonText,      QColor("#202020"));

        palette.setColor(QPalette::Highlight,       QColor("#0078d4"));
        palette.setColor(QPalette::HighlightedText, QColor("#ffffff"));

        palette.setColor(QPalette::Link,            QColor("#0067c5"));
        palette.setColor(QPalette::LinkVisited,     QColor("#5c2d91"));

        palette.setColor(QPalette::ToolTipBase,     QColor("#ffffff"));
        palette.setColor(QPalette::ToolTipText,     QColor("#202020"));
    }

    QApplication::setPalette(palette);
    if(dark){
        SettingsManager::instance().setValue("theme","dark");
        SettingsManager::instance().sync();
        osgwidget3D->setViewerColor(osg::Vec4(0.2f, 0.2f, 0.2f, 1.0f));
    }
    else{
        SettingsManager::instance().setValue("theme","light");
        SettingsManager::instance().sync();
        osgwidget3D->setViewerColor(osg::Vec4(0.9f, 0.9f, 0.9f, 1.0f));
    }

}
