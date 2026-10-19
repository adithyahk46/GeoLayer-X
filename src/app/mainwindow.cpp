#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <iostream>
#include <string>
#include <fstream>
using namespace std;

#include <osg/Notify>
#include <osgDB/FileUtils>
#include <osgEarth/Registry>


#include <QActionGroup>
#include <QFileInfo>
#include <QFileDialog>
#include <QKeyEvent>
#include <QMessageBox>
#include <QSplitter>
#include <QToolButton>

#include <optional>
#include <utility>

#include "App.h"
#include "SettingsManager/SettingsManager.h"
#include "DataManager/MapLoadModule.h"
#include "DrawingTool/PolygonDrawTool.h"
#include "PluginsDockWidget.h"



void qtLogToFile(QtMsgType type, const QMessageLogContext &context, const QString &msg)
{
    const std::string logTypes[] = { "Debug", "Warning", "Critical", "Fatal", "Info" };

    std::cout << "[Qt]    [" << logTypes[type] << "]    "
              << msg.toLocal8Bit().constData();

#ifndef NDEBUG
    std::cout << "    (" << context.file << ": " << context.line
              << ", " << context.function << ")";
#endif

    std::cout << std::endl;
}

class LogFileHandler : public osg::NotifyHandler
{
public:
    LogFileHandler(std::ofstream* log) : _log(log) {}

    void notify(osg::NotifySeverity severity, const char* msg) override
    {
        const char* tags[] = {
            "ALWAYS", "FATAL", "WARN", "NOTICE",
            "INFO", "DEBUG_INFO", "DEBUG_FP"
        };

        std::cout << "[OSG]   [" << tags[severity] << "]    " << msg;

        if (_log)
        {
            (*_log) << "[OSG]   [" << tags[severity] << "]    " << msg;
            _log->flush();
        }
    }

private:
    std::ofstream* _log = nullptr;
};


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

   const QString theme = settings.value("theme", "light").toString();
   // ui->actionTheme->setChecked(theme == "dark");
   setApplicationTheme(theme == "dark");

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
    GLXProjectsDialog dialog(this);

    dialog.exec();
}


QMenu *MainWindow::dockPanelsMenu() const { return ui->menuDockPanels;}

void MainWindow::osgViewerSetup()
{
    App::getInstance()->setMainwindow(this);
}

void MainWindow::initLogs()
{
    std::string glx_prefix_path =
        std::getenv("GLX_PREFIX_PATH") ?
        std::getenv("GLX_PREFIX_PATH") :
        QDir(QCoreApplication::applicationDirPath()).absoluteFilePath("..").toStdString();

    std::string glx_log_path =
        std::getenv("GLX_LOG_PATH") ?
        std::getenv("GLX_LOG_PATH") :
        glx_prefix_path + "/logs";

    if (!osgDB::fileExists(glx_log_path))
        osgDB::makeDirectory(glx_log_path);

    std::string logPath = glx_log_path + "/GLXLogs.txt";

    _log = new std::ofstream(logPath, std::ios::out | std::ios::app);

    if (!_log || !_log->is_open())
        return;

    // IMPORTANT: Don't redirect cout/cerr.
    // They remain connected to the console.

    qInstallMessageHandler(qtLogToFile);

    osgEarth::setNotifyLevel(osg::INFO);
    osgEarth::setNotifyHandler(new LogFileHandler(_log));

    osg::setNotifyLevel(osg::NOTICE);
    osg::setNotifyHandler(new LogFileHandler(_log));

    std::cout << "Program started: "
              << QDateTime::currentDateTime()
                     .toString("yyyy-MM-dd hh:mm:ss")
                     .toStdString()
              << std::endl;
}

void MainWindow::initCore()
{

    layerManager = LayerManagerWidget::getInstance(this);
    addDockWidget(Qt::RightDockWidgetArea, layerManager);
    //create central layout and central StackedWidget and insert osgwidget3D into it

    osgwidget3D = new OSGViewerWidget(this);
    App::setThreedWindow(osgwidget3D);

#if GLX_BUILD_2D_MAP
    //2D widget setUp
    osgwidget2D = new OSGViewerWidget(this);
    App::set2DWindow(osgwidget2D);
    osgwidget2D->getMapNode()->getMap()->setProfile(osgEarth::Profile::create(osgEarth::Profile::PLATE_CARREE));
    osgwidget2D->getManipulator()->getSettings()->bindMouse(
    osgEarth::Util::EarthManipulator::ACTION_NULL,osgGA::GUIEventAdapter::MIDDLE_MOUSE_BUTTON,0);
#endif

#if GLX_BUILD_2D_MAP
    mapViewSplitter = new QSplitter(Qt::Horizontal, this);
    mapViewSplitter->addWidget(osgwidget2D);
    mapViewSplitter->addWidget(osgwidget3D);
    mapViewSplitter->setStretchFactor(0, 1);
    mapViewSplitter->setStretchFactor(1, 1);
#endif

    _layerManagerCallback = LayerManagerWidget::createMapCallback(layerManager);
    osgwidget3D->getMapNode()->getMap()->addMapCallback(_layerManagerCallback);
#if GLX_BUILD_2D_MAP
    setCentralWidget(mapViewSplitter);
#else
    setCentralWidget(osgwidget3D);
#endif

    QObject::connect(osgwidget3D, &OSGViewerWidget::setupCompleted,this, &MainWindow::osgViewerSetup);

    MapLoadModule::LoadTMSLayer(osgwidget3D->getMapNode(), "https://readymap.org/readymap/tiles/1.0.0/7/");
#if GLX_BUILD_2D_MAP
    MapLoadModule::LoadTMSLayer(osgwidget2D->getMapNode(), "https://readymap.org/readymap/tiles/1.0.0/7/");
#endif


    mStatusBarWidget = new StatusbarHandlerWidget();
    statusBar()->addPermanentWidget(mStatusBarWidget);
    statusBar()->setMinimumHeight(20);
    statusBar()->layout()->setContentsMargins(0, 0, 0, 0);
    QObject::connect(osgwidget3D->getManipulator(), &GLXEarthManipulator::mouseMoveEvent, mStatusBarWidget, &StatusbarHandlerWidget::updateGeoCoordinates);
#if GLX_BUILD_2D_MAP
    QObject::connect(osgwidget2D->getManipulator(), &GLXEarthManipulator::mouseMoveEvent, mStatusBarWidget, &StatusbarHandlerWidget::updateGeoCoordinates);
#endif




}

void MainWindow::initSignalsAndSlots()
{
    //menubar "VIEW"
    {
#if GLX_BUILD_2D_MAP
      QObject::connect(ui->actionShow_2D, &QAction::triggered, this, [this](bool checked){
         osgwidget2D->setVisible(!osgwidget2D->isVisible());
         ui->actionShow_2D->setChecked(osgwidget2D->isVisible());
      });
#else
      ui->actionShow_2D->setVisible(false);
#endif
      QObject::connect(ui->actionShow_3D, &QAction::triggered, this, [this](bool checked){
            osgwidget3D->setVisible(!osgwidget3D->isVisible());
            ui->actionShow_3D->setChecked(osgwidget3D->isVisible());
      });
    }
    //connect signals and slots here
    QObject::connect(ui->actionOpenEarthFile, &QAction::triggered, this, [this]() {
        QString filePath = QFileDialog::getOpenFileName(this, "Open Earth File", "", "Earth Files (*.earth)");
        if (!filePath.isEmpty()) {
            MapLoadModule::LoadEarthFile(filePath);
        }
    });

    QObject::connect(ui->actionTheame, &QAction::toggled, this, [=](bool dark){
        setApplicationTheme(dark);
    });

    QObject::connect(ui->actionFullScreen, &QAction::triggered, this, [this](){
        if(isFullScreen()){showNormal();}
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
#if GLX_BUILD_2D_MAP
        QMenu *manipulatorMenu = new QMenu("", this);

        QAction *manip2d = manipulatorMenu->addAction("2D");
        QAction *manip3d = manipulatorMenu->addAction("3D");

        manip2d->setCheckable(true);
        manip3d->setCheckable(true);
        manip3d->setChecked(true);

        QToolButton *manipulatorButton = new QToolButton(this);
        const auto updateManipulatorButtonText = [manipulatorButton, manip2d, manip3d]() {
            if (manip2d->isChecked() && manip3d->isChecked())
              //  :/images/themes/default/sync_views.svg
                manipulatorButton->setIcon(QIcon(":/images/themes/default/sync_views.svg"));
            else if (manip2d->isChecked())
                manipulatorButton->setText("2D");
            else if (manip3d->isChecked())
                manipulatorButton->setText("3D");
            else
                manipulatorButton->setText("None");
        };
        updateManipulatorButtonText();
        manipulatorButton->setMenu(manipulatorMenu);
        manipulatorButton->setPopupMode(QToolButton::InstantPopup);

        ui->toolbarMapIntraction->addWidget(manipulatorButton);
#endif

#if GLX_BUILD_2D_MAP
        QObject::connect(manip2d, &QAction::toggled, this, [updateManipulatorButtonText](bool) {
            updateManipulatorButtonText();
        });
        QObject::connect(manip3d, &QAction::toggled, this, [updateManipulatorButtonText](bool) {
            updateManipulatorButtonText();
        });

        const auto applyToCheckedManipulators = [this, manip2d, manip3d](auto operation) {
            if (manip2d->isChecked())
                operation(osgwidget2D->getManipulator());
            if (manip3d->isChecked())
                operation(osgwidget3D->getManipulator());
        };
#else
        const auto applyToCheckedManipulators = [this](auto operation) {
            operation(osgwidget3D->getManipulator());
        };
#endif

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

    QObject::connect(ui->actionNew_Shapefile_Layer, &QAction::triggered, this, [this](){
        PolygonDrawTool* polygonDrawTool = new PolygonDrawTool(osgwidget3D->getMapNode(), osgwidget3D->getManipulator(), this);
    });


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

void MainWindow::keyPressEvent(QKeyEvent* event)
{

}

void MainWindow::setApplicationTheme(bool dark)
{
    QPalette palette;

    if(dark){
        palette.setColor(QPalette::Window,          QColor("#1e1e1e"));
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
#if GLX_BUILD_2D_MAP
        osgwidget2D->setViewerColor(osg::Vec4(0.2f, 0.2f, 0.2f, 1.0f));
#endif
        osgwidget3D->setViewerColor(osg::Vec4(0.2f, 0.2f, 0.2f, 1.0f));
    }
    else{
        SettingsManager::instance().setValue("theme","light");
#if GLX_BUILD_2D_MAP
        osgwidget2D->setViewerColor(osg::Vec4(0.9f, 0.9f, 0.9f, 1.0f));
#endif
        osgwidget3D->setViewerColor(osg::Vec4(0.9f, 0.9f, 0.9f, 1.0f));
    }


}
