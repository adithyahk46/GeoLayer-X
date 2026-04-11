#include "mainwindow.h"
#include "ui_mainwindow.h"



#include <app/app.h>
#include <app/toolbarmanager.h>
#include <app/StatusBarHandler.h>
#include <app/LayerManagerWidget.h>
#include <app/MapLoadModuleDialog.h>

#include "pluggins/MapLoadModule.h"

using namespace osgEarth;


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setWindowTitle("3D Viewer");

    osgWidget=new osgQOpenGLWidget();
    osgWidget->setMouseTracking(true);
    QObject::connect(osgWidget, &osgQOpenGLWidget::initialized, [this] {
        initOsg();

        App::getInstance()->setMainwindow(this);
        App::getInstance()->setMapNode(mapNode);
        App::getInstance()->setManipulator(manip);
        App::getInstance()->setOsgViewer(viewer);

        ToolBarManager::instance();

        this->statusBar()->setFixedHeight(20);
        StatusBarHandler::getInstance()->setStatusbar(ui->statusbar);
        StatusBarHandler::getInstance()->CreatMapReaders();


        QString raster = "C:/Users/pnmt1054/Adithya_working_directory/Data/raster/43J11.tif";
        MapLoadModule::LoadRasters(mapNode, raster,"kdfkmk");
        MapLoadModule::LoadElevation(mapNode,"C:/Users/pnmt1054/Adithya_working_directory/Data/Elevation/43J11.dt2","elev");


    });
    addDockWidget(Qt::LeftDockWidgetArea,LayerManagerWidget::getInstance());

    _initConnections();

    this->setCentralWidget(osgWidget);

}



MainWindow::~MainWindow()
{
    delete ui;
}

void printOsgAndOsgEarthInfo(osgViewer::Viewer* viewer)
{
    // auto printStr = [](GLenum e, const char* name)
    // {
    //     const GLubyte* s = glGetString(e);
    //     if (s)
    //         qDebug() << name << ": "<< reinterpret_cast<const char*>(s);
    // };

    // printStr(GL_VENDOR,   "Vendor");
    // printStr(GL_RENDERER, "Renderer");
    // printStr(GL_VERSION,  "OpenGL Version");
    // printStr(GL_SHADING_LANGUAGE_VERSION, "GLSL Version");

    // GLint profileMask = 0;
    // glGetIntegerv(GL_CONTEXT_PROFILE_MASK, &profileMask);

    // if (profileMask & GL_CONTEXT_CORE_PROFILE_BIT)
    //     qDebug() << "Profile: Core";
    // else if (profileMask & GL_CONTEXT_COMPATIBILITY_PROFILE_BIT)
    //     qDebug() << "Profile: Compatibility";
    // else
    //     qDebug() << "Profile: Unknown";

    // qDebug() << "OSG Runtime Version: "<< osgGetVersion();
    // qDebug() << "OSG SO Version: "<< osgGetSOVersion();
    // qDebug() << "osgEarth Version (compile-time): " << OSGEARTH_MAJOR_VERSION << "." << OSGEARTH_MINOR_VERSION << "." << OSGEARTH_PATCH_VERSION << "\n";


    // // ---- DisplaySettings ----
    // osg::DisplaySettings* ds = osg::DisplaySettings::instance();

    // qDebug() << "========== DISPLAY SETTINGS ==========";
    // qDebug() << "Shader Hint: " << ds->getShaderHint() ;
    // qDebug() << "MultiSamples: " << ds->getNumMultiSamples();
    // qDebug() << "Stencil Bits: " << ds->getMinimumNumStencilBits();
    // qDebug() << "Depth Buffer Enabled: " << ds->getDepthBuffer();

    // // ---- Camera Info ----
    // if (viewer && viewer->getCamera())
    // {
    //     osg::Camera* cam = viewer->getCamera();

    //     qDebug() << "========== CAMERA ==========";
    //     qDebug() << "Near/Far Ratio: "<< cam->getNearFarRatio();
    //     qDebug() << "Compute Near/Far Mode: "<< cam->getComputeNearFarMode();
    // }
}

void MainWindow::initOsg()
{
    viewer = osgWidget->getOsgViewer();
    viewer->setThreadingModel(osgViewer::Viewer::SingleThreaded);

    osgEarth::Util::LogarithmicDepthBuffer ldb;
    ldb.install(viewer->getCamera());
    viewer->setRealizeOperation(new osgEarth::GL3RealizeOperation());

    viewer->realize();

    osg::ref_ptr<osg::Node> earthNode =
        osgDB::readRefNodeFile("../resources/app/geocentric.earth");

    root = new osg::Group();

    osgUtil::Optimizer optimizer;
        optimizer.optimize(earthNode);
        optimizer.optimize(root);
        optimizer.reset();

    // osg::ref_ptr<osg::Group> mainRoot = new osg::Group;
    // mainRoot->addChild(root);

    root->addChild(earthNode);
    mapNode = osgEarth::MapNode::findMapNode(earthNode);

    manip = new EarthManipulator();
    manip->getSettings()->setThrowingEnabled(true);
    viewer->setCameraManipulator(manip);

    viewer->setSceneData(root);
    viewer->getCamera()->setClearColor(osg::Vec4(1,1,1,0.5));


    // viewer->realize();

    MyMapCallback* mapCallback = new MyMapCallback(manip);
    mapCallback->enableZoomToLayer(true);
    mapNode->getMap()->addMapCallback(mapCallback);


    mouseControlle = MouseEventHandler::Instance(mapNode);
    viewer->addEventHandler(mouseControlle);


    double hours = mapNode->externalConfig().child("sky").value("hours", 5.0);
    osg::ref_ptr<osgEarth::Util::SkyNode> sky = osgEarth::SkyNode::create();
    sky->setDateTime(osgEarth::DateTime(2020, 3, 6, hours));
    // sky->setLighting(true);
    // sky->addChild(mapNode);
    // sky->attach(viewer);
    // sky->getSunLight()->setAmbient(osg::Vec4(0.5f, 0.05f, 0.5f, 1.0f));


    // root->addChild(sky);

}

void MainWindow::_initConnections()
{
    connect(ui->actionLegends, &QAction::triggered, this,[=]{
        LayerManagerWidget::getInstance()->setVisible(!LayerManagerWidget::getInstance()->isVisible());
    });

    connect(ui->actionShow_Full_Screen, &QAction::toggled, this,[this](bool check){
        if(check) this->showFullScreen();
        else this->showMaximized();
    });

    QObject::connect(ui->actionLoad_Layers,&QAction::triggered,this,[=](){
           // LoadLayer("");
           MapLoadModuleDialog* _loadmap = new MapLoadModuleDialog(mapNode, this);
           _loadmap->setAttribute(Qt::WA_DeleteOnClose);
           _loadmap->show();

       });
}

#include <app/ViewshedAnalysisWidget.h>
ViewshedAnalysisWidget* _viewAnalysis = nullptr;
void MainWindow::on_actionViewshed_triggered()
{
    if (!_viewAnalysis)
    {
        _viewAnalysis = new ViewshedAnalysisWidget(this);

        connect(_viewAnalysis, &QObject::destroyed,
                this, [this]()
        {
            _viewAnalysis = nullptr;
        });

    }
    _viewAnalysis->show();

}


#include <app/RadialViewshedWidget.h>
RadialViewshedWidget* viewshedDialog = nullptr;
void MainWindow::on_actionRadar_Platter_triggered()
{
    if (!viewshedDialog)
    {
        viewshedDialog = new RadialViewshedWidget(mapNode,viewer, root,this);

        connect(viewshedDialog, &QObject::destroyed,
                this, [this]()
        {
            viewshedDialog = nullptr;
        });
    }

    viewshedDialog->show();
}

void MainWindow::on_actionDark_Theme_triggered(bool checked)
{
    static bool isDark = true; // Track state
        auto app = qApp;

        if (isDark) {
            // Switch to Light Theme
            app->setPalette(app->style()->standardPalette());
            app->setStyleSheet(""); // Clear custom dark styles if any
            // viewer->getCamera()->setClearColor(osg::Vec4(0.3,0.3,0.3,1.0));

        } else {
            // Switch to Dark Theme
            app->setStyle(QStyleFactory::create("Fusion"));
             // Optional: Set a light palette for the Fusion style
            QPalette lightPalette;
             lightPalette.setColor(QPalette::Window, QColor(240, 240, 240));  // Light grey
             lightPalette.setColor(QPalette::WindowText, Qt::black);
             lightPalette.setColor(QPalette::Base, QColor(245, 245, 245));   // White
             lightPalette.setColor(QPalette::AlternateBase, QColor(240, 240, 240));  // Light grey
             lightPalette.setColor(QPalette::ToolTipBase, Qt::white);
             lightPalette.setColor(QPalette::ToolTipText, Qt::black);
             lightPalette.setColor(QPalette::Text, Qt::black);
             lightPalette.setColor(QPalette::Button, QColor(240, 240, 240));  // Light grey
             lightPalette.setColor(QPalette::ButtonText, Qt::black);
             lightPalette.setColor(QPalette::BrightText, Qt::red);

             lightPalette.setColor(QPalette::Highlight, QColor(76, 163, 224));  // Blue highlight
             lightPalette.setColor(QPalette::HighlightedText, Qt::white);


            app->setPalette(lightPalette);

            // viewer->getCamera()->setClearColor(osg::Vec4(0.9,0.9,0.9,1.0));

        }
        isDark = !isDark;
}

