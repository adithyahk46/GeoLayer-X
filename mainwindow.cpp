#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <osgEarth/GLUtils>

#include <app/app.h>
#include <app/StatusBarHandler.h>
#include <app/LayerManagerWidget.h>
#include <app/MapLoadModule.h>

using namespace osgEarth;


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setWindowTitle("3D Viewer");

    // // Dock 1
    // QDockWidget *dock1 = new QDockWidget("Layers", this);
    // dock1->setWidget(new QListWidget());
    // addDockWidget(Qt::LeftDockWidgetArea, dock1);

    // // Dock 2
    // QDockWidget *dock2 = new QDockWidget("Browser", this);
    // dock2->setWidget(new QTextEdit());
    // addDockWidget(Qt::LeftDockWidgetArea, dock2);

    // // Dock 3
    // QDockWidget *dock3 = new QDockWidget("Processing", this);
    // dock3->setWidget(new QLabel("Processing Tools"));
    // addDockWidget(Qt::LeftDockWidgetArea, dock3);

    osgWidget=new osgQOpenGLWidget();
    osgWidget->setMouseTracking(true);
    QObject::connect(osgWidget, &osgQOpenGLWidget::initialized, [this] {
        initOsg();

        App::getInstance()->setMapNode(mapNode);
        App::getInstance()->setManipulator(manip);
        App::getInstance()->setOsgViewer(viewer);

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

    QString figPath = QCoreApplication::applicationDirPath()
                      + "/Data/earthFiles/simple.earth";

    osg::ref_ptr<osg::Node> earthNode =
        osgDB::readRefNodeFile(figPath.toStdString());

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
    viewer->getCamera()->setClearColor(osg::Vec4(0.3,0.3,0.3,1.0));


    // viewer->realize();

    MyMapCallback* mapCallback = new MyMapCallback(manip);
    mapCallback->enableZoomToLayer(true);
    mapNode->getMap()->addMapCallback(mapCallback);


    mouseControlle = MouseEventHandler::Instance(mapNode);
    viewer->addEventHandler(mouseControlle);

    ui->dockWidget->show();

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
           MapLoadModule* _loadmap = new MapLoadModule(mapNode, this);
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


#include <app/VisibilityTestAreaWidget.h>
VisibilityTestAreaWidget* viewshedDialog = nullptr;
void MainWindow::on_actionRadar_Platter_triggered()
{
    if (!viewshedDialog)
    {
        viewshedDialog = new VisibilityTestAreaWidget(mapNode,viewer, root,this);

        connect(viewshedDialog, &QObject::destroyed,
                this, [this]()
        {
            viewshedDialog = nullptr;
        });
    }

    viewshedDialog->show();
}


