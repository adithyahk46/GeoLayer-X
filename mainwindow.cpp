#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <osgEarth/GLUtils>



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
    });

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

    //     viewer->setRealizeOperation(new osgEarth::GL3RealizeOperation());

    // osgEarth::Util::LogarithmicDepthBuffer ldb;
    // ldb.install(viewer->getCamera());

    QString figPath = QCoreApplication::applicationDirPath()
                      + "/Data/earthFiles/simple.earth";

    osg::ref_ptr<osg::Node> earthNode =
        osgDB::readRefNodeFile(figPath.toStdString());

    // osg::ref_ptr<osg::Group> mainRoot = new osg::Group;
    root = new osg::Group();
    // mainRoot->addChild(root);
    root->addChild(earthNode);
    mapNode = osgEarth::MapNode::findMapNode(earthNode);

    manip = new EarthManipulator();
    manip->getSettings()->setThrowingEnabled(true);
    viewer->setCameraManipulator(manip);

    viewer->setSceneData(root);
    viewer->getCamera()->setClearColor(osg::Vec4(1.0,1.0,1.0,1.0));

    // viewer->realize();

    MyMapCallback* mapCallback = new MyMapCallback(manip);
    mapCallback->enableZoomToLayer(true);
    connect(mapCallback, &MyMapCallback::onMapLoaded,
            this, &MainWindow::OnMapLoaded);
    mapNode->getMap()->addMapCallback(mapCallback);


    mouseControlle = MouseEventHandler::Instance(mapNode);
    viewer->addEventHandler(mouseControlle);


    ui->dockWidget->show();

}

void MainWindow::_initConnections()
{
}


void MainWindow::OnMapLoaded(osgEarth::Layer* layer)
{
    //legend view setup
    {
    if (!layer) return;

    // Create the legend for this layer
    Legends* legend = new Legends(layer, mapNode, manip);

    // Determine layer type and add to corresponding category in QTreeWidget
    QString category;

    if (dynamic_cast<osgEarth::ImageLayer*>(layer))
    {
        category = "Raster Layers";
    }
    else if (dynamic_cast<osgEarth::ElevationLayer*>(layer))
    {
        category = "Elevation Layers";
    }
    else if (dynamic_cast<osgEarth::FeatureModelLayer*>(layer))
    {
        category = "Vector Layers";
    }
    else if (dynamic_cast<osgEarth::XYZImageLayer*>(layer))
    {
        category = "Tile Layers";
    }
    else if (dynamic_cast<osgEarth::AnnotationLayer*>(layer))
    {
        category = "Annotation Layers";
    }
    else
    {
        category = "Other Layers";
    }

    // Find or create category item in QTreeWidget
    QTreeWidgetItem* categoryItem = nullptr;
    QList<QTreeWidgetItem*> foundItems = ui->treeWidget->findItems(category, Qt::MatchExactly);
    if (!foundItems.isEmpty())
    {
        categoryItem = foundItems.first();
    }
    else
    {
        categoryItem = new QTreeWidgetItem(ui->treeWidget);
        categoryItem->setText(0, category);
        categoryItem->setFlags(Qt::ItemIsEnabled);
    }

    // Add new layer as a child item under the category
    QTreeWidgetItem* layerItem = new QTreeWidgetItem(categoryItem);
    //layerItem->setText(0, QString::fromStdString(layer->getName()));

    // Store pointer to legend widget for later retrieval (optional)
    ui->treeWidget->setItemWidget(layerItem, 0, legend);

    QObject::connect(legend, &QObject::destroyed, this, [=]() {
        QTreeWidgetItem* parent = layerItem->parent();
        delete layerItem;
        if (parent && parent->childCount() == 0) {
            delete parent; // remove empty category
        }
    });

    // Expand category for visibility
    ui->treeWidget->expandItem(categoryItem);

    }

}


#include <ViewshedAnalysis/ViewshedAreaAnalysisWidget.h>
ViewshedAreaAnalysisWidget* viewshedDialog = nullptr;
void MainWindow::on_actionViewshed_triggered()
{
    if (!viewshedDialog)
    {
        viewshedDialog = new ViewshedAreaAnalysisWidget(mapNode,viewer, root,this);

        connect(viewshedDialog, &QObject::destroyed,
                this, [this]()
        {
            viewshedDialog = nullptr;
        });
    }

    viewshedDialog->show();
    // viewshedDialog->raise();
    // viewshedDialog->activateWindow();

    LoadLayersOnMap load;
    QString raster = "C:/Users/pnmt1054/Adithya_working_directory/Data/raster/43J11.tif";
    load.LoadRasters(mapNode, raster,"kdfkmk");
    load.LoadElevation(mapNode,"C:/Users/pnmt1054/Adithya_working_directory/Data/Elevation/43J11.dt2","elev");

}


