#include "mainwindow.h"
#include "ui_mainwindow.h"


#include <QFileDialog>
#include <QMessageBox>
#include <QStatusBar>
#include <osgEarth/GeoData>
#include <osgEarth/FeatureModelLayer>
#include <osgEarth/FeatureSource>
#include <osgEarth/OGRFeatureSource>
#include <osgEarth/TiledFeatureModelLayer>

#include <osgEarth/Controls>
#include <osgEarth/LabelNode>

#include <osgEarth/ImageLayer>
#include <osgEarth/FileUtils>

#include <legends.h>
#include <osgEarth/GDAL>
#include <osgEarth/GLUtils>
#include <osgEarth/AttributesFilter>  // or the actual path where AttributesFilter is declared

#include <gdal.h>

#include <gdal_priv.h>
#include <gdal_alg.h>
#include <ogrsf_frmts.h>

#include <gdal_proxy.h>

#include <osgEarth/Style>
#include <osgEarth/FeatureModelLayer>


#include <osgEarth/Map>
#include <osgEarth/MapNode>
#include <osgEarth/OgrUtils>
#include <osgEarth/FeatureModelLayer>
#include <osgEarth/FeatureSource>
#include <osgEarth/Style>
#include <osgEarth/GeoMath>

#include <QLabel>

#include "PickEventHandler.h"

#include <osg/Notify>
#include <osg/DisplaySettings>
#include <osgEarth/ExampleResources>
#include <osgEarth/EarthManipulator>
#include <osgEarth/Controls>
// #include <osgEarth/ViewerWidget>
#include <osgEarth/Registry>
#include <osgEarth/Capabilities>
#include <QApplication>
#include <QMainWindow>
#include <QStatusBar>
#include <QVBoxLayout>
#include <osgQOpenGL/osgQOpenGLWidget>
// #include <osgWidget/VBox>

#include "MyMapCallback.h"


#include <QSlider>
#include <QPushButton>
#include <QtWidgets/QVBoxLayout>
#include <QTabWidget>

#ifdef Q_WS_X11
#include <X11/Xlib.h>
#endif


using namespace osgEarth;

#include <osgEarth/Map>
#include <iostream>


MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setWindowTitle("GeoLayer-X");

    osg::DisplaySettings::instance()->setNumOfHttpDatabaseThreadsHint(8);
    osg::DisplaySettings::instance()->setNumOfDatabaseThreadsHint(2);

    osgWidget=new osgQOpenGLWidget(this);
    osgWidget->resized();
    osgWidget->setMouseTracking(true);
    QObject::connect(osgWidget, &osgQOpenGLWidget::initialized, [this] { initOsg(); });
    // QObject::connect(osgWidget, &osgQOpenGLWidget::)
    setCentralWidget(osgWidget);


    _initConnections();

    ui->dockWidget->resize(450,ui->dockWidget->height());



}



MainWindow::~MainWindow()
{
    delete ui;
}



void MainWindow::initOsg()
{

    viewer = osgWidget->getOsgViewer();

    // load the data
    osg::ref_ptr<osg::Node> earthNode = osgDB::readRefNodeFile("D:/OSG1/osgEarth/tests/osm.earth");

    manip = new EarthManipulator();
    manip->getSettings()->setThrowingEnabled(true);
    viewer->setCameraManipulator(manip);

    osg::Group* root = new osg::Group();

    osgUtil::Optimizer optimizer;
    optimizer.optimize(earthNode);
    optimizer.optimize(root);
    optimizer.reset();

    root->addChild(earthNode);

    mapNode = MapNode::findMapNode(earthNode);
    annoGroup = new osg::Group();
    mapNode->addChild(annoGroup);

    MyMapCallback* mapCallback = new MyMapCallback();

        // Connect the signal to MainWindow slot
        connect(mapCallback, &MyMapCallback::onMapLoaded,
                this, &MainWindow::OnMapLoaded);

    mapNode->getMap()->addMapCallback(mapCallback);


    viewer->setSceneData(root);




    viewer->addEventHandler(new PickEventHandler(ui->statusbar, mapNode));


    // EarthManipulator::Settings* settings=manip->getSettings();
    // if(settings){
    //     settings->getBreakTetherActions().push_back(EarthManipulator::ACTION_GOTO);

    // }
    // //tether to the node and refresh viewpoint automatically
    // manip->getSettings()->setTetherMode(EarthManipulator::TetherMode::TETHER_CENTER_AND_HEADING);
    // // Set the minimum distance to something larger than the default
    // manip->getSettings()->setMinMaxDistance(10.0, manip->getSettings()->getMaxDistance());

    // // Sets the maximum focal point offsets (usually for tethering)
    // manip->getSettings()->setMaxOffset(5000.0, 5000.0);
    // manip->getSettings()->setArcViewpointTransitions(true);
    // viewer->getCamera()->setSmallFeatureCullingPixelSize(-1.0f);
    // LogarithmicDepthBuffer buf;
    // buf.install(viewer->getCamera());

    // 星空
        /*skyNode = osgEarth::Util::SkyNode::create();//提供天空、灯光和其他环境效果的类的接口
        //osg::ref_ptr<osgEarth::Util::Ephemeris> ephemeris = new osgEarth::Util::Ephemeris;//星历表给出了自然发生的天体的位置；在这种情况下，太阳和月亮；还包括一些相关的实用功能
        //skyNode->setEphemeris(ephemeris);//确定太阳和月亮的位置
        skyNode->setName("skyNode");
        //skyNode->setDateTime(osgEarth::DateTime());//现在
        skyNode->setDateTime(osgEarth::DateTime(2021, 4, 21, 22));
        skyNode->attach(ui.openGLWidget->getViewer(), 0);
        skyNode->setLighting(false);
        skyNode->addChild(m_mapNode);
        m_world->addChild(skyNode)*/

    double hours = mapNode->externalConfig().child("sky").value("hours", 5.0);
    osg::ref_ptr<osgEarth::Util::SkyNode> sky = osgEarth::SkyNode::create();
    sky->setDateTime(osgEarth::DateTime(2020, 3, 6, hours));
    sky->setLighting(false);
    sky->addChild(mapNode);
    sky->attach(viewer);
    sky->getSunLight()->setAmbient(osg::Vec4(5.0f, 0.04f, 0.5f, 1.0f));

    root->addChild(sky);

    viewer->realize();



    // manip->setViewpoint(osgEarth::Util::Viewpoint("", 75.43926116579506, 25.959264570876737, 500, 0.0f, -90.0f, 12000000), 0.1f);
   // manip->setViewpoint(osgEarth::Util::Viewpoint("",-84.8383, 42.5964, 0.0, 0.0f, -90.0f, 319303), 1.5f);


}

void MainWindow::_initConnections()
{
    /*
     * Controls Gui
     */
    {
        QVBoxLayout* buttonLayout = new QVBoxLayout(osgWidget);
        buttonLayout->addStretch();
        buttonLayout->setAlignment(Qt::AlignRight);

        QPushButton* zoomIn = new QPushButton("+", osgWidget);
        zoomIn->setFixedSize(30, 30);
        QPushButton* zoomOut = new QPushButton("-", osgWidget);
        zoomOut->setFixedSize(30, 30);

        QSlider* pitchSlider = new QSlider(Qt::Vertical);
        QSlider* headingSlider = new QSlider(Qt::Vertical);
        QSlider* rotationSlider = new QSlider(Qt::Vertical);

        // Optional: set ranges and initial values
        pitchSlider->setRange(-90, 90);
        headingSlider->setRange(-90, 90);
        rotationSlider->setRange(-180, 180);
        pitchSlider->setMaximumHeight(150);
        headingSlider->setMaximumHeight(150);
        rotationSlider->setMaximumHeight(150);

        pitchSlider->setValue(0);
        headingSlider->setValue(0);
        rotationSlider->setValue(0);

        // Optional: give tooltips or labels
        pitchSlider->setToolTip("Pitch");
        headingSlider->setToolTip("Heading");
        rotationSlider->setToolTip("Rotation");

        QHBoxLayout* sliderGroup = new QHBoxLayout();
        sliderGroup->addWidget(pitchSlider);
        sliderGroup->addWidget(headingSlider);
        sliderGroup->addWidget(rotationSlider);

        buttonLayout->addWidget(zoomIn);
        buttonLayout->addWidget(zoomOut);
        buttonLayout->addLayout(sliderGroup);

        connect(pitchSlider, &QSlider::sliderMoved, this, [=](int value) mutable {

            double delta = (value * 0.02) *  (M_PI / 180.0); // Convert degrees to radians
            this->manip->rotate(0, delta); // Only pitch (dy)
        });
        connect(pitchSlider, &QSlider::sliderReleased, this, [=]() mutable {
            pitchSlider->setValue(0);
        });

        connect(headingSlider, &QSlider::sliderMoved, this, [=](int value) mutable {

            double delta = (value * 0.02) *  (M_PI / 180.0); // Convert degrees to radians
            this->manip->rotate(delta,0 ); // Only pitch (dy)
        });
        connect(headingSlider, &QSlider::sliderReleased, this, [=]() mutable {
            headingSlider->setValue(0);
        });

        connect(rotationSlider, &QSlider::sliderMoved, this, [=](int value) mutable {

            double delta = (value * 0.02) *  (M_PI / 180.0); // Convert degrees to radians
            this->manip->pan(delta,0 ); // Only pitch (dy)
        });
        connect(rotationSlider, &QSlider::sliderReleased, this, [=]() mutable {
            rotationSlider->setValue(0);
        });


        QObject::connect(zoomIn, &QPushButton::clicked, this, [=]() {
            osgEarth::Viewpoint currentVP = this->manip->getViewpoint();
            if (currentVP.range().isSet()) {
                osgEarth::Distance currentDistance = currentVP.range().value();
                double currentRangeInMeters = currentDistance.as(osgEarth::Units::METERS);
                double targetRangeValue = currentRangeInMeters * 0.7;
                if (targetRangeValue < 10.0) {
                    targetRangeValue = 10.0;
                }
                osgEarth::Distance newDistance(targetRangeValue, osgEarth::Units::METERS);
                currentVP.setRange(newDistance);
                this->manip->setViewpoint(currentVP, 0.8);
            }
        });

        QObject::connect(zoomOut, &QPushButton::clicked, this, [=]() {
            osgEarth::Viewpoint currentVP = this->manip->getViewpoint();
            if (currentVP.range().isSet()) {
                osgEarth::Distance currentDistance = currentVP.range().value();
                double currentRangeInMeters = currentDistance.as(osgEarth::Units::METERS);
                double targetRangeValue = currentRangeInMeters * 1.3;
                if (targetRangeValue > 50000000.0) { // Example: Maximum range of 50,000,000 meter
                    targetRangeValue = 50000000.0;
                }
                osgEarth::Distance newDistance(targetRangeValue, osgEarth::Units::METERS);
                currentVP.setRange(newDistance);
                this->manip->setViewpoint(currentVP, 0.8);
            }
        });
    }

    {
    // ui->verticalLayout_legends->addStretch();
    QObject::connect(ui->actionLoad_Layers,&QAction::triggered,this,[=](){
        // LoadLayer("");
        MapLoadModule* _loadmap = new MapLoadModule(mapNode, manip);
        _loadmap->setAttribute(Qt::WA_DeleteOnClose);
        _loadmap->show();

    });

    QObject::connect(ui->actionLeft_Panel,&QAction::triggered,this,[=](){
        ui->dockWidget->setVisible(!ui->dockWidget->isVisible());
    });
    QObject::connect(ui->actionElevation_Layer, &QAction::triggered, this,[=](){

    });

    QObject::connect(ui->actionLegends, &QAction::triggered,this,[=](){
            ui->dockWidget->setWindowTitle("LEGENDS");
            ui->stackedWidget->setCurrentWidget(ui->legengs_page);
    });

    }
}

template class Legends<osgEarth::FeatureModelLayer>;
template class Legends<osgEarth::GDALElevationLayer>;
template class Legends<osgEarth::GDALImageLayer>;

template <typename T>

void MainWindow::CreateLegend(T* mapLayer)
{
    T* Layer = mapLayer;
    Legends<T>* legend = new Legends<T>(mapLayer,mapNode,manip);
    // legend->setSizeHint(legend->sizeHint());
    ui->verticalLayout_legends->addWidget(legend);
   // ui->verticalLayout_legends->addStretch();
}


void MainWindow::OnMapLoaded(osgEarth::Layer* layer){
    std::cout<< "Signal received Layer added: " << layer->getName() ;

    // Example for checking and casting to ImageLayer
        if (auto imageLayer = dynamic_cast<osgEarth::GDALImageLayer*>(layer)) {
            CreateLegend(imageLayer); // calls CreateLegend<osgEarth::ImageLayer>
        }
        // Example for checking and casting to ElevationLayer
        else if (auto elevationLayer = dynamic_cast<osgEarth::GDALElevationLayer*>(layer)) {
            CreateLegend(elevationLayer); // calls CreateLegend<osgEarth::ElevationLayer>
        }
        // Example for checking and casting to ModelLayer
        else if (auto modelLayer = dynamic_cast<osgEarth::FeatureModelLayer*>(layer)) {
            CreateLegend(modelLayer); // calls CreateLegend<osgEarth::ModelLayer>
        }
        else {
            std::cout << "Unknown layer type: " << layer->getName() << std::endl;
        }
    zoomToLayer(layer);
}



void MainWindow::zoomToLayer(Layer* layer)
{
    const osgEarth::SpatialReference* wgs84 = osgEarth::SpatialReference::get("wgs84");

    osgEarth::GeoExtent layerExtent = layer->getExtent().transform(wgs84);
    if (!layerExtent.isValid())
    {
        qDebug() << "Error: Border layer extent is invalid.";
        return;
    }


    GeoPoint p1(layerExtent.getSRS(), layerExtent.xMin(), layerExtent.yMin(),0.0);
    GeoPoint p2(layerExtent.getSRS(), layerExtent.xMax(), layerExtent.yMax(), 0.0);

    GeoPoint center(layerExtent.getSRS(),layerExtent.getCentroid().x(),layerExtent.getCentroid().y(),0.0);

    qDebug()<<layerExtent.getCentroid().x()<<" " <<layerExtent.getCentroid().y() <<center.x()<<" " <<center.y() << p1.distanceTo(p2) ;

    // Optional: set viewpoint with calculated range
    Viewpoint vp("",center.x(),center.y(),0.0f,0.0f,-90.0f, p1.distanceTo(p2)<1000?1000:p1.distanceTo(p2) );
    manip->setViewpoint(vp, 2.0); // with 2-second transition
    qDebug()<<"The Range Covered by the layer = "<< p1.distanceTo(p2);
}


void MainWindow::on_pushButton_clicked()
{

    // Inside the viewer loop (e.g., while (!viewer.done()))
    double currentRange = manip->getDistance();
    qDebug()<< "Current view range (eye to focal point distance): " << currentRange ;


    osg::Vec3d eye;
    osg::Vec3d center;
    osg::Vec3d up;

    viewer->getCamera()->getViewMatrixAsLookAt(eye, center, up);
    double computedRange = (eye - center).length();
    // qDebug() << "Computed eye-center distance: " << computedRange;

}

