#include "mainwindow.h"
#include "ui_mainwindow.h"


#include <QFileDialog>
#include <QMessageBox>
#include <QStatusBar>
#include <QLabel>
#include <QStatusBar>
#include <QVBoxLayout>
#include <QSlider>
#include <QPushButton>
#include <QtWidgets/QVBoxLayout>
#include <QTabWidget>
#include <osgEarth/AnnotationNode>

#include<QCoreApplication>

#include <osgEarth/ModelSymbol>
#include <osgEarth/ModelNode>
#include <osgEarth/GLUtils>



#ifdef Q_WS_X11
#include <X11/Xlib.h>
#endif

using namespace osgEarth;

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setWindowTitle("3D Viewer");
ui->dockWidget->hide();
    // QObject::connect(this,&MainWindow::)
    osgWidget=new osgQOpenGLWidget();
    osgWidget->setMouseTracking(true);
    QObject::connect(osgWidget, &osgQOpenGLWidget::initialized, [this] { initOsg();
    });

    // osgWidget->show();
    _initConnections();

    this->setCentralWidget(osgWidget);


    // ui->horizontalLayout_3->addWidget(osgWidget);

    // ui->dockWidget->resize(450,ui->dockWidget->height());
}



MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::initOsg()
{
    viewer = osgWidget->getOsgViewer();

    // This is optional, but it will mitigate near-plane clipping in a whole earth scene:
        osgEarth::Util::LogarithmicDepthBuffer ldb;
        ldb.install(viewer->getCamera());
        viewer->setRealizeOperation(new osgEarth::GL3RealizeOperation());

    viewer->realize();

    QString figPath = QCoreApplication::applicationDirPath() + "/Data/earthFiles/simple.earth";
    qDebug()<<figPath;
    osg::ref_ptr<osg::Node> earthNode = osgDB::readRefNodeFile(figPath.toStdString());

    manip = new EarthManipulator();
    manip->getSettings()->setThrowingEnabled(true);
    viewer->setCameraManipulator(manip);

    // manip->setValue();
    // qDebug()<<"get VALue" << manip->getValue();

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

    // viewer->addEventHandler(new PickEventHandler(ui->statusbar, mapNode));
    mouseControlle = MouseEventHandler::Instance(mapNode);

    viewer->addEventHandler(mouseControlle);

    double hours = mapNode->externalConfig().child("sky").value("hours", 5.0);
    osg::ref_ptr<osgEarth::Util::SkyNode> sky = osgEarth::SkyNode::create();
    sky->setDateTime(osgEarth::DateTime(2020, 3, 6, hours));
    sky->setLighting(false);
    sky->addChild(mapNode);
    sky->attach(viewer);
    sky->getSunLight()->setAmbient(osg::Vec4(5.0f, 0.04f, 0.5f, 1.0f));

    root->addChild(sky);


    //status bar initialization/ setup
    {
        QWidget* container = new QWidget;
        QHBoxLayout* layout = new QHBoxLayout(container);
        layout->setContentsMargins(8, 0, 8, 0); // Remove margins for status bar fit
        layout->addStretch();

        // _mapNode->get

        // Create and add latitude input
        layout->addWidget(new QLabel("Lat:"));
        _lat = new QLineEdit;
        _lat->setReadOnly(true);
        //_lat->setFixedWidth(100);
        layout->addWidget(_lat);

        // Create and add longitude input
        layout->addWidget(new QLabel("Lon:"));
        _long = new QLineEdit;
        _long->setReadOnly(true);
       // _long->setFixedWidth(100);
        layout->addWidget(_long);

        // Create and add height input
        layout->addWidget(new QLabel("alt:"));
        _height = new QLineEdit;
        _height->setReadOnly(true);
        //_height->setFixedWidth(100);
        layout->addWidget(_height);

        // Create and add CRS combo box
        layout->addWidget(new QLabel("CRS:"));
        _CRS = new QComboBox;
        _CRS->addItem("WGS84(EPSG:4326)");
        _CRS->addItem("Web Mercator(EPSG:3857");
        layout->addWidget(_CRS);

        // Add the container to the status bar
        ui->statusbar->addPermanentWidget(container);

        QObject::connect(mouseControlle,&MouseEventHandler::mouseMoveEvent, this ,[=](const double lon, const double lat,const double alt,const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa){
            osgUtil::LineSegmentIntersector::Intersections intersections;
            if (viewer->computeIntersections(ea.getX(), ea.getY(), intersections))
            {
                auto hit = *intersections.begin();
                osg::Vec3d world = hit.getWorldIntersectPoint();

                osgEarth::GeoPoint mapPoint;
                mapPoint.fromWorld(mapNode->getMapSRS(), world);

                const osgEarth::SpatialReference* srs4326 = osgEarth::SpatialReference::get("epsg:4326");
                const osgEarth::SpatialReference* srs3857 = osgEarth::SpatialReference::get("epsg:3857");

                osgEarth::GeoPoint transformed;
                if ( _CRS->currentIndex() == 0) {
                    mapPoint.transform(srs4326, transformed);
                } else {
                    mapPoint.transform(srs3857, transformed);
                }

                QString latUnit = (transformed.getSRS()->isGeographic()) ? "°" : "m";
                QString lonUnit = (transformed.getSRS()->isGeographic()) ? "°" : "m";

                QString latText = QString("%1 %2").arg(transformed.y(), 0, 'f', 4).arg(latUnit);
                QString lonText = QString("%1 %2").arg(transformed.x(), 0, 'f', 4).arg(lonUnit);
                QString heightText = QString("%1 m").arg(transformed.z(), 0, 'f', 2);

                // Update the UI fields
                _lat->setText(latText);
                _long->setText(lonText);
                _height->setText(heightText);
            }
        } );
    }

    ui->dockWidget->show();

    LoadLayersOnMap load;
    QString raster = "C:/Users/pnmt1054/Downloads/DGRE/Eelvation_Data/MOPT_DEM_colored7.tif";
    load.LoadRasters(mapNode, raster,"kdfkmk");
    load.LoadElevation(mapNode,"C:/Users/pnmt1054/Downloads/DGRE/Eelvation_Data/MOPT_DEM_30m.dt2","elev");

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
        LoadLayersOnMap* _loadmap = new LoadLayersOnMap(this, mapNode, manip);
        _loadmap->setAttribute(Qt::WA_DeleteOnClose);
        _loadmap->show();

    });

    QObject::connect(ui->actionLeft_Panel,&QAction::triggered,this,[=](){
        ui->dockWidget->setVisible(!ui->dockWidget->isVisible());
    });

    QObject::connect(ui->actionLegends, &QAction::triggered,this,[=](){
            ui->dockWidget->setWindowTitle("LEGENDS");
            ui->stackedWidget->setCurrentWidget(ui->legengs_page);
    });

    QObject::connect(ui->actionSave_File, &QAction::triggered,this,[=](){


        createTempShapeFile( m_currentShapefileInfo.outputPath,
                                             m_currentShapefileInfo.encoding,
                                             m_currentShapefileInfo.geometryType,
                                             m_currentShapefileInfo.srs,
                                             annotationLayer) ;

        ui->actionCreate_Shape_File->setEnabled(false);
        ui->actionSave_File->setEnabled(true);
        ui->actionClose_File->setEnabled(true);
    });

    QObject::connect(ui->actionClose_File, &QAction::triggered,this,[=](){

        toggleCreateShapeFile = false;
        listFeatureNode.clear();
        mapNode->getMap()->removeLayer(annotationLayer);

        ui->actionCreate_Shape_File->setEnabled(true);
        ui->actionSave_File->setEnabled(false);
        ui->actionClose_File->setEnabled(false);

    });

    }

   }

void MainWindow::on_actionBaseMaps_triggered()
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

    // Finally, zoom to the layer
    zoomToLayer(layer);
}


void MainWindow::zoomToLayer(Layer* layer)
{

    const osgEarth::SpatialReference* wgs84 = osgEarth::SpatialReference::get("wgs84");

    osgEarth::GeoExtent layerExtent = layer->getExtent().transform(wgs84);
    if (!layerExtent.isValid())
    {
        qDebug() << "Error:layer extent is invalid.";
        return;
    }

    GeoPoint p1(layerExtent.getSRS(), layerExtent.xMin(), layerExtent.yMin(),0.0);
    GeoPoint p2(layerExtent.getSRS(), layerExtent.xMax(), layerExtent.yMax(), 0.0);

    GeoPoint center(layerExtent.getSRS(),layerExtent.getCentroid().x(),layerExtent.getCentroid().y(),0.0);

    // Optional: set viewpoint with calculated range
    Viewpoint vp("",center.x(),center.y(),0.0f,0.0f,-90.0f, p1.distanceTo(p2)<1000?1000:p1.distanceTo(p2) );
    manip->setViewpoint(vp, 2.0); // with 2-second transition
    qDebug()<<"The Range Covered by the layer = "<<p1.distanceTo(p2);

}

void MainWindow::on_actionMissileTracker_triggered()
{
    //     if(clientDialog == NULL  )
    //     {
    //         clientDialog = new UdpClient(viewer, osgWidget, manip, mapNode, annoGroup, this);
    //         clientDialog->setAttribute(Qt::WA_DeleteOnClose);

    // std::cout << "Size of Flight Simulator befor Memory optimization: " << sizeof(*clientDialog) << " bytes" << std::endl;


    //         connect(clientDialog, &QDialog::finished, this, [this]()  {
    //             // ui->actionStartClient->setChecked(false);
    //             delete clientDialog;
    //             clientDialog = NULL;
    //         });
    //     }

    //     if(!clientDialog->isVisible()){
    //         clientDialog->show();
    //     }

    if(udpClientDialog == NULL  )
    {
        udpClientDialog = new FlightSimulator(viewer,osgWidget, manip,annoGroup,mapNode, this);
        udpClientDialog->setWindowTitle("Flight Simulator");
        udpClientDialog->setAttribute(Qt::WA_DeleteOnClose);

        std::cout << "Size of Flight Simulator module: " << sizeof(*udpClientDialog) << " bytes" << std::endl;


        connect(udpClientDialog, &QDialog::finished, this, [this]()  {
            // ui->actionStartClient->setChecked(false);
            delete udpClientDialog;
            udpClientDialog = NULL;
        });
    }

    if(!udpClientDialog->isVisible()){
        udpClientDialog->show();
    }
}

void MainWindow::on_actionFind_Path_triggered()
{
    if(!findpath){
        findpath  = new RoutingWidget(mapNode,mouseControlle);
        ui->stackedWidget->addWidget(findpath);
    }
    ui->stackedWidget->setWindowTitle("Find Path");
    ui->stackedWidget->setCurrentWidget(findpath);
}


void MainWindow::on_actionGoTo_triggered()
{
    if(!goToWidgwt){

    goToWidgwt = new QDockWidget("GoTo", osgWidget);
    goToWidgwt->setFeatures(QDockWidget::NoDockWidgetFeatures);
    goToWidgwt->setStyleSheet("background-color: white;"); // affects dock frame & title bar

    // Main widget for dock
    QWidget* dockContent = new QWidget(goToWidgwt);
    QVBoxLayout* mainLayout = new QVBoxLayout(dockContent);

    // === From Section ===
    QHBoxLayout* fromLayout = new QHBoxLayout(dockContent);
    QLabel* fromLabel = new QLabel("Enter a Location:");
    QLineEdit* fromLatEdit = new QLineEdit(dockContent);
    fromLatEdit->setPlaceholderText("Lat");
    QLineEdit* fromLonEdit = new QLineEdit(dockContent);
    fromLonEdit->setPlaceholderText("Lon");

    // fromLayout->addWidget(fromLabel);
    fromLayout->addWidget(fromLonEdit);
    fromLayout->addWidget(fromLatEdit);


    // === Buttons Section ===
    QPushButton* findPathButton = new QPushButton("goto",dockContent);
    // Add layouts to main layout
    mainLayout->addWidget(fromLabel);
    mainLayout->addLayout(fromLayout);
    mainLayout->addWidget(findPathButton);

    dockContent->setLayout(mainLayout);
    goToWidgwt->setWidget(dockContent);
   // controlPanel->setAllowedAreas(Qt::


    QObject::connect(findPathButton, &QPushButton::clicked, this, [=]() mutable {

        double lat = fromLatEdit->text().toDouble();
        double lon = fromLonEdit->text().toDouble();

        QString figPath = QCoreApplication::applicationDirPath() + "/Data/placemark32.png";
        Style pm;
        pm.getOrCreate<IconSymbol>()->url().mutable_value().setLiteral(figPath.toStdString().c_str());
        pm.getOrCreate<IconSymbol>()->declutter() = true;
        pm.getOrCreate<TextSymbol>()->halo() = Color("#5f5f5f");

        // Remove old marker if it exists
        if (locationMarker) {
            annoGroup->removeChild(locationMarker);
            locationMarker = nullptr;
        }

        // Create new marker
        locationMarker = new osgEarth::PlaceNode(
            osgEarth::GeoPoint(mapNode->getMapSRS(), lon, lat),
            "New York",
            pm
        );
        // annoGroup->addChild(locationMarker);

        // Set viewpoint
        osgEarth::Util::Viewpoint vp("", lon, lat, 500, 0.0f, -90.0f, 1000);
        manip->setViewpoint(vp, 0.1f);
    });

    goToWidgwt->show();

    }
    else {
        // Remove old marker if it exists
        if (locationMarker) {
            annoGroup->removeChild(locationMarker);
            locationMarker = nullptr;
        }
        goToWidgwt->deleteLater();
        goToWidgwt = nullptr;
    }

}

osgEarth::ModelNode* modelNode=nullptr;
osg::ref_ptr<osgEarth::GeoTransform> geoModel ;
void MainWindow::on_pushButton_clicked()
{

    osg::Matrix scaleMat = osg::Matrix::scale(2, 2, 2);
       geoModel->setMatrix(scaleMat * geoModel->getMatrix());

       return;

    QString filePath = QFileDialog::getOpenFileName(this,"Open Files","","*.obj");
    if (filePath.isEmpty())
        return;

    ////////////////////////////////////to add Flight Object///////////////////////////////

    QString flightModel = QCoreApplication::applicationDirPath() + "/Data/simulator/missile_2obj.obj";
    // a model node with auto scaling.
        {

            // style.getOrCreate<ModelSymbol>()->url()->setLiteral("C:/Users/pnmt1054/Adithya-working-directory/Data/3D-models/akm__free_lowpoly/scene.gltf");
            // osg::ref_ptr<osgEarth::ModelNode> modelNode = new osgEarth::ModelNode(_mapNode, style);
            // modelNode->setName("threed model");
            // //modelNode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
            // //mViewer->getCamera()->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

            // modelNode->setPosition(osgEarth::GeoPoint(_mapNode->getMapSRS(), 0, 0));
            // _annoGroup->addChild(modelNode);
            // osg::Vec3d Scale = modelNode->getScale();
            // Scale *= 2;
            // modelNode->setScale(Scale);
            // osg::Quat quata = modelNode->getLocalRotation();
            // // quata.z() = sin(z);
            // // quata.w() = cos(z);
            // // modelNode->setLocalRotation(quata);

        // osgEarth::Style style;
        // style.getOrCreate<osgEarth::ModelSymbol>()->autoScale() = true;
        // style.getOrCreate<osgEarth::ModelSymbol>()->url().mutable_value().setLiteral("C:/Users/pnmt1054/Adithya-working-directory/Data/3D-models/akm__free_lowpoly/scene.gltf");
        // style.getOrCreate<osgEarth::ModelSymbol>()->scale() = 2000.f;
        // style.getOrCreate<osgEarth::ModelSymbol>()->minAutoScale() = 2000.0f;
        // style.getOrCreate<osgEarth::ModelSymbol>()->maxAutoScale() = 2000.0f;
        // modelNode = new osgEarth::ModelNode(_mapNode, style);
        // // modelNode->setPosition(GeoPoint(geoSRS, -100, 52));
        // modelNode->setPosition(osgEarth::GeoPoint(_mapNode->getMapSRS(), 0, 0,0));
        // _annoGroup->addChild(modelNode);


        //////////////////////////////////////////////////////////////////////////////////////////////////
            // osgEarth::Style style;
            // style.getOrCreate<osgEarth::ModelSymbol>()->autoScale() = true;
            // style.getOrCreate<osgEarth::ModelSymbol>()->url().mutable_value().setLiteral("C:/Users/pnmt1054/Adithya-working-directory/QT_PROJECTS/GeoLayer-X/Data/simulator/missile_2obj.obj");
            // style.getOrCreate<osgEarth::ModelSymbol>()->scale() = 2000.f;
            // style.getOrCreate<osgEarth::ModelSymbol>()->minAutoScale() = 2000.0f;
            // style.getOrCreate<osgEarth::ModelSymbol>()->maxAutoScale() = 2000.0f;
            // modelNode = new osgEarth::ModelNode(_mapNode, style);
            // // modelNode->setPosition(GeoPoint(geoSRS, -100, 52));
            // modelNode->setPosition(osgEarth::GeoPoint(_mapNode->getMapSRS(), 0, 0,0));
            // _annoGroup->addChild(modelNode);
        //////////////////////////////////////////////////////////////////////////////////////////////

        osgEarth::ModelLayer::Options modelOpts;
        modelOpts.url() = osgEarth::URI(filePath.toStdString().c_str());   // path to your OBJ
        // modelOpts.set()
        modelOpts.location() =  osgEarth::GeoPoint(
                    mapNode->getMapSRS(),
                    77.59,    // Longitude
                    12.97,    // Latitude
                    0.0,      // Elevation in meters
                    osgEarth::ALTMODE_RELATIVE);

        // Create ModelLayer
        osg::ref_ptr<osgEarth::ModelLayer> modelLayer =
            new osgEarth::ModelLayer( modelOpts);

        // Add it to the map
        mapNode->getMap()->addLayer(modelLayer.get());


        // OPTIONAL: Move the camera to focus on this model
        osgEarth::Viewpoint vp("Model View", 77.59, 12.97, 20.0, 0.0, -90.0, 1000.0);
        manip->setViewpoint(vp);

        return;


        if(modelNode){
            annoGroup->removeChild(modelNode);
        }
        // Create and configure the style
        osgEarth::Style style;
        auto* modelSymbol = style.getOrCreate<osgEarth::ModelSymbol>();

        // Configure the model properties
        modelSymbol->autoScale() = false;  // Disable auto-scaling for better control
        modelSymbol->url().mutable_value().setLiteral(filePath.toStdString().c_str());
        modelSymbol->scale() = 10.0f;  // Adjust if too small or too big

        // Create the ModelNode with this style
        modelNode = new osgEarth::ModelNode(mapNode, style);
        osgEarth::ModelLayer* layer = new osgEarth::ModelLayer();


        // Set the model position (example: Bangalore)
        modelNode->setPosition(osgEarth::GeoPoint(
            mapNode->getMapSRS(),
            77.59,    // Longitude
            12.97,    // Latitude
            0.0,      // Elevation in meters
            osgEarth::ALTMODE_RELATIVE));  // Place relative to terrain

        // Disable face culling so reversed faces are visible
        // modelNode->getOrCreateStateSet()->setMode(GL_CULL_FACE, osg::StateAttribute::OFF);

        // Add the model to your annotation group
        annoGroup->addChild(modelNode);

        // OPTIONAL: Move the camera to focus on this model
        osgEarth::Viewpoint vp1("Model View", 77.59, 12.97, 20.0, 0.0, -90.0, 1000.0);
        manip->setViewpoint(vp1);

        }
}

void MainWindow::createTempShapeFile(const QString &outputPath,
                                     const QString &encoding,
                                     const QString &geometryType,
                                     const QString &srs,
                                     const osgEarth::AnnotationLayer* Layer) // ✅ MODIFIED
{
    QString shapefilePath = outputPath;

    // ✅ Check Layer validity
    if (!Layer || listFeatureNode.size() == 0)  // ✅ MODIFIED
    {
        qDebug() << "No annotations in the layer to export!";
        QMessageBox::warning(this, "Layer Warning", "No annotations in the layer to export");
        return;
    }

    // ✅ Delete existing shapefile files (.shp, .shx, .dbf, etc.)
    QStringList extensions = {".shp", ".shx", ".dbf", ".prj", ".cpg"};
    for (const QString &ext : extensions) {
        QFile::remove(shapefilePath.left(shapefilePath.length() - 4) + ext);
    }

    // ✅ Register GDAL drivers
    GDALAllRegister();

    GDALDriver *driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
    if (!driver) {
        QMessageBox::critical(this, "GDAL Error", "Could not get ESRI Shapefile driver.");
        return;
    }

    GDALDataset *dataset = driver->Create(shapefilePath.toUtf8().constData(), 0, 0, 0, GDT_Unknown, nullptr);
    if (!dataset) {
        QMessageBox::critical(this, "GDAL Error", "Failed to create shapefile.");
        return;
    }

    // ✅ Setup Spatial Reference
    OGRSpatialReference oSRS;
    if (srs.contains("4326")) {
        oSRS.SetWellKnownGeogCS("WGS84");
    } else if (srs.contains("3857")) {
        oSRS.importFromEPSG(3857);
    } else {
        oSRS.importFromEPSG(4326); // default
    }

    // ✅ Determine Geometry Type
    OGRwkbGeometryType ogrGeomType = wkbUnknown;
    if (geometryType == "Point") ogrGeomType = wkbPoint;
    else if (geometryType == "LineString") ogrGeomType = wkbLineString;
    else if (geometryType == "Polygon") ogrGeomType = wkbPolygon;
    else if (geometryType == "MultiPoint") ogrGeomType = wkbMultiPoint;
    else if (geometryType == "MultiPolygon") ogrGeomType = wkbMultiPolygon;

    // ✅ Create Layer
    OGRLayer *layer = dataset->CreateLayer("layer", &oSRS, ogrGeomType, nullptr);
    if (!layer) {
        QMessageBox::critical(this, "GDAL Error", "Failed to create layer in shapefile.");
        GDALClose(dataset);
        return;
    }

    // ✅ MODIFIED: No manual fields creation from QVariantMap.
    // Instead, we will create fields dynamically from the first feature's attributes.

    bool fieldsCreated = false; // ✅ MODIFIED

    // ✅ Iterate all children in AnnotationLayer
    for (unsigned int i = 0; i < listFeatureNode.size(); ++i)  // ✅ MODIFIED
    {
        osgEarth::FeatureNode* featureNode = listFeatureNode[i];  // ✅ Correct way
        if (!featureNode) continue;

        osgEarth::Feature* feature = featureNode->getFeature();
        if (!feature) continue;

        // ✅ Create fields dynamically from first feature's attributes
        if (!fieldsCreated)  // ✅ MODIFIED
        {
            const osgEarth::AttributeTable& attrs = feature->getAttrs();
            for (const auto& attrPair : attrs)
            {
                const std::string& attrName = attrPair.first;
                const osgEarth::AttributeValue& attrValue = attrPair.second;

                OGRFieldDefn oField(attrName.c_str(), OFTString);

#ifdef _MSC_VER
                if (attrValue.type == osgEarth::ATTRTYPE_INT)
                    oField.SetType(OFTInteger);
                else if (attrValue.type == osgEarth::ATTRTYPE_DOUBLE)
                    oField.SetType(OFTReal);
                else if (attrValue.type == osgEarth::ATTRTYPE_BOOL)
                    oField.SetType(OFTInteger);
                else
                    oField.SetType(OFTString);
#elif defined(__MINGW32__) || defined(__MINGW64__)

/*                if (attrValue.getType == osgEarth::ATTRTYPE_INT)
                    oField.SetType(OFTInteger);
                else if (attrValue.getType == osgEarth::ATTRTYPE_DOUBLE)
                    oField.SetType(OFTReal);
                else if (attrValue.getType == osgEarth::ATTRTYPE_BOOL)
                    oField.SetType(OFTInteger);
                else
                    oField.SetType(OFTString)*/;
#endif

                oField.SetWidth(32);
                layer->CreateField(&oField);
            }
            fieldsCreated = true;
        }

        // ✅ Convert osgEarth Geometry to WKT
        std::string wkt;
        osgEarth::Geometry* geom = feature->getGeometry();
        if (geom)
        {
            std::ostringstream wktStream;
            if (geom->isPolygon())
            {
                wktStream << "POLYGON((";
                for (unsigned j = 0; j < geom->size(); ++j)
                {
                    const osg::Vec3d& v = (*geom)[j];
                    wktStream << v.x() << " " << v.y();
                    if (j < geom->size() - 1) wktStream << ",";
                }
                const osg::Vec3d& first = (*geom)[0];
                wktStream << "," << first.x() << " " << first.y();
                wktStream << "))";
            }
            wkt = wktStream.str();
        }

        // ✅ Create new OGR Feature
        OGRFeature* poFeature = OGRFeature::CreateFeature(layer->GetLayerDefn());

        // ✅ Copy attributes from osgEarth Feature
        const osgEarth::AttributeTable& attrs = feature->getAttrs();
        for (const auto& attrPair : attrs)
        {
            const std::string& attrName = attrPair.first;
            const osgEarth::AttributeValue& attrValue = attrPair.second;

            // if (attrValue.type == osgEarth::ATTRTYPE_STRING)
            //     poFeature->SetField(attrName.c_str(), attrValue.getString().c_str());
            // else if (attrValue.type == osgEarth::ATTRTYPE_INT)
            //     poFeature->SetField(attrName.c_str(), static_cast<int>(attrValue.getInt()));
            // else if (attrValue.type == osgEarth::ATTRTYPE_DOUBLE)
            //     poFeature->SetField(attrName.c_str(), attrValue.getDouble());
            // else
            //     poFeature->SetField(attrName.c_str(), attrValue.getString().c_str());
        }

        // ✅ Set Geometry
        OGRGeometry* poGeometry = nullptr;
        if (OGRGeometryFactory::createFromWkt(wkt.c_str(), nullptr, &poGeometry) == OGRERR_NONE)
        {
            poFeature->SetGeometry(poGeometry);
        }

        // ✅ Add feature to layer
        if (layer->CreateFeature(poFeature) != OGRERR_NONE)
        {
            qDebug() << "Failed to create feature in shapefile.";
        }

        OGRFeature::DestroyFeature(poFeature);
        if (poGeometry) OGRGeometryFactory::destroyGeometry(poGeometry);
    }

    GDALClose(dataset);

    QMessageBox::information(this, "Success", "Shapefile created at:\n" + shapefilePath);
}

void MainWindow::on_actionCreate_Shape_File_triggered()
{

    toggleCreateShapeFile = true;


    CreateShapeFileDialog *dia = new CreateShapeFileDialog(this);
        dia->setAttribute(Qt::WA_DeleteOnClose);
        dia->show();

        connect(dia, &CreateShapeFileDialog::createShapeFile,
                this, [this](const QString &outputPath,
                             const QString &encoding,
                             const QString &geometryType,
                             const QString &srs,
                             const QList<QVariantMap> &fields)
        {
            // 1. Create AnnotationLayer
            annotationLayer = new osgEarth::AnnotationLayer();

            // 2. Extract file name (without extension) from outputPath and set as layer name
            QFileInfo fileInfo(outputPath);
            QString layerName = fileInfo.baseName(); // gets filename without extension
            annotationLayer->setName(layerName.toStdString());

            // 3. Add the layer to the map
            if (mapNode && mapNode->getMap())
            {
                mapNode->getMap()->addLayer(annotationLayer);
            }
            else
            {
                qWarning() << "MapNode or Map is not available!";
                return;
            }

            // 4. Save received info into m_currentShapefileInfo
            m_currentShapefileInfo.outputPath = outputPath;
            m_currentShapefileInfo.encoding = encoding;
            m_currentShapefileInfo.geometryType = geometryType;
            m_currentShapefileInfo.srs = srs;
            m_currentShapefileInfo.fields = fields;

            ui->actionCreate_Shape_File->setEnabled(false);
            ui->actionSave_File->setEnabled(true);
            ui->actionClose_File->setEnabled(true);

        });

}
#include <osgEarth/FeatureSource>

#include <QDialog>
#include <QTreeWidget>
#include <QVBoxLayout>
#include <QPushButton>
#include <osg/Node>
#include <osg/Group>
#include <vector>

void MainWindow::collectNodes(osg::Group* group, std::vector<osg::Node*>& nodes)
{
    if (!group) return;

    for (unsigned int i = 0; i < group->getNumChildren(); ++i)
    {
        osg::Node* child = group->getChild(i);
        nodes.push_back(child);

        osg::Group* subgroup = child->asGroup();
        if (subgroup)
        {
            collectNodes(subgroup, nodes); // recursive for sub-children
        }
    }
}


void MainWindow::on_actionClear_Annotations_triggered()
{

    // Collect nodes
    std::vector<osg::Node*> nodes;
    collectNodes(annoGroup, nodes);

    // Create a dialog
    QDialog* dialog = new QDialog(this);
    dialog->setWindowTitle("Annotations List");
    dialog->resize(400, 300);

    QVBoxLayout* layout = new QVBoxLayout(dialog);

    // Create tree widget
    QTreeWidget* tree = new QTreeWidget(dialog);
    tree->setHeaderLabels({"Node Name", "Type"});
    layout->addWidget(tree);

    // Populate tree
    for (auto* node : nodes)
    {
        QTreeWidgetItem* item = new QTreeWidgetItem(tree);
        item->setText(0, QString::fromStdString(node->getName()));
        item->setText(1, node->className()); // OSG provides class name
    }

    // Add a Clear button
    QPushButton* clearBtn = new QPushButton("Clear All", dialog);
    layout->addWidget(clearBtn);

    connect(clearBtn, &QPushButton::clicked, this, [this, dialog]() {
        if (annoGroup)
        {
            annoGroup->removeChildren(0, annoGroup->getNumChildren());
        }
        dialog->accept();
    });

    dialog->exec();

}

void MainWindow::on_actionDrawCircle_triggered()
{
    ui->actionDrawCircle->setEnabled(false);

    if(!PolygonGroup){
        PolygonGroup = new osg::Group();
        annoGroup->addChild(PolygonGroup);
    }

    if(toggleCreateShapeFile){

        DrawCircle* drawCircle = new DrawCircle(mapNode, mouseControlle);

        connect(drawCircle, &DrawCircle::DrawCircleFinished, this , [=, &drawCircle](osgEarth::FeatureNode* featureNode){

            QObject::disconnect(drawCircle, &DrawCircle::DrawCircleFinished, this ,nullptr);


            listFeatureNode.append(featureNode);
            annotationLayer->addChild(featureNode);
            annotationLayer->dirty();

            AttributeDialog dialog(featureNode,m_currentShapefileInfo.fields, this);
            dialog.exec();

            ui->actionDrawCircle->setEnabled(true);

        });
    }
    else{
        DrawCircle* drawCircle = new DrawCircle(mapNode, mouseControlle);

        connect(drawCircle, &DrawCircle::DrawCircleFinished, this , [=, &drawCircle](osgEarth::FeatureNode* featureNode){

            QObject::disconnect(drawCircle, &DrawCircle::DrawCircleFinished, this ,nullptr);

            PolygonGroup->addChild(featureNode);

            ui->actionDrawCircle->setEnabled(true);

        });
    }

}

void MainWindow::on_actionDrawPolygon_triggered(bool checked)
{

        if(!PolygonGroup){
            PolygonGroup = new osg::Group();
            annoGroup->addChild(PolygonGroup);
        }

        if(toggleCreateShapeFile){

            DrawPolygonTool* drawPolygonevent = new DrawPolygonTool(mapNode, mouseControlle);

            ui->actionDrawPolygon->setEnabled(false);

            QObject::connect(drawPolygonevent, &DrawPolygonTool::DrawPolygonFinished, this, [=](osgEarth::FeatureNode* featureNode){

                QObject::disconnect(drawPolygonevent, &DrawPolygonTool::DrawPolygonFinished, this,nullptr);

                listFeatureNode.append(featureNode);
                annotationLayer->addChild(featureNode);
                annotationLayer->dirty();

                AttributeDialog dialog(featureNode,m_currentShapefileInfo.fields, this);
                dialog.exec();

                ui->actionDrawPolygon->setEnabled(true);

            });

        }
        else{

            DrawPolygonTool* drawPolygonevent = new DrawPolygonTool(mapNode, mouseControlle);

            ui->actionDrawPolygon->setEnabled(false);

            QObject::connect(drawPolygonevent, &DrawPolygonTool::DrawPolygonFinished, this, [=](osgEarth::FeatureNode* featureNode){
                PolygonGroup->addChild(featureNode);

                QObject::disconnect(drawPolygonevent, &DrawPolygonTool::DrawPolygonFinished, this,nullptr);

                ui->actionDrawPolygon->setEnabled(true);


            });
        }
}

#include <osgEarth/LabelNode>
#include <osgEarth/AnnotationUtils>
#include <osgEarth/TextSymbol>
#include <osgEarth/Style>

// Assuming you already have _mapNode (osgEarth::MapNode*) and a group for annotations
void MainWindow::on_actionAddLabel_triggered()
{
    ui->actionAddLabel->setEnabled(false);

     static LabelingDialog* dialog =  new LabelingDialog(mapNode,mouseControlle,this);
    dialog->setAttribute(Qt::WA_DeleteOnClose);
    dialog->show();

    connect(dialog, &LabelingDialog::LabelCreated, this ,[=](LabelNode* _measureLabel){
        if(LabelGroup == nullptr)
        {
            LabelGroup = new osg::Group();
            annoGroup->addChild(LabelGroup);
        }
        LabelGroup->addChild(_measureLabel);
    } );

    connect(dialog,&LabelingDialog::destroyed,this, [=](){
        disconnect(dialog, nullptr, this , nullptr);
        dialog->deleteLater();
        dialog = nullptr;
        ui->actionAddLabel->setEnabled(true);
    });


    return;

    Style labelStyle;
    labelStyle.getOrCreate<TextSymbol>()->content()->setLiteral("Label with offsets");
    labelStyle.getOrCreate<TextSymbol>()->size() = 24.0f;
    labelStyle.getOrCreate<TextSymbol>()->alignment() = TextSymbol::ALIGN_CENTER_CENTER;
    labelStyle.getOrCreate<TextSymbol>()->fill().mutable_value().color().set(1, 1, 1, 1);
    labelStyle.getOrCreate<TextSymbol>()->halo().mutable_value().color().set(.3, .3, .3, 1);
    labelStyle.getOrCreate<BBoxSymbol>()->fill().mutable_value().color().set(1, 0.5, 0.0, 0.5);
    labelStyle.getOrCreate<BBoxSymbol>()->border().mutable_value().color().set(.8, .8, .8, 1);
    labelStyle.getOrCreate<BBoxSymbol>()->border().mutable_value().width().mutable_value() = osgEarth::Distance(2.0, osgEarth::Units::PIXELS);
    labelStyle.getOrCreate<BBoxSymbol>()->margin().mutable_value() = 3.0f;
    labelStyle.getOrCreate<TextSymbol>()->declutter() = true;

    labelStyle.getOrCreate<IconSymbol>()->url().mutable_value().setLiteral( "C:/Users/pnmt1054/Adithya-working-directory/QT_PROJECTS/GeoLayer-X/icons/drawTools/pin.png" );

    labelStyle.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;
    labelStyle.getOrCreate<AltitudeSymbol>()->technique() = AltitudeSymbol::TECHNIQUE_DRAPE;

    GeoPoint pos(
        mapNode->getMapSRS(),
        -120.0, 16.0, 0.0,  // lon, lat, altitude
        ALTMODE_ABSOLUTE);

    LabelNode* _measureLabel = new LabelNode();
    _measureLabel->setDynamic(true);
    _measureLabel->setStyle(labelStyle);
    _measureLabel->setPosition(pos);
    // _measureLabel->setNodeMask(0);
    // mapNode->addChild(_measureLabel);

    annoGroup->addChild(_measureLabel);
}

void MainWindow::on_actionLoad3D_Model_triggered()
{
    // QString filePath = QFileDialog::getOpenFileName(this,"Open Files","","*.obj");
    // if (filePath.isEmpty())
    //     return;

    // osgEarth::ModelLayer::Options modelOpts;
    // modelOpts.url() = osgEarth::URI(filePath.toStdString().c_str());   // path to your OBJ
    // modelOpts.location() =  osgEarth::GeoPoint(
    //             mapNode->getMapSRS(),
    //             77.59,    // Longitude
    //             12.97);

    // // Create ModelLayer
    // osg::ref_ptr<osgEarth::ModelLayer> modelLayer =
    //     new osgEarth::ModelLayer( modelOpts);

    // // Add it to the map
    // mapNode->getMap()->addLayer(modelLayer.get());


    // // OPTIONAL: Move the camera to focus on this model
    // osgEarth::Viewpoint vp("Model View", 77.59, 12.97, 20.0, 0.0, -90.0, 1000.0);
    // manip->setViewpoint(vp);

// ------------------------------------------------------------------------------------------------
    // File dialog for selecting model
       QString filePath = QFileDialog::getOpenFileName(
           this,
           "Open 3D Model",
           "",
           "3D Models (*.obj *.gltf *.glb)"
       );

       if (filePath.isEmpty()) {
           qDebug() << "No file selected.";
           return;
       }

       qDebug() << "Selected model:" << filePath;

       // Load the model using OSG
       osg::ref_ptr<osg::Node> model = osgDB::readNodeFile(filePath.toStdString());
       if (!model.valid()) {
           qDebug() << "❌ Failed to load model:" << filePath;
           return;
       }

       qDebug() << "✅ Model loaded successfully:" << model->className();

       // Apply default shader
       osgEarth::Registry::shaderGenerator().run(model.get());

       // Create GeoTransform to position the model
       osg::ref_ptr<osgEarth::GeoTransform> geoXform = new osgEarth::GeoTransform();

       // Example coordinates (Bengaluru)
       osgEarth::GeoPoint geoPoint(
           mapNode->getMapSRS(),
           77.59,   // Longitude
           12.97,   // Latitude
           0.0,     // Altitude
           osgEarth::ALTMODE_RELATIVE
       );

       geoXform->setPosition(geoPoint);
       // geoXform->setScale(osg::Vec3d(1000.0, 1000.0, 1000.0)); // Adjust for visibility
       geoXform->addChild(model.get());

       // Add to map node
       mapNode->addChild(geoXform.get());
       qDebug() << "✅ Model added to scene.";

       /*
       // ✅ Alternative: Add as ModelLayer instead
       osg::ref_ptr<osgEarth::ModelLayer> modelLayer = new osgEarth::ModelLayer();
       modelLayer->setName("CustomModel");
       modelLayer->setNode(model.get());

       osgEarth::GeoPoint location(mapNode->getMapSRS(), 77.59, 12.97);
       modelLayer->setLocation(location);
       mapNode->getMap()->addLayer(modelLayer.get());
       qDebug() << "✅ Model added as a ModelLayer.";
       */

    return;

    // QString filePath = QFileDialog::getOpenFileName(this, "Open Model", "", "*.obj");
    // if (filePath.isEmpty())
    //     return;

    // // Create ModelLayer
    // osg::ref_ptr<osgEarth::ModelLayer> modelLayer = new osgEarth::ModelLayer();

    // // Set URL to load .obj file
    // modelLayer->setURL(osgEarth::URI(filePath.toStdString()));

    // // Set initial placement (Bengaluru coordinates as example)
    // osgEarth::GeoPoint location(
    //     mapNode->getMapSRS(),
    //     77.59,   // lon
    //     12.97,   // lat
    //     0.0,     // alt
    //     osgEarth::ALTMODE_ABSOLUTE);

    // modelLayer->setLocation(location);

    // // Optional orientation (heading, pitch, roll in degrees)
    // modelLayer->setOrientation(osg::Vec3(0.0, 0.0, 0.0));

    // // Optional: scale model (not always supported by ModelLayer, better via node transform)
    // modelLayer->setLODScale(10.0f);

    // // Add to the map
    // mapNode->getMap()->addLayer(modelLayer);






}

#include <osgEarth/MapNode>
#include <osgEarth/GeoTransform>
#include <osgEarth/SpatialReference>
#include <osg/ShapeDrawable>
#include <osg/Geode>
#include <osg/Group>
#include <cstdlib>
#include <ctime>
#include "RadarDomSimulationWidget.h"
RadarDomSimulationWidget* radarWidget = nullptr;
void MainWindow::on_actionRadar_dome_triggered()
{
    ui->actionRadar_dome->setChecked(true);

    if(!radarWidget)
    {
        radarWidget= new RadarDomSimulationWidget(mapNode,nullptr);
        ui->stackedWidget->addWidget(radarWidget);

        QObject::connect(radarWidget,&RadarDomSimulationWidget::closeWindowClicked, this, [=](){
           ui->stackedWidget->removeWidget(radarWidget);
           radarWidget->deleteLater();
           radarWidget= nullptr;
        });
    }

    ui->stackedWidget->setCurrentWidget(radarWidget);

}

#include "Mesurements/VolumeCalculatorWidget.h"
VolumeCalculatorWidget* volumewidget = nullptr;
void MainWindow::on_action3D_Volume_triggered()
{
    if(!volumewidget){

        volumewidget = new VolumeCalculatorWidget();
        ui->stackedWidget->addWidget(volumewidget);

    }

    ui->stackedWidget->setCurrentWidget(volumewidget);

}


#include "Mesurements/DistanceCalculator.h"
DistanceCalculator* distance = nullptr;
void MainWindow::on_actionDistanceCalculator_triggered()
{

    if(!distance){
        distance = new DistanceCalculator(mapNode);
    }
}

#include"LineOfSightWidget.h"
LineOfSightWidget* los = nullptr;
void MainWindow::on_actionLineOfSight_triggered()
{
    if(!los){
        los = new LineOfSightWidget(mapNode);
        ui->stackedWidget->addWidget(los);

    }
    ui->stackedWidget->setCurrentWidget(los);
}

