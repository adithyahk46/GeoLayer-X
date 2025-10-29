#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <osgQOpenGL/osgQOpenGLWidget>
#include <osg/Group>
#include <osg/Node>
#include <osgDB/ReadFile>

#include <osgUtil/Optimizer>
#include <osgViewer/Viewer>

#include <osgEarth/EarthManipulator>
#include <osgEarth/Map>
#include <osgEarth/MapNode>
#include <osgEarth/LogarithmicDepthBuffer>
#include <osgEarth/Sky>

#include <osgEarth/Layer>
#include <osgEarth/GDAL>
#include <osgEarth/FeatureModelLayer>
#include <osgEarth/OGRFeatureSource>
#include <osgEarth/XYZ>
#include <osgEarth/SpatialReference>
#include <osgEarth/GeoData>
#include <osgEarth/Viewpoint>

#include <osgEarth/AnnotationLayer>

#include <gdal.h>
#include <gdal_priv.h>
#include <gdal_alg.h>
#include <ogrsf_frmts.h>
#include <gdal_proxy.h>
#include <ogr_api.h>
#include <ogr_spatialref.h>
#include <QDir>

#include "MapLoadModule/LoadLayersOnMap.h"
#include "MapLoadModule/MyMapCallback.h"
#include "Simulation/tcpclient.h"
#include "legends.h"
#include "Simulation/FlightSimulator.h"
#include "RouteFinder/RoutingWidget.h"
#include "MouseEventHandler.h"
#include "Annotation/Annotation.h"
#include "Annotation/DrawPolygonTool.h"
#include "Annotation/DrawCircle.h"
#include "Annotation/LabelingDialog.h"
#include "Annotation/CreateShapeFileDialog.h"


using namespace osgEarth;

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void zoomToLayer(Layer *layer);


//user defined Private Slots:
private slots:
    void initOsg();

    void OnMapLoaded(osgEarth::Layer* layer = nullptr);

    void createTempShapeFile(const QString &outputPath,
                              const QString &encoding,
                              const QString &geometryType,
                              const QString &srs,
                              const osgEarth::AnnotationLayer* Layer);


//Application Defined Slots
private slots:
    void on_actionMissileTracker_triggered();

//user defined member functions
    void on_actionBaseMaps_triggered();

    void on_actionFind_Path_triggered();

    void on_actionGoTo_triggered();

    void on_pushButton_clicked();

    void on_actionCreate_Shape_File_triggered();

    void on_actionClear_Annotations_triggered();

    void on_actionDrawPolygon_triggered(bool checked);

    void on_actionDrawCircle_triggered();

    void on_actionAddLabel_triggered();

    void on_actionLoad3D_Model_triggered();

    void on_actionRadar_dome_triggered();

    void on_action3D_Volume_triggered();

    void on_actionDistanceCalculator_triggered();

    void on_actionLineOfSight_triggered();

private:
    void _initConnections();

//user defined member variables
private:
    Ui::MainWindow *ui;
    osgQOpenGLWidget *osgWidget{nullptr};
    osgViewer::Viewer *viewer{nullptr};
    EarthManipulator* manip = nullptr;
    MapNode* mapNode = nullptr;


    osg::Group* annoGroup;
    osg::Group* PolygonGroup = nullptr;
    osg::Group* LabelGroup = nullptr;



    MouseEventHandler* mouseControlle = nullptr;
    QLineEdit* _lat;
    QLineEdit* _long;
    QLineEdit* _height;
    QComboBox* _CRS;
    QLabel* _label;


    UdpClient* clientDialog = NULL;
    FlightSimulator* udpClientDialog = NULL;
    QDockWidget* goToWidgwt = nullptr;
    osgEarth::PlaceNode* locationMarker = nullptr;

    Annotation* annotationWidget = nullptr;

    RoutingWidget* findpath =nullptr;


    osg::ref_ptr<osgEarth::AnnotationLayer> annotationLayer{nullptr};
    QList<osgEarth::FeatureNode*> listFeatureNode;
    void collectNodes(osg::Group *group, std::vector<osg::Node *> &nodes);

    struct ShapefileInfo {
            QString outputPath;
            QString encoding;
            QString geometryType;
            QString srs;
            QList<QVariantMap> fields;
        };
        ShapefileInfo m_currentShapefileInfo;

        bool toggleCreateShapeFile = false;
};
#endif // MAINWINDOW_H
