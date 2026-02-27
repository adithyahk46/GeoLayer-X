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
#include "legends.h"
#include "MouseEventHandler.h"

#include <osgEarth/PlaceNode>
#include <osgEarth/FeatureNode>

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


//user defined Private Slots:
private slots:
    void initOsg();

    void OnMapLoaded(osgEarth::Layer* layer = nullptr);

 //Application Defined Slots
private slots:
    void on_actionViewshed_triggered();

private:
    void _initConnections();

//user defined member variables
private:
    Ui::MainWindow *ui;
    osgQOpenGLWidget *osgWidget{nullptr};
    osgViewer::Viewer *viewer{nullptr};
    EarthManipulator* manip = nullptr;
    MapNode* mapNode = nullptr;
    osg::Group* root = nullptr;


    MouseEventHandler* mouseControlle = nullptr;
    QLineEdit* _lat;
    QLineEdit* _long;
    QLineEdit* _height;
    QComboBox* _CRS;
    QLabel* _label;

};
#endif // MAINWINDOW_H
