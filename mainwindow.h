#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include <osgQOpenGL/osgQOpenGLWidget>
#include <QtWidgets/qboxlayout.h>
#include <qmainwindow.h>
#include <qmenubar.h>
#include <qdockwidget.h>

#include <osgDB/ReadFile>
#include <osgUtil/Optimizer>
#include <osg/CoordinateSystemNode>
#include <osg/ShapeDrawable>

#include <osg/Switch>
#include <osg/Types>
#include <osgText/Text>

#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgEarth/Sky>


#include <osgGA/TrackballManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/DriveManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/StateSetManipulator>
#include <osgGA/AnimationPathManipulator>
#include <osgGA/TerrainManipulator>
#include <osgGA/SphericalManipulator>

#include <osgGA/Device>
#include <osgEarth/GeoMath>
#include <osgEarth/GeoTransform>
#include <osgEarth/MapNode>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/Viewpoint>
#include <osgEarth/EarthManipulator>
#include <osgEarth/ExampleResources>
#include <osgEarth/LogarithmicDepthBuffer>
#include <osgEarth/ViewFitter>
#include <osgEarth/AnnotationUtils>
#include <osgEarth/LabelNode>
#include <osgEarth/Style>
#include <osgEarth/ScreenSpaceLayout>
#include <osgEarth/ModelNode>
#include <osgEarth/LineDrawable>
#include <osgUtil/Optimizer>
#include <osgEarth/FeatureNode>
#include <osgEarth/FeatureIndex>
#include <osgEarth/IntersectionPicker>
#include <osgEarth/Registry>
#include <osgEarth/RTTPicker>
#include <osgEarth/GLUtils>
#include <osgEarth/GeoPositionNodeAutoScaler>
#include <osgEarth/ElevationQuery>
#include <osgEarth/FeatureModelLayer>
#include <osgEarth/OGRFeatureSource>
#include <QApplication>
#include <QSurfaceFormat>
#include <QTextCodec>

// #include "DistanceCaculator.h"
// #include "AreaCaculator.h"
// #include "AngleCaculator.h"

#include <osgEarth/MapNode>
#include <osgEarth/Map>
#include <osgEarth/TMS>
#include <osgEarth/EarthManipulator>
#include <osg/ArgumentParser>
#include <osgViewer/Viewer>
#include <osgDB/ReadFile>
#include <osgViewer/Viewer>
#include <osgEarth/ElevationLayer>
#include <osgEarth/GDAL>
#include <osgEarth/GLUtils>

#include <osgEarth/Units>
#include <osgEarth/Expression>
#include <osgViewer/CompositeViewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/StateSetManipulator>

#include <QMessageBox>

#include <QDebug>
#include <QListWidgetItem>
#include <QMouseEvent>

#include <osgEarth/Controls>

#include <QLinkedList>
#include <any>
#include <vector>
#include <string>
#include <iostream>


#include "MapLoadModule.h"
#include "MyMapCallback.h"


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



private slots:

    void initOsg();

    void on_pushButton_clicked();

    void OnMapLoaded(osgEarth::Layer* layer = nullptr);

protected:


private:

    template<typename T>
    void CreateLegend(T *mapLayer = nullptr);

    // template <typename T>
    // QListWidgetItem* findItemForLegend(Legends<T>* legend);

    void _initConnections();


private:
    Ui::MainWindow *ui;
    // Ui::OpenLayersDialog *LayersDialog;

    osgQOpenGLWidget *osgWidget{nullptr};
    osgViewer::Viewer *viewer{nullptr};
    EarthManipulator* manip = nullptr;
    MapNode* mapNode = nullptr;
    osg::Group* annoGroup;
    // osgEarth::SkyNode skyNode;
    FeatureModelLayer* buildingLayer = nullptr;


};
#endif // MAINWINDOW_H
