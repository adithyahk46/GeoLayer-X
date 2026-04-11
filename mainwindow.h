#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "app/MapLoadModuleDialog.h"
#include "app/MyMapCallback.h"
#include "app/legends.h"
#include "app/MouseEventHandler.h"


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

 //Application Defined Slots
private slots:
    void on_actionViewshed_triggered();

    void on_actionRadar_Platter_triggered();

    void on_actionDark_Theme_triggered(bool checked);

private:
    void _initConnections();

//user defined member variables
private:
    Ui::MainWindow *ui;
    osgQOpenGLWidget *osgWidget{nullptr};
    osg::ref_ptr<osgViewer::Viewer> viewer;
    osg::ref_ptr<osgEarth::EarthManipulator> manip;
    osg::ref_ptr<osgEarth::MapNode>mapNode;
    osg::ref_ptr<osg::Group> root;


    MouseEventHandler* mouseControlle = nullptr;


};
#endif // MAINWINDOW_H
