#ifndef APP_H
#define APP_H

#include <QMainWindow>

#include <osgEarth/MapNode>
#include <osgEarth/EarthManipulator>



class App
{
public:
    static App* getInstance(){
        if(!_app){
            _app = new App();
        }
        return _app;
    }
    ~App();

    void setMapNode(osgEarth::MapNode* mapNode){_mapNode = mapNode;}
    osgEarth::MapNode* getMapNode(){return _mapNode;}

    void setManipulator(osgEarth::EarthManipulator* manip){_manip = manip;}
    osgEarth::EarthManipulator* getManipulator(){return _manip;}

    void setOsgViewer(osgViewer::Viewer* viewer){_viewer = viewer;}
    osgViewer::Viewer* getOsgViewer(){return _viewer;}

    static void setMainwindow(QMainWindow* window){_mainWindow = window;}
    static QMainWindow* getMainwindow(){return _mainWindow;}

private:
    App(){

    }

    static App* _app;
    static QMainWindow* _mainWindow;

    osg::ref_ptr<osgEarth::MapNode> _mapNode;
    osg::ref_ptr<osgEarth::EarthManipulator> _manip;
    osg::ref_ptr<osgViewer::Viewer> _viewer;



};

#endif // APP_H
