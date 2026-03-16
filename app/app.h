#ifndef APP_H
#define APP_H

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

private:
    App(){

    }

    static App* _app;

    osg::ref_ptr<osgEarth::MapNode> _mapNode;
    osg::ref_ptr<osgEarth::EarthManipulator> _manip;
    osg::ref_ptr<osgViewer::Viewer> _viewer;



};

#endif // APP_H
