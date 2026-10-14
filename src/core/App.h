#ifndef APP_H
#define APP_H

#include <QMainWindow>

#include <osgEarth/MapNode>
#include <osgEarth/EarthManipulator>
#include <osgViewer/Viewer>

#include "osgviewerwidget.h"

class OSGViewerWidget;


#include "osgearthcore_global.h"


class OSGEARTHCORE_EXPORT App
{
public:
    static App* getInstance();
    ~App();
    static void setThreedWindow(OSGViewerWidget* window){_threeDWindow = window;}
    static OSGViewerWidget* getThreedWindow(){return _threeDWindow;}

    static void set2DWindow(OSGViewerWidget* window){_2DWindow = window;}
    static OSGViewerWidget* get2DWindow(){return _2DWindow;}

    static void setMainwindow(QMainWindow* window){_mainWindow = window;}
    static QMainWindow* getMainwindow(){return _mainWindow;}

    void sync2D3DView(OSGViewerWidget* ViewerWidget2D, OSGViewerWidget* ViewerWidget3D);
    void unsync2D3DView(OSGViewerWidget* ViewerWidget2D, OSGViewerWidget* ViewerWidget3D);
    bool is2D3DViewSynced(OSGViewerWidget* ViewerWidget2D, OSGViewerWidget* ViewerWidget3D);

private:
    App() = default;

    static App* _app;
    static QMainWindow* _mainWindow;
    static OSGViewerWidget* _threeDWindow;
    static OSGViewerWidget* _2DWindow;
};

#endif // APP_H
