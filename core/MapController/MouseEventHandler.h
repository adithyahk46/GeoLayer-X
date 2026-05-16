#ifndef MOUSEEVENTHANDLER_H
#define MOUSEEVENTHANDLER_H

#include <iostream>


class MouseEventHandler :public QObject, public osgGA::GUIEventHandler
{
    Q_OBJECT

public:

    static MouseEventHandler* Instance(osgEarth::MapNode* mapNode = nullptr);

    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa) override;

    void disableMouseAction(bool state);



signals:
    void mouseClickEvent(const double lon, const double lat,const double alt,const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
    void mouseDoubleClickEvent(const double lon, const double lat,const double alt,const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
    void mouseMoveEvent(const double lon, const double lat,const double alt,const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
    void mouseScrollEvent(const double lon, const double lat,const double alt,const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);


private:
    explicit MouseEventHandler(osgEarth::MapNode* mapNode = nullptr);


    double lon;
    double lat;
    double alt;

    osgViewer::View* view = nullptr;
    osg::ref_ptr<osgEarth::MapNode> _mapNode;

    bool _disableMouseAction = false;


    bool getLatLonFromScreen(osgViewer::View *view, float x, float y, double &lat, double &lon, double &height);
};

#endif // MOUSEEVENTHANDLER_H
