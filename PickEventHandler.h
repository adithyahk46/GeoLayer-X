#ifndef PICKEVENTHANDLER_H
#define PICKEVENTHANDLER_H

#include <osgEarth/MapNode>
#include <osgEarth/EarthManipulator>
#include <osgViewer/Viewer>
#include <osgGA/GUIEventHandler>
#include <osgEarth/GeoData>
#include <osgEarth/SpatialReference>
#include <osgUtil/LineSegmentIntersector>

#include <QLabel>
#include <QStatusBar>
#include <QLineEdit>
#include <QComboBox>

class PickEventHandler : public osgGA::GUIEventHandler
{
public:
    explicit PickEventHandler(QStatusBar* StatusBar = nullptr, osgEarth::MapNode* mapNode = nullptr);

    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa) override;

private:
    osg::ref_ptr<osgEarth::MapNode> _mapNode;
    QStatusBar* _statusBar = nullptr;
    QLineEdit* _lat;
    QLineEdit* _long;
    QLineEdit* _height;
    QComboBox* _CRS;
    QLabel* _label;



    bool getLatLonFromScreen(osgViewer::View* view, float x, float y, double& lat, double& lon, double& height);
};

#endif // PICKEVENTHANDLER_H
