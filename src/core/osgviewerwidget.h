#ifndef OSGVIEWERWIDGET_H
#define OSGVIEWERWIDGET_H

#include <osg/Node>
#include <osg/Group>
#include <osgViewer/Viewer>

#include <osgQOpenGL/osgQOpenGLWidget>

#include <osgEarth/MapNode>
#include <osgEarth/EarthManipulator>

#include "osgearthcore_global.h"

class OSGEARTHCORE_EXPORT OSGViewerWidget : public osgQOpenGLWidget
{
    Q_OBJECT

public:
    enum BaseMapType
    {
        OpenStreetMap,
        BingMaps,
        GoogleMaps,
        Mapbox,
        ArcGIS
    };
    static OSGViewerWidget* instance();

    static osg::Group* getRoot()  { return _root.get(); }
    static void setRoot(osg::Group* root);

    static osgEarth::MapNode* getMapNode() { return _mapNode.get(); }
    static void setMapNode(osgEarth::MapNode* mapNode);

    static osgEarth::EarthManipulator* getManipulator()  { return _manipulator.get(); }
    static void setManipulator(osgEarth::EarthManipulator* manipulator);

    static void setViewer(osgViewer::Viewer* viewer);
    static osgViewer::Viewer* getViewer()  { return _viewer.get(); }

    void setViewerColor(const osg::Vec4& color);

    void setBaseMap(BaseMapType type);

signals:
    void setupCompleted();

private:
    bool setupOsgEarth();

protected:
    OSGViewerWidget(QWidget* parent = nullptr);

    ~OSGViewerWidget() override;

private:
    static OSGViewerWidget* _instance;
    static osg::ref_ptr<osg::Group> _root;
    static osg::ref_ptr<osgViewer::Viewer> _viewer;
    static osg::ref_ptr<osgEarth::MapNode> _mapNode;
    static osg::ref_ptr<osgEarth::EarthManipulator> _manipulator;

    osg::Vec4 _viewerColor = osg::Vec4(0.2f, 0.2f, 0.2f, 1.0f);

    BaseMapType _baseMap =  BaseMapType::GoogleMaps;
    QVector<QString> _baseMapURL= {
        "https://a.tile.openstreetmap.org/{z}/{x}/{y}.png",
        "https://ecn.t3.tiles.virtualearth.net/tiles/a{q}.jpeg?g=1",
        "https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}",
        "https://api.mapbox.com/styles/v1/mapbox/satellite-v9/tiles/256/{z}/{x}/{y}@2x?access_token=YOUR_MAPBOX_ACCESS_TOKEN",
        "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"
    };



//declare private function's here
private:
};
#endif // OSGVIEWERWIDGET_H
