#ifndef OSGEARTHVIEWERWIDGET_H
#define OSGEARTHVIEWERWIDGET_H

#include <osg/Node>
#include <osg/Group>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>

#include <osgQOpenGL/osgQOpenGLWidget>

#include <osgEarth/MapNode>
#include <osgEarth/EarthManipulator>

#ifndef OSGEARTHVIEWERWIDGET_EXPORT
#define OSGEARTHVIEWERWIDGET_EXPORT
#endif


class OSGEARTHVIEWERWIDGET_EXPORT OsgEarthViewerWidget : public osgQOpenGLWidget
{
    Q_OBJECT

public:
    static OsgEarthViewerWidget* instance();

    void setSceneData(osg::Node* node);
    void setMapNode(osgEarth::MapNode* mapNode);
    osgEarth::MapNode* getMapNode() const;
    void setManipulator(osgEarth::EarthManipulator* manipulator);
    osgEarth::EarthManipulator* getManipulator() const;

private:
    bool setupOsgEarth();

protected:
    OsgEarthViewerWidget(QWidget* parent = nullptr);

    ~OsgEarthViewerWidget() override;

private:
    static OsgEarthViewerWidget* _instance;
    osg::ref_ptr<osg::Group> _root;
    osg::ref_ptr<osgViewer::Viewer> _viewer;
    osg::ref_ptr<osgEarth::MapNode> _mapNode;
    osg::ref_ptr<osgEarth::EarthManipulator> _manipulator;
};

#endif // OSGEARTHVIEWERWIDGET_H
