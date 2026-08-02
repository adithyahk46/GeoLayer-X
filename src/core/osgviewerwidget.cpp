
#include "osgviewerwidget.h"

#include <QDebug>
#include <QObject>
#include <osgEarth/GLUtils>

#include <osgEarth/Map>
#include <osgEarth/Profile>
#include <osgEarth/XYZ>
#include <osgUtil/Optimizer>

OSGViewerWidget* OSGViewerWidget::_instance = nullptr;

osg::ref_ptr<osg::Group> OSGViewerWidget::_root = nullptr;
osg::ref_ptr<osgViewer::Viewer> OSGViewerWidget::_viewer = nullptr;
osg::ref_ptr<osgEarth::MapNode> OSGViewerWidget::_mapNode = nullptr;
osg::ref_ptr<osgEarth::EarthManipulator> OSGViewerWidget::_manipulator = nullptr;

OSGViewerWidget* OSGViewerWidget::instance()
{
    if (!_instance)
    {
        _instance = new OSGViewerWidget();
    }
    return _instance;
}

void OSGViewerWidget::setRoot(osg::Group *root)
{
    _root = nullptr;
    _root = root;
}

void OSGViewerWidget::setMapNode(osgEarth::MapNode *mapNode)
{
    _mapNode = nullptr;
    _mapNode = mapNode;
}

void OSGViewerWidget::setManipulator(osgEarth::Util::EarthManipulator *manipulator)
{
    _manipulator= nullptr;
    _manipulator = manipulator;
}

void OSGViewerWidget::setViewer(osgViewer::Viewer *viewer)
{
    _viewer = nullptr;
    _viewer = viewer;
}

OSGViewerWidget::OSGViewerWidget(QWidget* parent)
    : osgQOpenGLWidget(parent)
{
    setMouseTracking(true);
    _root = new osg::Group();
    _mapNode = new osgEarth::MapNode;
    _root->addChild(_mapNode);
    _manipulator = new osgEarth::EarthManipulator();
    _manipulator->getSettings()->setThrowingEnabled(true);

    // osgUtil::Optimizer optimizer;
    // if (_root.valid())
    // {
    //     optimizer.optimize(_root);
    // }
    // optimizer.reset();

    QObject::connect(this, &osgQOpenGLWidget::initialized, [this] { setupOsgEarth();
    });
}

OSGViewerWidget::~OSGViewerWidget()
{
    _manipulator = nullptr;
    _mapNode = nullptr;
    _viewer = nullptr;
    _root = nullptr;
}

bool OSGViewerWidget::setupOsgEarth()
{
    _viewer = getOsgViewer();
    if (!_viewer)
    {
        qWarning() << "Failed to acquire osg viewer instance.";
        return false;
    }
    _viewer->setThreadingModel(osgViewer::ViewerBase::SingleThreaded);
    OE_WARN << "Forcing --SingleThreaded mode because we are sharing an OSG Graphics Context (GC), "
        "and OSG does not support multi-threading of a shared GC" << std::endl;
    _viewer->setRealizeOperation(new osgEarth::GL3RealizeOperation());
    _viewer->setCameraManipulator(_manipulator.get());
    int width  = this->width();
    int height = this->height();

    double aspectRatio =
        static_cast<double>(width) /
        static_cast<double>(height);

    _viewer->getCamera()->setProjectionMatrixAsPerspective(
        30.0,
        aspectRatio,
        1.0,
        50000000.0
    );
    _viewer->getCamera()->setClearColor(_viewerColor);
    _viewer->setSceneData(_root);

    return true;
}

void OSGViewerWidget::setViewerColor(const osg::Vec4& color)
{
            _viewerColor = color;

    if (_viewer)
    {
        _viewer->getCamera()->setClearColor(color);
    }
}