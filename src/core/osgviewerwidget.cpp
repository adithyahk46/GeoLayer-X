
#include "osgviewerwidget.h"

#include <QDebug>
#include <QObject>
#include <osgEarth/GLUtils>

#include <osgEarth/LogarithmicDepthBuffer>

void OSGViewerWidget::setMapNode(osgEarth::MapNode *mapNode)
{
    if(mapNode)
    {
        _root->removeChild(_mapNode);
        _mapNode = nullptr;
        _mapNode = mapNode;
        _manipulator->setMapNode(_mapNode.get());
        _root->addChild(mapNode);
    }
    
    
}

OSGViewerWidget::OSGViewerWidget(QWidget* parent)
    : osgQOpenGLWidget(parent)
{
    setMouseTracking(true);
    _root = new osg::Group();
    _mapNode = new osgEarth::MapNode;
    _root->addChild(_mapNode);

    _manipulator = new GLXEarthManipulator(_mapNode.get());
    _manipulator->getSettings()->setThrowingEnabled(true);

    QObject::connect(this, &osgQOpenGLWidget::initialized, [this] {
        setupOsgEarth();
        emit setupCompleted();
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
    _viewer->setRealizeOperation(new osgEarth::GL3RealizeOperation());
    // _viewer->getCamera()->setComputeNearFarMode(osg::CullSettings::COMPUTE_NEAR_FAR_USING_BOUNDING_VOLUMES);
    // _viewer->getCamera()->setNearFarRatio(0.00001);

    _viewer->setCameraManipulator(_manipulator.get());

    osgEarth::Util::LogarithmicDepthBuffer logarithmicDepth;
    logarithmicDepth.install(_viewer->getCamera());

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