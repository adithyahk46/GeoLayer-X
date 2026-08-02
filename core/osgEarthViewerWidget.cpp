
#include "osgEarthViewerWidget.h"

#include <QCoreApplication>
#include <QDebug>
#include <QObject>

#include <osgDB/ReadFile>
#include <osgUtil/Optimizer>

OsgEarthViewerWidget* OsgEarthViewerWidget::_instance = nullptr;

OsgEarthViewerWidget* OsgEarthViewerWidget::instance()
{
    if (!_instance)
    {
        _instance = new OsgEarthViewerWidget();
    }
    return _instance;
}

OsgEarthViewerWidget::OsgEarthViewerWidget(QWidget* parent)
    : osgQOpenGLWidget(parent)
{

    _root = new osg::Group();

    setMouseTracking(true);


    _manipulator = new osgEarth::EarthManipulator();
    _manipulator->getSettings()->setThrowingEnabled(true);

    //create a default mapNode with google statallite imagery 
    QString figPath = QCoreApplication::applicationDirPath() + "/Data/earthFiles/simple.earth";
    osg::ref_ptr<osg::Node> earthNode = osgDB::readNodeFile(figPath.toStdString());
    if(!earthNode)
    {
        qWarning() << "Failed to load the default .earth file: " << figPath;
    }
    else
    {
        _root->addChild(earthNode);
        _mapNode = osgEarth::MapNode::findMapNode(earthNode);
    }

    osgUtil::Optimizer optimizer;
    if (earthNode)
    {
        optimizer.optimize(earthNode);
    }
    optimizer.optimize(_root);
    optimizer.reset();

    QObject::connect(this, &osgQOpenGLWidget::initialized, [this] { setupOsgEarth();
    });
}

OsgEarthViewerWidget::~OsgEarthViewerWidget()
{
    _manipulator = nullptr;
    _mapNode = nullptr;
    _viewer = nullptr;
    _root = nullptr;
}

bool OsgEarthViewerWidget::setupOsgEarth()
{
    _viewer = getOsgViewer();
    if (!_viewer)
    {
        qWarning() << "Failed to acquire osg viewer instance.";
        return false;
    }

    _viewer->setThreadingModel(osgViewer::Viewer::SingleThreaded);
    _viewer->setCameraManipulator(_manipulator.get());
    _viewer->addEventHandler(new osgViewer::StatsHandler());

    _viewer->setSceneData(_root);

    _viewer->realize();

    return true;
}

void OsgEarthViewerWidget::setSceneData(osg::Node* node)
{
    if (!node)
    {
        qWarning() << "Cannot set scene data: node is null.";
        return;
    }

    if (!_root)
    {
        _root = new osg::Group();
    }

    _root->removeChildren(0, _root->getNumChildren());
    _root->addChild(node);

    _mapNode = osgEarth::MapNode::findMapNode(node);

    if (_viewer)
    {
        _viewer->setSceneData(_root.get());
    }
}

void OsgEarthViewerWidget::setMapNode(osgEarth::MapNode* mapNode)
{
    _mapNode = mapNode;
}

osgEarth::MapNode* OsgEarthViewerWidget::getMapNode() const
{
    return _mapNode.get();
}

void OsgEarthViewerWidget::setManipulator(osgEarth::EarthManipulator* manipulator)
{
    if (!manipulator)
    {
        qWarning() << "Cannot set manipulator: manipulator is null.";
        return;
    }

    _manipulator = manipulator;

    if (_viewer)
    {
        _viewer->setCameraManipulator(_manipulator.get());
    }
}

osgEarth::EarthManipulator* OsgEarthViewerWidget::getManipulator() const
{
    return _manipulator.get();
}
