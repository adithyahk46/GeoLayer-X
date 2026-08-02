#include "MainWindow.h"

#include <QApplication>

// int main(int argc, char* argv[])
// {

//     osg::setNotifyLevel(osg::WARN);
//         osgEarth::Registry::instance()->getCapabilities();

//         QSurfaceFormat format = QSurfaceFormat::defaultFormat();
//         // QTextCodec *codec = QTextCodec::codecForLocale();
//         #ifdef OSG_GL3_AVAILABLE
//         format.setVersion(3, 2);
//         format.setProfile(QSurfaceFormat::CoreProfile);
//         format.setRenderableType(QSurfaceFormat::OpenGL);
//         format.setOption(QSurfaceFormat::DebugContext);
//         #else
//         format.setVersion(2, 0);
//         format.setProfile(QSurfaceFormat::CompatibilityProfile);
//         format.setRenderableType(QSurfaceFormat::OpenGL);
//         format.setOption(QSurfaceFormat::DebugContext);
//         #endif
//         format.setDepthBufferSize(24);
//         //format.setAlphaBufferSize(8);
//         format.setStencilBufferSize(8);
//         format.setSwapBehavior(QSurfaceFormat::DoubleBuffer);
//         format.setColorSpace(QSurfaceFormat::DefaultColorSpace); // avoid sRGB auto


//         QSurfaceFormat::setDefaultFormat(format);


//         osgEarth::initialize();
// 	QApplication app(argc, argv);

// 	MainWindow window;
//     window.showMaximized();

// 	return app.exec();
// }


#include <osgEarth/MapNode>
#include <osgEarth/TMS>
#include <osgEarth/EarthManipulator>
#include <osgEarth/GLUtils>
#include <osg/ArgumentParser>
#include <osgViewer/Viewer>

int main(int argc, char** argv)
{
    osgEarth::initialize();

    osg::ArgumentParser args(&argc, argv);
    osgViewer::Viewer viewer(args);
    viewer.setRealizeOperation(new osgEarth::GL3RealizeOperation());

    auto imagery = new osgEarth::TMSImageLayer();
    imagery->setURL("https://readymap.org/readymap/tiles/1.0.0/7/");

    auto mapNode = new osgEarth::MapNode();
    mapNode->getMap()->addLayer(imagery);

    viewer.setSceneData(mapNode);
    viewer.setCameraManipulator(new osgEarth::EarthManipulator(args));

    return viewer.run();
}