#include <QApplication>
#include "mainwindow.h"
#include <QSurfaceFormat>
#include <QTimer>
#include <QQuickStyle>

#include <osg/Notify>
#include <osgEarth/Registry>

extern "C" __declspec(dllexport) int glx_main(int argc, char* argv[])
{
    QQuickStyle::setStyle("Fusion");

    //set osgNotification level to WARN
    osg::setNotifyLevel(osg::WARN);
    
    osgEarth::initialize();
    osgEarth::Registry::instance()->unRefImageDataAfterApply() = false;

    QSurfaceFormat format = QSurfaceFormat::defaultFormat();
    #ifdef OSG_GL3_AVAILABLE
    format.setVersion(3, 3);
    format.setProfile(QSurfaceFormat::CoreProfile);
    format.setRenderableType(QSurfaceFormat::OpenGL);
    format.setOption(QSurfaceFormat::DebugContext);
    #else
    format.setVersion(2, 0);
    format.setProfile(QSurfaceFormat::CompatibilityProfile);
    format.setRenderableType(QSurfaceFormat::OpenGL);
    format.setOption(QSurfaceFormat::DebugContext);
    #endif
    format.setDepthBufferSize(24);
    //format.setAlphaBufferSize(8);
    format.setStencilBufferSize(8);
    // format.setSamples(4);
    format.setSwapBehavior(QSurfaceFormat::DoubleBuffer);
    format.setColorSpace(QSurfaceFormat::DefaultColorSpace); // avoid sRGB auto


    QSurfaceFormat::setDefaultFormat(format);


    QApplication app(argc, argv);

    QCoreApplication::setOrganizationName("SpereGIS");
    QCoreApplication::setApplicationName("GeoLayer-X");


    QApplication::setStyle("Fusion");

    MainWindow window;
    window.initAll();
    window.showMaximized();
    QTimer::singleShot(0, &window, &MainWindow::showWelcomeScreen);

    return app.exec();
}
