#include <QApplication>
#include "mainwindow.h"
#include <QFile>
#include <QTextStream>
#include <QStyleHints>

#include <QApplication>
#include <QStyle>
#include <QPalette>
#include <QStyleFactory>
#include <QSurfaceFormat>
#include <QTimer>

#include <osgEarth/MapNode>

QString loadFile(const QString& path) {
    QFile file(path);
    if (file.open(QFile::ReadOnly | QFile::Text)) {
        return QString::fromUtf8(file.readAll());
    }
    return QString();
}


#define LC "[osgearth_windows] "

using namespace osgEarth;
using namespace osgEarth::Util;

// int usage(const char* name)
// {
//     OE_NOTICE
//         << "\nUsage: " << name << " file.earth"
//         << "\n          --views [num] : Number of windows to open"
//         << "\n          --shared      : Share a single OSG graphics context across all windows (forces SingleThreaded mode)"
//         << "\n          --updates [num] : Number of update traversals"
//         << "\n"
//         << MapNodeHelper().usage() << std::endl;

//     return 0;
// }

int main(int argc, char* argv[])
{

    //set osgNotification level to WARN
    osg::setNotifyLevel(osg::WARN);
    
    osgEarth::initialize();
    osgEarth::Registry::instance()->unRefImageDataAfterApply() = false;

    QSurfaceFormat format = QSurfaceFormat::defaultFormat();
    // QTextCodec *codec = QTextCodec::codecForLocale();
    #ifdef OSG_GL3_AVAILABLE
    format.setVersion(3, 2);
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

    qDebug() << qApp->styleSheet();

    return app.exec();
}
