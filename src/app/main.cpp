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

#include <osgEarth/MapNode>

QString loadFile(const QString& path) {
    QFile file(path);
    if (file.open(QFile::ReadOnly | QFile::Text)) {
        return QString::fromUtf8(file.readAll());
    }
    return QString();
}

void setTheme(QApplication& app, const QString& colorThemeName) {
    // 1. Read layout/structural rules
    QString baseStyle = loadFile(":/themes/base.qss");
    // 2. Read color rules (e.g., "dark" or "light")
    QString colorStyle = loadFile(":/themes/" + colorThemeName + ".qss");
    // 3. Combine them together!
    app.setStyleSheet(baseStyle + "\n" + colorStyle);
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
    format.setSwapBehavior(QSurfaceFormat::DoubleBuffer);
    format.setColorSpace(QSurfaceFormat::DefaultColorSpace); // avoid sRGB auto


    QSurfaceFormat::setDefaultFormat(format);



    QApplication app(argc, argv);

    QCoreApplication::setOrganizationName("MyCompany");
    QCoreApplication::setApplicationName("MyGIS");

    // QGuiApplication::styleHints()->setColorScheme(Qt::ColorScheme::Light);

    // setTheme(app, "light");
    // Set default palette provided by the current style


    for (const QString &style : QStyleFactory::keys())
        qDebug() << style;

    // QApplication::setStyle(QStyleFactory::create("Fusion"));

    // qApp->setStyleSheet("");
    QApplication::setStyle("Fusion");
    // QApplication::setPalette(QApplication::style()->standardPalette());

    MainWindow window;
    window.showMaximized();

    qDebug() << qApp->styleSheet();

    return app.exec();
}
