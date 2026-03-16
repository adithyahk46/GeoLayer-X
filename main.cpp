#include <cstdint>


#ifdef Q_OS_WIN
#include <windows.h>
#include <stdio.h>
#endif

#include "mainwindow.h"
#include <QApplication>
// #include <proj.h>
#include <QDebug>
#include <QTimer>
#include <QDir>
#include <proj.h>
#include <QStyleFactory>


// QString url = "http://[abc].tile.openstreetmap.org/{z}/{x}/{y}.png";
// QString url = "http://172.22.22.213:8000/{z}/{x}/{y}.png";
// QString url = "http://127.0.0.1:8000/{z}/{x}/{y}.png";

// #include <QTextCodec>
void setUpEnvironments(){

    qputenv("PROJ_LIB", "./share/proj");
    qputenv("FONTCONFIG_PATH", "./etc/fonts");

}

int main(int argc, char *argv[])
{

    setUpEnvironments();

    osg::setNotifyLevel(osg::WARN);
    osgEarth::Registry::instance()->getCapabilities();

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


    osgEarth::initialize();
    QApplication a(argc, argv);

     a.setStyle(QStyleFactory::create("Fusion"));
      // Optional: Set a light palette for the Fusion style
     QPalette lightPalette;
      lightPalette.setColor(QPalette::Window, QColor(240, 240, 240));  // Light grey
      lightPalette.setColor(QPalette::WindowText, Qt::black);
      lightPalette.setColor(QPalette::Base, QColor(255, 255, 255));   // White
      lightPalette.setColor(QPalette::AlternateBase, QColor(240, 240, 240));  // Light grey
      lightPalette.setColor(QPalette::ToolTipBase, Qt::white);
      lightPalette.setColor(QPalette::ToolTipText, Qt::black);
      lightPalette.setColor(QPalette::Text, Qt::black);
      lightPalette.setColor(QPalette::Button, QColor(240, 240, 240));  // Light grey
      lightPalette.setColor(QPalette::ButtonText, Qt::black);
      lightPalette.setColor(QPalette::BrightText, Qt::red);

      lightPalette.setColor(QPalette::Highlight, QColor(76, 163, 224));  // Blue highlight
      lightPalette.setColor(QPalette::HighlightedText, Qt::white);

      // a.setPalette(lightPalette);


    QFile  styleFile("Atlas.qss");

      if (styleFile.open(QFile::ReadOnly))
      {
        QString  style(styleFile.readAll());
        a.setStyleSheet(style);
      }

    MainWindow w;
    w.setWindowIcon(QIcon(":/new/prefix1/icons/ApplicationIcon.png"));
    w.showMaximized();

    return a.exec();
}

