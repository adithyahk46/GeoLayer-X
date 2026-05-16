#include "MainWindow.h"

#include <QApplication>

int main(int argc, char* argv[])
{

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
	QApplication app(argc, argv);

	MainWindow window;
	window.show();

	return app.exec();
}
