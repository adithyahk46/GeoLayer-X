#include <cstdint>
#include "mainwindow.h"
#include <QApplication>
// #include <proj.h>
#include <QDebug>
#include <QTimer>
#include "SplashScreen.h"


int main(int argc, char *argv[])
{
    osgEarth::initialize();

    // PJ_CONTEXT *ctx = proj_context_create();
    // PJ *P = proj_create_crs_to_crs(ctx, "EPSG:4326", "EPSG:3857", nullptr);
    // if (!P) {
    //     qDebug() << "PROJ failed: missing proj.db or wrong path.";
    // } else {
    //     qDebug() << "PROJ CRS transformation object created.";
    // }

    // A trick to get higher fps than 30
      QSurfaceFormat format = QSurfaceFormat::defaultFormat();
      format.setSwapInterval(0);
      QSurfaceFormat::setDefaultFormat(format);


    QApplication a(argc, argv);


    QFile  styleFile("Atlas.qss");

      if (styleFile.open(QFile::ReadOnly))
      {
        QString  style(styleFile.readAll());
        a.setStyleSheet(style);
      }

    // Create splash screen
    QPixmap pixmap(":/new/prefix1/icons/splashScreen.jpg");
    QSplashScreen *splash = new QSplashScreen(pixmap);
    splash->setFont(QFont("Arial", 10));
    splash->show();
    MainWindow w;
    splash->showMessage("Starting application...", Qt::AlignBottom | Qt::AlignCenter, Qt::white);

    // Simulate loading steps using QTimers
    QTimer::singleShot(500, [=]() {
        splash->showMessage("Loading modules...", Qt::AlignBottom | Qt::AlignCenter, Qt::white);
    });

    QTimer::singleShot(1000, [=]() {
        splash->showMessage("Connecting to services...", Qt::AlignBottom | Qt::AlignCenter, Qt::white);
    });

    QTimer::singleShot(1500, [=]() {
        splash->showMessage("Initializing UI...", Qt::AlignBottom | Qt::AlignCenter, Qt::white);
    });

    QTimer::singleShot(2000, [=]() {
        splash->showMessage("Finalizing...", Qt::AlignBottom | Qt::AlignCenter, Qt::white);
    });



    // Show main window and close splash after 2.5 seconds
    QTimer::singleShot(2500, splash, SLOT(close()));
    QTimer::singleShot(2500, &w, SLOT(showMaximized()));
    w.setWindowIcon(QIcon(":/new/prefix1/icons/ApplicationIcon.png"));
    // QObject::connect(&w, SIGNAL(sendTotalInitSteps(int)), splash, SLOT(setTotalInitSteps(int)));
    //     QObject::connect(&w, SIGNAL(sendNowInitName(const QString&)), splash, SLOT(setNowInitName(const QString&)));



   // w.showMaximized();
    return a.exec();
}
