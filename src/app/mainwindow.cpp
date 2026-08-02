#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QFileDialog>
#include "DataManager/MapLoadModule.h"
#include "DataManager/MyMapCallback.h"
#include "DataManager/LayerManagerWidget.h"

#include "MapLoadModuleDialog.h"


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);


    initCore();

    initSignalsAndSlots();


}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::setApplicationTheme(bool dark)
{
    QPalette palette;

    if(dark){
        palette.setColor(QPalette::Window,          QColor("#1e1e1e"));
       palette.setColor(QPalette::WindowText,      QColor("#ffffff"));

       palette.setColor(QPalette::Base,            QColor("#2d2d2d"));
       palette.setColor(QPalette::AlternateBase,   QColor("#001a68"));

       palette.setColor(QPalette::Text,            QColor("#ffffff"));
       palette.setColor(QPalette::PlaceholderText, QColor("#80ffffff"));

       palette.setColor(QPalette::Button,          QColor("#3c3c3c"));
       palette.setColor(QPalette::ButtonText,      QColor("#ffffff"));

       palette.setColor(QPalette::Highlight,       QColor("#0078d4"));
       palette.setColor(QPalette::HighlightedText, QColor("#ffffff"));

       palette.setColor(QPalette::Link,            QColor("#99ebff"));
       palette.setColor(QPalette::LinkVisited,     QColor("#4cc2ff"));

       palette.setColor(QPalette::ToolTipBase,     QColor("#3c3c3c"));
       palette.setColor(QPalette::ToolTipText,     QColor("#d4d4d4"));
    }
    else {
        palette.setColor(QPalette::Window,          QColor("#f5f5f5"));
        palette.setColor(QPalette::WindowText,      QColor("#202020"));

        palette.setColor(QPalette::Base,            QColor("#ffffff"));
        palette.setColor(QPalette::AlternateBase,   QColor("#f0f0f0"));

        palette.setColor(QPalette::Text,            QColor("#202020"));
        palette.setColor(QPalette::PlaceholderText, QColor("#80000000"));

        palette.setColor(QPalette::Button,          QColor("#e5e5e5"));
        palette.setColor(QPalette::ButtonText,      QColor("#202020"));

        palette.setColor(QPalette::Highlight,       QColor("#0078d4"));
        palette.setColor(QPalette::HighlightedText, QColor("#ffffff"));

        palette.setColor(QPalette::Link,            QColor("#0067c5"));
        palette.setColor(QPalette::LinkVisited,     QColor("#5c2d91"));

        palette.setColor(QPalette::ToolTipBase,     QColor("#ffffff"));
        palette.setColor(QPalette::ToolTipText,     QColor("#202020"));
    }

    QApplication::setPalette(palette);

}

void MainWindow::initCore()
{

    LayerManagerWidget* layerManager = LayerManagerWidget::getInstance(this);
    addDockWidget(Qt::RightDockWidgetArea, layerManager);
    //create central layout and central StackedWidget and insert osgwidget into it

    osgwidget = OSGViewerWidget::instance();
    osgwidget->getMapNode()->getMap()->addMapCallback(layerManager);
    setCentralWidget(osgwidget);

    MapLoadModule::LoadTMSLayer(osgwidget->getMapNode(), "https://readymap.org/readymap/tiles/1.0.0/7/");
}

void MainWindow::initSignalsAndSlots()
{
    //connect signals and slots here
    QObject::connect(ui->actionOpenEarthFile, &QAction::triggered, this, [this]() {
        QString filePath = QFileDialog::getOpenFileName(this, "Open Earth File", "", "Earth Files (*.earth)");
        if (!filePath.isEmpty()) {
            MapLoadModule::LoadEarthFile(filePath);
        }
    });

    QObject::connect(ui->actionTheame, &QAction::toggled, this, [=](bool dark){
        setApplicationTheme(dark);
    });

    QObject::connect(ui->actionFullScreen, &QAction::triggered, this, [this](){
        if(isFullScreen()){showNormal();}
        else{showFullScreen(); }
    });

    QObject::connect(ui->actionLoad_layers, &QAction::triggered, this, [this](){
        //clear the memeory when dialog is closed
        MapLoadModuleDialog* dialog = new MapLoadModuleDialog(this);
        QObject::connect(dialog, &QDialog::finished, dialog, [&dialog](){
            // dialog->deleteLater();
            dialog = nullptr;
        });
        dialog->show();

    });

}