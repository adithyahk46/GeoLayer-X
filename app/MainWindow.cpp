#include "MainWindow.h"

#include "ui_MainWindow.h"

#include "app.h"
#include <LayersManager/LayerManagerWidget.h>
#include <MapController/MouseEventHandler.h>
#include <UIManager/StatusBarHandler.h>

#include "MapLoadModuleDialog.h"
#include "MyMapCallback.h"

#include <QApplication>
#include <QListWidgetItem>
#include <QMessageBox>


MainWindow::MainWindow(QWidget* parent)
	: QMainWindow(parent)
	, ui(new Ui::MainWindow)
{
	ui->setupUi(this);

	setWindowTitle("GeoLayer X");


    osgWidget = OsgEarthViewerWidget::instance();
    setCentralWidget(osgWidget);

    _pluginManager = PluginManager::instance();
    connect(_pluginManager, &PluginManager::pluginsChanged, this, &MainWindow::_refreshPluginsDock);

    QObject::connect(osgWidget, &osgQOpenGLWidget::initialized, [this] {

        App::getInstance()->setMainwindow(this);
        App::getInstance()->setMapNode(osgWidget->getMapNode());
        App::getInstance()->setManipulator(osgWidget->getManipulator());
        App::getInstance()->setOsgViewer(osgWidget->getOsgViewer());


        MyMapCallback* mapCallback = new MyMapCallback(osgWidget->getManipulator());
        mapCallback->enableZoomToLayer(true);
        osgWidget->getMapNode()->getMap()->addMapCallback(mapCallback);


        osgWidget->getOsgViewer()->addEventHandler(MouseEventHandler::Instance(osgWidget->getMapNode()));

        StatusBarHandler::getInstance()->setStatusbar(ui->statusBar);
        StatusBarHandler::getInstance()->CreatMapReaders();

        PluginHostContext pluginContext;
        pluginContext.application = qApp;
        pluginContext.mainWindow = this;
        pluginContext.mapNode = osgWidget->getMapNode();
        _pluginManager->setHostContext(pluginContext);
        _pluginManager->scanPlugins();



    });
    addDockWidget(Qt::LeftDockWidgetArea,LayerManagerWidget::getInstance());

    setCorner(Qt::BottomLeftCorner, Qt::LeftDockWidgetArea);
    setCorner(Qt::BottomRightCorner, Qt::RightDockWidgetArea);

    _refreshPluginsDock();
    _initConnections();

}

MainWindow::~MainWindow()
{
	delete ui;
}


void MainWindow::_initConnections()
{
    connect(ui->actionPlugins, &QAction::triggered, this, [this] {
        ui->pluginsDockWidget->setVisible(!ui->pluginsDockWidget->isVisible());
    });

    QAction* managePluginsAction = ui->menuPlugins->addAction("Manage Plugins...");
    connect(managePluginsAction, &QAction::triggered, this, [this] {
        _pluginManager->showManagerDialog(this);
    });

    connect(ui->actionLegends, &QAction::triggered, this,[=]{
        LayerManagerWidget::getInstance()->setVisible(!LayerManagerWidget::getInstance()->isVisible());
    });

    connect(ui->actionFull_Screen, &QAction::toggled, this,[this](bool check){
        if(check) this->showFullScreen();
        else this->showMaximized();
    });

    QObject::connect(ui->actionLoad_Layers,&QAction::triggered,this,[=](){
           // LoadLayer("");
           MapLoadModuleDialog* _loadmap = new MapLoadModuleDialog(App::getInstance()->getMapNode(), this);
           _loadmap->setAttribute(Qt::WA_DeleteOnClose);
           _loadmap->show();

       });

    connect(ui->pluginListWidget, &QListWidget::itemDoubleClicked, this, [this](QListWidgetItem* item) {
        if (!item)
        {
            return;
        }

        const QString pluginId = item->data(Qt::UserRole).toString();
        if (pluginId.isEmpty())
        {
            return;
        }

        QString errorMessage;
        if (!_pluginManager->executePluginCommand(pluginId, QStringLiteral("showDialog"), {}, nullptr, &errorMessage))
        {
            QMessageBox::warning(this, QStringLiteral("Plugin"), errorMessage);
        }
    });
}

void MainWindow::_refreshPluginsDock()
{
    ui->pluginListWidget->clear();

    const QList<PluginManager::PluginInfo> plugins = _pluginManager->plugins();
    for (const PluginManager::PluginInfo& plugin : plugins)
    {
        QString label = plugin.name.isEmpty() ? plugin.id : plugin.name;
        if (plugin.loaded)
        {
            label += " [loaded]";
        }
        else if (!plugin.valid)
        {
            label += " [invalid]";
        }

        QListWidgetItem* item = new QListWidgetItem(label, ui->pluginListWidget);
        item->setData(Qt::UserRole, plugin.id);

        if (!plugin.valid)
        {
            item->setForeground(Qt::red);
            item->setToolTip(plugin.error);
        }
        else
        {
            item->setToolTip(QStringLiteral("Double-click to open plugin dialog."));
        }
    }
}
