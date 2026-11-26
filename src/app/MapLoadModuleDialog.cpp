#include "MapLoadModuleDialog.h"
#include "App.h"
#include "ui_MapLoadModuleDialog.h"


#include <DataManager/MapLoadModule.h>
#include "osgviewerwidget.h"

#include <QFileDialog>
#include <QFileInfo>
#include <QAbstractItemView>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QListWidget>
#include <QListView>
#include <QStackedWidget>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QUrl>
#include <QUrlQuery>
#include <QtConcurrent/QtConcurrentRun>

#include "gdal.h"
#include "gdal_priv.h"
#include "ogrsf_frmts.h"

#include <osgEarth/WMS>

namespace
{
    void addWMSLayerItems(QTreeWidget* tree,
                          QTreeWidgetItem* parent,
                          const osgEarth::WMS::Layer::LayerList& layers)
    {
        for (const auto& layer : layers) {
            if (!layer.valid())
                continue;

            QTreeWidgetItem* item = parent
                ? new QTreeWidgetItem(parent)
                : new QTreeWidgetItem(tree);
            item->setText(0, QString::fromStdString(layer->getName()));
            item->setText(1, QString::fromStdString(layer->getTitle()));
            item->setText(2, QString::fromStdString(layer->getAbstract()));

            const auto& children = layer->getLayers();
            if (children.empty()) {
                item->setData(0, Qt::UserRole, QString::fromStdString(layer->getName()));
            }
            else {
                item->setFlags(item->flags() & ~Qt::ItemIsSelectable);
                addWMSLayerItems(tree, item, children);
            }
        }
    }

    QUrl makeWMSCapabilitiesUrl(const QString& serviceUrl)
    {
        QUrl url(serviceUrl);
        QUrlQuery query(url);
        if (!query.hasQueryItem(QStringLiteral("SERVICE")))
            query.addQueryItem(QStringLiteral("SERVICE"), QStringLiteral("WMS"));
        if (!query.hasQueryItem(QStringLiteral("VERSION")))
            query.addQueryItem(QStringLiteral("VERSION"), QStringLiteral("1.1.1"));
        if (!query.hasQueryItem(QStringLiteral("REQUEST")))
            query.addQueryItem(QStringLiteral("REQUEST"), QStringLiteral("GetCapabilities"));
        url.setQuery(query);
        return url;
    }
}

MapLoadModuleDialog::MapLoadModuleDialog( QWidget* parent )
    :QDialog(parent)
    ,ui(new Ui::MapLoadModuleDialog)
{
    ui->setupUi(this);
    setupHorizontalTabs();
    setWindowTitle("Open Layers");
    setWindowIcon(QIcon(":/new/prefix1/icons/Layers/layers.png"));
    ui->tree_wmsLayers->setHeaderLabels({ tr("Name"), tr("Title"), tr("Abstract") });
    ui->tree_wmsLayers->setSelectionMode(QAbstractItemView::SingleSelection);
    ui->tree_wmsLayers->header()->setSectionResizeMode(2, QHeaderView::Stretch);

  _initConnections();

  adjustSize();
}

void MapLoadModuleDialog::setupHorizontalTabs()
{
    QTabWidget* tabWidget = ui->tabWidget;
    const int initialIndex = tabWidget->currentIndex();
    QWidget* tabContainer = new QWidget(tabWidget->parentWidget());
    auto* layout = new QHBoxLayout(tabContainer);
    layout->setContentsMargins(0, 0, 0, 0);

    _tabSelector = new QListWidget(tabContainer);
    _tabSelector->setSelectionMode(QAbstractItemView::SingleSelection);
    _tabSelector->setMovement(QListView::Static);
    _tabSelector->setResizeMode(QListView::Adjust);
    _tabSelector->setIconSize(QSize(24, 24));
    _tabSelector->setFixedWidth(150);

    _tabContent = new QStackedWidget(tabContainer);
    layout->addWidget(_tabSelector);
    layout->addWidget(_tabContent, 1);

    while (tabWidget->count() > 0)
    {
        QWidget* page = tabWidget->widget(0);
        const QString title = tabWidget->tabText(0);
        const QIcon icon = tabWidget->tabIcon(0);
        tabWidget->removeTab(0);
        _tabSelector->addItem(new QListWidgetItem(icon, title));
        _tabContent->addWidget(page);
    }

    if (QLayout* parentLayout = tabWidget->parentWidget()->layout())
        parentLayout->replaceWidget(tabWidget, tabContainer);

    delete tabWidget;
    ui->tabWidget = nullptr;

    connect(_tabSelector, &QListWidget::currentRowChanged,
        _tabContent, &QStackedWidget::setCurrentIndex);
    _tabSelector->setCurrentRow(initialIndex);
}

void MapLoadModuleDialog::_initConnections()
{

    ui->le_rasterLabelName->setEnabled(!ui->cb_raster->isChecked());
    ui->le_rasterLabelName->setReadOnly(ui->cb_raster->isChecked());

    ui->le_vectorLayerName->setEnabled(!ui->cb_vectorLayerName->isChecked());
    ui->le_vectorLayerName->setReadOnly(ui->cb_vectorLayerName->isChecked());

    ui->le_eleLayerName->setEnabled(!ui->cb_eleLayerName->isChecked());
    ui->le_eleLayerName->setReadOnly(ui->cb_eleLayerName->isChecked());

    ui->le_sceneLayerName->setEnabled(!ui->cb_scenelayerName->isChecked());
    ui->le_sceneLayerName->setReadOnly(ui->cb_scenelayerName->isChecked());


    QObject::connect(ui->fromServer_radioButton,&QRadioButton::toggled,this,[=](){
        if(ui->fromServer_radioButton->isChecked()){
            ui->xyzUrlLabel->setText("Enter Server Url :");
            ui->pb_xyzUrl->hide();
            ui->le_xyzUrl->hide();
            ui->cb_ServerUrl->show();
            ui->cb_xyzLayerName->setChecked(false);
            ui->cb_xyzLayerName->setCheckable(false);
        }
    });
    QObject::connect(ui->fromLocal_radioButton,&QRadioButton::toggled,this,[=](){
        if(ui->fromLocal_radioButton->isChecked()){
            ui->xyzUrlLabel->setText("Select Directory : ");
            ui->pb_xyzUrl->show();
            ui->le_xyzUrl->show();
            ui->cb_ServerUrl->hide();

        }
    });
    ui->fromServer_radioButton->setChecked(true);
    ui->le_xyzLayername->setEnabled(!ui->cb_xyzLayerName->isChecked());
    ui->le_xyzLayername->setReadOnly(ui->cb_xyzLayerName->isChecked());

    connect(ui->pb_cancel, &QPushButton::clicked, this, &QDialog::reject);
    connect(ui->le_wmsUrl, &QLineEdit::textChanged, this, [this] {
        ui->tree_wmsLayers->clear();
        ui->le_wmsLayerName->clear();
        ui->wmsWarningLabel->clear();
    });
    connect(ui->tree_wmsLayers, &QTreeWidget::currentItemChanged, this,
        [this](QTreeWidgetItem* current) {
            if (!current)
                return;
            const QString serviceLayerName = current->data(0, Qt::UserRole).toString();
            if (serviceLayerName.isEmpty())
                return;

            const QString title = current->text(1).trimmed();
            ui->le_wmsLayerName->setText(title.isEmpty() ? serviceLayerName : title);
        });
    connect(ui->pb_wmsFetchLayers, &QPushButton::clicked, this, [this] {
        const QString serviceUrl = ui->le_wmsUrl->text().trimmed();
        const QUrl parsedUrl(serviceUrl);
        if (!parsedUrl.isValid() || parsedUrl.isEmpty()) {
            ui->wmsWarningLabel->setText(tr("Enter a valid WMS service URL."));
            return;
        }

        ui->tree_wmsLayers->clear();
        ui->le_wmsLayerName->clear();
        ui->wmsWarningLabel->clear();

        const QUrl capabilitiesUrl = makeWMSCapabilitiesUrl(serviceUrl);
        osg::ref_ptr<osgEarth::WMS::Capabilities> capabilities =
            osgEarth::WMS::CapabilitiesReader::read(
                capabilitiesUrl.toString(QUrl::FullyEncoded).toStdString(), nullptr);
        if (!capabilities.valid()) {
            ui->wmsWarningLabel->setText(tr("Could not read WMS capabilities from this service."));
            return;
        }

        addWMSLayerItems(ui->tree_wmsLayers, nullptr, capabilities->getLayers());
        ui->tree_wmsLayers->expandAll();
        if (ui->tree_wmsLayers->topLevelItemCount() == 0)
            ui->wmsWarningLabel->setText(tr("The WMS service did not advertise any layers."));
    });

    QObject::connect(ui->pb_rasterUrl,&QPushButton::clicked,this,[=](){
        QString filePath = OpenFiles(FileType::RASTER);
            if (filePath.isEmpty())
                return;
            QFileInfo finfo(filePath);
            QString MapName = finfo.baseName().toStdString().c_str();
            QString fileExt = finfo.suffix().toStdString().c_str();  // Get the file extension

            ui->le_rasterUrl->setText(filePath);
            ui->le_rasterLabelName->setText(QString(MapName+"."+fileExt));
    });
    QObject::connect(ui->pb_vectorUrl,&QPushButton::clicked,this,[=](){
        QString filePath = OpenFiles(FileType::VECTOR);
            if (filePath.isEmpty())
                return;
            QFileInfo finfo(filePath);
            QString MapName = finfo.baseName().toStdString().c_str();
            QString fileExt = finfo.suffix().toStdString().c_str();  // Get the file extension

            ui->le_vectorUrl->setText(filePath);
            ui->le_vectorLayerName->setText(QString(MapName+"."+fileExt));
    });

    QObject::connect(ui->pb_eleUrl,&QPushButton::clicked,this,[=](){
        QString filePath = OpenFiles(FileType::ELEVATION);
            if (filePath.isEmpty())
                return;
            QFileInfo finfo(filePath);
            QString MapName = finfo.baseName().toStdString().c_str();
            QString fileExt = finfo.suffix().toStdString().c_str();  // Get the file extension

            ui->le_eleUrl->setText(filePath);
            ui->le_eleLayerName->setText(QString(MapName+fileExt));
    });

    QObject::connect(ui->pb_sceneUrl,&QPushButton::clicked,this,[=](){
        QString filePath = OpenFiles(FileType::VECTOR);
            if (filePath.isEmpty())
                return;
            QFileInfo finfo(filePath);
            QString MapName = finfo.baseName().toStdString().c_str();
            QString fileExt = finfo.suffix().toStdString().c_str();  // Get the file extension

            ui->le_sceanUrl->setText(filePath);
            ui->le_sceneLayerName->setText(QString(MapName+fileExt));
            getAttributes(filePath);
    });

    QObject::connect(ui->pb_sceneXml,&QPushButton::clicked,this,[=](){
        QString filePath = OpenFiles(FileType::TEXTURE);
            if (filePath.isEmpty())
                return;
            ui->le_sceneXml->setText(filePath);
    });

    QObject::connect(ui->pb_xyzUrl, &QPushButton::clicked, this, [=]() {
        // Let user pick a directory
        // QString FilePaths = QFileDialog::get(this,"Open Files","",Filter);

        QString folderPath = QFileDialog::getExistingDirectory(this, "Select XYZ Tile Folder");

        if (folderPath.isEmpty())
            return;
        // Example: convert local folder to file URL
        QUrl url = QUrl::fromLocalFile(folderPath );
        // Extract folder name
        QFileInfo finfo(folderPath);
        QString mapName = finfo.fileName();  // Just folder name
        // Set values to the UI
        ui->le_xyzUrl->setText(url.toString());
        ui->le_xyzLayername->setText(mapName);
    });



    QObject::connect(ui->cb_raster, &QCheckBox::toggled,this,[=](bool state){
            ui->le_rasterLabelName->setEnabled(!ui->cb_raster->isChecked());
            ui->le_rasterLabelName->setReadOnly(ui->cb_raster->isChecked());
    } );

    QObject::connect(ui->cb_vectorLayerName, &QCheckBox::toggled,this,[=](bool state){
            ui->le_vectorLayerName->setEnabled(!ui->cb_vectorLayerName->isChecked());
            ui->le_vectorLayerName->setReadOnly(ui->cb_vectorLayerName->isChecked());
    } );

    QObject::connect(ui->cb_eleLayerName, &QCheckBox::toggled,this,[=](bool state){
            ui->le_eleLayerName->setEnabled(!ui->cb_eleLayerName->isChecked());
            ui->le_eleLayerName->setReadOnly(ui->cb_eleLayerName->isChecked());
    } );

    QObject::connect(ui->cb_scenelayerName, &QCheckBox::toggled,this,[=](bool state){
            ui->le_eleLayerName->setEnabled(!ui->cb_scenelayerName->isChecked());
            ui->le_eleLayerName->setReadOnly(ui->cb_scenelayerName->isChecked());
    } );

  }


void MapLoadModuleDialog::getAttributes(QString FilePath){
 // OGR Shapefile support
 // Register all GDAL drivers
    GDALAllRegister();

    // Open the shapefile
    GDALDataset* dataset = static_cast<GDALDataset*>(
        GDALOpenEx(FilePath.toStdString().c_str(), GDAL_OF_VECTOR, nullptr, nullptr, nullptr)
    );

    if (!dataset) {
        qWarning("Failed to open shapefile.");
        return;
    }

    // Get the first layer
    OGRLayer* layer = dataset->GetLayer(0);
    if (!layer) {
        qWarning("Failed to get layer.");
        GDALClose(dataset);
        return;
    }

    // Get the layer definition (schema)
    OGRFeatureDefn* defn = layer->GetLayerDefn();
    if (!defn) {
        qWarning("Failed to get layer definition.");
        GDALClose(dataset);
        return;
    }

    // Clear the combo box
    ui->cb_heightAttribute->clear();

    // Loop through all fields and add their names to the combo box
    for (int i = 0; i < defn->GetFieldCount(); ++i) {
        OGRFieldDefn* fieldDefn = defn->GetFieldDefn(i);
        if (fieldDefn) {
            QString fieldName = QString::fromUtf8(fieldDefn->GetNameRef());
            ui->cb_heightAttribute->addItem(fieldName);
        }
    }

    // Clean up
    GDALClose(dataset);

}


MapLoadModuleDialog::~MapLoadModuleDialog()
{
    delete ui;

}

QString MapLoadModuleDialog::OpenFiles(FileType Type )
{
    QString Filter ="";
    switch(Type)
    {
    case FileType::RASTER:
        Filter = "Raster Files (*.tif *.tiff *.img *.asc *.bil);;All Files (*.*)";
        break;
    case FileType::VECTOR:
        Filter = "vector Files (*.shp );;All Files (*.*)";
        break;
    case FileType::ELEVATION:
        Filter = "Elevation Files (*.tif *.tiff *.dt2 *dt1 *dt0);;All Files (*.*)";
        break;
    case FileType::TEXTURE:
        Filter = "Texture Files(*.xml);; All Files (*.*)";
        break;
    case FileType::MODEL3D:
        Filter = "3D Model File (*.obj)";
    case FileType::ALLFILES:
        Filter = "All Files (*.*)";
        break;
    }

    QString FilePaths = QFileDialog::getOpenFileName(this,"Open Files","",Filter);

    return FilePaths;

}

void MapLoadModuleDialog::on_pb_openMap_clicked()
{
    if(_tabSelector->currentRow() == 0)
    {
        if (ui->le_rasterUrl->text().isEmpty() )
            return;

            MapLoadModule::LoadRasters(App::getThreedWindow()->getMapNode(), ui->le_rasterUrl->text(), ui->le_rasterLabelName->text());

    }

    else if(_tabSelector->currentRow() == 1)
    {
        if (ui->le_vectorUrl->text().isEmpty())
            return;

        // Run LoadVectors in a separate thread
           QtConcurrent::run([=]() {

               QMetaObject::invokeMethod(this, [=]() {

                   MapLoadModule::LoadVectors(App::getThreedWindow()->getMapNode(), ui->le_vectorUrl->text(), ui->le_vectorLayerName->text());

                  }, Qt::QueuedConnection);
           });
    }

    else if(_tabSelector->currentRow() == 2){

        if (ui->le_eleUrl->text().isEmpty() || !App::getThreedWindow()->getMapNode())
            return;

        MapLoadModule::LoadElevation(App::getThreedWindow()->getMapNode(),ui->le_eleUrl->text() , ui->le_eleLayerName->text());
    }

    else if(_tabSelector->currentRow() == 3){
        if(ui->le_sceanUrl->text().isEmpty())
            return;

        if(ui->le_sceneLayerName->text().isEmpty()) return;

        QString LayerName = ui->le_sceneLayerName->text();
        QString heightText = ui->cb_heightAttribute->currentText().trimmed();
        if (heightText.isEmpty())
            heightText = "10";

        MapLoadModule::Load3Dbuildings(App::getThreedWindow()->getMapNode(),
                                       ui->le_sceanUrl->text().trimmed(),
                                       heightText,
                                       LayerName,
                                       ui->le_sceneXml->text().trimmed());
    }

    else if(_tabSelector->currentRow() == 4)
        {
            if(ui->le_xyzLayername->text().isEmpty() || ui->le_xyzUrl->text().isEmpty()) {
                ui->warning_label->setText("⚠️ Warning: Please enter url or Layer Name");
                return;
            }
            QString LayerUrl;
            QString LayerName;

            if(ui->fromServer_radioButton->isChecked()) {

                LayerUrl = ui->cb_ServerUrl->currentText();

                if (!LayerUrl.contains("{z}") || !LayerUrl.contains("{x}") || !LayerUrl.contains("{y}")) {
                    ui->warning_label->setText("⚠️ Warning: URL must contain {z}/{x}/{y} placeholders");
                    return;
                }

            }
            else if(ui->fromLocal_radioButton->isChecked()) {

                QString inputUrl = ui->le_xyzUrl->text().trimmed();

                QFileInfo dirInfo(inputUrl);

                if (!dirInfo.isDir()) {
                    ui->warning_label->setText("⚠️ Warning: Invalid directory path");
                    return;
                }
                LayerName = dirInfo.completeBaseName().isEmpty() ? dirInfo.fileName() : dirInfo.completeBaseName();
                ui->le_xyzLayername->setText(LayerName);

                #ifdef _WIN32
                    // Remove "file:///" prefix (which is 8 characters long) on Windows
                    if (inputUrl.startsWith("file:///"))
                        inputUrl.remove(0, 8);
                    inputUrl += "/{z}/{x}/{y}.png";

                #else
                    // On Linux/macOS, "file://" is usually the prefix (7 characters)
                    if (inputUrl.startsWith("file://"))
                        inputUrl.remove(0, 7);
                    inputUrl += "{z}/{x}/{y}.png";
                #endif
                LayerUrl = inputUrl;

            }

            ui->warning_label->clear();
            MapLoadModule::LoadXYZTiles(App::getThreedWindow()->getMapNode(),LayerUrl, ui->le_xyzLayername->text());
        }
#ifdef OSGEARTH_HAVE_CESIUM_NODEKIT
    else if(_tabSelector->currentRow() == 5)
    {
        OSGViewerWidget* viewer = App::getThreedWindow();
        if (!viewer || !viewer->getMapNode())
        {
            ui->cesiumWarningLabel->setText(tr("The 3D map is not available."));
            return;
        }

        const QString accessToken = ui->cesiumAccessTokenLineEdit->text().trimmed();
        bool assetIdValid = false;
        const uint assetId = ui->cesiumAssetIdLineEdit->text().trimmed().toUInt(&assetIdValid);
        const QString layerName = ui->cesiumLayerNameLineEdit->text().trimmed();
        const int layerTypeIndex = ui->cesiumLayerTypeComboBox->currentIndex();

        if (accessToken.isEmpty())
        {
            ui->cesiumWarningLabel->setText(tr("Enter a Cesium Ion access token."));
            return;
        }
        if (!assetIdValid || assetId == 0)
        {
            ui->cesiumWarningLabel->setText(tr("Enter a valid Cesium Ion asset ID."));
            return;
        }
        if (layerName.isEmpty())
        {
            ui->cesiumWarningLabel->setText(tr("Enter a layer name."));
            return;
        }
        if (layerTypeIndex < 0 || layerTypeIndex > 2)
        {
            ui->cesiumWarningLabel->setText(tr("Select a Cesium layer type."));
            return;
        }

        ui->cesiumWarningLabel->clear();
        MapLoadModule::LoadCesiumLayer(
            viewer->getMapNode(),
            accessToken,
            static_cast<MapLoadModule::CesiumLayerType>(layerTypeIndex),
            assetId,
            layerName);
    }
#endif
    else if(_tabSelector->currentRow() == 6)
    {
        OSGViewerWidget* viewer = App::getThreedWindow();
        if (!viewer || !viewer->getMapNode()) {
            ui->tmsWarningLabel->setText(tr("The map is not available."));
            return;
        }

        const QString url = ui->le_tmsUrl->text().trimmed();
        const QString layerName = ui->le_tmsLayerName->text().trimmed();
        if (url.isEmpty() || layerName.isEmpty()) {
            ui->tmsWarningLabel->setText(tr("Enter a TMS URL and layer name."));
            return;
        }

        ui->tmsWarningLabel->clear();
        MapLoadModule::LoadTMSLayer(
            viewer->getMapNode(), url, layerName, ui->cb_tmsElevation->isChecked());
    }
    else if(_tabSelector->currentRow() == 7)
    {
        OSGViewerWidget* viewer = App::getThreedWindow();
        if (!viewer || !viewer->getMapNode()) {
            ui->wmsWarningLabel->setText(tr("The map is not available."));
            return;
        }

        QTreeWidgetItem* selectedItem = ui->tree_wmsLayers->currentItem();
        const QString serviceLayerName = selectedItem
            ? selectedItem->data(0, Qt::UserRole).toString()
            : QString();
        const QString layerName = ui->le_wmsLayerName->text().trimmed();
        if (ui->le_wmsUrl->text().trimmed().isEmpty() ||
            serviceLayerName.isEmpty() || layerName.isEmpty()) {
            ui->wmsWarningLabel->setText(
                tr("Fetch and select a WMS service layer, then enter a map layer name."));
            return;
        }

        ui->wmsWarningLabel->clear();
        MapLoadModule::LoadWMSLayer(
            viewer->getMapNode(),
            ui->le_wmsUrl->text().trimmed(),
            serviceLayerName,
            layerName);
    }
}
