#include "GLXNewVectorLayerDialog.h"

#include <QComboBox>
#include <QFileInfo>
#include <QIntValidator>
#include <QTreeWidget>
#include <QMessageBox>
#include <QStringList>
#include <QSet>
#include <QTreeWidgetItem>
#include <QTreeWidgetItemIterator>
#include <QValidator>
#include <QFileDialog>
#include <QDir>
#include <QStandardPaths>
#include <QRegularExpression>
#include <QDebug>

#include <gdal.h>
#include <ogr_api.h>
#include <ogr_srs_api.h>
#include <cpl_conv.h>
#include <cpl_vsi.h>
#include <gdal_priv.h>
#include <ogr_feature.h>
#include <ogr_spatialref.h>
#include <ogrsf_frmts.h>

#include "App.h"
#include "DataManager/MapLoadModule.h"

namespace {

bool resolveCrs(const QString& text, OGRSpatialReference& srs)
{
    const QString crsText = text.trimmed();
    qDebug().noquote() << "[CRS] Resolving:" << crsText;

    if (crsText.isEmpty())
        return false;

    if (crsText.compare(QStringLiteral("EPSG:4326"), Qt::CaseInsensitive) == 0) {
        const bool ok = srs.SetWellKnownGeogCS("WGS84") == OGRERR_NONE;
        qDebug().noquote() << "[CRS] WGS84 construction result:" << ok;
        return ok;
    }

    if (crsText.startsWith(QStringLiteral("EPSG:"), Qt::CaseInsensitive)) {
        bool isInteger = false;
        const int epsgCode = crsText.section(':', 1, 1).trimmed().toInt(&isInteger);
        if (!isInteger || epsgCode <= 0)
            return false;

        // These UTM entries can be constructed directly and do not depend on EPSG lookup tables.
        if (epsgCode >= 32601 && epsgCode <= 32660) {
            const int zone = epsgCode - 32600;
            const bool ok = srs.SetUTM(zone, TRUE) == OGRERR_NONE
                         && srs.SetWellKnownGeogCS("WGS84") == OGRERR_NONE;
            qDebug().noquote() << "[CRS] Direct UTM" << zone << "result:" << ok;
            return ok;
        }

        const OGRErr result = srs.importFromEPSG(epsgCode);
        qDebug().noquote() << "[CRS] importFromEPSG(" << epsgCode << ") result:" << result
                           << "lastError:" << CPLGetLastErrorMsg();
        return result == OGRERR_NONE;
    }

    const OGRErr result = srs.SetFromUserInput(crsText.toUtf8().constData());
    qDebug().noquote() << "[CRS] SetFromUserInput result:" << result
                       << "lastError:" << CPLGetLastErrorMsg();
    return result == OGRERR_NONE;
}

}

GLXNewVectorLayerDialog::GLXNewVectorLayerDialog(QWidget *parent)
    : QDialog(parent)
{
    setupUi(this);
    GDALAllRegister();

    connect(mAddAttributeButton, &QToolButton::clicked,
            this, &GLXNewVectorLayerDialog::mAddAttributeButton_clicked);
    connect(mRemoveAttributeButton, &QToolButton::clicked,
            this, &GLXNewVectorLayerDialog::mRemoveAttributeButton_clicked);
    connect(mFileFormatComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &GLXNewVectorLayerDialog::mFileFormatComboBox_currentIndexChanged);
    connect(mTypeBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &GLXNewVectorLayerDialog::mTypeBox_currentIndexChanged);
    connect(buttonBox, &QDialogButtonBox::helpRequested,
            this, &GLXNewVectorLayerDialog::showHelp);
    connect(mButtonUp, &QPushButton::clicked, this, &GLXNewVectorLayerDialog::moveFieldsUp);
    connect(mButtonDown, &QPushButton::clicked, this, &GLXNewVectorLayerDialog::moveFieldsDown);
    connect(mNameEdit, &QLineEdit::textChanged, this, &GLXNewVectorLayerDialog::nameChanged);
    connect(mAttributeView, &QTreeWidget::itemSelectionChanged,
            this, &GLXNewVectorLayerDialog::selectionChanged);
    connect(mBrowseButton, &QPushButton::clicked,
            this, &GLXNewVectorLayerDialog::browseForFilename);

    mTypeBox->addItem(tr("Text (String)"), QStringLiteral("String"));
    mTypeBox->addItem(tr("Integer (32 bit)"), QStringLiteral("Integer"));
    mTypeBox->addItem(tr("Decimal (Double)"), QStringLiteral("Real"));
    mTypeBox->addItem(tr("Date"), QStringLiteral("Date"));
    mTypeBox->addItem(tr("Boolean"), QStringLiteral("Boolean"));

    mWidth->setValidator(new QIntValidator(1, 255, this));
    mPrecision->setValidator(new QIntValidator(0, 15, this));

    mGeometryTypeBox->addItem(tr("Point"), QStringLiteral("Point"));
    mGeometryTypeBox->addItem(tr("LineString"), QStringLiteral("LineString"));
    mGeometryTypeBox->addItem(tr("Polygon"), QStringLiteral("Polygon"));
    mGeometryTypeBox->setCurrentIndex(mGeometryTypeBox->findData(QStringLiteral("Polygon")));

    mFileFormatComboBox->addItem(tr("ESRI Shapefile"), QStringLiteral("ESRI Shapefile"));
    mFileFormatComboBox->setVisible(true);
    mFileFormatLabel->setVisible(true);

    mFileEncoding->addItems(QStringList()
        << QStringLiteral("UTF-8") << QStringLiteral("System")
        << QStringLiteral("ISO-8859-1") << QStringLiteral("Windows-1252"));
    mFileEncoding->setCurrentText(QStringLiteral("UTF-8"));

    mAttributeView->addTopLevelItem(new QTreeWidgetItem(
        QStringList() << QStringLiteral("id") << QStringLiteral("Integer")
                      << QStringLiteral("10") << QString()));

    const QString defaultDir = QDir(QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)).filePath(QStringLiteral("GeoLayer_X"));
    QDir().mkpath(defaultDir);
    if (mFileName->text().trimmed().isEmpty())
        setFilename(QDir(defaultDir).filePath(QStringLiteral("new_layer.shp")));
    mCrsComboBox->clear();
    mCrsComboBox->addItem(QStringLiteral("EPSG:4326 — WGS 84"), QStringLiteral("EPSG:4326"));
    mCrsComboBox->addItem(QStringLiteral("EPSG:3857 — WGS 84 / Pseudo-Mercator"), QStringLiteral("EPSG:3857"));
    mCrsComboBox->addItem(QStringLiteral("EPSG:32643 — WGS 84 / UTM zone 43N"), QStringLiteral("EPSG:32643"));
    mCrsComboBox->addItem(QStringLiteral("EPSG:32644 — WGS 84 / UTM zone 44N"), QStringLiteral("EPSG:32644"));
    mCrsComboBox->setCurrentIndex(0);
    qDebug().noquote() << "[CRS] Dialog default index:" << mCrsComboBox->currentIndex()
                       << "text:" << mCrsComboBox->currentText()
                       << "data:" << mCrsComboBox->currentData(Qt::UserRole).toString();

    connect(mGeometryTypeBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int) {
        updateExtension();
        checkOk();
    });
    connect(mFileName, &QLineEdit::textChanged, this, [this](const QString &) { checkOk(); });
    connect(mCrsComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this](int) { checkOk(); });

    mOkButton = buttonBox->button(QDialogButtonBox::Ok);
    mOkButton->setEnabled(false);
    mAddAttributeButton->setEnabled(false);
    mRemoveAttributeButton->setEnabled(false);
    mButtonUp->setEnabled(false);
    mButtonDown->setEnabled(false);

    mTypeBox_currentIndexChanged(mTypeBox->currentIndex());
    checkOk();
}

void GLXNewVectorLayerDialog::mFileFormatComboBox_currentIndexChanged(int)
{
    mNameEdit->setMaxLength(mFileFormatComboBox->currentText() == tr("ESRI Shapefile") ? 10 : 32767);
    updateExtension();
}

void GLXNewVectorLayerDialog::mTypeBox_currentIndexChanged(int index)
{
    if (index == 0) {
        mWidth->setText(QStringLiteral("80"));
        mWidth->setEnabled(true);
        mPrecision->setEnabled(false);
        mPrecision->clear();
        mWidth->setValidator(new QIntValidator(1, 255, this));
    } else if (index == 1) {
        mWidth->setText(QStringLiteral("10"));
        mWidth->setEnabled(true);
        mPrecision->setEnabled(false);
        mPrecision->clear();
        mWidth->setValidator(new QIntValidator(1, 10, this));
    } else if (index == 2) {
        mWidth->setText(QStringLiteral("20"));
        mPrecision->setText(QStringLiteral("6"));
        mWidth->setEnabled(true);
        mPrecision->setEnabled(true);
        mWidth->setValidator(new QIntValidator(1, 20, this));
    } else {
        mWidth->clear();
        mPrecision->clear();
        mWidth->setEnabled(false);
        mPrecision->setEnabled(false);
    }
}

void GLXNewVectorLayerDialog::mAddAttributeButton_clicked()
{
    const QString type = mTypeBox->currentData(Qt::UserRole).toString();
    const QString precision = mPrecision->isEnabled() ? mPrecision->text() : QString();
    mAttributeView->addTopLevelItem(new QTreeWidgetItem(
        QStringList() << mNameEdit->text() << type << mWidth->text() << precision));
    mNameEdit->clear();
    checkOk();
}

void GLXNewVectorLayerDialog::mRemoveAttributeButton_clicked()
{
    delete mAttributeView->currentItem();
    checkOk();
}

void GLXNewVectorLayerDialog::attributes(QList<QPair<QString, QString>> &result) const
{
    result.clear();
    QTreeWidgetItemIterator it(mAttributeView);
    while (*it) {
        QTreeWidgetItem *item = *it;
        result.push_back(qMakePair(item->text(0),
            item->text(1) + QStringLiteral(";") + item->text(2) + QStringLiteral(";") + item->text(3)));
        ++it;
    }
}

QString GLXNewVectorLayerDialog::selectedFileFormat() const
{
    return mFileFormatComboBox->currentData(Qt::UserRole).toString();
}

QString GLXNewVectorLayerDialog::selectedFileEncoding() const
{
    return mFileEncoding->currentText();
}

QString GLXNewVectorLayerDialog::filename() const
{
    return mFileName->text();
}

void GLXNewVectorLayerDialog::setFilename(const QString &filename)
{
    mFileName->setText(filename);
}

QString GLXNewVectorLayerDialog::crs() const
{
    const int index = mCrsComboBox->currentIndex();
    if (index >= 0) {
        const QString value = mCrsComboBox->itemData(index, Qt::UserRole).toString().trimmed();
        if (!value.isEmpty())
            return value;

        // Fallback: read the EPSG code from the visible combo-box text.
        const QRegularExpression epsg(QStringLiteral("(?i)EPSG\\s*:\\s*(\\d+)"));
        const QRegularExpressionMatch match = epsg.match(mCrsComboBox->currentText());
        if (match.hasMatch())
            return QStringLiteral("EPSG:%1").arg(match.captured(1));
    }

    qWarning() << "[CRS] No combo selection; using EPSG:4326";
    return QStringLiteral("EPSG:4326");
}

void GLXNewVectorLayerDialog::setCrs(const QString &crs)
{
    const int index = mCrsComboBox->findData(crs.trimmed(), Qt::UserRole);
    if (index >= 0) {
        mCrsComboBox->setCurrentIndex(index);
    } else {
        mCrsComboBox->addItem(crs.trimmed(), crs.trimmed());
        mCrsComboBox->setCurrentIndex(mCrsComboBox->count() - 1);
    }
}

QString GLXNewVectorLayerDialog::selectedType() const
{
    return mGeometryTypeBox->currentData(Qt::UserRole).toString();
}

void GLXNewVectorLayerDialog::nameChanged(const QString &name)
{
    mAddAttributeButton->setEnabled(!name.isEmpty()
        && mAttributeView->findItems(name, Qt::MatchExactly).isEmpty());
}

void GLXNewVectorLayerDialog::selectionChanged()
{
    const bool hasSelection = !mAttributeView->selectedItems().isEmpty();
    mRemoveAttributeButton->setEnabled(hasSelection);
    mButtonUp->setEnabled(hasSelection);
    mButtonDown->setEnabled(hasSelection);
}

void GLXNewVectorLayerDialog::moveFieldsUp()
{
    const int row = mAttributeView->currentIndex().row();
    if (row <= 0)
        return;
    mAttributeView->insertTopLevelItem(row - 1, mAttributeView->takeTopLevelItem(row));
    mAttributeView->setCurrentIndex(mAttributeView->model()->index(row - 1, 0));
}

void GLXNewVectorLayerDialog::moveFieldsDown()
{
    const int row = mAttributeView->currentIndex().row();
    if (row < 0 || row >= mAttributeView->topLevelItemCount() - 1)
        return;
    mAttributeView->insertTopLevelItem(row + 1, mAttributeView->takeTopLevelItem(row));
    mAttributeView->setCurrentIndex(mAttributeView->model()->index(row + 1, 0));
}

void GLXNewVectorLayerDialog::checkOk()
{
    const bool ok = !mFileName->text().trimmed().isEmpty()
                    && mAttributeView->topLevelItemCount() > 0
                    && mGeometryTypeBox->currentIndex() != -1;
    if (mOkButton)
        mOkButton->setEnabled(ok);
}

void GLXNewVectorLayerDialog::browseForFilename()
{
    const QString selected = QFileDialog::getSaveFileName(this, tr("Create Shapefile"), filename(),
                                                          tr("ESRI Shapefile (*.shp)"));
    if (!selected.isEmpty()) {
        QString path = selected;
        if (!path.endsWith(QStringLiteral(".shp"), Qt::CaseInsensitive))
            path += QStringLiteral(".shp");
        setFilename(path);
    }
}

#include <cpl_string.h>

bool GLXNewVectorLayerDialog::CreateEmptyShapefile(
    const std::string& filepath,
    const std::string& encoding,
    const std::string& geometryType,
    const std::string& crs,
    const std::vector<std::pair<std::string, OGRFieldType>>& fields)
{
    GDALAllRegister();

    qDebug().noquote() << "[CRS] CreateEmptyShapefile input:" << QString::fromStdString(crs);

    const QString qtPath = QString::fromStdString(filepath);
    const QFileInfo shapefileInfo(qtPath);
    if (!shapefileInfo.absolutePath().isEmpty())
        QDir().mkpath(shapefileInfo.absolutePath());

    const QString basePath = QDir(shapefileInfo.absolutePath()).filePath(shapefileInfo.completeBaseName());
    for (const QString& suffix : {QStringLiteral(".shp"), QStringLiteral(".shx"),
                                  QStringLiteral(".dbf"), QStringLiteral(".prj"),
                                  QStringLiteral(".cpg"), QStringLiteral(".qix")}) {
        QFile::remove(basePath + suffix);
    }

    GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
    if (!driver) {
        qWarning() << "[CRS] ESRI Shapefile GDAL driver is unavailable";
        return false;
    }
    qDebug().noquote() << "[CRS] ESRI Shapefile GDAL driver found:" << driver->GetDescription();

    GDALDataset* dataset = driver->Create(filepath.c_str(), 0, 0, 0, GDT_Unknown, nullptr);
    if (!dataset) {
        qWarning() << "[CRS] GDAL could not create the shapefile:" << CPLGetLastErrorMsg();
        return false;
    }

    OGRSpatialReference srs;
    QString crsText = QString::fromStdString(crs).trimmed();
    if (!resolveCrs(crsText, srs)) {
        qWarning() << "[CRS] Selected CRS failed; falling back to EPSG:4326";
        if (!resolveCrs(QStringLiteral("EPSG:4326"), srs)) {
            qWarning() << "[CRS] Even the WGS84 fallback failed";
            GDALClose(dataset);
            return false;
        }
    }
    srs.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

    OGRwkbGeometryType geomType = wkbPolygon;
    if (geometryType == "POINT")
        geomType = wkbPoint;
    else if (geometryType == "LINESTRING")
        geomType = wkbLineString;

    char** options = nullptr;
    if (!encoding.empty()) {
        if (encoding != "System")
            options = CSLSetNameValue(options, "ENCODING", encoding.c_str());
    }

    OGRLayer* layer = dataset->CreateLayer(
        shapefileInfo.completeBaseName().toUtf8().constData(), &srs, geomType, options);
    CSLDestroy(options);

    if (!layer) {
        qWarning() << "[CRS] GDAL CreateLayer failed for the shapefile:" << CPLGetLastErrorMsg();
        GDALClose(dataset);
        return false;
    }

    for (const auto& field : fields) {
        OGRFieldDefn fieldDef(field.first.c_str(), field.second);
        if (layer->CreateField(&fieldDef) != OGRERR_NONE) {
            qWarning() << "[CRS] Could not create field" << field.first.c_str()
                       << ":" << CPLGetLastErrorMsg();
            GDALClose(dataset);
            return false;
        }
    }

    dataset->FlushCache();
    GDALClose(dataset);

    QFile cpg(basePath + QStringLiteral(".cpg"));
    if (cpg.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
        cpg.write(encoding == "UTF-8" ? "UTF-8" : encoding.c_str());
        cpg.close();
    }

    const QString shpPath = basePath + QStringLiteral(".shp");
    const QString prjPath = basePath + QStringLiteral(".prj");

    qDebug().noquote() << "[CRS] GDAL outputs - SHP:" << QFileInfo::exists(shpPath)
                       << "PRJ:" << QFileInfo::exists(prjPath)
                       << "SHP path:" << shpPath;

    if (!QFileInfo::exists(shpPath)) {
        qWarning() << "[CRS] Shapefile was not created at:" << shpPath;
        return false;
    }

    // ESRI Shapefile normally writes .prj from CreateLayer. Write it explicitly only when GDAL did not.
    if (!QFileInfo::exists(prjPath)) {
        char* wkt = nullptr;
        if (srs.exportToWkt(&wkt) == OGRERR_NONE && wkt && wkt[0] != '\0') {
            QFile prj(prjPath);
            if (prj.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
                const qint64 written = prj.write(wkt);
                prj.close();
                qDebug().noquote() << "[CRS] Fallback .prj written, bytes:" << written;
            } else {
                qWarning() << "[CRS] Could not open fallback .prj for writing:" << prjPath;
            }
        } else {
            qWarning() << "[CRS] Could not export CRS WKT:" << CPLGetLastErrorMsg();
        }
        CPLFree(wkt);
    } else {
        qDebug().noquote() << "[CRS] GDAL created .prj automatically:" << prjPath;
    }

    const bool success = QFileInfo::exists(shpPath) && QFileInfo::exists(prjPath);
    qDebug().noquote() << "[CRS] Final shapefile result:" << success;
    return success;
}
void GLXNewVectorLayerDialog::updateExtension()
{
    QString path = filename();
    if (selectedFileFormat() == QStringLiteral("ESRI Shapefile") && !path.isEmpty()) {
        const QString lower = path.toLower();
        if (!lower.endsWith(QStringLiteral(".shp")) && !lower.endsWith(QStringLiteral(".dbf")))
            path += QStringLiteral(".shp");
        setFilename(path);
    }
}

void GLXNewVectorLayerDialog::accept()
{
    if (!mNameEdit->text().trimmed().isEmpty()) {
        const QString pendingName = mNameEdit->text().trimmed();
        if (mAttributeView->findItems(pendingName, Qt::MatchExactly).isEmpty()) {
            if (QMessageBox::question(this, windowTitle(),
                    tr("The field \u201c%1\u201d has not been added to the fields list. Discard it?").arg(pendingName),
                    QMessageBox::Ok | QMessageBox::Cancel) != QMessageBox::Ok)
                return;
        }
    }

    updateExtension();

    QString path = filename().trimmed();
    if (path.isEmpty()) {
        const QString defaultDir = QDir(QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)).filePath(QStringLiteral("GeoLayer_X"));
        QDir().mkpath(defaultDir);
        path = QDir(defaultDir).filePath(QStringLiteral("new_layer.shp"));
        setFilename(path);
    } else if (!path.endsWith(QStringLiteral(".shp"), Qt::CaseInsensitive)) {
        path += QStringLiteral(".shp");
        setFilename(path);
    }

    const QFileInfo shpInfo(path);
    if (shpInfo.suffix().compare(QStringLiteral("shp"), Qt::CaseInsensitive) != 0) {
        QMessageBox::critical(this, tr("New Vector Layer"), tr("The output filename must be a .shp file."));
        return;
    }
    if (!QDir().mkpath(shpInfo.absolutePath())) {
        QMessageBox::critical(this, tr("New Vector Layer"), tr("Could not create the output directory."));
        return;
    }

    const QString geometry = selectedType().toUpper();
    if (geometry != QStringLiteral("POINT") && geometry != QStringLiteral("LINESTRING") && geometry != QStringLiteral("POLYGON")) {
        QMessageBox::critical(this, tr("New Vector Layer"), tr("Select Point, LineString, or Polygon."));
        return;
    }

    QString crsText = crs().trimmed();
    if (crsText.isEmpty()) {
        crsText = QStringLiteral("EPSG:4326");
        setCrs(crsText);
    }

    qDebug().noquote() << "[CRS] Selected combo index:" << mCrsComboBox->currentIndex()
                       << "text:" << mCrsComboBox->currentText()
                       << "passed CRS:" << crsText;

    // The UI supplies predefined EPSG codes, so validate the code syntax instead of
    // depending on GDAL's optional EPSG database lookup here.
    const QRegularExpression epsgCode(QStringLiteral("(?i)^EPSG\\s*:\\s*(\\d+)\\s*$"));
    const QRegularExpressionMatch crsMatch = epsgCode.match(crsText);
    if (!crsMatch.hasMatch()) {
        QMessageBox::critical(this, tr("New Vector Layer"),
            tr("Enter a valid coordinate reference system, such as EPSG:4326."));
        return;
    }

    bool validCode = false;
    const int epsgCodeValue = crsMatch.captured(1).toInt(&validCode);
    if (!validCode || epsgCodeValue <= 0) {
        QMessageBox::critical(this, tr("New Vector Layer"),
            tr("Enter a valid coordinate reference system, such as EPSG:4326."));
        return;
    }

    std::vector<std::pair<std::string, OGRFieldType>> fields;
    QSet<QString> fieldNames;
    for (int i = 0; i < mAttributeView->topLevelItemCount(); ++i) {
        const QTreeWidgetItem* item = mAttributeView->topLevelItem(i);
        const QString name = item->text(0).trimmed();
        if (name.isEmpty()) {
            QMessageBox::critical(this, tr("New Vector Layer"), tr("Field names cannot be empty."));
            return;
        }
        if (name.length() > 10) {
            QMessageBox::critical(this, tr("New Vector Layer"), tr("Shapefile field name '%1' exceeds the 10-character limit.").arg(name));
            return;
        }
        const QString key = name.toLower();
        if (fieldNames.contains(key)) {
            QMessageBox::critical(this, tr("New Vector Layer"), tr("Duplicate field name: %1").arg(name));
            return;
        }
        fieldNames.insert(key);

        OGRFieldType fieldType = OFTString;
        const QString type = item->text(1);
        if (type == QStringLiteral("Integer")) fieldType = OFTInteger;
        else if (type == QStringLiteral("Real")) fieldType = OFTReal;
        else if (type == QStringLiteral("Date")) fieldType = OFTDate;
        else if (type == QStringLiteral("Boolean")) fieldType = OFTInteger;
        fields.emplace_back(name.toStdString(), fieldType);
    }

    if (fields.empty()) {
        QMessageBox::critical(this, tr("New Vector Layer"), tr("Add at least one attribute field."));
        return;
    }

    if (QFileInfo::exists(path)) {
        if (QMessageBox::warning(this, tr("New Vector Layer"),
                tr("The layer already exists. Are you sure you want to overwrite it?"),
                QMessageBox::Yes | QMessageBox::Cancel, QMessageBox::Cancel) != QMessageBox::Yes)
            return;
    }

    const std::string encoding = selectedFileEncoding().toStdString();
    const bool created = CreateEmptyShapefile(
        shpInfo.absoluteFilePath().toStdString(), encoding,
        geometry.toStdString(), crsText.toStdString(), fields);

    if (!created) {
        QMessageBox::critical(this, tr("New Vector Layer"), tr("Could not create the shapefile and its spatial reference metadata."));
        return;
    }

    if (App::getThreedWindow() && App::getThreedWindow()->getMapNode()) {
        MapLoadModule::LoadVectors(App::getThreedWindow()->getMapNode(), path,
                                   QFileInfo(path).completeBaseName());
    } else {
        QMessageBox::warning(this, tr("New Vector Layer"),
                             tr("The shapefile was created, but no map is currently available to load it."));
    }

    QDialog::accept();
}

void GLXNewVectorLayerDialog::showHelp()
{
    QMessageBox::information(this, tr("New Vector Layer"),
        tr("Specify a filename, geometry type, coordinate reference system, fields, and encoding."));
}
