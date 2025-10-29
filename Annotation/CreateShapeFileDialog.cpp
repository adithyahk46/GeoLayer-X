#include "CreateShapeFileDialog.h"
#include "ui_CreateShapeFileDialog.h"

CreateShapeFileDialog::CreateShapeFileDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::CreateShapeFileDialog)
{
    ui->setupUi(this);

    // ✅ Output Path Browse Button
    connect(ui->pb_browsOutPutPath, &QPushButton::clicked, this, &CreateShapeFileDialog::onBrowseOutputPath);

    // ✅ Populate File Encoding ComboBox (default UTF-8)
    QStringList encodings;
    for (const QByteArray &codec : QTextCodec::availableCodecs()) {
        encodings << QString::fromUtf8(codec);
    }
    for (const QString &encoding : encodings) {
        ui->cb_FileEncoding->addItem(encoding);
    }

    int utf8Index = ui->cb_FileEncoding->findText("UTF-8", Qt::MatchFixedString);
    if (utf8Index >= 0)
        ui->cb_FileEncoding->setCurrentIndex(utf8Index);

    // ✅ Populate Geometry Types (Common for Shapefiles)
    ui->cb_GeometryType->addItems({"Point", "LineString", "Polygon", "MultiPoint", "MultiPolygon"});

    // ✅ Populate SRS (Projections - Simplified List)
    ui->cb_SRS->addItems({
        "EPSG:4326 - WGS84",
        "EPSG:3857 - Web Mercator",
        "EPSG:32643 - UTM Zone 43N"
        // You can dynamically load from PROJ database if available
    });

    // ✅ Setup Field Type ComboBox
    ui->cb_fieldtype->addItems({"String", "Integer", "Real", "Date"});

    // ✅ Field Length SpinBox
    ui->sb_fieldLength->setMinimum(1);
    ui->sb_fieldLength->setMaximum(255);
    ui->sb_fieldLength->setValue(80);

    // ✅ Setup Table Model for Fields
    fieldModel = new QStandardItemModel(this);
    fieldModel->setColumnCount(3);
    fieldModel->setHeaderData(0, Qt::Horizontal, "Field Name");
    fieldModel->setHeaderData(1, Qt::Horizontal, "Type");
    fieldModel->setHeaderData(2, Qt::Horizontal, "Length");
    ui->tableView_fieldList->setModel(fieldModel);
    ui->tableView_fieldList->horizontalHeader()->setStretchLastSection(true);

    // ✅ Add Field Button (connect le_fieldName, cb_fieldtype, sb_fieldLength to table)
    connect(ui->pb_AddField, &QPushButton::clicked, this, &CreateShapeFileDialog::onAddField);
    connect(ui->pb_RemoveField, &QPushButton::clicked, this, &CreateShapeFileDialog::onRemoveField);

    // ✅ OK and Cancel buttons
    connect(ui->Pb_Ok, &QPushButton::clicked, this, &CreateShapeFileDialog::onOkClicked);
    connect(ui->pb_Cancel, &QPushButton::clicked, this, &QDialog::reject);
}

CreateShapeFileDialog::~CreateShapeFileDialog() {
    delete ui;
}


void CreateShapeFileDialog::onBrowseOutputPath() {
    QString filePath = QFileDialog::getSaveFileName(this, "Select Shapefile Output", "", "Shapefile (*.shp)");
    if (!filePath.isEmpty()) {
        if (!filePath.endsWith(".shp"))
            filePath += ".shp";
        ui->le_OutPutPath->setText(filePath);
    }
}

void CreateShapeFileDialog::onAddField() {
    QString fieldName = ui->le_fieldName->text().trimmed();
    if (fieldName.isEmpty()) {
        QMessageBox::warning(this, "Invalid Field", "Field name cannot be empty.");
        return;
    }
    QString fieldType = ui->cb_fieldtype->currentText();
    int fieldLength = ui->sb_fieldLength->value();

    QList<QStandardItem *> row;
    row << new QStandardItem(fieldName)
        << new QStandardItem(fieldType)
        << new QStandardItem(QString::number(fieldLength));
    fieldModel->appendRow(row);

    ui->le_fieldName->clear();
}


void CreateShapeFileDialog::onRemoveField() {
    QModelIndexList selected = ui->tableView_fieldList->selectionModel()->selectedRows();
    for (int i = selected.count() - 1; i >= 0; --i) {
        fieldModel->removeRow(selected.at(i).row());
    }
}

void CreateShapeFileDialog::onOkClicked() {
    QString outputPath = ui->le_OutPutPath->text().trimmed();
    if (outputPath.isEmpty()) {
        QMessageBox::warning(this, "Output Path", "Please select an output path.");
        return;
    }

    QString encoding = ui->cb_FileEncoding->currentText();
    QString geometryType = ui->cb_GeometryType->currentText();
    QString srs = ui->cb_SRS->currentText();

    // Collect fields
    QList<QVariantMap> fields;
    for (int row = 0; row < fieldModel->rowCount(); ++row) {
        QVariantMap field;
        field["name"] = fieldModel->item(row, 0)->text();
        field["type"] = fieldModel->item(row, 1)->text();
        field["length"] = fieldModel->item(row, 2)->text().toInt();
        fields.append(field);
    }

    // Emit signal
    emit createShapeFile(outputPath, encoding, geometryType, srs, fields);
    accept();
}
