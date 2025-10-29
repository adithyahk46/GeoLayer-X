#include "Annotation.h"
#include "ui_Annotation.h"

#include <QColorDialog>
#include <QColor>
#include <QFileInfoList>
#include <QDir>
#include <QFileInfo>

Annotation::Annotation(osgEarth::MapNode* mapNode,MouseEventHandler* mouseControlle,QWidget *parent) :
    _mapNode(mapNode)
  ,_mouseControlle(mouseControlle),
    QDialog(parent),
    ui(new Ui::Annotation)
{
    ui->setupUi(this);

    annoGroup = new osg::Group();
    _mapNode->addChild(annoGroup);

    labelGroup = new osg::Group();
    annoGroup->addChild( labelGroup );

    QObject::connect(_mouseControlle, &MouseEventHandler::mouseClickEvent,this, &Annotation::mouseClickEvent);

    ui->pb_fontColor->setStyleSheet("background-color: white;");
    ui->pb_outlineColor->setStyleSheet("background-color: black;");

    connect(ui->pb_fontColor, &QPushButton::clicked, this, [=]() {
        QColor color = QColorDialog::getColor(Qt::white, this, "Select Font Color");
        if (color.isValid()) {
            ui->pb_fontColor->setStyleSheet(QString("background-color: %1").arg(color.name()));
        }
    });

    connect(ui->pb_outlineColor, &QPushButton::clicked, this, [=]() {
        QColor color = QColorDialog::getColor(Qt::black, this, "Select Outline Color");
        if (color.isValid()) {
            ui->pb_outlineColor->setStyleSheet(QString("background-color: %1").arg(color.name()));
        }
    });


    QStringList fontDirs = {
           "/usr/share/fonts",          // Linux
           "/usr/local/share/fonts",    // Linux alt
           "C:/Windows/Fonts"           // Windows
       };

       QStringList fontExtensions = {"ttf", "otf"};

       for (const QString &dirPath : fontDirs) {
           QDir dir(dirPath);
           if (!dir.exists()) continue;

           QFileInfoList fontFiles = dir.entryInfoList(QDir::Files | QDir::NoSymLinks);
           for (const QFileInfo &file : fontFiles) {
               if (fontExtensions.contains(file.suffix().toLower())) {
                   ui->cb_font->addItem(file.fileName(), file.absoluteFilePath());
               }
           }
       }


    annoGroup = new osg::Group();
    _mapNode->addChild(annoGroup);
}

Annotation::~Annotation()
{
    delete ui;
}

void Annotation::on_pb_Add_clicked()
{
    _addText = true;

}

void Annotation::mouseClickEvent(const double lon, const double lat, const double alt, const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa)
{
    if(_addText){
        osgEarth::Style labelStyle;
        osgEarth::TextSymbol* textSymbol = labelStyle.getOrCreate<osgEarth::TextSymbol>();
        // text->content() = StringExpression( "[name]" );
        //       text->priority() = NumericExpression( "[pop]" );
        textSymbol->font() = ui->cb_font->currentText().toStdString();
        // QString fontFilePath = ui->cb_font->itemData(ui->cb_font->currentIndex()).toString();
        // textSymbol->font()->setValue(fontFilePath.toStdString());  // Apply chosen font

        textSymbol->size() = ui->sb_InputFontSize->value();
        textSymbol->fill()->color() = osgEarth::Color::Green;
        textSymbol->halo()->color() = osgEarth::Color::Red;  // Better readability

        textSymbol->halo()->width() = osgEarth::Distance(5.0f, osgEarth::Units::PIXELS);                   // Thickness
        textSymbol->alignment() = osgEarth::TextSymbol::ALIGN_CENTER_CENTER;
        textSymbol->declutter() = false;  // Ensure it doesn't disappear

        osgEarth::GeoPoint mapPoint(_mapNode->getMapSRS(), lon, lat, 0.0, osgEarth::ALTMODE_ABSOLUTE);
        osgEarth::LabelNode* label = new osgEarth::LabelNode(mapPoint, ui->le_InputText->text().toStdString(), labelStyle);
        labelGroup->addChild(label);

        _addText = false;
    }
    // 1. Define the style for the label

}


#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QPushButton>
#include <QHeaderView>
#include <QLineEdit>
#include <QDateEdit>
#include <QIntValidator>
#include <QDoubleValidator>

AttributeDialog::AttributeDialog(osgEarth::FeatureNode* featureNode,
                                 const QList<QVariantMap>& fields,
                                 QWidget* parent)
    : QDialog(parent),
      m_fields(fields),
      m_featureNode(featureNode)
{
    setWindowTitle("Enter Attribute Values");
    setMinimumWidth(400);

    setupUI();
    populateTable();
}

AttributeDialog::~AttributeDialog() {}

void AttributeDialog::setupUI()
{
    QVBoxLayout* layout = new QVBoxLayout(this);

    m_table = new QTableWidget(m_fields.size(), 2, this);
    m_table->setHorizontalHeaderLabels({"Field", "Value"});
    m_table->horizontalHeader()->setStretchLastSection(true);
    m_table->verticalHeader()->setVisible(false);

    layout->addWidget(m_table);

    // Buttons
    QHBoxLayout* btnLayout = new QHBoxLayout();
    QPushButton* okBtn = new QPushButton("OK");
    QPushButton* cancelBtn = new QPushButton("Cancel");
    btnLayout->addStretch();
    btnLayout->addWidget(okBtn);
    btnLayout->addWidget(cancelBtn);
    layout->addLayout(btnLayout);

    connect(okBtn, &QPushButton::clicked, this, [this]() {
        applyAttributesToFeature();
        accept();
    });
    connect(cancelBtn, &QPushButton::clicked, this, &QDialog::reject);
}

void AttributeDialog::populateTable()
{
    osgEarth::Feature* feature = m_featureNode ? m_featureNode->getFeature() : nullptr;

    int row = 0;
    for (const QVariantMap& field : m_fields)
    {
        QString name = field.value("name").toString();
        QString type = field.value("type").toString(); // String, Integer, Double, Date

        // Column 0: Field Name
        QTableWidgetItem* nameItem = new QTableWidgetItem(name);
        nameItem->setFlags(Qt::ItemIsEnabled); // read-only
        m_table->setItem(row, 0, nameItem);

        // Existing value (if feature already has attribute)
        QString existingValue;
        // if (feature)
        // {
        //     const osgEarth::AttributeValue& attr = feature->get(name.toStdString());
        //     if (!attr.isNull())
        //         existingValue = QString::fromStdString(attr.getString());
        // }

        // Column 1: Value editor
        if (type.compare("Float", Qt::CaseInsensitive) == 0 ||
            type.compare("Integer", Qt::CaseInsensitive) == 0)
        {
            QLineEdit* editor = new QLineEdit();
            editor->setValidator(new QIntValidator(editor));
            if (!existingValue.isEmpty())
                editor->setText(existingValue);
            m_table->setCellWidget(row, 1, editor);
        }
        else if (type.compare("Double", Qt::CaseInsensitive) == 0 ||
                 type.compare("Real", Qt::CaseInsensitive) == 0)
        {
            QLineEdit* editor = new QLineEdit();
            editor->setValidator(new QDoubleValidator(editor));
            if (!existingValue.isEmpty())
                editor->setText(existingValue);
            m_table->setCellWidget(row, 1, editor);
        }
        else if (type.compare("Date", Qt::CaseInsensitive) == 0)
        {
            QDateEdit* dateEdit = new QDateEdit(QDate::currentDate());
            dateEdit->setCalendarPopup(true);
            if (!existingValue.isEmpty())
                dateEdit->setDate(QDate::fromString(existingValue, Qt::ISODate));
            m_table->setCellWidget(row, 1, dateEdit);
        }
        else // String
        {
            QLineEdit* editor = new QLineEdit();
            if (!existingValue.isEmpty())
                editor->setText(existingValue);
            m_table->setCellWidget(row, 1, editor);
        }

        // Store type in the nameItem
        nameItem->setData(Qt::UserRole, type);
        row++;
    }
}

void AttributeDialog::applyAttributesToFeature()
{
    if (!m_featureNode) return;

    osgEarth::Feature* feature = m_featureNode->getFeature();
    if (!feature) return;

    for (int i = 0; i < m_table->rowCount(); ++i)
    {
        QString fieldName = m_table->item(i, 0)->text();
        QString type = m_table->item(i, 0)->data(Qt::UserRole).toString();
        QString value;

        if (type.compare("Integer", Qt::CaseInsensitive) == 0 ||
            type.compare("Double", Qt::CaseInsensitive) == 0 ||
            type.compare("Real", Qt::CaseInsensitive) == 0 ||
            type.compare("String", Qt::CaseInsensitive) == 0)
        {
            QLineEdit* editor = qobject_cast<QLineEdit*>(m_table->cellWidget(i, 1));
            value = editor ? editor->text() : "";
        }
        else if (type.compare("Date", Qt::CaseInsensitive) == 0)
        {
            QDateEdit* dateEdit = qobject_cast<QDateEdit*>(m_table->cellWidget(i, 1));
            value = dateEdit ? dateEdit->date().toString(Qt::ISODate) : "";
        }

        feature->set(fieldName.toStdString(), value.toStdString());
    }
}

