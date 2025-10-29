#include "LabelingDialog.h"
#include "ui_LabelingDialog.h"

#include <osgEarth/TextSymbol>

LabelingDialog::LabelingDialog(osgEarth::MapNode* mapNode, MouseEventHandler* mousehandler, QWidget *parent)
    : QDialog(parent)
    ,_mapNode(mapNode)
    ,_mousehandler(mousehandler)
    , ui(new Ui::LabelingDialog)
{
    ui->setupUi(this);

    ui->pb_GetStyle->hide();
    ui->pb_AddText->show();
    // Default values
    ui->sb_fontSize->setValue(22);
    ui->le_fillColor->setText("##ffffff");
    ui->le_haloColor->setText("#fbff01");
    ui->le_LabelBoxFillColor->setText("##c1ffbd");
    ui->le_LabelBoxBorderColor->setText("#ff0000");


    ui->sp_haloWidth->setValue(0);
    ui->sb_LabelBoxBorderWidth->setValue(2);
    ui->cb_enableLabelBox->setChecked(false);
    ui->GB_LabelBox->setVisible(false);

    // Populate fonts
    QStringList fonts = {"Arial", "Times New Roman", "Courier New"}; // load from osgEarth if possible
    ui->cb_Font->addItems(fonts);

    // Alignment options (osgEarth::Symbology::TextSymbol::Alignment)
    {
        ui->cb_textAlignment->addItem("Left Top",       osgEarth::TextSymbol::ALIGN_LEFT_TOP);
        ui->cb_textAlignment->addItem("Left Center",    osgEarth::TextSymbol::ALIGN_LEFT_CENTER);
        ui->cb_textAlignment->addItem("Left Bottom",    osgEarth::TextSymbol::ALIGN_LEFT_BOTTOM);
        ui->cb_textAlignment->addItem("Center Top",     osgEarth::TextSymbol::ALIGN_CENTER_TOP);
        ui->cb_textAlignment->addItem("Center Center",  osgEarth::TextSymbol::ALIGN_CENTER_CENTER);
        ui->cb_textAlignment->addItem("Center Bottom",  osgEarth::TextSymbol::ALIGN_CENTER_BOTTOM);
        ui->cb_textAlignment->addItem("Right Top",      osgEarth::TextSymbol::ALIGN_RIGHT_TOP);
        ui->cb_textAlignment->addItem("Right Center",   osgEarth::TextSymbol::ALIGN_RIGHT_CENTER);
        ui->cb_textAlignment->addItem("Right Bottom",   osgEarth::TextSymbol::ALIGN_RIGHT_BOTTOM);
        ui->cb_textAlignment->addItem("Left Baseline",  osgEarth::TextSymbol::ALIGN_LEFT_BASE_LINE);
        ui->cb_textAlignment->addItem("Center Baseline",osgEarth::TextSymbol::ALIGN_CENTER_BASE_LINE);
        ui->cb_textAlignment->addItem("Right Baseline", osgEarth::TextSymbol::ALIGN_RIGHT_BASE_LINE);
        ui->cb_textAlignment->addItem("Left Bottom BL", osgEarth::TextSymbol::ALIGN_LEFT_BOTTOM_BASE_LINE);
        ui->cb_textAlignment->addItem("Center Bottom BL", osgEarth::TextSymbol::ALIGN_CENTER_BOTTOM_BASE_LINE);
        ui->cb_textAlignment->addItem("Right Bottom BL", osgEarth::TextSymbol::ALIGN_RIGHT_BOTTOM_BASE_LINE);

        // Default selection
        ui->cb_textAlignment->setCurrentIndex(1); // ALIGN_LEFT_CENTER, for example

    }

    // Connect buttons
    connect(ui->pb_fillColor, &QPushButton::clicked, this, [=](){

        QString color = getColor();
        if(color.isEmpty())
            return;
        ui->le_fillColor->setText(color);
    });
    connect(ui->pb_haloColor, &QPushButton::clicked, this, [=](){
        QString color = getColor();
        if(color.isEmpty())
            return;
        ui->le_haloColor->setText(color);
    });
    connect(ui->pb_LabelBoxFillColor, &QPushButton::clicked, this, [=](){
        QString color = getColor();
        if(color.isEmpty())
            return;
        ui->le_LabelBoxFillColor->setText(color);
    });
    connect(ui->pb_LabelBoxFillColor_2, &QPushButton::clicked, this, [=](){
        QString color = getColor();
        if(color.isEmpty())
            return;
        ui->le_LabelBoxBorderColor->setText(color);
    });

    connect(ui->cb_enableLabelBox, &QCheckBox::toggled, ui->GB_LabelBox, &QGroupBox::setVisible);

    connect(ui->pb_LocationPicker, &QPushButton::clicked, [=](){
        connect(mousehandler, &MouseEventHandler::mouseClickEvent,
                this, &LabelingDialog::onMapClicked);
    });

    connect(ui->pb_AddText, &QPushButton::clicked, this, &LabelingDialog::onAddTextClicked);
    connect(ui->pb_reset, &QPushButton::clicked, this, &LabelingDialog::onReset);
    connect(ui->pb_cancel, &QPushButton::clicked, this, &LabelingDialog::reject);
}

LabelingDialog::~LabelingDialog()
{
    delete ui;
}

void LabelingDialog::OpenTOGetStyle()
{
    ui->widget->setVisible(false);
    ui->pb_GetStyle->show();
    ui->pb_AddText->hide();
    show();

}


QString LabelingDialog::getColor(){
    QColor color = QColorDialog::getColor(Qt::white, this, "Choose Halo Color");
    if(color.isValid())
        return color.name();

   return "";
}

void LabelingDialog::onMapClicked(const double lon, const double lat, const double alt,
                                  const osgGA::GUIEventAdapter&, osgGA::GUIActionAdapter&)
{
    ui->le_Lon->setText(QString::number(lon, 'f', 6));
    ui->le_Lat->setText(QString::number(lat, 'f', 6));
    disconnect(_mousehandler, &MouseEventHandler::mouseClickEvent,
            this, &LabelingDialog::onMapClicked);
}


osgEarth::Style LabelingDialog::getStyle(){

    Style labelStyle;
    // labelStyle.getOrCreate<TextSymbol>()->content()->setLiteral(ui->le_InputText->text().toStdString());
    labelStyle.getOrCreate<TextSymbol>()->font() = ui->cb_Font->currentText().toStdString();
    labelStyle.getOrCreate<TextSymbol>()->size() = ui->sb_fontSize->value();
    // labelStyle.getOrCreate<TextSymbol>()->alignment() = TextSymbol::ALIGN_CENTER_CENTER;
    int alignEnum = ui->cb_textAlignment->currentData().toInt();
    labelStyle.getOrCreate<TextSymbol>()->alignment() = static_cast<osgEarth::TextSymbol::Alignment>(alignEnum);
    osgEarth::BBoxSymbol* bbox = labelStyle.getOrCreate<osgEarth::BBoxSymbol>();

    labelStyle.getOrCreate<TextSymbol>()->fill() = Color(ui->le_fillColor->text().toStdString());
    if(ui->sp_haloWidth->value() !=0){
        labelStyle.getOrCreate<TextSymbol>()->halo() = Color(ui->le_haloColor->text().toStdString());
        labelStyle.getOrCreate<TextSymbol>()->halo().mutable_value().width() = osgEarth::Distance(ui->sp_haloWidth->value(), osgEarth::Units::PIXELS);
    }

    if (ui->cb_enableLabelBox->isChecked())
    {
        bbox->fill() = osgEarth::Fill(
            osgEarth::Color(ui->le_LabelBoxFillColor->text().toStdString())
        );
        // Brder (stroke)
        osgEarth::Stroke stroke;
        stroke.color() = osgEarth::Color(ui->le_LabelBoxBorderColor->text().toStdString());
        stroke.width() = osgEarth::Distance(ui->sb_LabelBoxBorderWidth->value(),
                                            osgEarth::Units::PIXELS);
        bbox->border() = stroke;
        // bbox->margin() = 3.0f;
    }

    labelStyle.getOrCreate<TextSymbol>()->declutter() = true;


    return labelStyle;
}


void LabelingDialog::onAddTextClicked()
{

    if (ui->le_InputText->text().isEmpty()) {
        return; // guard
    }

    GeoPoint pos(
        _mapNode->getMapSRS(),
                ui->le_Lon->text().toDouble(),
                ui->le_Lat->text().toDouble(), 0.0,  // lon, lat, altitude
        ALTMODE_ABSOLUTE);


    // Style labelStyle = getStyle().getOrCreate<TextSymbol>()->content()->setLiteral(ui->le_InputText->text().toStdString());
    LabelNode* _measureLabel = new LabelNode(pos,ui->le_InputText->text().toStdString(), getStyle());
    // _measureLabel->setDynamic(true);
    // _measureLabel->setStyle(labelStyle);
    // _measureLabel->setPosition(pos);

    // _mapNode->addChild(_measureLabel);

    emit LabelCreated(_measureLabel);

}


void LabelingDialog::onReset()
{
    ui->le_InputText->clear();
    ui->le_Lon->clear();
    ui->le_Lat->clear();
    ui->cb_Font->setCurrentIndex(0);
    ui->sb_fontSize->setValue(22);
    ui->sp_haloWidth->setValue(0);
    ui->cb_enableLabelBox->setChecked(false);
    ui->sb_LabelBoxBorderWidth->setValue(2);

    ui->le_fillColor->setText("##ffffff");
    ui->le_haloColor->setText("#fbff01");
    ui->le_LabelBoxFillColor->setText("##c1ffbd");
    ui->le_LabelBoxBorderColor->setText("#ff0000");

}

