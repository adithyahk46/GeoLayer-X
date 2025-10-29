#include "LineOfSightWidget.h"
#include "ui_LineOfSightWidget.h"

#include <MouseEventHandler.h>
#include <osgEarth/Color>
#include <osgEarth/GeoData>
#include <QDebug>
#include <osgEarth/LinearLineOfSight>
#include <osgEarth/LineOfSight>
#include <osg/LineWidth>
#include <osgEarth/Feature>
#include <osgEarth/FeatureNode>

MouseEventHandler* handler = nullptr;

using namespace osgEarth;

LineOfSightWidget::LineOfSightWidget(osgEarth::MapNode* mapNode, QWidget *parent)
    : QWidget(parent),
      ui(new Ui::LineOfSightWidget),
      _mapNode(mapNode)
{
    qDebug() << "helow 1";

    ui->setupUi(this);
    handler = MouseEventHandler::Instance();

    updateButtonStyles();

    connect(ui->pb_visbleColor, &QPushButton::clicked, this, &LineOfSightWidget::on_pb_visbleColor_clicked);
    connect(ui->pb_invisibleColor, &QPushButton::clicked, this, &LineOfSightWidget::on_pb_invisibleColor_clicked);
    connect(ui->btnClear, &QPushButton::clicked, this, &LineOfSightWidget::on_btnClear_clicked);
    connect(ui->pb_close, &QPushButton::clicked, this, &LineOfSightWidget::on_pb_close_clicked);
}

LineOfSightWidget::~LineOfSightWidget()
{
    delete ui;
}

void LineOfSightWidget::updateButtonStyles()
{
    // Set Good Color button style (Green)
    ui->pb_visbleColor->setStyleSheet(
        "QPushButton {"
        "    background-color: #4CAF50;"  // Green
        "    color: white;"
        "    border: 1px solid #388E3C;"
        "    border-radius: 3px;"
        "    padding: 5px;"
        "}"
        "QPushButton:hover {"
        "    background-color: #45a049;"
        "}"
        "QPushButton:pressed {"
        "    background-color: #3d8b40;"
        "}"
    );

    // Set Bad Color button style (Red)
    ui->pb_invisibleColor->setStyleSheet(
        "QPushButton {"
        "    background-color: #F44336;"  // Red
        "    color: white;"
        "    border: 1px solid #D32F2F;"
        "    border-radius: 3px;"
        "    padding: 5px;"
        "}"
        "QPushButton:hover {"
        "    background-color: #e53935;"
        "}"
        "QPushButton:pressed {"
        "    background-color: #d32f2f;"
        "}"
    );
}

void LineOfSightWidget::on_pb_sourceLocation_clicked()
{
    QObject::disconnect(handler, &MouseEventHandler::mouseClickEvent, this, nullptr);

    QObject::connect(handler, &MouseEventHandler::mouseClickEvent, this,
        [=](const double lon, const double lat, const double alt,
            const osgGA::GUIEventAdapter&, osgGA::GUIActionAdapter&)
    {
        GeoPoint Start(_mapNode->getMapSRS(), lon, lat, alt, ALTMODE_ABSOLUTE);
        start = Start;

        ui->le_souceLon->setText(QString::number(lon, 'f', 6));
        ui->le_sourceLat->setText(QString::number(lat, 'f', 6));
        ui->le_sourceAlt->setText(QString::number(alt, 'f', 2));

        QObject::disconnect(handler, &MouseEventHandler::mouseClickEvent, this, nullptr);
    });
}

void LineOfSightWidget::on_pb_targetLocation_clicked()
{
    QObject::disconnect(handler, &MouseEventHandler::mouseClickEvent, this, nullptr);

    QObject::connect(handler, &MouseEventHandler::mouseClickEvent, this,
        [=](const double lon, const double lat, const double alt,
            const osgGA::GUIEventAdapter&, osgGA::GUIActionAdapter&)
    {
        GeoPoint ENDing(_mapNode->getMapSRS(), lon, lat, alt, ALTMODE_ABSOLUTE);
        end = ENDing;

        ui->le_targetLon->setText(QString::number(lon, 'f', 6));
        ui->le_targetLat->setText(QString::number(lat, 'f', 6));
        ui->le_targetAlt->setText(QString::number(alt, 'f', 2));

        QObject::disconnect(handler, &MouseEventHandler::mouseClickEvent, this, nullptr);
    });
}

void LineOfSightWidget::on_btnUpdate_clicked()
{
    if (!_mapNode) return;

    osgEarth::Contrib::LinearLineOfSightNode losNode(_mapNode, start, end);

    const GeoPoint& startGeo = losNode.getStart();
    const GeoPoint& endGeo = losNode.getEnd();
    const GeoPoint& hitGeo = losNode.getHit();

    bool hasLOS = losNode.getHasLOS();

    float lineWidth = static_cast<float>(ui->sb_lineWidth->value());

    if (hasLOS) {
        drawLine(startGeo, endGeo, goodColor, lineWidth);
        ui->le_resultLon->setText("");
        ui->le_resultLat->setText("");
        ui->le_resultAlt->setText("Visible");
    } else {
        drawLine(startGeo, hitGeo, goodColor, lineWidth);
        drawLine(hitGeo, endGeo, badColor, lineWidth);
        ui->le_resultLon->setText(QString::number(hitGeo.x(), 'f', 6));
        ui->le_resultLat->setText(QString::number(hitGeo.y(), 'f', 6));
        ui->le_resultAlt->setText(QString::number(hitGeo.z(), 'f', 2));
    }
}

void LineOfSightWidget::on_btnClear_clicked()
{
    ui->le_souceLon->clear();
    ui->le_sourceLat->clear();
    ui->le_sourceAlt->clear();
    ui->le_targetLon->clear();
    ui->le_targetLat->clear();
    ui->le_targetAlt->clear();
    ui->le_resultLon->clear();
    ui->le_resultLat->clear();
    ui->le_resultAlt->clear();
    for(auto i : lineNodes){
        _mapNode->removeChild(i);
    }
}

void LineOfSightWidget::on_pb_close_clicked()
{
    this->close();
}

#include <QColorDialog>

void LineOfSightWidget::on_pb_visbleColor_clicked()
{
    QColor currentColor = QColor::fromRgbF(goodColor.r(), goodColor.g(), goodColor.b(), goodColor.a());
    QColor newColor = QColorDialog::getColor(currentColor, this, "Select Good Color");

    if (newColor.isValid()) {
        goodColor = osg::Vec4f(
            newColor.redF(),
            newColor.greenF(),
            newColor.blueF(),
            1.0f
        );

        ui->pb_visbleColor->setStyleSheet(
            QString("QPushButton { background-color: %1; color: white; border: 1px solid %2; border-radius: 3px; padding: 5px; }")
            .arg(newColor.name())
            .arg(newColor.darker().name())
        );
    }
}

void LineOfSightWidget::on_pb_invisibleColor_clicked()
{
    QColor currentColor = QColor::fromRgbF(badColor.r(), badColor.g(), badColor.b(), badColor.a());
    QColor newColor = QColorDialog::getColor(currentColor, this, "Select Bad Color");

    if (newColor.isValid()) {
        badColor = osg::Vec4f(
            newColor.redF(),
            newColor.greenF(),
            newColor.blueF(),
            1.0f
        );

        ui->pb_invisibleColor->setStyleSheet(
            QString("QPushButton { background-color: %1; color: white; border: 1px solid %2; border-radius: 3px; padding: 5px; }")
            .arg(newColor.name())
            .arg(newColor.darker().name())
        );
    }
}

void LineOfSightWidget::drawLine(const GeoPoint& p1, const GeoPoint& p2, const osg::Vec4f& color, float width)
{
    if (!_mapNode) return;

    osg::ref_ptr<osgEarth::LineString> line = new osgEarth::LineString();
    line->push_back(osg::Vec3d(p1.x(), p1.y(), p1.z()));
    line->push_back(osg::Vec3d(p2.x(), p2.y(), p2.z()));

    osg::ref_ptr<osgEarth::Feature> feature = new osgEarth::Feature(line, _mapNode->getMapSRS());
    feature->setGeometry(line);

    osgEarth::Style style;
    osg::ref_ptr<osgEarth::LineSymbol> ls = style.getOrCreate<osgEarth::LineSymbol>();
    ls->stroke()->color() = color;
    ls->stroke()->width() = osgEarth::Distance(width, osgEarth::Units::PIXELS);

    QString lineStyle = ui->cb_lineStyle->currentText();
    if (lineStyle == "Dotted Line") {
        ls->stroke()->stipplePattern() = 0xAAAA; // Dotted pattern
    } else if (lineStyle == "Dashed Line") {
        ls->stroke()->stipplePattern() = 0xFF00; // Dashed pattern
    }

    osg::ref_ptr<osgEarth::FeatureNode> featureNode = new osgEarth::FeatureNode(feature.get(), style);
    lineNodes.push_back(featureNode);
    _mapNode->addChild(featureNode.get());
}
