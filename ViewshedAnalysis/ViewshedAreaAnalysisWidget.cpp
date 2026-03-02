#include "ViewshedAreaAnalysisWidget.h"
#include "ui_ViewshedAreaAnalysisWidget.h"

#include "MouseEventHandler.h"



#include <QColorDialog>

ColorPickerCheckBox::ColorPickerCheckBox(const QColor& color, QWidget* parent)
    : QCheckBox(parent),
      m_color(color)
{
    setCheckable(false);
    setTristate(false);
    setFocusPolicy(Qt::NoFocus);
    setCursor(Qt::PointingHandCursor);

    updateAppearance();

    connect(this, &QCheckBox::clicked,
            this, &ColorPickerCheckBox::onClicked);
}

QColor ColorPickerCheckBox::color() const
{
    return m_color;
}

void ColorPickerCheckBox::onClicked()
{
    QColor newColor = QColorDialog::getColor(
        m_color,
        this,
        "Select Color",
        QColorDialog::ShowAlphaChannel
    );

    if (!newColor.isValid())
        return;

    m_color = newColor;
    updateAppearance();
    emit colorChanged(m_color);
}

void ColorPickerCheckBox::updateAppearance()
{
    setText(m_color.name().toUpper());

    setStyleSheet(QString(R"(
        QCheckBox::indicator {
            width: 18px;
            height: 18px;
            background-color: %1;
        }
    )").arg(m_color.name()));
}


ViewshedAreaAnalysisWidget::ViewshedAreaAnalysisWidget(osgEarth::MapNode* mapNode, osgViewer::Viewer *viewer,osg::Group* root,QWidget *parent) :
    QDialog(parent),
    _mapNode(mapNode),
    _viewer(viewer),
    _root(root),
    ui(new Ui::ViewshedAreaAnalysisWidget)
{
    ui->setupUi(this);
    setWindowTitle("3D Viewshed Analysis");
    setAttribute(Qt::WA_DeleteOnClose, true);



    ColorPickerCheckBox* visibleColor = new ColorPickerCheckBox(QColor("#00FF00"), this);
    ColorPickerCheckBox* hiddenColor = new ColorPickerCheckBox(QColor("#ff0000"), this);
    ColorPickerCheckBox* boundaryColor = new ColorPickerCheckBox(QColor("#004cff"), this);

    ui->vl_visibleAreaColor->addWidget(visibleColor);
    ui->vl_hiddenAreaColor->addWidget(hiddenColor);
    ui->vl_boundaryColor->addWidget(boundaryColor);

    connect(boundaryColor, &ColorPickerCheckBox::colorChanged,this, [this](const QColor& c)
    {
        qDebug() << "Color changed to:" << c.name();
        osg::Vec4 osgColor(c.redF(),c.greenF(),c.blueF(),ui->sb_boundarylineOpacity->value()/100);
            // viewShed->setBorderLineColor(osgColor);
    });

    connect(visibleColor, &ColorPickerCheckBox::colorChanged,this, [this](const QColor& c)
    {
        qDebug() << "Color changed to:" << c.name();
        osg::Vec4 osgColor(c.redF(),c.greenF(),c.blueF(),ui->sb_visibleAreaOpacity->value()/100);
            // viewShed->setVisibleAreaColor(osgColor);
    });

    connect(hiddenColor, &ColorPickerCheckBox::colorChanged,this, [this](const QColor& c)
    {
        qDebug() << "Color changed to:" << c.name();
        osg::Vec4 osgColor(c.redF(),c.greenF(),c.blueF(),ui->sb_hiddenAreaOpacity->value()/100);
            // viewShed->setInvisibleAreaColor(osgColor);
    });


    ui->sb_distance->setValue(5000);
    ui->sb_verticleAngle->setValue(90);


}

ViewshedAreaAnalysisWidget::~ViewshedAreaAnalysisWidget()
{
    delete ui;

     if(viewShed) {
         delete viewShed;
     }
     _root->removeChild(_shadowScene);
}


void ViewshedAreaAnalysisWidget::on_pb_pickLocation_clicked()
{
    connect(MouseEventHandler::Instance(), &MouseEventHandler::mouseClickEvent, this,
            [this](const double lon, const double lat,const double alt,const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
    {

        ui->sb_longitude->setValue(lon);
        ui->sb_latitude->setValue(lat);
        ui->sb_altitude->setValue(alt);
        disconnect(MouseEventHandler::Instance(), &MouseEventHandler::mouseClickEvent, this, nullptr);
    });


}


osg::Vec3 ViewshedAreaAnalysisWidget::geoPointsToVev3(double lon, double lat, double alt){
const osgEarth::SpatialReference* mapSRS = _mapNode->getMapSRS();
const osgEarth::SpatialReference* geoSRS = mapSRS->getGeographicSRS();

    // 1️⃣ Convert lon/lat/alt → world coordinates
    osgEarth::GeoPoint geoPoint(mapSRS, lon, lat, alt, osgEarth::ALTMODE_ABSOLUTE);
        // Convert to world coordinates
        osg::Vec3d worldPos;
        geoPoint.toWorld(worldPos,_mapNode->getTerrain());

        osg::Vec3 localepoints = osg::Vec3(worldPos);

        return localepoints;
}


void ViewshedAreaAnalysisWidget::on_pb_runORupdate_clicked()
{
    viewShed = new VisibilityTestArea(_mapNode->asGroup(), _viewer
                                      ,geoPointsToVev3(ui->sb_longitude->value(),ui->sb_latitude->value(),ui->sb_altitude->value())
                                      ,int(ui->sb_distance->value()));
    // viewShed->setRadius(int(ui->sb_distance->value()));
    // viewShed->setViwerPosition(geoPointsToVev3(ui->sb_longitude->value(),ui->sb_latitude->value(),ui->sb_altitude->value()+200));

    viewShed->buildModel();
}

void ViewshedAreaAnalysisWidget::on_sb_verticleAngle_valueChanged(double arg1)
{
    // if(viewShed) viewShed->setVerticalFOV((int)arg1);
}

void ViewshedAreaAnalysisWidget::on_sb_roll_valueChanged(double arg1)
{
    if(viewShed){}
}


void ViewshedAreaAnalysisWidget::on_sb_horizantalAngle_valueChanged(double arg1)
{
    if(viewShed){}

}


void ViewshedAreaAnalysisWidget::on_sb_longitude_valueChanged(double arg1)
{
    if(viewShed) viewShed->setViwerPosition(geoPointsToVev3(arg1,ui->sb_latitude->value(),ui->sb_altitude->value()));
}


void ViewshedAreaAnalysisWidget::on_sb_latitude_valueChanged(double arg1)
{
    if(viewShed) viewShed->setViwerPosition(geoPointsToVev3(ui->sb_longitude->value(),arg1,ui->sb_altitude->value()));
}


void ViewshedAreaAnalysisWidget::on_sb_altitude_valueChanged(double arg1)
{
    if(viewShed) viewShed->setViwerPosition(geoPointsToVev3(ui->sb_longitude->value(),ui->sb_latitude->value(),arg1));
}

void ViewshedAreaAnalysisWidget::on_sb_distance_valueChanged(double arg1)
{
     if(viewShed) viewShed->setRadius((int)arg1);
}

