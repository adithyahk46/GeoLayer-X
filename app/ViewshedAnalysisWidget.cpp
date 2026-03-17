#include "ViewshedAnalysisWidget.h"
#include "ui_ViewshedAnalysisWidget.h"
#include "MouseEventHandler.h"

#include <QColorDialog>

#include "app/customwidgets/colorpickercheckbox.h"
#include "app/app.h"


ViewshedAnalysisWidget::ViewshedAnalysisWidget(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ViewshedAnalysisWidget)
{
    ui->setupUi(this);
    setWindowTitle("3D Viewshed Analysis");
    setAttribute(Qt::WA_DeleteOnClose, true);



    ColorPickerCheckBox* cb_visibleColor = new ColorPickerCheckBox(QColor("#00FF00"), this);
    ColorPickerCheckBox* cb_hiddenColor = new ColorPickerCheckBox(QColor("#ff0000"), this);
    // ColorPickerCheckBox* cb_boundaryColor = new ColorPickerCheckBox(QColor("#004cff"), this);

    ui->vl_visibleAreaColor->addWidget(cb_visibleColor);
    ui->vl_hiddenAreaColor->addWidget(cb_hiddenColor);
    // ui->vl_boundaryColor->addWidget(cb_boundaryColor);

    // connect(cb_boundaryColor, &ColorPickerCheckBox::colorChanged,this, [this](const QColor& c)
    // {
    //     if(viewShed){
    //         float alpha = static_cast<float>(ui->sb_boundarylineOpacity->value()) / 100.0f;
    //          osg::Vec4 osgColor(c.redF(), c.greenF(), c.blueF(), alpha);
    //             // viewShed->setBorderLineColor(osgColor);
    //     }

    // });

    connect(cb_visibleColor, &ColorPickerCheckBox::colorChanged,this, [this](const QColor& c)
    {

        if(viewShed){
            float alpha = static_cast<float>(ui->sb_visibleAreaOpacity->value()) / 100.0f;
            osg::Vec4 osgColor(c.redF(), c.greenF(), c.blueF(), alpha);
            visibleColor = osgColor;
            viewShed->setVisibleAreaColor(visibleColor);
        }

    });

    connect(cb_hiddenColor, &ColorPickerCheckBox::colorChanged,this, [this](const QColor& c)
    {
        if(viewShed){
            float alpha = static_cast<float>(ui->sb_hiddenAreaOpacity->value()) / 100.0f;
            osg::Vec4 osgColor(c.redF(),c.greenF(),c.blueF(), alpha);
            inVisibleColor = osgColor;
            viewShed->setHiddenAreaColor(inVisibleColor);
        }

    });

    ui->sb_distance->setValue(5000);

    adjustSize();


}

ViewshedAnalysisWidget::~ViewshedAnalysisWidget()
{
    delete ui;

     if(viewShed) {
         delete viewShed;
     }
}


void ViewshedAnalysisWidget::on_pb_pickLocation_clicked()
{
    connect(MouseEventHandler::Instance(), &MouseEventHandler::mouseClickEvent, this,
            [this](const double lon, const double lat,const double alt,const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
    {
        ui->sb_longitude->setValue(lon);
        ui->sb_latitude->setValue(lat);
        ui->sb_altitude->setValue(alt);

        osgEarth::GeoPoint geoPoint(App::getInstance()->getMapNode()->getMapSRS(), lon, lat, alt, osgEarth::ALTMODE_ABSOLUTE);

        _lightPos = geoPoint;

        disconnect(MouseEventHandler::Instance(), &MouseEventHandler::mouseClickEvent, this, nullptr);

    });


}

void ViewshedAnalysisWidget::on_pb_runORupdate_clicked()
{
    viewShed = new ViewshedAnalysis(App::getInstance()->getMapNode(), _lightPos,int(ui->sb_distance->value()));
    viewShed->buildModel();
}

void ViewshedAnalysisWidget::on_sb_verticleAngle_valueChanged(double arg1)
{
    if(viewShed) viewShed->setVerticalFOV((int)arg1);
}

void ViewshedAnalysisWidget::on_sb_horizantalAngle_valueChanged(double arg1)
{
    if(viewShed)viewShed->setHorizontalFOV((int)arg1);

}

void ViewshedAnalysisWidget::on_sb_longitude_valueChanged(double arg1)
{
    _lightPos.x() = arg1;
    if(viewShed) viewShed->setCameraPosition(_lightPos);
}

void ViewshedAnalysisWidget::on_sb_latitude_valueChanged(double arg1)
{
    _lightPos.y() = arg1;
    if(viewShed) viewShed->setCameraPosition(_lightPos);
}

void ViewshedAnalysisWidget::on_sb_altitude_valueChanged(double arg1)
{
    _lightPos.z() = arg1;
    if(viewShed) viewShed->setCameraPosition(_lightPos);

}

void ViewshedAnalysisWidget::on_sb_distance_valueChanged(double arg1)
{
     if(viewShed) viewShed->setDistance((float)arg1);
}

void ViewshedAnalysisWidget::on_sb_visibleAreaOpacity_valueChanged(int arg1)
{
    if(viewShed) {
        float alpha = static_cast<float>(ui->sb_visibleAreaOpacity->value()) / 100.0f;
        visibleColor[3] = alpha;
        viewShed->setVisibleAreaColor(visibleColor);
    }
}

void ViewshedAnalysisWidget::on_sb_hiddenAreaOpacity_valueChanged(int arg1)
{
    if(viewShed) {
        float alpha = static_cast<float>(ui->sb_hiddenAreaOpacity->value()) / 100.0f;
        inVisibleColor[3] = alpha;
        viewShed->setHiddenAreaColor(inVisibleColor);
    }
}

void ViewshedAnalysisWidget::on_sb_boundarylineOpacity_valueChanged(int arg1)
{

}

void ViewshedAnalysisWidget::on_sb_xRotation_valueChanged(double arg1)
{
    static double oldRoll = 0;
    double roll = arg1 - oldRoll;
    oldRoll = arg1;
    // Pass both current roll and current pitch to the setter
    if(viewShed) {viewShed->setRotation(roll,ViewshedAnalysis::Rotation::HORIZANTAL);}
}

void ViewshedAnalysisWidget::on_sb_yRotation_valueChanged(double arg1)
{
    static double oldy = 0;
    double roll = arg1 - oldy;
    oldy = arg1;
    if(viewShed) {viewShed->setRotation(roll, ViewshedAnalysis::Rotation::VERTICAL);}
}

