#include "VisibilityTestAreaWidget.h"
#include "ui_VisibilityTestAreaWidget.h"

#include "MouseEventHandler.h"
#include "app/customwidgets/colorpickercheckbox.h"


VisibilityTestAreaWidget::VisibilityTestAreaWidget(osgEarth::MapNode* mapNode, osgViewer::Viewer *viewer,osg::Group* root,QWidget *parent) :
    QDialog(parent),
    _mapNode(mapNode),
    _viewer(viewer),
    _root(root),
    ui(new Ui::VisibilityTestAreaWidget)
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
            viewShed->setInvisibleAreaColor(inVisibleColor);
        }

    });

    ui->label_bColor->hide();
    ui->label_bWidth->hide();
    ui->label_bOpacity->hide();
    // ui->vl_boundaryColor->;
    ui->sb_boundarylineOpacity->hide();
    ui->sb_boundaryLineWidth->hide();

    ui->sb_distance->setValue(5000);

    adjustSize();


}

VisibilityTestAreaWidget::~VisibilityTestAreaWidget()
{
    delete ui;

     if(viewShed) {
         delete viewShed;
     }
     _root->removeChild(_shadowScene);
}


void VisibilityTestAreaWidget::on_pb_pickLocation_clicked()
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


osg::Vec3 VisibilityTestAreaWidget::geoPointsToVev3(double lon, double lat, double alt){
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


void VisibilityTestAreaWidget::on_pb_runORupdate_clicked()
{
    viewShed = new VisibilityTestArea(_mapNode->asGroup(), _viewer
                                      ,geoPointsToVev3(ui->sb_longitude->value(),ui->sb_latitude->value(),ui->sb_altitude->value())
                                      ,int(ui->sb_distance->value()));
    // viewShed->setRadius(int(ui->sb_distance->value()));
    // viewShed->setViwerPosition(geoPointsToVev3(ui->sb_longitude->value(),ui->sb_latitude->value(),ui->sb_altitude->value()+200));

    viewShed->buildModel();
}

void VisibilityTestAreaWidget::on_sb_verticleAngle_valueChanged(double arg1)
{
    // if(viewShed) viewShed->setVerticalFOV((int)arg1);
}

void VisibilityTestAreaWidget::on_sb_roll_valueChanged(double arg1)
{
    if(viewShed){}
}


void VisibilityTestAreaWidget::on_sb_horizantalAngle_valueChanged(double arg1)
{
    if(viewShed){}

}


void VisibilityTestAreaWidget::on_sb_longitude_valueChanged(double arg1)
{
    if(viewShed) viewShed->setViwerPosition(geoPointsToVev3(arg1,ui->sb_latitude->value(),ui->sb_altitude->value()));
}


void VisibilityTestAreaWidget::on_sb_latitude_valueChanged(double arg1)
{
    if(viewShed) viewShed->setViwerPosition(geoPointsToVev3(ui->sb_longitude->value(),arg1,ui->sb_altitude->value()));
}


void VisibilityTestAreaWidget::on_sb_altitude_valueChanged(double arg1)
{
    if(viewShed) viewShed->setViwerPosition(geoPointsToVev3(ui->sb_longitude->value(),ui->sb_latitude->value(),arg1));
}

void VisibilityTestAreaWidget::on_sb_distance_valueChanged(double arg1)
{
     if(viewShed) viewShed->setRadius((int)arg1);
}


void VisibilityTestAreaWidget::on_sb_visibleAreaOpacity_valueChanged(int arg1)
{
    if(viewShed) {
        float alpha = static_cast<float>(ui->sb_visibleAreaOpacity->value()) / 100.0f;
        visibleColor[3] = alpha;
        viewShed->setVisibleAreaColor(visibleColor);
    }
}


void VisibilityTestAreaWidget::on_sb_hiddenAreaOpacity_valueChanged(int arg1)
{
    if(viewShed) {
        float alpha = static_cast<float>(ui->sb_hiddenAreaOpacity->value()) / 100.0f;
        inVisibleColor[3] = alpha;
        viewShed->setInvisibleAreaColor(inVisibleColor);
    }
}


void VisibilityTestAreaWidget::on_sb_boundarylineOpacity_valueChanged(int arg1)
{

}

