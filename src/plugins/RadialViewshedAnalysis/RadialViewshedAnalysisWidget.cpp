#include "RadialViewshedAnalysisWidget.h"
#include "ui_RadialViewshedAnalysisWidget.h"

#include "MapController/GLXEarthManipulator.h"

#include <QColorDialog>

#include "colorpickercheckbox.h"
#include "osgviewerwidget.h"
#include "App.h"

RadialViewshedAnalysisWidget::RadialViewshedAnalysisWidget(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::RadialViewshedAnalysisWidget)
{
    ui->setupUi(this);

    setWindowTitle("3D Radial Viewshed Analysis");
    setAttribute(Qt::WA_DeleteOnClose, true);

    ColorPickerCheckBox* cb_visibleColor = new ColorPickerCheckBox(QColor("#00FF00"), this);
    ColorPickerCheckBox* cb_hiddenColor = new ColorPickerCheckBox(QColor("#ff0000"), this);

    ui->vl_visibleAreaColor->addWidget(cb_visibleColor);
    ui->vl_hiddenAreaColor->addWidget(cb_hiddenColor);

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

    ui->sb_distance->setValue(5000);

    adjustSize();
}

RadialViewshedAnalysisWidget::~RadialViewshedAnalysisWidget()
{
    delete ui;

    if(viewShed) {
        delete viewShed;
    }
}

void RadialViewshedAnalysisWidget::on_pb_pickLocation_clicked()
{
    GLXEarthManipulator* manipulator = App::getThreedWindow()->getManipulator();
    if (!manipulator)
        return;

    disconnect(manipulator, &GLXEarthManipulator::mouseClickEvent, this, nullptr);
    connect(manipulator, &GLXEarthManipulator::mouseClickEvent, this,
            [this, manipulator](osgEarth::GeoPoint mapPoint)
            {
                osgEarth::GeoPoint geographicPoint;
                mapPoint.transform(osgEarth::SpatialReference::get("epsg:4326"), geographicPoint);

                _lightPos = geographicPoint;
                ui->sb_longitude->setValue(geographicPoint.x());
                ui->sb_latitude->setValue(geographicPoint.y());
                ui->sb_altitude->setValue(geographicPoint.z());

                disconnect(manipulator, &GLXEarthManipulator::mouseClickEvent, this, nullptr);

            });


}

void RadialViewshedAnalysisWidget::on_pb_createORupdate_clicked()
{
    osg::Vec3d worldPosition;
    _lightPos.toWorld(worldPosition);
    viewShed = new RadialViewshedAnalysis(
        App::getThreedWindow()->getMapNode(),
        App::getThreedWindow()->getViewer(),
        osg::Vec3(worldPosition),
        static_cast<int>(ui->sb_distance->value()));
    viewShed->buildModel();
}

void RadialViewshedAnalysisWidget::on_sb_longitude_valueChanged(double arg1)
{
    _lightPos.x() = arg1;
    if (viewShed) {
        osg::Vec3d worldPosition;
        _lightPos.toWorld(worldPosition);
        viewShed->setViwerPosition(osg::Vec3(worldPosition));
    }
}

void RadialViewshedAnalysisWidget::on_sb_latitude_valueChanged(double arg1)
{
    _lightPos.y() = arg1;
    if (viewShed) {
        osg::Vec3d worldPosition;
        _lightPos.toWorld(worldPosition);
        viewShed->setViwerPosition(osg::Vec3(worldPosition));
    }
}

void RadialViewshedAnalysisWidget::on_sb_altitude_valueChanged(double arg1)
{
    _lightPos.z() = arg1;
    if (viewShed) {
        osg::Vec3d worldPosition;
        _lightPos.toWorld(worldPosition);
        viewShed->setViwerPosition(osg::Vec3(worldPosition));
    }
}

void RadialViewshedAnalysisWidget::on_sb_distance_valueChanged(double arg1)
{
    if (viewShed) viewShed->setRadius(static_cast<int>(arg1));
}

void RadialViewshedAnalysisWidget::on_sb_visibleAreaOpacity_valueChanged(int arg1)
{
    if(viewShed) {
        float alpha = static_cast<float>(ui->sb_visibleAreaOpacity->value()) / 100.0f;
        visibleColor[3] = alpha;
        viewShed->setVisibleAreaColor(visibleColor);
    }
}

void RadialViewshedAnalysisWidget::on_sb_hiddenAreaOpacity_valueChanged(int arg1)
{
    if(viewShed) {
        float alpha = static_cast<float>(ui->sb_hiddenAreaOpacity->value()) / 100.0f;
        inVisibleColor[3] = alpha;
        viewShed->setInvisibleAreaColor(inVisibleColor);
    }
}