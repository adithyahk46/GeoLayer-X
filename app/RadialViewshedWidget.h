#ifndef RADIALVIEWSHEDWIDGET_H
#define RADIALVIEWSHEDWIDGET_H

#include <QDialog>
#include <osgEarth/MapNode>
#include <osgViewer/Viewer>



#include "pluggins/VisibilityTestArea/RadialViewshedAnalysis.h"


namespace Ui {
class RadialViewshedWidget;
}

class RadialViewshedWidget : public QDialog
{
    Q_OBJECT

public:
    explicit RadialViewshedWidget(osgEarth::MapNode* mapNode, osgViewer::Viewer *viewer,osg::Group* root,QWidget *parent = nullptr);
    ~RadialViewshedWidget();

private slots:
    void on_pb_pickLocation_clicked();

    void on_pb_runORupdate_clicked();

    void on_sb_verticleAngle_valueChanged(double arg1);
    void on_sb_roll_valueChanged(double arg1);

    void on_sb_horizantalAngle_valueChanged(double arg1);

    void on_sb_longitude_valueChanged(double arg1);

    void on_sb_latitude_valueChanged(double arg1);

    void on_sb_altitude_valueChanged(double arg1);

    void on_sb_distance_valueChanged(double arg1);

    void on_sb_visibleAreaOpacity_valueChanged(int arg1);

    void on_sb_hiddenAreaOpacity_valueChanged(int arg1);

    void on_sb_boundarylineOpacity_valueChanged(int arg1);

private:
    Ui::RadialViewshedWidget *ui;
    osgEarth::MapNode* _mapNode = nullptr;
    osgViewer::Viewer* _viewer = nullptr;
    osg::Group* _root = nullptr;
    RadialViewshedAnalysis* viewShed =nullptr;

    osg::ref_ptr<osg::Group> _shadowSceneparent = nullptr;
    osg::ref_ptr<osg::Group>_shadowScene = nullptr;

    osg::Vec4  visibleColor   = osg::Vec4(159.0f / 255.0f, 255.0f / 255.0f, 61.0f / 255.0f, 1.0f);
    osg::Vec4  inVisibleColor = osg::Vec4(255.0f / 255.0f, 87.0f / 255.0f, 61.0f / 255.0f, 1.0f);

    osg::Vec3 geoPointsToVev3(double lon, double lat, double alt);
};

#endif // RADIALVIEWSHEDWIDGET_H
