#ifndef VIEWSHEDANALYSISWIDGET_H
#define VIEWSHEDANALYSISWIDGET_H

#include <QDialog>

#include "pluggins/VisibilityTestArea/ViewshedAnalysis.h"

namespace Ui {
class ViewshedAnalysisWidget;
}


class ViewshedAnalysisWidget : public QDialog
{
    Q_OBJECT

public:
    explicit ViewshedAnalysisWidget(QWidget *parent = nullptr);
    ~ViewshedAnalysisWidget();

private slots:
    void on_pb_pickLocation_clicked();

    void on_pb_runORupdate_clicked();

    void on_sb_verticleAngle_valueChanged(double arg1);


    void on_sb_horizantalAngle_valueChanged(double arg1);

    void on_sb_longitude_valueChanged(double arg1);

    void on_sb_latitude_valueChanged(double arg1);

    void on_sb_altitude_valueChanged(double arg1);

    void on_sb_distance_valueChanged(double arg1);

    void on_sb_visibleAreaOpacity_valueChanged(int arg1);

    void on_sb_hiddenAreaOpacity_valueChanged(int arg1);

    void on_sb_boundarylineOpacity_valueChanged(int arg1);


    void on_sb_xRotation_valueChanged(double arg1);

    void on_sb_yRotation_valueChanged(double arg1);

    void on_sb_zRotation_valueChanged(double arg1);

private:
    Ui::ViewshedAnalysisWidget *ui;

    ViewshedAnalysis* viewShed =nullptr;

    osgEarth::GeoPoint _lightPos;

    osg::Vec4  visibleColor   = osg::Vec4(159.0f / 255.0f, 255.0f / 255.0f, 61.0f / 255.0f, 1.0f);
    osg::Vec4  inVisibleColor = osg::Vec4(255.0f / 255.0f, 87.0f / 255.0f, 61.0f / 255.0f, 1.0f);

};

#endif // VIEWSHEDANALYSISWIDGET_H
