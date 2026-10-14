#ifndef RADIALVIEWSHEDANALYSISWIDGET_H
#define RADIALVIEWSHEDANALYSISWIDGET_H

#include <QDialog>

#include "RadialViewshedAnalysis.h"
#include <osgEarth/GeoData>

namespace Ui {
class RadialViewshedAnalysisWidget;
}

class RadialViewshedAnalysisWidget : public QDialog
{
    Q_OBJECT

public:
    explicit RadialViewshedAnalysisWidget(QWidget *parent = nullptr);
    ~RadialViewshedAnalysisWidget();

private slots:
    void on_pb_pickLocation_clicked();

    void on_pb_createORupdate_clicked();

    void on_sb_longitude_valueChanged(double arg1);

    void on_sb_latitude_valueChanged(double arg1);

    void on_sb_altitude_valueChanged(double arg1);

    void on_sb_distance_valueChanged(double arg1);

    void on_sb_visibleAreaOpacity_valueChanged(int arg1);

    void on_sb_hiddenAreaOpacity_valueChanged(int arg1);

private:
    Ui::RadialViewshedAnalysisWidget *ui;

    RadialViewshedAnalysis* viewShed =nullptr;

    osgEarth::GeoPoint _lightPos;

    osg::Vec4  visibleColor   = osg::Vec4(159.0f / 255.0f, 255.0f / 255.0f, 61.0f / 255.0f, 1.0f);
    osg::Vec4  inVisibleColor = osg::Vec4(255.0f / 255.0f, 87.0f / 255.0f, 61.0f / 255.0f, 1.0f);

};

#endif // RADIALVIEWSHEDANALYSISWIDGET_H
