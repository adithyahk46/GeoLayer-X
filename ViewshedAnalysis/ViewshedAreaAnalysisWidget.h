#ifndef VIEWSHEDAREAANALYSISWIDGET_H
#define VIEWSHEDAREAANALYSISWIDGET_H

#include <QDialog>
#include <osgEarth/MapNode>
#include <osgViewer/Viewer>



#include "VisibilityTestArea/VisibilityTestArea.h"


namespace Ui {
class ViewshedAreaAnalysisWidget;
}

class ViewshedAreaAnalysisWidget : public QDialog
{
    Q_OBJECT

public:
    explicit ViewshedAreaAnalysisWidget(osgEarth::MapNode* mapNode, osgViewer::Viewer *viewer,osg::Group* root,QWidget *parent = nullptr);
    ~ViewshedAreaAnalysisWidget();

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

private:
    Ui::ViewshedAreaAnalysisWidget *ui;
    osgEarth::MapNode* _mapNode = nullptr;
    osgViewer::Viewer* _viewer = nullptr;
    osg::Group* _root = nullptr;
    VisibilityTestArea* viewShed =nullptr;

    osg::ref_ptr<osg::Group> _shadowSceneparent = nullptr;
    osg::ref_ptr<osg::Group>_shadowScene = nullptr;




    osg::Vec3 geoPointsToVev3(double lon, double lat, double alt);
};



#include <QCheckBox>
#include <QColor>

class ColorPickerCheckBox : public QCheckBox
{
    Q_OBJECT

public:
    explicit ColorPickerCheckBox(const QColor& color,
                                 QWidget* parent = nullptr);

    QColor color() const;

signals:
    void colorChanged(const QColor& color);

private slots:
    void onClicked();

private:

    QColor m_color;

    void updateAppearance();
};


#endif // VIEWSHEDAREAANALYSISWIDGET_H
