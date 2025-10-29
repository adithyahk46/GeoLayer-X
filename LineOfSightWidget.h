#ifndef LINEOFSIGHTWIDGET_H
#define LINEOFSIGHTWIDGET_H

#include <QWidget>
#include <osgEarth/MapNode>
#include <osgEarth/GeoData>
#include <osgEarth/FeatureNode>
#include <vector>
namespace Ui {
class LineOfSightWidget;
}

class LineOfSightWidget : public QWidget
{
    Q_OBJECT

public:
    explicit LineOfSightWidget(osgEarth::MapNode* mapNode, QWidget *parent = nullptr);
    ~LineOfSightWidget();

private slots:
    void on_pb_sourceLocation_clicked();
    void on_pb_targetLocation_clicked();
    void on_btnUpdate_clicked();
    void on_btnClear_clicked();
    void on_pb_close_clicked();
    void on_pb_visbleColor_clicked();
    void on_pb_invisibleColor_clicked();

private:
    Ui::LineOfSightWidget *ui;
    osgEarth::MapNode* _mapNode;

    osgEarth::GeoPoint start;
    osgEarth::GeoPoint end;

    osg::Vec4f goodColor = osg::Vec4f(0.0f, 1.0f, 0.0f, 1.0f); // Green
    osg::Vec4f badColor = osg::Vec4f(1.0f, 0.0f, 0.0f, 1.0f);  // Red

    osg::ref_ptr<osgEarth::FeatureNode> GoodNode = nullptr;
    osg::ref_ptr<osgEarth::FeatureNode> BadNode = nullptr;

    void drawLine(const osgEarth::GeoPoint &p1, const osgEarth::GeoPoint &p2, const osg::Vec4f &color, float width);
    std::vector<osg::ref_ptr<osgEarth::FeatureNode>> lineNodes ;
    void updateButtonStyles();
};

#endif
