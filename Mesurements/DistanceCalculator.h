#ifndef DISTANCECALCULATOR_H
#define DISTANCECALCULATOR_H

#include <QWidget>
#include <osgGA/GUIEventHandler>
#include <osgEarth/MapNode>
#include <osgEarth/FeatureNode>
#include <osgEarth/Geometry>
#include <osgEarth/Style>
#include <vector>


namespace Ui {
class DistanceCalculator;
}

class DistanceCalculator: public QObject
{
    Q_OBJECT

public:
    explicit DistanceCalculator(osgEarth::MapNode* mapNode);
    ~DistanceCalculator();

signals:
    void DrawPolygonFinished(osg::ref_ptr<osgEarth::FeatureNode>);

private slots:
    void mouseClickEvent(const double lon, const double lat,const double alt,const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
    void mouseDoubleClickEvent(const double lon, const double lat,const double alt,const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
    void mouseMoveEvent(const double lon, const double lat,const double alt,const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);


private:
    Ui::DistanceCalculator *ui;

    osgEarth::MapNode* _mapNode = nullptr;
    osg::ref_ptr<osgEarth::Geometry> _geom;
    osgEarth::Style _lineStyle;
    osg::ref_ptr<osgEarth::Feature> feature;
    osg::ref_ptr<osgEarth::FeatureNode> _previewNode;

    // osg::ref_ptr<osgEarth::LabelNode> _distanceLabel;


    bool _isDrawing;
};

#endif // DISTANCECALCULATOR_H
