#ifndef LEGENDS_H
#define LEGENDS_H

#include <QCheckBox>
#include <QWidget>
#include <QDialog>
#include <QMouseEvent>
#include <QMenu>
#include <QDebug>
#include <QFormLayout>
#include <QLineEdit>
#include <QDoubleSpinBox>
#include <QDialogButtonBox>

#include <osg/ComputeBoundsVisitor>

#include <osgEarth/MapNode>
#include <osgEarth/EarthManipulator>
#include <osgEarth/Style>
#include <osgEarth/Map>
#include <osgEarth/Layer>
#include <osgEarth/GDAL>
#include <osgEarth/FeatureModelLayer>
#include <osgEarth/OGRFeatureSource>
#include <osgEarth/XYZ>
#include <osgEarth/AnnotationLayer>
#include <osgEarth/ModelLayer>



class Legends : public QCheckBox {
    Q_OBJECT
public:
    explicit Legends(osgEarth::Layer* mapLayer = nullptr, osgEarth::MapNode* mapNode = nullptr,
                     osgEarth::EarthManipulator* manip = nullptr, QWidget* parent = nullptr);
    ~Legends();

    void modelLayerPropertyDialog(osgEarth::ModelLayer* modelLayer,const osgEarth::SpatialReference* srs);

protected:
    void mousePressEvent(QMouseEvent* event) override;

private:
    osgEarth::Layer* m_mapLayer = nullptr;
    osgEarth::EarthManipulator* _manip = nullptr;
    osgEarth::MapNode* _mapNode = nullptr;
    QMenu menu;
};

#endif // LEGENDS_H
