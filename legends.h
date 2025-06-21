#ifndef LEGENDS_H
#define LEGENDS_H
#include <QCheckBox>
#include <QWidget>
#include <QMouseEvent>
#include <QMenu>
#include <QDebug>

#include <osgEarth/MapNode>
#include <osgEarth/EarthManipulator>
#include <osgEarth/Style>
#include <osgEarth/Map>
#include <osgEarth/FeatureModelLayer>
#include <osgEarth/ElevationQuery>
#include <osgEarth/FeatureModelLayer>
#include <osgEarth/OGRFeatureSource>
#include <osgEarth/ElevationLayer>
#include <osgEarth/GDAL>
#include <osgEarth/GLUtils>


// Non-template base class to hold signals and Q_OBJECT
class LegendBase : public QCheckBox {
    Q_OBJECT
public:
    using QCheckBox::QCheckBox;

signals:
    void zoom_to_layer();
    void remove_layer();
    void move_to_top();
    void move_to_bottom();
};

// Template class inherits from LegendBase
template <typename T>
class Legends : public LegendBase {
public:
    explicit Legends(T* mapLayer = nullptr, osgEarth::MapNode* mapNode = nullptr, osgEarth::EarthManipulator* manip = nullptr, QWidget* parent = nullptr);
    // void setLayer(T* layer);,
    T* mapLayer() const { return m_mapLayer; }

protected:
    void mousePressEvent(QMouseEvent* event) override;

private:
    T* m_mapLayer;
    QMenu menu;

    osgEarth::EarthManipulator* _manip = nullptr;
    osgEarth::MapNode* _mapNode = nullptr;

    osgEarth::StyleSheet* styleSheet;
};

#endif // LEGENDS_H
