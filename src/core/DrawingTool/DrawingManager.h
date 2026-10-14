#ifndef DRAWINGMANAGER_H
#define DRAWINGMANAGER_H

#include <vector>
#include <memory>

#include <QObject>

#include <osg/Group>
#include <osg/Vec3d>
#include <osg/ref_ptr>
#include <osgEarth/GeoData>
#include <osgEarth/MapNode>

#include "DrawingTool.h"
#include "FeatureStore.h"
#include "osgearthcore_global.h"

class OSGEARTHCORE_EXPORT DrawingManager : public QObject
{
    Q_OBJECT

public:
    explicit DrawingManager(
        osgEarth::MapNode* mapNode,
        QObject* parent = nullptr);

    void setTool(std::unique_ptr<DrawingTool> tool);

    void mousePress(const osgEarth::GeoPoint& point, Qt::MouseButton button);
    void mouseMove(const osgEarth::GeoPoint& point);
    void mouseRelease(const osgEarth::GeoPoint& point, Qt::MouseButton button);
    void keyPress(QKeyEvent* event);

    void cancel();
    void finish();

    void undo();
    void redo();
    void clear();

    bool hasActiveTool() const;

    FeatureStore* featureStore();

private:
    void commitActiveToolGeometry();
    std::vector<osg::Vec3d> toWorldPoints(const std::vector<osgEarth::GeoPoint>& points) const;
    std::vector<osgEarth::GeoPoint> rectanglePoints(const std::vector<osgEarth::GeoPoint>& points) const;
    std::vector<osgEarth::GeoPoint> circlePoints(const std::vector<osgEarth::GeoPoint>& points) const;

    osg::ref_ptr<osgEarth::MapNode> m_mapNode;
    osg::ref_ptr<osg::Group> m_drawingGroup;
    std::unique_ptr<DrawingTool> m_activeTool;
    FeatureStore m_featureStore;
};

#endif // DRAWINGMANAGER_H