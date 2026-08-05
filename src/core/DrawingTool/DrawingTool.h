#ifndef DRAWINGTOOL_H
#define DRAWINGTOOL_H

#include <vector>

#include <QKeyEvent>
#include <Qt>

#include <osgEarth/GeoData>

#include "osgearthcore_global.h"

class OSGEARTHCORE_EXPORT DrawingTool
{
public:
    enum class ShapeType
    {
        Point,
        Polyline,
        Polygon,
        Rectangle,
        Circle
    };

    virtual ~DrawingTool() = default;

    virtual void activate() = 0;
    virtual void deactivate() = 0;

    virtual void mousePress(
        const osgEarth::GeoPoint& point,
        Qt::MouseButton button) = 0;

    virtual void mouseMove(
        const osgEarth::GeoPoint& point) = 0;

    virtual void mouseRelease(
        const osgEarth::GeoPoint& point,
        Qt::MouseButton button) = 0;

    virtual void keyPress(QKeyEvent* event) = 0;

    virtual void cancel() = 0;

    virtual bool isFinished() const = 0;

    virtual ShapeType shapeType() const = 0;
    virtual std::vector<osgEarth::GeoPoint> resultPoints() const = 0;
};

#endif // DRAWINGTOOL_H