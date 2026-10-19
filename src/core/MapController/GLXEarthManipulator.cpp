#include "GLXEarthManipulator.h"

#include <cmath>
#include <osgUtil/LineSegmentIntersector>
#include <osgViewer/View>

GLXEarthManipulator::GLXEarthManipulator(osgEarth::MapNode* mapNode, QObject* parent)
    : QObject(parent),
    _mapNode(mapNode)
{
    qRegisterMetaType<osgEarth::GeoPoint>("osgEarth::GeoPoint");
}

void GLXEarthManipulator::setMapNode(osgEarth::MapNode* mapNode)
{
    _mapNode = mapNode;
}

bool GLXEarthManipulator::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
    const auto eventType = ea.getEventType();
    const bool needsGeoPoint =
        eventType == osgGA::GUIEventAdapter::PUSH ||
        eventType == osgGA::GUIEventAdapter::DOUBLECLICK ||
        eventType == osgGA::GUIEventAdapter::MOVE ||
        eventType == osgGA::GUIEventAdapter::DRAG ||
        eventType == osgGA::GUIEventAdapter::SCROLL;
    auto* view = needsGeoPoint ? dynamic_cast<osgViewer::View*>(&aa) : nullptr;
    const bool hasGeoPoint = view && getGeoPointFromScreen(view, ea.getX(), ea.getY());
    emit defaultHandleEvent(ea, aa);

    if (hasGeoPoint)
    {
        switch (eventType)
        {
        case osgGA::GUIEventAdapter::PUSH:
            emit mouseClickEvent(_currentGeoPos);
            break;
        case osgGA::GUIEventAdapter::DOUBLECLICK:
            emit mouseDoubleClickEvent(_currentGeoPos);
            break;
        case osgGA::GUIEventAdapter::MOVE:
        case osgGA::GUIEventAdapter::DRAG:
            emit mouseMoveEvent(_currentGeoPos);
            break;
        case osgGA::GUIEventAdapter::SCROLL:
            emit mouseScrollEvent(_currentGeoPos);
            break;
        default:
            break;
        }
    }

    const bool handledByEarthManipulator = _disableMouseAction
                                               ? false
                                               : osgEarth::EarthManipulator::handle(ea, aa);
    return _disableMouseAction || handledByEarthManipulator;
}

void GLXEarthManipulator::disableMouseAction(bool state)
{
    _disableMouseAction = state;
}

void GLXEarthManipulator::zoomToFullExtent()
{
    home(0.5);
}

void GLXEarthManipulator::zoomIn(double factor)
{
    zoomBy(factor);
}

void GLXEarthManipulator::zoomOut(double factor)
{
    zoomBy(factor);
}

void GLXEarthManipulator::zoomBy(double factor)
{
    if (!std::isfinite(factor) || factor <= 0.0)
        return;

    auto viewpoint = getViewpoint();
    if (!viewpoint.range().isSet())
        return;

    osgEarth::Distance range = *viewpoint.range();
    range = osgEarth::Distance(range.as(osgEarth::Units::METERS) * factor,
                               osgEarth::Units::METERS);
    viewpoint.range() = range;
    setViewpoint(viewpoint, 0.3);
}

void GLXEarthManipulator::flyTo(const osgEarth::Viewpoint& viewpoint, double duration)
{
    setViewpoint(viewpoint, duration);
}

void GLXEarthManipulator::flyTo(double lon,
                                double lat,
                                double altitude,
                                double range,
                                double heading,
                                double pitch,
                                double duration)
{
    flyTo(osgEarth::Viewpoint("", lon, lat, altitude, heading, pitch, range), duration);
}

void GLXEarthManipulator::zoomToExtent(osgEarth::GeoExtent extent, double duration)
{

}


osgEarth::Viewpoint GLXEarthManipulator::currentViewpoint() const
{
    return getViewpoint();
}

bool GLXEarthManipulator::getGeoPointFromScreen(osgViewer::View* view, float x, float y)
{
    if (!_mapNode.valid())
        return false;

    osgUtil::LineSegmentIntersector::Intersections intersections;
    if (!view->computeIntersections(x, y, intersections))
        return false;

    const auto& hit = *intersections.begin();
    _currentWorldPos = hit.getWorldIntersectPoint();
    _currentGeoPos.fromWorld(_mapNode->getMapSRS(), _currentWorldPos);
    return _currentGeoPos.isValid();
}