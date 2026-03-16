#include "MouseEventHandler.h"

static MouseEventHandler* mouse = nullptr;


MouseEventHandler* MouseEventHandler::Instance(osgEarth::MapNode* mapNode){

    if(!mouse){
        mouse = new MouseEventHandler(mapNode);
    }

    return mouse;

}
MouseEventHandler::MouseEventHandler(osgEarth::MapNode* mapNode)
    : _mapNode(mapNode)
{


}


bool MouseEventHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{

    if (!view)
        view = dynamic_cast<osgViewer::View*>(&aa);

    if (getLatLonFromScreen(view, ea.getX(), ea.getY(), lat, lon, alt))
    {
        switch (ea.getEventType())
        {
            case osgGA::GUIEventAdapter::PUSH:
                emit mouseClickEvent(lon, lat, alt,ea,aa);
            break;

            case osgGA::GUIEventAdapter::DOUBLECLICK:
                emit mouseDoubleClickEvent(lon, lat, alt,ea,aa);
                // return true;
            break;

            case osgGA::GUIEventAdapter::MOVE:
                emit mouseMoveEvent(lon, lat, alt,ea,aa);
            break;

            case osgGA::GUIEventAdapter::DRAG:
                emit mouseMoveEvent(lon, lat, alt,ea,aa);
                // return true;
            break;

            case osgGA::GUIEventAdapter::SCROLL:
                emit mouseScrollEvent(lon, lat, alt,ea,aa);

            default:
                break;
        }
    }

        return _disableMouseAction;

    // return false;
}

void MouseEventHandler::disableMouseAction(bool state)
{
_disableMouseAction = state;
}

bool MouseEventHandler::getLatLonFromScreen(osgViewer::View* view, float x, float y, double& lat, double& lon, double& height)
{
    osgUtil::LineSegmentIntersector::Intersections intersections;
    if (view->computeIntersections(x, y, intersections))
    {
        auto hit = *intersections.begin();
        osg::Vec3d world = hit.getWorldIntersectPoint();

        osgEarth::GeoPoint mapPoint;
        mapPoint.fromWorld(_mapNode->getMapSRS(), world);

        const osgEarth::SpatialReference* srs4326 = osgEarth::SpatialReference::get("epsg:4326");

        osgEarth::GeoPoint transformed;

            mapPoint.transform(srs4326, transformed);

        lon = transformed.x();
        lat = transformed.y();
        alt = transformed.z();

         return true;
    }
    return false;
}
