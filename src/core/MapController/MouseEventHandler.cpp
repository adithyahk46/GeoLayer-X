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
                emit mouseClickEvent(_currentGeoPos);
            break;

            case osgGA::GUIEventAdapter::DOUBLECLICK:
                emit mouseDoubleClickEvent(_currentGeoPos);
                // return true;
            break;

            case osgGA::GUIEventAdapter::MOVE:
                emit mouseMoveEvent(_currentGeoPos);
            break;

            case osgGA::GUIEventAdapter::DRAG:
                emit mouseMoveEvent(_currentGeoPos);
                // return true;
            break;

            case osgGA::GUIEventAdapter::SCROLL:
                emit mouseScrollEvent(_currentGeoPos);

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
        // osg::Vec3d world = hit.getWorldIntersectPoint();

        _nearestIntesection = hit;
        _currentLocalPos    = _nearestIntesection.getLocalIntersectPoint();
        _currentWorldPos    = _nearestIntesection.getWorldIntersectPoint();

        //replace null if its not correct

        osgEarth::GeoPoint mapPoint;
        mapPoint.fromWorld(_mapNode->getMapSRS(), _currentWorldPos);

        const osgEarth::SpatialReference* srs4326 = osgEarth::SpatialReference::get("epsg:4326");

        // osgEarth::GeoPoint transformed;
        mapPoint.transform(_mapNode->getMapSRS(), _currentGeoPos);

         return true;
    }
    return false;
}