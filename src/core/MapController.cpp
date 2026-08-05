#include "MapController.h"


#include "MapController.h"
#include "osgviewerwidget.h"

#include <osgViewer/Viewer>

#include <osgEarth/MapNode>
#include <osgEarth/Viewpoint>
#include <osgEarth/Geodata>


#include <osgViewer/Viewer>

MapController::MapController()
{
}

bool MapController::isValid()
{
    return OSGViewerWidget::getViewer() && OSGViewerWidget::getManipulator() ;
}

void MapController::zoomToFullExtent()
{
    if (!isValid())
        return;

    OSGViewerWidget::getManipulator()->home(0.5);
}

void MapController::zoomIn(double factor)
{
    if (!isValid())
        return;

    auto vp = OSGViewerWidget::getManipulator()->getViewpoint();

    if (vp.range().isSet())
    {
        osgEarth::Distance range = *vp.range();
        range = osgEarth::Distance(range.as(osgEarth::Units::METERS) * factor,
                                   osgEarth::Units::METERS);

        vp.range() = range;
    }

    OSGViewerWidget::getManipulator()->setViewpoint(vp, 0.3);
}

void MapController::zoomOut(double factor)
{
    if (!isValid())
        return;

    auto vp = OSGViewerWidget::getManipulator()->getViewpoint();

    if (vp.range().isSet())
    {
        osgEarth::Distance range = *vp.range();
        range = osgEarth::Distance(range.as(osgEarth::Units::METERS) * factor,
                                   osgEarth::Units::METERS);

        vp.range() = range;
    }

    OSGViewerWidget::getManipulator()->setViewpoint(vp, 0.3);
}

void MapController::flyTo(const osgEarth::Viewpoint& vp,
                          double duration)
{
    if (!isValid())
        return;

    OSGViewerWidget::getManipulator()->setViewpoint(vp, duration);
}

void MapController::flyTo(double lon,
                          double lat,
                          double altitude,
                          double range,
                          double heading,
                          double pitch,
                          double duration)
{
    flyTo(osgEarth::Viewpoint(
              "",
              lon,
              lat,
              altitude,
              heading,
              pitch,
              range),
          duration);
}

void MapController::zoomToExtent(double duration)
{
    if (!isValid())
        return;

    //allow user to select a rectangle on the map and zoom to that area using mouse drag and drop
    // //write a code to allow user to select a rectangle on the map and zoom to that area using mouse drag and drop

    // osgEarth::GeoPoint center = extent.getCentroid();

    // double lon = center.x();
    // double lat = center.y();

    // double range = std::max(extent.width(), extent.height()) * 120000.0;

    // osgEarth::Viewpoint vp(
    //     "Extent",
    //     lon,
    //     lat,
    //     0.0,
    //     0.0,
    //     -90.0,
    //     range);

    // OSGViewerWidget::getManipulator()->setViewpoint(vp, duration);
}

osgEarth::Viewpoint MapController::currentViewpoint() const
{
    return OSGViewerWidget::getManipulator()->getViewpoint();
}