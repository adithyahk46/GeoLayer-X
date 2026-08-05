#ifndef MAPCONTROLLER_H
#define MAPCONTROLLER_H

#include "osgearthcore_global.h"

#include <osgEarth/EarthManipulator>

namespace osgViewer
{
class Viewer;
}

namespace osgEarth
{
class MapNode;
class GeoExtent;
class Viewpoint;
}


class OSGEARTHCORE_EXPORT MapController : public osgEarth::EarthManipulator
{

public:
    explicit MapController();

    static bool isValid();

    // Camera
    static void zoomToFullExtent();

    static void zoomIn(double factor = 0.8);
    static void zoomOut(double factor = 1.25);

    static void flyTo(const osgEarth::Viewpoint& vp,
               double duration = 1.0);

     static void flyTo(double lon, double lat, double altitude, double range, 
        double heading = 0.0,double pitch = -90.0, double duration = 1.0);

    static void zoomToExtent(double duration = 1.5);

    // Camera state
    osgEarth::Viewpoint currentViewpoint() const;

};

#endif // MAPCONTROLLER_H
