#include "DrawTool.h"

using namespace osgEarth;

DrawTool::DrawTool(QObject* parent)
    : QObject(parent)
{
    _style.getOrCreate<osgEarth::PolygonSymbol>()->fill().mutable_value().color() = osgEarth::Color(1.0, 0.0, 0.0, 0.5); // Red fill with 50% opacity

    _style.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;
    _style.getOrCreate<AltitudeSymbol>()->technique() = AltitudeSymbol::TECHNIQUE_DRAPE;


    _style.getOrCreate<osgEarth::LineSymbol>()->stroke()->color() = osgEarth::Color(0.0, 0.0, 1.0, 1.0); // Blue stroke with full opacity
    _style.getOrCreate<osgEarth::LineSymbol>()->stroke()->width() = osgEarth::Distance(3.0f, osgEarth::Units::PIXELS); // Stroke width of 2.0
    _style.getOrCreate<osgEarth::LineSymbol>()->tessellationSize() = osgEarth::Distance(75000, osgEarth::Units::METERS);

    _style.getOrCreate<osgEarth::RenderSymbol>()->depthOffset()->range() = osgEarth::Distance(1.0, osgEarth::Units::KILOMETERS);

}

DrawTool::~DrawTool()
{
}

// void DrawTool::pickPoint(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
// {
//     // Implement the logic to handle the picked point
//     // For example, you can emit a signal or store the point for further processing
// }