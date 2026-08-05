#include "DrawingManager.h"

#include <algorithm>
#include <cmath>

#include <QDebug>

#include <osgEarth/Feature>
#include <osgEarth/FeatureNode>
#include <osgEarth/Style>

namespace
{
osgEarth::Style styleForShape(DrawingTool::ShapeType shapeType)
{
	osgEarth::Style style;

	auto* altitude = style.getOrCreate<osgEarth::AltitudeSymbol>();
	altitude->clamping() = osgEarth::AltitudeSymbol::CLAMP_TO_TERRAIN;
	altitude->technique() = osgEarth::AltitudeSymbol::TECHNIQUE_DRAPE;

	auto* render = style.getOrCreate<osgEarth::RenderSymbol>();
	render->lighting() = false;
	render->depthOffset()->enabled() = true;
	render->depthOffset().mutable_value().automatic() = true;

	switch (shapeType)
	{
		case DrawingTool::ShapeType::Point:
		{
			auto* point = style.getOrCreate<osgEarth::PointSymbol>();
			point->fill() = osgEarth::Color(1.0f, 0.2f, 0.2f, 1.0f);
			point->size() = 9.0f;
			point->smooth() = true;
			break;
		}

		case DrawingTool::ShapeType::Polyline:
		{
			auto* line = style.getOrCreate<osgEarth::LineSymbol>();
			line->stroke().mutable_value().color() = osgEarth::Color(1.0f, 0.95f, 0.15f, 1.0f);
			line->stroke().mutable_value().width() = osgEarth::Distance(3.0f, osgEarth::Units::PIXELS);
			line->tessellationSize() = osgEarth::Distance(75000.0, osgEarth::Units::METERS);
			break;
		}

		case DrawingTool::ShapeType::Polygon:
		case DrawingTool::ShapeType::Rectangle:
		case DrawingTool::ShapeType::Circle:
		{
			auto* poly = style.getOrCreate<osgEarth::PolygonSymbol>();
			poly->fill()->color() = osgEarth::Color(0.2f, 1.0f, 0.35f, 0.35f);

			auto* line = style.getOrCreate<osgEarth::LineSymbol>();
			line->stroke().mutable_value().color() = osgEarth::Color(0.2f, 1.0f, 0.35f, 1.0f);
			line->stroke().mutable_value().width() = osgEarth::Distance(3.0f, osgEarth::Units::PIXELS);
			line->tessellationSize() = osgEarth::Distance(75000.0, osgEarth::Units::METERS);
			break;
		}
	}

	return style;
}
}

DrawingManager::DrawingManager(osgEarth::MapNode* mapNode, QObject* parent)
	: QObject(parent)
	, m_mapNode(mapNode)
	, m_drawingGroup(new osg::Group)
{
	if (m_mapNode.valid()) {
		m_drawingGroup->setName("DrawingGroup");
		m_mapNode->addChild(m_drawingGroup);
	}
}

void DrawingManager::setTool(std::unique_ptr<DrawingTool> tool)
{
	if (m_activeTool) {
		m_activeTool->deactivate();
	}

	m_activeTool = std::move(tool);

	if (m_activeTool) {
		m_activeTool->activate();
	}
}

void DrawingManager::mousePress(const osgEarth::GeoPoint& point, Qt::MouseButton button)
{
	if (!m_activeTool) {
		return;
	}

	m_activeTool->mousePress(point, button);

	if (m_activeTool->isFinished()) {
		commitActiveToolGeometry();
		finish();
	}
}

void DrawingManager::mouseMove(const osgEarth::GeoPoint& point)
{
	if (!m_activeTool) {
		return;
	}

	m_activeTool->mouseMove(point);
}

void DrawingManager::mouseRelease(const osgEarth::GeoPoint& point, Qt::MouseButton button)
{
	if (!m_activeTool) {
		return;
	}

	m_activeTool->mouseRelease(point, button);

	if (m_activeTool->isFinished()) {
		commitActiveToolGeometry();
		finish();
	}
}

void DrawingManager::keyPress(QKeyEvent* event)
{
	if (!m_activeTool || event == nullptr) {
		return;
	}

	m_activeTool->keyPress(event);

	if (m_activeTool->isFinished()) {
		commitActiveToolGeometry();
		finish();
	}
}

void DrawingManager::cancel()
{
	if (!m_activeTool) {
		return;
	}

	m_activeTool->cancel();
	m_activeTool->deactivate();
	m_activeTool.reset();
}

void DrawingManager::finish()
{
	if (!m_activeTool) {
		return;
	}

	m_activeTool->deactivate();
	m_activeTool.reset();
}

void DrawingManager::undo()
{
	qInfo() << "Drawing undo is not implemented yet.";
}

void DrawingManager::redo()
{
	qInfo() << "Drawing redo is not implemented yet.";
}

void DrawingManager::clear()
{
	cancel();
	if (m_drawingGroup.valid()) {
		m_drawingGroup->removeChildren(0, m_drawingGroup->getNumChildren());
	}
	m_featureStore.clear();
}

bool DrawingManager::hasActiveTool() const
{
	return static_cast<bool>(m_activeTool);
}

FeatureStore* DrawingManager::featureStore()
{
	return &m_featureStore;
}

void DrawingManager::commitActiveToolGeometry()
{
	if (!m_activeTool || !m_drawingGroup.valid() || !m_mapNode.valid()) {
		return;
	}

	std::vector<osgEarth::GeoPoint> geoPoints = m_activeTool->resultPoints();
	if (geoPoints.empty()) {
		return;
	}

	const DrawingTool::ShapeType shapeType = m_activeTool->shapeType();
	if (shapeType == DrawingTool::ShapeType::Rectangle) {
		geoPoints = rectanglePoints(geoPoints);
	}
	if (shapeType == DrawingTool::ShapeType::Circle) {
		geoPoints = circlePoints(geoPoints);
	}

	if (geoPoints.empty()) {
		return;
	}

	const osgEarth::SpatialReference* mapSrs = m_mapNode->getMapSRS();
	if (mapSrs == nullptr) {
		return;
	}

	osg::ref_ptr<osgEarth::Geometry> geometry;
	switch (shapeType)
	{
		case DrawingTool::ShapeType::Point:
			if (!geoPoints.empty()) {
				auto* pointSet = new osgEarth::PointSet();
				pointSet->push_back(osg::Vec3d(geoPoints.front().x(), geoPoints.front().y(), geoPoints.front().z()));
				geometry = pointSet;
			}
			break;

		case DrawingTool::ShapeType::Polyline:
			if (geoPoints.size() >= 2) {
				auto* line = new osgEarth::LineString();
				for (const auto& point : geoPoints) {
					line->push_back(osg::Vec3d(point.x(), point.y(), point.z()));
				}
				geometry = line;
			}
			break;

		case DrawingTool::ShapeType::Polygon:
		case DrawingTool::ShapeType::Rectangle:
		case DrawingTool::ShapeType::Circle:
			if (geoPoints.size() >= 3) {
				auto* polygon = new osgEarth::Polygon();
				for (const auto& point : geoPoints) {
					polygon->push_back(osg::Vec3d(point.x(), point.y(), point.z()));
				}
				geometry = polygon;
			}
			break;
	}

	if (!geometry.valid()) {
		return;
	}

	osg::ref_ptr<osgEarth::Feature> feature = new osgEarth::Feature(geometry, mapSrs);
	feature->geoInterp() = osgEarth::GEOINTERP_RHUMB_LINE;

	osgEarth::Style style = styleForShape(shapeType);
	osg::ref_ptr<osgEarth::FeatureNode> featureNode = new osgEarth::FeatureNode(feature, style);

	m_drawingGroup->addChild(featureNode);
	m_featureStore.addFeature(feature);
}

std::vector<osgEarth::GeoPoint> DrawingManager::rectanglePoints(const std::vector<osgEarth::GeoPoint>& points) const
{
	if (!m_mapNode.valid() || points.size() < 2 || m_mapNode->getMapSRS() == nullptr) {
		return points;
	}

	const osgEarth::SpatialReference* mapSrs = m_mapNode->getMapSRS();
	osgEarth::GeoPoint p1;
	osgEarth::GeoPoint p2;

	if (points[0].getSRS() == mapSrs) {
		p1 = points[0];
	} else {
		points[0].transform(mapSrs, p1);
	}

	if (points[1].getSRS() == mapSrs) {
		p2 = points[1];
	} else {
		points[1].transform(mapSrs, p2);
	}

	const double xMin = std::min(p1.x(), p2.x());
	const double xMax = std::max(p1.x(), p2.x());
	const double yMin = std::min(p1.y(), p2.y());
	const double yMax = std::max(p1.y(), p2.y());
	const double z = p1.z();

	std::vector<osgEarth::GeoPoint> rect;
	rect.reserve(4);
	rect.emplace_back(mapSrs, xMin, yMin, z, osgEarth::ALTMODE_ABSOLUTE);
	rect.emplace_back(mapSrs, xMax, yMin, z, osgEarth::ALTMODE_ABSOLUTE);
	rect.emplace_back(mapSrs, xMax, yMax, z, osgEarth::ALTMODE_ABSOLUTE);
	rect.emplace_back(mapSrs, xMin, yMax, z, osgEarth::ALTMODE_ABSOLUTE);

	return rect;
}

std::vector<osgEarth::GeoPoint> DrawingManager::circlePoints(const std::vector<osgEarth::GeoPoint>& points) const
{
	if (!m_mapNode.valid() || points.size() < 2 || m_mapNode->getMapSRS() == nullptr) {
		return points;
	}

	const osgEarth::SpatialReference* mapSrs = m_mapNode->getMapSRS();
	osgEarth::GeoPoint center;
	osgEarth::GeoPoint edge;

	if (points[0].getSRS() == mapSrs) {
		center = points[0];
	} else {
		points[0].transform(mapSrs, center);
	}

	if (points[1].getSRS() == mapSrs) {
		edge = points[1];
	} else {
		points[1].transform(mapSrs, edge);
	}

	const double radius = std::hypot(edge.x() - center.x(), edge.y() - center.y());
	if (radius <= 0.0) {
		return {};
	}

	constexpr int segmentCount = 64;
	constexpr double twoPi = 6.283185307179586;

	std::vector<osgEarth::GeoPoint> circle;
	circle.reserve(segmentCount);

	for (int i = 0; i < segmentCount; ++i) {
		const double angle = (twoPi * static_cast<double>(i)) / static_cast<double>(segmentCount);
		const double x = center.x() + (radius * std::cos(angle));
		const double y = center.y() + (radius * std::sin(angle));
		circle.emplace_back(mapSrs, x, y, center.z(), osgEarth::ALTMODE_ABSOLUTE);
	}

	return circle;
}
