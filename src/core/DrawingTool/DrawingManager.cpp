#include "DrawingManager.h"

#include <algorithm>
#include <cmath>

#include <QDebug>

#include <osg/Geometry>
#include <osg/Geode>
#include <osg/LineWidth>
#include <osg/Point>
#include <osg/StateSet>

namespace
{
osg::ref_ptr<osg::Geode> createRenderable(
	const std::vector<osg::Vec3d>& points,
	GLenum primitive,
	const osg::Vec4& color,
	float lineWidth,
	float pointSize)
{
	if (points.empty()) {
		return nullptr;
	}

	osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array;
	vertices->reserve(points.size());
	for (const auto& point : points) {
		vertices->push_back(point);
	}

	osg::ref_ptr<osg::Geometry> geometry = new osg::Geometry;
	geometry->setVertexArray(vertices);
	geometry->addPrimitiveSet(new osg::DrawArrays(primitive, 0, static_cast<GLsizei>(vertices->size())));

	osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
	colors->push_back(color);
	geometry->setColorArray(colors, osg::Array::BIND_OVERALL);

	osg::ref_ptr<osg::Geode> geode = new osg::Geode;
	geode->addDrawable(geometry);

	osg::StateSet* stateSet = geode->getOrCreateStateSet();
	stateSet->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

	if (primitive == GL_POINTS) {
		stateSet->setAttributeAndModes(new osg::Point(pointSize), osg::StateAttribute::ON);
	} else {
		stateSet->setAttributeAndModes(new osg::LineWidth(lineWidth), osg::StateAttribute::ON);
	}

	return geode;
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

	const std::vector<osg::Vec3d> worldPoints = toWorldPoints(geoPoints);
	if (worldPoints.empty()) {
		return;
	}

	osg::ref_ptr<osg::Geode> geode;
	switch (shapeType)
	{
		case DrawingTool::ShapeType::Point:
			geode = createRenderable(worldPoints, GL_POINTS, osg::Vec4(1.0f, 0.2f, 0.2f, 1.0f), 2.0f, 9.0f);
			break;

		case DrawingTool::ShapeType::Polyline:
			if (worldPoints.size() >= 2) {
				geode = createRenderable(worldPoints, GL_LINE_STRIP, osg::Vec4(1.0f, 0.95f, 0.15f, 1.0f), 3.0f, 1.0f);
			}
			break;

		case DrawingTool::ShapeType::Polygon:
		case DrawingTool::ShapeType::Rectangle:
		case DrawingTool::ShapeType::Circle:
			if (worldPoints.size() >= 3) {
				geode = createRenderable(worldPoints, GL_LINE_LOOP, osg::Vec4(0.2f, 1.0f, 0.35f, 1.0f), 3.0f, 1.0f);
			}
			break;
	}

	if (geode.valid()) {
		m_drawingGroup->addChild(geode);
	}
}

std::vector<osg::Vec3d> DrawingManager::toWorldPoints(const std::vector<osgEarth::GeoPoint>& points) const
{
	std::vector<osg::Vec3d> worldPoints;
	if (!m_mapNode.valid() || m_mapNode->getMapSRS() == nullptr) {
		return worldPoints;
	}

	const osgEarth::SpatialReference* mapSrs = m_mapNode->getMapSRS();
	worldPoints.reserve(points.size());

	for (const auto& sourcePoint : points) {
		osgEarth::GeoPoint mapPoint;
		if (sourcePoint.getSRS() == mapSrs) {
			mapPoint = sourcePoint;
		} else {
			sourcePoint.transform(mapSrs, mapPoint);
		}

		osg::Vec3d worldPoint;
		if (mapPoint.toWorld(worldPoint)) {
			worldPoints.push_back(worldPoint);
		}
	}

	return worldPoints;
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
