#include "PolygonTool.h"

#include <QDebug>

void PolygonTool::activate()
{
	m_active = true;
	m_finished = false;
	m_points.clear();
}

void PolygonTool::deactivate()
{
	m_active = false;
}

void PolygonTool::mousePress(const osgEarth::GeoPoint& point, Qt::MouseButton button)
{
	if (!m_active || button != Qt::LeftButton) {
		return;
	}

	m_points.push_back(point);
}

void PolygonTool::mouseMove(const osgEarth::GeoPoint& point)
{
	Q_UNUSED(point);
}

void PolygonTool::mouseRelease(const osgEarth::GeoPoint& point, Qt::MouseButton button)
{
	Q_UNUSED(point);
	Q_UNUSED(button);
}

void PolygonTool::keyPress(QKeyEvent* event)
{
	if (event == nullptr) {
		return;
	}

	if (event->key() == Qt::Key_Escape) {
		cancel();
		return;
	}

	if (event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter) {
		if (m_points.size() >= 3) {
			qInfo() << "Polygon created with" << m_points.size() << "vertices";
			m_finished = true;
		}
	}
}

void PolygonTool::cancel()
{
	m_points.clear();
	m_finished = true;
}

bool PolygonTool::isFinished() const
{
	return m_finished;
}

DrawingTool::ShapeType PolygonTool::shapeType() const
{
	return ShapeType::Polygon;
}

std::vector<osgEarth::GeoPoint> PolygonTool::resultPoints() const
{
	return m_points;
}
