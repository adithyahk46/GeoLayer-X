#include "PointTool.h"

#include <QDebug>

void PointTool::activate()
{
	m_active = true;
	m_finished = false;
	m_points.clear();
}

void PointTool::deactivate()
{
	m_active = false;
}

void PointTool::mousePress(const osgEarth::GeoPoint& point, Qt::MouseButton button)
{
	if (!m_active || button != Qt::LeftButton) {
		return;
	}

	m_points.clear();
	m_points.push_back(point);
	qInfo() << "Point created at" << point.x() << point.y() << point.z();
	m_finished = true;
}

void PointTool::mouseMove(const osgEarth::GeoPoint& point)
{
	Q_UNUSED(point);
}

void PointTool::mouseRelease(const osgEarth::GeoPoint& point, Qt::MouseButton button)
{
	Q_UNUSED(point);
	Q_UNUSED(button);
}

void PointTool::keyPress(QKeyEvent* event)
{
	if (event != nullptr && event->key() == Qt::Key_Escape) {
		cancel();
	}
}

void PointTool::cancel()
{
	m_points.clear();
	m_finished = true;
}

bool PointTool::isFinished() const
{
	return m_finished;
}

DrawingTool::ShapeType PointTool::shapeType() const
{
	return ShapeType::Point;
}

std::vector<osgEarth::GeoPoint> PointTool::resultPoints() const
{
	return m_points;
}
