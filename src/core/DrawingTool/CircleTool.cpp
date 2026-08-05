#include "CircleTool.h"

#include <QDebug>

void CircleTool::activate()
{
	m_active = true;
	m_finished = false;
	m_points.clear();
}

void CircleTool::deactivate()
{
	m_active = false;
}

void CircleTool::mousePress(const osgEarth::GeoPoint& point, Qt::MouseButton button)
{
	if (!m_active || button != Qt::LeftButton) {
		return;
	}

	m_points.push_back(point);

	if (m_points.size() == 2) {
		qInfo() << "Circle created with center"
				<< m_points.front().x() << m_points.front().y()
				<< "and edge point"
				<< m_points.back().x() << m_points.back().y();
		m_finished = true;
	}
}

void CircleTool::mouseMove(const osgEarth::GeoPoint& point)
{
	Q_UNUSED(point);
}

void CircleTool::mouseRelease(const osgEarth::GeoPoint& point, Qt::MouseButton button)
{
	Q_UNUSED(point);
	Q_UNUSED(button);
}

void CircleTool::keyPress(QKeyEvent* event)
{
	if (event != nullptr && event->key() == Qt::Key_Escape) {
		cancel();
	}
}

void CircleTool::cancel()
{
	m_points.clear();
	m_finished = true;
}

bool CircleTool::isFinished() const
{
	return m_finished;
}

DrawingTool::ShapeType CircleTool::shapeType() const
{
	return ShapeType::Circle;
}

std::vector<osgEarth::GeoPoint> CircleTool::resultPoints() const
{
	return m_points;
}
