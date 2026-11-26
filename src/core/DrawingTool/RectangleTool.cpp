#include "RectangleTool.h"

#include <QDebug>

void RectangleTool::activate()
{
	m_active = true;
	m_finished = false;
	m_points.clear();
}

void RectangleTool::deactivate()
{
	m_active = false;
}

void RectangleTool::mousePress(const osgEarth::GeoPoint& point, Qt::MouseButton button)
{
	if (!m_active || button != Qt::LeftButton) {
		return;
	}

	m_points.push_back(point);

	if (m_points.size() == 2) {
		qInfo() << "Rectangle created from corners"
				<< m_points.front().x() << m_points.front().y()
				<< "and"
				<< m_points.back().x() << m_points.back().y();
		m_finished = true;
	}
}

void RectangleTool::mouseMove(const osgEarth::GeoPoint& point)
{
	Q_UNUSED(point);
}

void RectangleTool::mouseRelease(const osgEarth::GeoPoint& point, Qt::MouseButton button)
{
	Q_UNUSED(point);
	Q_UNUSED(button);
}

void RectangleTool::keyPress(QKeyEvent* event)
{
	if (event != nullptr && event->key() == Qt::Key_Escape) {
		cancel();
	}
}

void RectangleTool::cancel()
{
	m_points.clear();
	m_finished = true;
}

bool RectangleTool::isFinished() const
{
	return m_finished;
}

DrawingTool::ShapeType RectangleTool::shapeType() const
{
	return ShapeType::Rectangle;
}

std::vector<osgEarth::GeoPoint> RectangleTool::resultPoints() const
{
	return m_points;
}
