#include "LineTool.h"

#include <QDebug>

void LineTool::activate()
{
	m_active = true;
	m_finished = false;
	m_points.clear();
}

void LineTool::deactivate()
{
	m_active = false;
}

void LineTool::mousePress(const osgEarth::GeoPoint& point, Qt::MouseButton button)
{
	if (!m_active || button != Qt::LeftButton) {
		return;
	}

	m_points.push_back(point);

	if (m_points.size() == 2) {
		qInfo() << "Line created from"
				<< m_points.front().x() << m_points.front().y()
				<< "to"
				<< m_points.back().x() << m_points.back().y();
		m_finished = true;
	}
}

void LineTool::mouseMove(const osgEarth::GeoPoint& point)
{
	Q_UNUSED(point);
}

void LineTool::mouseRelease(const osgEarth::GeoPoint& point, Qt::MouseButton button)
{
	Q_UNUSED(point);
	Q_UNUSED(button);
}

void LineTool::keyPress(QKeyEvent* event)
{
	if (event != nullptr && event->key() == Qt::Key_Escape) {
		cancel();
	}
}

void LineTool::cancel()
{
	m_points.clear();
	m_finished = true;
}

bool LineTool::isFinished() const
{
	return m_finished;
}

DrawingTool::ShapeType LineTool::shapeType() const
{
	return ShapeType::Polyline;
}

std::vector<osgEarth::GeoPoint> LineTool::resultPoints() const
{
	return m_points;
}
