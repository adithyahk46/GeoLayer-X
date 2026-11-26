#ifndef LINETOOL_H
#define LINETOOL_H

#include <vector>

#include "DrawingTool.h"
#include "osgearthcore_global.h"

class OSGEARTHCORE_EXPORT LineTool : public DrawingTool
{
public:
	void activate() override;
	void deactivate() override;

	void mousePress(const osgEarth::GeoPoint& point, Qt::MouseButton button) override;
	void mouseMove(const osgEarth::GeoPoint& point) override;
	void mouseRelease(const osgEarth::GeoPoint& point, Qt::MouseButton button) override;
	void keyPress(QKeyEvent* event) override;

	void cancel() override;
	bool isFinished() const override;
	ShapeType shapeType() const override;
	std::vector<osgEarth::GeoPoint> resultPoints() const override;

private:
	bool m_active = false;
	bool m_finished = false;
	std::vector<osgEarth::GeoPoint> m_points;
};

#endif // LINETOOL_H
