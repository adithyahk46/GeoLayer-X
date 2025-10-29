#include <Windows.h>
// #include "common.h"
#include "DrawTool.h"
#include <osg/Math>
#include <osgUtil/LineSegmentIntersector>
#include <osgEarth/TextSymbol>
#include <osgEarth/IconSymbol>

#include <QCoreApplication>

DrawTool::DrawTool(osgEarth::MapNode* mapNode, osg::Group* drawGroup)
	: _mapNode(mapNode)
	, _drawGroup(drawGroup)
	, _active(true)
	, _dbClick(false)
	, _intersectionMask(0x1)
	, _tmpGroup(new osg::Group)
{
    QString figPath = QCoreApplication::applicationDirPath() + "/Data/placemark32.png";

    _pnStyle.getOrCreate<osgEarth::IconSymbol>()->url()->setLiteral(figPath.toStdString());
    _pnStyle.getOrCreate<osgEarth::TextSymbol>()->size() = 14;
	_mapNode->addChild(_tmpGroup);
}

bool DrawTool::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
	if (!_active)
		return false;

	_view = static_cast<osgViewer::View*>(aa.asView());

	const osgGA::GUIEventAdapter::EventType eventType = ea.getEventType();

	if (eventType == osgGA::GUIEventAdapter::KEYDOWN == eventType && ea.getKey() == osgGA::GUIEventAdapter::KEY_Escape) {
		resetDraw();
	}

	switch (ea.getEventType()) {
		// 鼠标点击
	case osgGA::GUIEventAdapter::PUSH: {
		if (ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON) {
			_mouseDownX = ea.getX();
			_mouseDownY = ea.getY();
		}
		break;
	}
	// 鼠标移动
	case osgGA::GUIEventAdapter::MOVE: {
		osg::Vec3d pos;
		getLocationAt(_view, ea.getX(), ea.getY(), pos.x(), pos.y(), pos.z());
		std::string coord = osgEarth::Stringify() << pos.x() << " " << pos.y() << " " << pos.z();
		if (_coordPn.valid()) {
			_coordPn->setPosition(osgEarth::GeoPoint::GeoPoint(getMapNode()->getMapSRS(), pos));
			_coordPn->setText(coord);
		}
		moveDraw(pos);
		aa.requestRedraw();
		break;
	}
	 // 鼠标释放
	case osgGA::GUIEventAdapter::RELEASE: {
		osg::Vec3d pos;
		getLocationAt(_view, ea.getX(), ea.getY(), pos.x(), pos.y(), pos.z());
		float eps = 1.0f;

		if (ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON) {
			if (osg::equivalent(ea.getX(), _mouseDownX, eps) && osg::equivalent(ea.getY(), _mouseDownY, eps)) {
				if (!_coordPn.valid()) {
					std::string coord = osgEarth::Stringify() << pos.x() << " " << pos.y() << " " << pos.z();
                    _coordPn = new osgEarth::PlaceNode(osgEarth::GeoPoint(getMapNode()->getMapSRS(), pos), coord, _pnStyle);
					_tmpGroup->addChild(_coordPn);
				}
				beginDraw(pos);
				aa.requestRedraw();
			}
		}
		else if (ea.getButton() == osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON) {
			if (_coordPn.valid()) {
				_tmpGroup->removeChild(_coordPn);
				_coordPn = NULL;
			}
			resetDraw();
			aa.requestRedraw();
		}
		break;
	}
	}

	return false;
}

void DrawTool::drawCommand(osg::Node *node)
{
	_drawGroup->addChild(node);
}

void DrawTool::drawCommand(const osg::NodeList &nodes)
{
	for (auto& node : nodes)
		_drawGroup->addChild(node);
}

#include <osgEarth/SpatialReference>
bool DrawTool::getLocationAt(osgViewer::View* view, double x, double y, double& lon, double& lat, double& alt)
{
    osgUtil::LineSegmentIntersector::Intersections results;
    if (getMapNode() && view->computeIntersections(x, y, results, _intersectionMask))
    {
        osgUtil::LineSegmentIntersector::Intersection first = *(results.begin());
        osg::Vec3d worldPoint = first.getWorldIntersectPoint();

        // Get the map's SRS (Spatial Reference System)
        const osgEarth::SpatialReference* mapSRS = getMapNode()->getMapSRS();
        if (!mapSRS)
            return false;

        // Transform world XYZ to geographic (lon, lat, alt)
        osg::Vec3d geoPoint;
        mapSRS->transform(worldPoint, geoPoint, mapSRS->getGeodeticSRS());

        lon = geoPoint.x();
        lat = geoPoint.y();
        alt = geoPoint.z();

        return true;
    }
    return false;
}
