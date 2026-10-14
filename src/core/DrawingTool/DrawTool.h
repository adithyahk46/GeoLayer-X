#ifndef DRAWTOOL_H
#define DRAWTOOL_H

#include <QObject>
#include <osgEarth/Style>
#include <osgEarth/GeometryFactory>

#include <osg/Group>
#include <osgEarth/MapNode>
#include <osgEarth/Feature>

using namespace osgEarth;
using namespace osgEarth::Util;
using namespace osgEarth::Util;


class DrawTool : public QObject
{
    Q_OBJECT

public:
    explicit DrawTool(QObject* parent = nullptr) ;
    virtual ~DrawTool();

    osgEarth::Style getStyle() const { return _style; }

private:
    osgEarth::Style _style;

};
#endif // DRAWTOOL_H