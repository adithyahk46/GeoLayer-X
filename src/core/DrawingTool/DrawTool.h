#ifndef DRAWTOOL_H
#define DRAWTOOL_H

#include <QObject>
#include <osgEarth/Style>


class DrawTool : public QObject
{
    Q_OBJECT

public:
    explicit DrawTool(QObject* parent = nullptr) ;
    virtual ~DrawTool() = default;

    osgEarth::Style getStyle() const { return _style; }

private:
    osgEarth::Style _style;

};
#endif // DRAWTOOL_H