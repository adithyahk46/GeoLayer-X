#ifndef STATUSBARHANDLERWIDGET_H
#define STATUSBARHANDLERWIDGET_H

#include "MapController/GeoPointMetaType.h"

#include <QWidget>

namespace Ui {
class StatusbarHandlerWidget;
}

class StatusbarHandlerWidget : public QWidget
{
    Q_OBJECT

public:
    explicit StatusbarHandlerWidget(QWidget *parent = nullptr);
    ~StatusbarHandlerWidget();

public slots:
    void updateGeoCoordinates(osgEarth::GeoPoint mapPoint);
    void clearGeoCoordinates();

private:
    Ui::StatusbarHandlerWidget *ui;
};

#endif // STATUSBARHANDLERWIDGET_H
