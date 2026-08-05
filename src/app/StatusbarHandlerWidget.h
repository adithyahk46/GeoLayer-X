#ifndef STATUSBARHANDLERWIDGET_H
#define STATUSBARHANDLERWIDGET_H

#include <QWidget>
#include <osgEarth/GeoData>

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

private:
    Ui::StatusbarHandlerWidget *ui;
};

#endif // STATUSBARHANDLERWIDGET_H
