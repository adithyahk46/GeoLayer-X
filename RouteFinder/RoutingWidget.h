#ifndef ROUTINGWIDGET_H
#define ROUTINGWIDGET_H

#include <QWidget>
#include <osgEarth/MapNode>

#include "MouseEventHandler.h"


QT_BEGIN_NAMESPACE
namespace Ui { class RoutingWidget; }
QT_END_NAMESPACE

class RoutingWidget : public QWidget
{
    Q_OBJECT
public:
    explicit RoutingWidget(osgEarth::MapNode* mapNode, MouseEventHandler* mouseControlle);
    ~RoutingWidget();
private slots:
    void selectShapefile();
    void computePath();
    void resetFields();
    void appendLog(const QString &msg);
    void onRoutingFinished(const QString &outPath);

    void on_pushButton_clicked();

    void on_pushButton_2_clicked();

private:
    Ui::RoutingWidget *ui;
    QString shapefilePath;
    void validateShapefile(const QString &path);
    void runRoutingInThread(double sx, double sy, double ex, double ey);

    osgEarth::MapNode* _mapnode =nullptr;
    MouseEventHandler* _mouseControlle = nullptr;

    QString prevOP;
};

#endif // ROUTINGWIDGET_H
