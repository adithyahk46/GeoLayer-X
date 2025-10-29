#ifndef VOLUMECALCULATORWIDGET_H
#define VOLUMECALCULATORWIDGET_H

#include <QWidget>
#include <QVector>
#include <osg/Vec3d>
#include <osgGA/GUIEventAdapter>
#include <osgGA/GUIActionAdapter>

namespace Ui {
class VolumeCalculatorWidget;
}

class VolumeCalculatorWidget : public QWidget
{
    Q_OBJECT

public:
    explicit VolumeCalculatorWidget(QWidget *parent = nullptr);
    ~VolumeCalculatorWidget();

private slots:
    // Mouse event slots
    // void mouseClickEvent(const double lon, const double lat, const double alt,
    //                      const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
    // void mouseDoubleClickEvent(const double lon, const double lat, const double alt,
    //                            const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);

    // Button click slots
    void onSelectRegionClicked();
    void onUpdatePolygonClicked();
    void onCalculateVolumeClicked();
    // void onClearPointsClicked();

    void onMouseClickEvent(double lon, double lat, double alt, const osgGA::GUIEventAdapter &, osgGA::GUIActionAdapter &);
private:
    Ui::VolumeCalculatorWidget *ui;

    // Store selected points
    QVector<osg::Vec3d> _selectedPoints;

    // Mouse controller reference
    class MouseEventHandler* _mouseController;
};

#endif // VOLUMECALCULATORWIDGET_H
