#ifndef MAPREADOUTSDOCKWIDGET_H
#define MAPREADOUTSDOCKWIDGET_H

#include <array>

#include "MapController/GeoPointMetaType.h"

#include "DockWidgetPanels.h"

class QLabel;
class QShowEvent;
class QHideEvent;
class QTimer;
class GLXEarthManipulator;

namespace osgEarth {
class MapNode;
}

class MapReadoutsDockWidget : public DockWidgetPanels
{
    Q_OBJECT

public:
    /**
     * Creates the dock and connects it to the active 3D viewer and manipulator.
     * @param parent Parent widget, normally the main window.
     */
    explicit MapReadoutsDockWidget(QWidget* parent = nullptr);

    /**
     * Releases the dock's Qt-owned resources.
     */
    ~MapReadoutsDockWidget() override;

    /**
     * Sets the dock title.
     * @param title Title to display.
     */
    void setWindowTitle(const QString& title) override;

    /**
     * Sets the dock object name.
     * @param name Object name used by Qt.
     */
    void setObjectName(const QString& name) override;

protected:
    /**
     * Starts periodic camera readout updates when the dock becomes visible.
     * @param event Show event.
     */
    void showEvent(QShowEvent* event) override;

    /**
     * Stops periodic camera readout updates when the dock is hidden.
     * @param event Hide event.
     */
    void hideEvent(QHideEvent* event) override;

private slots:
    /**
     * Updates the mouse and terrain readouts from a valid map intersection.
     * @param mapPoint Geographic position beneath the pointer.
     */
    void updateMouseReadouts(osgEarth::GeoPoint mapPoint);

    /**
     * Clears mouse and terrain values when the pointer has no map intersection.
     */
    void clearMouseReadouts();

    /**
     * Refreshes the current viewpoint, map reference system, and scale.
     */
    void updateReadouts();

private:
    /**
     * Builds the dock's grouped readout interface.
     */
    void buildInterface();

    /**
     * Populates geographic and map-local coordinate fields.
     * @param point Point to display.
     * @param geographicValues Latitude, longitude, and altitude values.
     * @param localValues X, Y, and Z OSG world coordinates in meters.
     */
    void setCoordinateValues(const osgEarth::GeoPoint& point,
                             const std::array<QLabel*, 3>& geographicValues,
                             const std::array<QLabel*, 3>& localValues);

    /**
     * Returns the map currently displayed by the 3D viewer.
     */
    osgEarth::MapNode* currentMapNode() const;

    /**
     * Returns the manipulator attached to the 3D viewer.
     */
    GLXEarthManipulator* currentManipulator() const;

    std::array<QLabel*, 3> _viewGeoValues = {};
    std::array<QLabel*, 3> _viewLocalValues = {};
    std::array<QLabel*, 3> _mouseGeoValues = {};
    std::array<QLabel*, 3> _mouseLocalValues = {};

    QLabel* _headingValue = nullptr;
    QLabel* _pitchValue = nullptr;
    QLabel* _heightValue = nullptr;
    QLabel* _rangeValue = nullptr;
    QLabel* _srsValue = nullptr;
    QLabel* _projectionValue = nullptr;
    QLabel* _datumValue = nullptr;
    QLabel* _scaleValue = nullptr;
    QLabel* _resolutionValue = nullptr;
    QLabel* _viewModeValue = nullptr;
    QLabel* _terrainElevationValue = nullptr;
    QLabel* _slopeValue = nullptr;
    QLabel* _aspectValue = nullptr;
    QTimer* _updateTimer = nullptr;
};

#endif // MAPREADOUTSDOCKWIDGET_H
