#pragma once

#include <QScopedPointer>

#include <osgEarth/Config>
#include <osgEarth/MapNode>
#include <osgEarth/Viewpoint>

#include "DockWidgetPanels.h"

#include <vector>

class GLXEarthManipulator;
class QShowEvent;

namespace Ui {
class ViewpointsGUI;
}

class ViewpointsGUI : public DockWidgetPanels
{
    Q_OBJECT

public:
    explicit ViewpointsGUI(QWidget* parent = nullptr);
    ~ViewpointsGUI() override;

protected:
    void setWindowTitle(const QString& title) override { QDockWidget::setWindowTitle(title); }
    void setObjectName(const QString& name) override { QDockWidget::setObjectName(name); }
    void showEvent(QShowEvent* event) override;

private slots:
    void addCurrentViewpoint();
    void zoomToSelectedViewpoint();
    void deleteSelectedViewpoint();
    void updateButtonStates();

private:
    osgEarth::MapNode* currentMapNode() const;
    GLXEarthManipulator* currentManipulator() const;
    void loadViewpointSection(const osgEarth::Config& config);
    void reloadViewpoints();
    void persistViewpoints(osgEarth::MapNode* mapNode);

    QScopedPointer<Ui::ViewpointsGUI> ui;
    std::vector<osgEarth::Viewpoint> _viewpoints;
    float _duration = 1.0f;
};