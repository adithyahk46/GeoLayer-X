#pragma once

#include <QDialog>
#include <QScopedPointer>
#include <QColor>

#include <osg/observer_ptr>
#include <osgEarth/Ephemeris>
#include <osgEarth/Sky>
#include <osgEarth/ShaderLoader>
#include <osgEarth/Shadowing>
#include <osgEarth/VirtualProgram>
#include <osgEarth/WindLayer>

#include <osgEarth/MapNode>

#include "DockWidgetPanels.h"

using namespace osgEarth::Util;


namespace Ui {
    class EnvironmentGUI;
}

class EnvironmentGUI : public DockWidgetPanels
{
    Q_OBJECT

public:
    explicit EnvironmentGUI(QWidget *parent = nullptr);
    ~EnvironmentGUI() override;

protected:
    void setWindowTitle(const QString &title) override { QDockWidget::setWindowTitle(title); }
    void setObjectName(const QString &name) override { QDockWidget::setObjectName(name); }

 private:
    QScopedPointer<Ui::EnvironmentGUI> ui;

        osg::observer_ptr<MapNode> _mapNode;
        osg::observer_ptr<SkyNode> _skyNode;
        osg::observer_ptr<ShadowCaster> _shadowCaster;
        osg::observer_ptr<WindLayer> _windLayer;
        bool _showDetails = false;
        float _hour;
        int _day, _month, _year;
        float _exposure = 3.5f;
        float _contrast = 1.0f;
        float _ambient = 0.033f;
        float _max_ambient_intensity = 0.75;
        bool _first = true;
        bool _shadows = false;
        float _haze_cutoff = 0.0f, _haze_strength = 16.0f;
        float _shadow_darkness = 0.5f, _shadow_blur = 0.001f;
        float _wind_power = 1.0f;

};