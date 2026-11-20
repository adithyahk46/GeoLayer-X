#pragma once

#include <QScopedPointer>

#include <osg/observer_ptr>
#include <osg/NodeCallback>
#include <osg/Timer>
#include <osgEarth/MapNode>
#include <osgEarth/Shadowing>
#include <osgEarth/Sky>
#include <osgEarth/WindLayer>

#include "DockWidgetPanels.h"

#include <functional>
#include <atomic>

class QShowEvent;
class QTimer;

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
    void showEvent(QShowEvent *event) override;

private:
    struct TimeCycle : public osg::NodeCallback
    {
        std::atomic<bool> playing{ false };
        std::atomic<float> hoursPerSecond{ 0.25f };

        void operator()(osg::Node* node, osg::NodeVisitor* nv) override;

    private:
        osg::Timer_t _lastTick = osg::Timer::instance()->tick();
        double _seconds = 0.0;
        bool _wasPlaying = false;
    };

    osgEarth::MapNode *currentMapNode() const;
    osgEarth::SkyNode *currentSkyNode() const;
    bool queueUpdate(const std::function<void(osgViewer::Viewer&)> &operation);
    void changeSky(const std::function<void(osgEarth::SkyNode&)> &change);
    void toggleEnvironment();
    void detachEnvironment();
    void installShadows();
    void applyCloudOptions();
    void resetEnvironment();
    void syncFromScene();
    void syncFromSceneLater();
    void refreshSunColorButton(const osg::Vec4& color, bool remember = true);

    QScopedPointer<Ui::EnvironmentGUI> ui;
    osg::observer_ptr<osgEarth::MapNode> _mapNode;
    osg::observer_ptr<osgEarth::SkyNode> _skyNode;
    osg::observer_ptr<osgEarth::Util::ShadowCaster> _shadowCaster;
    osg::observer_ptr<osgEarth::WindLayer> _windLayer;
    osg::ref_ptr<TimeCycle> _timeCycle;
    QTimer* _cycleUiTimer = nullptr;
    float _timeCycleSpeed = 0.25f;
    float _groundReflectance = 0.15f;
    float _hazeCutoff = 0.0f;
    float _hazeStrength = 16.0f;
    osg::Vec4 _sunColor = osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f);
    bool _updatingControls = false;
    bool _windDebug = false;
};
