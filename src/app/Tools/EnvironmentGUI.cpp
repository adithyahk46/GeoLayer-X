#include "EnvironmentGUI.h"
#include "ui_EnvironmentGUI.h"
#include "App.h"
#include "OSGViewerWidget.h"

#include <QColorDialog>
#include <QComboBox>
#include <QDialog>
#include <QDoubleSpinBox>
#include <QMetaObject>
#include <QPointer>
#include <QShowEvent>
#include <QSignalBlocker>
#include <QSlider>
#include <QTimer>
#include <QDebug>

#include <osgEarth/NodeUtils>
#include <osgEarth/ShaderLoader>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/Utils>
#include <osgEarth/VirtualProgram>

#ifdef OSGEARTH_HAVE_PRESTIGE_NODEKIT
#include <osgEarthPrestige/CloudLayer>
#include <osgEarthPrestige/SkyNode>
#endif

#include <algorithm>
#include <cmath>
#include <vector>

using namespace osgEarth;
using namespace osgEarth::Util;

namespace
{
    const char* render_wind = R"(
#version 450
#pragma vp_function oe_ui_render_wind_vert, vertex_view
out vec3 viewpos3_wind;
void oe_ui_render_wind_vert(inout vec4 vertex) {
    viewpos3_wind = vertex.xyz;
}
[break]
#version 450
#pragma vp_function oeui_render_wind_texture, fragment_output
#pragma import_defines(OE_WIND_TEX)
#pragma import_defines(OE_WIND_TEX_MATRIX)
in vec3 viewpos3_wind;
out vec4 frag_out;
#ifdef OE_WIND_TEX
uniform sampler3D OE_WIND_TEX;
uniform mat4 OE_WIND_TEX_MATRIX;
uniform vec3 oe_Camera;
uniform float osg_FrameTime;
#pragma import_defines(OE_TWEAKABLE)
#ifdef OE_TWEAKABLE
#define tweakable uniform
#else
#define tweakable const
#endif
tweakable float oe_wind_power = 1.0;
#define MAX_WIND_SPEED 50.0
#endif
void oeui_render_wind_texture(inout vec4 color)
{
    frag_out = color;
  #ifdef OE_WIND_TEX
    vec4 texel = textureProj(OE_WIND_TEX, (OE_WIND_TEX_MATRIX*vec4(viewpos3_wind,1)));
    float speed = texel.a * oe_wind_power;
    vec3 wind_view = 2.0*texel.xyz - 1.0;
    vec3 wind_clip = mat3(gl_ProjectionMatrix) * wind_view;
    vec2 wind_screen = wind_clip.xy * oe_Camera.xy;
    vec2 coord = (gl_FragCoord.xy - 0.5);
    vec2 rv = normalize(wind_screen);
    vec2 coordProj = mat2(rv.x, -rv.y, rv.y, rv.x) * coord;
    const float oe_wind_animation_speed = 2.8 * MAX_WIND_SPEED * speed;
    int cx = int(coordProj.x - int(osg_FrameTime*oe_wind_animation_speed));
    int ci = cx % 32;
    const int stipple = 0x00000001;
    int pattern32 = 0xffffffff & (stipple & (1 << ci));
    if (pattern32 != 0)
        frag_out = vec4(0.5,0.5,1,0.5);
  #endif
}
)";

    void configureSlider(ImGuiSlider* slider, int scale, float minimum, float maximum)
    {
        slider->setValueScale(scale);
        slider->setValuePrecision(scale == 1 ? 0 : 3);
        slider->setRange(qRound(minimum * scale), qRound(maximum * scale));
    }

    void setSliderValue(ImGuiSlider* slider, float value)
    {
        slider->setValue(qRound(value * slider->valueScale()));
    }

    float sliderValue(const ImGuiSlider* slider)
    {
        return float(slider->value()) / float(slider->valueScale());
    }
}

void EnvironmentGUI::TimeCycle::operator()(osg::Node* node, osg::NodeVisitor* nv)
{
    const osg::Timer_t now = osg::Timer::instance()->tick();
    const double elapsed = osg::Timer::instance()->delta_s(_lastTick, now);
    _lastTick = now;
    const bool running = playing.load();

    if (running && _wasPlaying && elapsed > 0.0)
    {
        _seconds += elapsed * hoursPerSecond.load() * 3600.0;
        const double wholeSeconds = std::floor(_seconds);
        _seconds -= wholeSeconds;
        if (wholeSeconds >= 1.0)
        {
            auto* sky = static_cast<SkyNode*>(node);
            const TimeStamp stamp = sky->getDateTime().asTimeStamp();
            const TimeStamp secondsOfDay = (stamp % 86400 + 86400) % 86400;
            const TimeStamp advance = static_cast<TimeStamp>(std::fmod(wholeSeconds, 86400.0));
            const TimeStamp next = (secondsOfDay + advance) % 86400;
            sky->setDateTime(DateTime(stamp - secondsOfDay + next));
        }
    }
    if (!running)
        _seconds = 0.0;
    _wasPlaying = running;
    traverse(node, nv);
}

EnvironmentGUI::EnvironmentGUI(QWidget *parent)
    : DockWidgetPanels(parent)
    , ui(new Ui::EnvironmentGUI)
{
    ui->setupUi(this);
    setObjectName(QStringLiteral("EnvironmentDock"));

    configureSlider(ui->horizontalSlider_Hour, 1000, 0.0f, 24.0f);
    configureSlider(ui->horizontalSlider_day, 1, 1.0f, 31.0f);
    configureSlider(ui->horizontalSlider_month, 1, 1.0f, 12.0f);
    configureSlider(ui->horizontalSlider_year, 1, 1970.0f, 2061.0f);
    configureSlider(ui->horizontalSlider_spped, 1000, 0.01f, 24.0f);

    configureSlider(ui->horizontalSlider_Exposure, 1000, 0.01f, 16.0f);
    configureSlider(ui->horizontalSlider_AmbientMin, 1000, 0.0f, 1.0f);
    configureSlider(ui->horizontalSlider_AmbientMax, 1000, 0.0f, 1.0f);
    configureSlider(ui->horizontalSlider_sunIntensity, 1000, 0.0f, 40.0f);
    configureSlider(ui->horizontalSlider_skyEnv, 1000, 0.0f, 4.0f);
    configureSlider(ui->horizontalSlider_nightFill, 1000, 0.0f, 1.0f);
    configureSlider(ui->horizontalSlider_NormalBoost, 1000, 1.0f, 5.0f);
    configureSlider(ui->horizontalSlider_groundReflectance, 1000, 0.0f, 1.0f);
    configureSlider(ui->horizontalSlider_hazeCutoff, 1000, 0.0f, 0.2f);
    configureSlider(ui->horizontalSlider_hazeStrength, 1000, 0.0f, 24.0f);

    configureSlider(ui->horizontalSlider_cloudCoverage, 1000, 0.0f, 1.0f);
    configureSlider(ui->horizontalSlider_cloudDensity, 1000, 0.0f, 5.0f);
    configureSlider(ui->horizontalSlider_cloudBase, 1, 0.0f, 19000.0f);
    configureSlider(ui->horizontalSlider_cloudTop, 1, 100.0f, 20000.0f);
    configureSlider(ui->horizontalSlider_cloudFadeStart, 1, 20000.0f, 1000000.0f);
    configureSlider(ui->horizontalSlider_cloudHiddenAbove, 1, 20100.0f, 10000000.0f);
    configureSlider(ui->horizontalSlider_cloudSize, 1, 500.0f, 15000.0f);
    configureSlider(ui->horizontalSlider_cloudErosion, 1000, 0.0f, 1.0f);
    configureSlider(ui->horizontalSlider_cloudShadows, 1000, 0.0f, 1.0f);
    configureSlider(ui->horizontalSlider_shadowCoverage, 1000, 1.0f, 200.0f);
    configureSlider(ui->horizontalSlider_farShadowCoverage, 1000, 1.25f, 1000.0f);
    configureSlider(ui->horizontalSlider_rayQuality, 1, 0.0f, 2.0f);
    configureSlider(ui->horizontalSlider_rayStrength, 1000, 0.0f, 1.0f);
    configureSlider(ui->horizontalSlider_rayIntensity, 1000, 0.0f, 8.0f);
    configureSlider(ui->horizontalSlider_rayHaze, 1000, 0.0f, 4.0f);
    configureSlider(ui->horizontalSlider_rayDistance, 1, 1000.0f, 100000.0f);

    configureSlider(ui->horizontalSlider_cascades, 1, 1.0f, 4.0f);
    configureSlider(ui->horizontalSlider_distance, 1, 50.0f, 20000.0f);
    configureSlider(ui->horizontalSlider_softness, 1000, 0.0f, 8.0f);
    configureSlider(ui->horizontalSlider_sunlightInShadow, 1000, 0.0f, 1.0f);
    configureSlider(ui->horizontalSlider_deothBias, 1000, 0.0f, 4.0f);
    configureSlider(ui->horizontalSlider_casterReach, 1, 0.0f, 10000.0f);
    configureSlider(ui->horizontalSlider_casterBlend, 1000, 0.0f, 0.3f);

    ui->comboBox_mapResolution->addItems({ tr("512"), tr("1024"), tr("2048"), tr("4096"), tr("8192") });
    ui->comboBox_filtering->addItems({ tr("Fast"), tr("Balanced"), tr("Soft") });
    ui->grpDetails->setVisible(ui->checkBox_2->isChecked());
    ui->grpWind->setVisible(ui->checkBox->isChecked());
    ui->lblDay->setVisible(ui->checkBox_2->isChecked());
    ui->lblMonth->setVisible(ui->checkBox_2->isChecked());
    ui->lblYear->setVisible(ui->checkBox_2->isChecked());
    ui->horizontalSlider_day->setVisible(ui->checkBox_2->isChecked());
    ui->horizontalSlider_month->setVisible(ui->checkBox_2->isChecked());
    ui->horizontalSlider_year->setVisible(ui->checkBox_2->isChecked());

    ui->pushButton_reset->setToolTip(tr("Restore the environment controls to their default values."));
    ui->chkLighting->setToolTip(tr("Scene lighting override. Atmosphere visibility is controlled separately."));
    ui->checkBox_2->setToolTip(tr("Show advanced environment and shadow settings."));
    ui->checkBoxCycleTime->setToolTip(tr("Repeat this day automatically. Continues with the Sky panel or GUI hidden."));
    ui->horizontalSlider_spped->setToolTip(tr("Simulated hours per real second."));
    ui->btnNow->setToolTip(tr("Set the environment date and time to now."));
    ui->horizontalSlider_Exposure->setToolTip(tr("Sky exposure in linear space."));
    ui->horizontalSlider_sunIntensity->setToolTip(
        tr("Solar irradiance drives both atmospheric scattering and direct sunlight."));
    ui->horizontalSlider_nightFill->setToolTip(tr("Minimum diffuse fill. Set to zero for a physically dark night."));
    ui->horizontalSlider_AmbientMax->setToolTip(
        tr("Daytime ambient fill. Fades out as Sky environment approaches 1."));
    ui->horizontalSlider_skyEnv->setToolTip(
        tr("Controls sky fill and reflections. Ambient min remains separate."));
    ui->horizontalSlider_groundReflectance->setToolTip(
        tr("Assumed linear reflectance of surrounding ground for indirect light."));
    ui->horizontalSlider_NormalBoost->setToolTip(tr("Boosts normal detail in the sky lighting shader."));
    ui->horizontalSlider_hazeCutoff->setToolTip(tr("Atmospheric haze cutoff."));
    ui->horizontalSlider_hazeStrength->setToolTip(tr("Atmospheric haze strength."));
    ui->comboCloudQuality->setToolTip(tr("Cloud rendering quality: Low, Balanced, or High."));
    ui->horizontalSlider_cloudCoverage->setToolTip(tr("Fraction of the cloud volume covered by clouds."));
    ui->horizontalSlider_cloudDensity->setToolTip(tr("Cloud density multiplier."));
    ui->horizontalSlider_cloudBase->setToolTip(tr("Minimum cloud-shell altitude in meters."));
    ui->horizontalSlider_cloudTop->setToolTip(tr("Maximum cloud-shell altitude in meters."));
    ui->horizontalSlider_cloudFadeStart->setToolTip(tr("Camera altitude where cloud visibility begins to fade."));
    ui->horizontalSlider_cloudHiddenAbove->setToolTip(tr("Camera altitude above which clouds are hidden."));
    ui->horizontalSlider_cloudSize->setToolTip(tr("Procedural cloud feature size in meters."));
    ui->horizontalSlider_cloudErosion->setToolTip(tr("Amount of detail erosion applied to cloud density."));
    ui->_farDensityDetails->setToolTip(
        tr("Average detail smaller than a cloud pixel or ray step to reduce shimmer."));
    ui->horizontalSlider_cloudShadows->setToolTip(tr("Strength of the shadows cast by clouds."));
    ui->horizontalSlider_shadowCoverage->setToolTip(
        tr("Full width of the ground area receiving cloud shadows. Smaller values give finer local shadows; distant shadows fade out."));
    ui->checkBox_farCloudShadow->setToolTip(
        tr("Add a coarser distant shadow cascade. Extra GPU work; near detail is unchanged."));
    ui->horizontalSlider_farShadowCoverage->setToolTip(
        tr("Full width of the far shadow area; blends beyond the near cascade. At least 25 percent wider than the near area."));
    ui->checkBox_crepussularRays->setToolTip(tr("Cloud shadows in atmospheric haze. Optional extra GPU work."));
    ui->horizontalSlider_rayQuality->setToolTip(tr("0: Low  1: Balanced  2: High"));
    ui->horizontalSlider_rayStrength->setToolTip(
        tr("Blend from existing lighting to shadowed air and additional haze."));
    ui->horizontalSlider_rayIntensity->setToolTip(
        tr("Sunlight scattering without increasing haze opacity. 1 is unamplified."));
    ui->horizontalSlider_rayHaze->setToolTip(
        tr("Extra low-altitude haze that reveals sunbeams. Fades out as cloud coverage falls from 0.6 to 0.2. Zero uses existing air only."));
    ui->horizontalSlider_rayDistance->setToolTip(tr("Local atmospheric shadow integration range in meters."));
    ui->chkWindDebug->setToolTip(tr("Visualize the wind field used by the cloud layer."));
    ui->comboBox->setToolTip(tr("Select the shadow draw-submission path."));
    ui->comboBox_mapResolution->setToolTip(tr("Shadow-map resolution per cascade."));
    ui->comboBox_filtering->setToolTip(tr("Select the shadow filtering quality."));
    ui->horizontalSlider_cascades->setToolTip(tr("Number of shadow cascades."));
    ui->horizontalSlider_distance->setToolTip(tr("Maximum shadow distance in meters."));
    ui->checkBox_cacheStaticCaster->setToolTip(
        tr("For static caster shaders and render state. Callbacks and paging bypass reuse. Keep caching off for animated shaders; invalidate custom state changes."));
    ui->horizontalSlider_softness->setToolTip(tr("Shadow filter radius in texels."));
    ui->horizontalSlider_deothBias->setToolTip(tr("Receiver depth bias in texels."));
    ui->horizontalSlider_casterReach->setToolTip(tr("Maximum reach for shadow-casting geometry in meters."));
    ui->horizontalSlider_casterBlend->setToolTip(tr("Fraction of each cascade used for transition blending."));

    _cycleUiTimer = new QTimer(this);
    _cycleUiTimer->setInterval(250);
    connect(_cycleUiTimer, &QTimer::timeout, this, &EnvironmentGUI::syncFromScene);

#ifndef OSGEARTH_HAVE_PRESTIGE_NODEKIT
    ui->grpPrestige->hide();
#endif

    const auto connectSlider = [this](ImGuiSlider* slider, const std::function<void(float)>& change) {
        connect(slider, &QSlider::valueChanged, this, [this, slider, change](int value) {
            if (!_updatingControls)
                change(float(value) / float(slider->valueScale()));
        });
    };
    const auto changeShadows = [this](const std::function<void(ShadowCaster&)>& change) {
        if (_updatingControls)
            return;
        osg::ref_ptr<ShadowCaster> caster = _shadowCaster.get();
        if (caster.valid())
            queueUpdate([caster, change](osgViewer::Viewer&) {
                if (caster.valid())
                    change(*caster);
            });
    };

    connect(ui->btnNow, &QPushButton::clicked, this, [this] {
        if (_timeCycle)
            _timeCycle->playing = false;
        ui->checkBoxCycleTime->setChecked(false);
        changeSky([](SkyNode& sky) { sky.setDateTime(DateTime()); });
    });
    connect(ui->pushButton_reset, &QPushButton::clicked, this, &EnvironmentGUI::resetEnvironment);
    connect(ui->pushButton_toggleEnvironment, &QPushButton::clicked, this, &EnvironmentGUI::toggleEnvironment);
    connect(ui->chkLighting, &QCheckBox::toggled, this, [this](bool enabled) {
        if (!_updatingControls)
            changeSky([enabled](SkyNode& sky) {
                sky.setLighting((enabled ? osg::StateAttribute::ON : osg::StateAttribute::OFF) |
                    osg::StateAttribute::OVERRIDE);
            });
    });
    connect(ui->chkAtmosphere, &QCheckBox::toggled, this, [this](bool enabled) {
        if (!_updatingControls)
            changeSky([enabled](SkyNode& sky) { sky.setAtmosphereVisible(enabled); });
    });
    connect(ui->chkSun, &QCheckBox::toggled, this, [this](bool enabled) {
        if (!_updatingControls)
            changeSky([enabled](SkyNode& sky) { sky.setSunVisible(enabled); });
    });
    connect(ui->chkMoon, &QCheckBox::toggled, this, [this](bool enabled) {
        if (!_updatingControls)
            changeSky([enabled](SkyNode& sky) { sky.setMoonVisible(enabled); });
    });
    connect(ui->chkStars, &QCheckBox::toggled, this, [this](bool enabled) {
        if (!_updatingControls)
            changeSky([enabled](SkyNode& sky) { sky.setStarsVisible(enabled); });
    });
    connect(ui->chkShadows, &QCheckBox::toggled, this, [this](bool enabled) {
        if (_updatingControls)
            return;
        if (!_shadowCaster.valid()) {
            if (enabled)
                installShadows();
            return;
        }
        osg::ref_ptr<ShadowCaster> caster = _shadowCaster.get();
        QPointer<EnvironmentGUI> panel(this);
        queueUpdate([caster, enabled, panel](osgViewer::Viewer&) {
            if (caster.valid())
                caster->setEnabled(enabled);
            if (panel)
                panel->syncFromSceneLater();
        });
    });

    const auto setDateTime = [this] {
        const int year = ui->horizontalSlider_year->value();
        const int month = ui->horizontalSlider_month->value();
        const int day = ui->horizontalSlider_day->value();
        const double hour = sliderValue(ui->horizontalSlider_Hour);
        if (_timeCycle)
            _timeCycle->playing = false;
        ui->checkBoxCycleTime->setChecked(false);
        changeSky([year, month, day, hour](SkyNode& sky) {
            sky.setDateTime(DateTime(year, month, day, hour));
        });
    };
    connect(ui->horizontalSlider_Hour, &QSlider::valueChanged, this, [this, setDateTime] {
        if (!_updatingControls) setDateTime();
    });
    connect(ui->horizontalSlider_day, &QSlider::valueChanged, this, [this, setDateTime] {
        if (!_updatingControls) setDateTime();
    });
    connect(ui->horizontalSlider_month, &QSlider::valueChanged, this, [this, setDateTime] {
        if (!_updatingControls) setDateTime();
    });
    connect(ui->horizontalSlider_year, &QSlider::valueChanged, this, [this, setDateTime] {
        if (!_updatingControls) setDateTime();
    });
    connect(ui->checkBoxCycleTime, &QCheckBox::toggled, this, [this](bool enabled) {
        if (_timeCycle)
            _timeCycle->playing = enabled && _skyNode.valid();
        if (enabled && _skyNode.valid())
            _cycleUiTimer->start();
        else
            _cycleUiTimer->stop();
    });
    connectSlider(ui->horizontalSlider_spped, [this](float value) {
        _timeCycleSpeed = value;
        if (_timeCycle)
            _timeCycle->hoursPerSecond = value;
        ui->label_4->setText(tr("%1 seconds").arg(24.0f / value, 0, 'f', 1));
    });

    const auto setUniform = [this](const char* name, float value) {
        changeSky([name, value](SkyNode& sky) {
            sky.getOrCreateStateSet()->getOrCreateUniform(name, osg::Uniform::FLOAT)->set(value);
        });
    };
    connectSlider(ui->horizontalSlider_Exposure, [this, setUniform](float value) {
        changeSky([value](SkyNode& sky) {
#ifdef OSGEARTH_HAVE_PRESTIGE_NODEKIT
            if (auto* prestige = dynamic_cast<osgEarthPrestige::SkyNode*>(&sky)) {
                prestige->setExposure(value);
                return;
            }
#endif
            sky.getOrCreateStateSet()->getOrCreateUniform("oe_sky_exposure", osg::Uniform::FLOAT)->set(value);
        });
    });
    connectSlider(ui->horizontalSlider_AmbientMin, [this](float value) {
        changeSky([value](SkyNode& sky) {
#ifdef OSGEARTH_HAVE_PRESTIGE_NODEKIT
            if (auto* prestige = dynamic_cast<osgEarthPrestige::SkyNode*>(&sky)) {
                prestige->setAmbientIntensity(value);
                return;
            }
#endif
            if (sky.getSunLight())
                sky.getSunLight()->setAmbient(osg::Vec4(value, value, value, 1.0f));
        });
    });
    connectSlider(ui->horizontalSlider_AmbientMax, [setUniform](float value) {
        setUniform("oe_sky_maxAmbientIntensity", value);
    });
    connectSlider(ui->horizontalSlider_sunIntensity, [this](float value) {
        changeSky([value](SkyNode& sky) {
#ifdef OSGEARTH_HAVE_PRESTIGE_NODEKIT
            if (auto* prestige = dynamic_cast<osgEarthPrestige::SkyNode*>(&sky))
                prestige->setSunIntensity(value);
#else
            Q_UNUSED(sky);
#endif
        });
    });
    connectSlider(ui->horizontalSlider_skyEnv, [this](float value) {
        changeSky([value](SkyNode& sky) {
#ifdef OSGEARTH_HAVE_PRESTIGE_NODEKIT
            if (auto* prestige = dynamic_cast<osgEarthPrestige::SkyNode*>(&sky)) {
                prestige->setEnvironmentIntensity(value);
                return;
            }
#endif
            auto* uniform = sky.getOrCreateStateSet()->getUniform("oe_sky_iblStrength");
            if (uniform)
                uniform->set(value);
        });
    });
    connectSlider(ui->horizontalSlider_nightFill, [this](float value) {
        changeSky([value](SkyNode& sky) {
#ifdef OSGEARTH_HAVE_PRESTIGE_NODEKIT
            if (auto* prestige = dynamic_cast<osgEarthPrestige::SkyNode*>(&sky))
                prestige->setAmbientIntensity(value);
#else
            Q_UNUSED(sky);
#endif
        });
    });
    connectSlider(ui->horizontalSlider_NormalBoost, [setUniform](float value) {
        setUniform("oe_normal_boost", value);
    });
    connectSlider(ui->horizontalSlider_groundReflectance, [setUniform](float value) {
        setUniform("oe_sky_groundReflectance", value);
    });
    connectSlider(ui->horizontalSlider_hazeCutoff, [setUniform](float value) {
        setUniform("atmos_haze_cutoff", value);
    });
    connectSlider(ui->horizontalSlider_hazeStrength, [setUniform](float value) {
        setUniform("atmos_haze_strength", value);
    });

    connect(ui->pushButton_sunColor, &QPushButton::clicked, this, [this] {
        const osg::Vec4 previous = _sunColor;
        QColorDialog dialog(QColor::fromRgbF(previous.r(), previous.g(), previous.b()), this);
        dialog.setWindowTitle(tr("Sun color"));
        dialog.setOption(QColorDialog::DontUseNativeDialog);
        dialog.setOption(QColorDialog::ShowAlphaChannel, false);
        connect(&dialog, &QColorDialog::currentColorChanged, this, [this](const QColor& color) {
            if (!color.isValid())
                return;
            const osg::Vec4 diffuse(color.redF(), color.greenF(), color.blueF(), 1.0f);
            changeSky([diffuse](SkyNode& sky) {
                if (sky.getSunLight())
                    sky.getSunLight()->setDiffuse(diffuse);
            });
            refreshSunColorButton(diffuse, false);
        });
        if (dialog.exec() == QDialog::Accepted) {
            const QColor selected = dialog.currentColor();
            if (selected.isValid()) {
                const osg::Vec4 diffuse(selected.redF(), selected.greenF(), selected.blueF(), 1.0f);
                _sunColor = diffuse;
                changeSky([diffuse](SkyNode& sky) {
                    if (sky.getSunLight())
                        sky.getSunLight()->setDiffuse(diffuse);
                });
                refreshSunColorButton(diffuse, false);
            }
        }
        else {
            _sunColor = previous;
            changeSky([previous](SkyNode& sky) {
                if (sky.getSunLight())
                    sky.getSunLight()->setDiffuse(previous);
            });
            refreshSunColorButton(previous, false);
        }
    });

    connect(ui->checkBox_clouds, &QCheckBox::toggled, this, [this] {
        if (!_updatingControls) applyCloudOptions();
    });
    connect(ui->comboCloudQuality, qOverload<int>(&QComboBox::currentIndexChanged), this, [this] {
        if (!_updatingControls) applyCloudOptions();
    });
    for (ImGuiSlider* slider : {
        ui->horizontalSlider_cloudCoverage, ui->horizontalSlider_cloudDensity,
        ui->horizontalSlider_cloudBase, ui->horizontalSlider_cloudTop,
        ui->horizontalSlider_cloudFadeStart, ui->horizontalSlider_cloudHiddenAbove,
        ui->horizontalSlider_cloudSize, ui->horizontalSlider_cloudErosion,
        ui->horizontalSlider_cloudShadows, ui->horizontalSlider_shadowCoverage,
        ui->horizontalSlider_farShadowCoverage, ui->horizontalSlider_rayQuality,
        ui->horizontalSlider_rayStrength, ui->horizontalSlider_rayIntensity,
        ui->horizontalSlider_rayHaze, ui->horizontalSlider_rayDistance
    }) {
        connectSlider(slider, [this](float) {
            if (!_updatingControls) applyCloudOptions();
        });
    }
    connect(ui->_farDensityDetails, &QCheckBox::toggled, this, [this] {
        if (!_updatingControls) applyCloudOptions();
    });
    connect(ui->checkBox_farCloudShadow, &QCheckBox::toggled, this, [this] {
        if (!_updatingControls) applyCloudOptions();
    });
    connect(ui->checkBox_crepussularRays, &QCheckBox::toggled, this, [this] {
        if (!_updatingControls) applyCloudOptions();
    });

    connect(ui->spinWindPower, qOverload<double>(&QDoubleSpinBox::valueChanged), this, [this](double value) {
        if (_updatingControls)
            return;
        osg::ref_ptr<WindLayer> wind = _windLayer.get();
        if (wind.valid())
            queueUpdate([wind, value](osgViewer::Viewer&) { wind->setSpeedFactor(float(value)); });
    });
    connect(ui->chkWindDebug, &QCheckBox::toggled, this, [this](bool enabled) {
        if (_updatingControls)
            return;
        osg::ref_ptr<MapNode> map = _mapNode.get();
        if (!map.valid())
            return;
        _windDebug = enabled;
        queueUpdate([map, enabled](osgViewer::Viewer&) {
            auto* program = VirtualProgram::getOrCreate(map->getOrCreateStateSet());
            if (enabled)
                ShaderLoader::load(program, render_wind);
            else
                ShaderLoader::unload(program, render_wind);
        });
    });

    connectSlider(ui->horizontalSlider_cascades, [this, changeShadows](float value) {
        const int count = qRound(value);
        const int distance = ui->horizontalSlider_distance->value();
        changeShadows([count, distance](ShadowCaster& caster) {
            caster.setCascades(unsigned(count), float(distance));
        });
    });
    connectSlider(ui->horizontalSlider_distance, [this, changeShadows](float value) {
        const int count = ui->horizontalSlider_cascades->value();
        changeShadows([count, value](ShadowCaster& caster) {
            caster.setCascades(unsigned(count), value);
        });
    });
    connectSlider(ui->horizontalSlider_softness, [changeShadows](float value) {
        changeShadows([value](ShadowCaster& caster) { caster.setFilterRadius(value); });
    });
    connectSlider(ui->horizontalSlider_sunlightInShadow, [changeShadows](float value) {
        changeShadows([value](ShadowCaster& caster) { caster.setShadowColor(value); });
    });
    connectSlider(ui->horizontalSlider_deothBias, [changeShadows](float value) {
        changeShadows([value](ShadowCaster& caster) { caster.setDepthBias(value); });
    });
    connectSlider(ui->horizontalSlider_casterReach, [changeShadows](float value) {
        changeShadows([value](ShadowCaster& caster) { caster.setCasterDistance(value); });
    });
    connectSlider(ui->horizontalSlider_casterBlend, [changeShadows](float value) {
        changeShadows([value](ShadowCaster& caster) { caster.setCascadeBlend(value); });
    });
    connect(ui->checkBox_cacheStaticCaster, &QCheckBox::toggled, this, [this, changeShadows](bool enabled) {
        if (_updatingControls) return;
        changeShadows([enabled](ShadowCaster& caster) { caster.setCacheEnabled(enabled); });
    });
    connect(ui->comboBox, qOverload<int>(&QComboBox::currentIndexChanged), this, [this, changeShadows](int value) {
        if (_updatingControls) return;
        if (value >= int(ShadowCaster::AUTOMATIC) && value <= int(ShadowCaster::LEGACY)) {
            changeShadows([value](ShadowCaster& caster) {
                caster.setSubmission(static_cast<ShadowCaster::Submission>(value));
            });
        }
    });
    connect(ui->comboBox_mapResolution, qOverload<int>(&QComboBox::currentIndexChanged), this,
        [this, changeShadows](int index) {
            if (_updatingControls) return;
            static const unsigned resolutions[] = { 512u, 1024u, 2048u, 4096u, 8192u };
            if (index >= 0 && index < int(std::size(resolutions))) {
                const unsigned resolution = resolutions[index];
                changeShadows([resolution](ShadowCaster& caster) { caster.setTextureSize(resolution); });
            }
        });
    connect(ui->comboBox_filtering, qOverload<int>(&QComboBox::currentIndexChanged), this,
        [this, changeShadows](int value) {
            if (_updatingControls) return;
            if (value >= int(ShadowCaster::HARD) && value <= int(ShadowCaster::SOFT)) {
                changeShadows([value](ShadowCaster& caster) {
                    caster.setFilter(static_cast<ShadowCaster::Filter>(value));
                });
            }
        });

    connect(ui->checkBox_2, &QCheckBox::toggled, ui->grpDetails, &QWidget::setVisible);
    connect(ui->checkBox, &QCheckBox::toggled, ui->grpWind, &QWidget::setVisible);

    syncFromScene();
}

EnvironmentGUI::~EnvironmentGUI()
{
    if (_cycleUiTimer)
        _cycleUiTimer->stop();
    if (_timeCycle) {
        _timeCycle->playing = false;
        osg::ref_ptr<SkyNode> sky = _skyNode.get();
        osg::ref_ptr<TimeCycle> cycle = _timeCycle.get();
        if (sky.valid()) {
            queueUpdate([sky, cycle](osgViewer::Viewer&) {
                if (sky.valid())
                    sky->removeUpdateCallback(cycle.get());
            });
        }
    }
    if (_windDebug && _mapNode.valid()) {
        osg::ref_ptr<MapNode> map = _mapNode.get();
        queueUpdate([map](osgViewer::Viewer&) {
            ShaderLoader::unload(VirtualProgram::getOrCreate(map->getOrCreateStateSet()), render_wind);
        });
    }
}

void EnvironmentGUI::showEvent(QShowEvent *event)
{
    DockWidgetPanels::showEvent(event);
    syncFromScene();
}

MapNode *EnvironmentGUI::currentMapNode() const
{
    auto *widget = App::getThreedWindow();
    auto *viewer = widget ? widget->getViewer() : nullptr;
    auto *scene = viewer ? viewer->getSceneData() : nullptr;
    auto *map = scene ? MapNode::findMapNode(scene) : nullptr;
    return map ? map : (widget ? widget->getMapNode() : nullptr);
}

SkyNode *EnvironmentGUI::currentSkyNode() const
{
    auto *map = currentMapNode();
    return map ? findFirstParentOfType<SkyNode>(map) : nullptr;
}

bool EnvironmentGUI::queueUpdate(const std::function<void(osgViewer::Viewer&)> &operation)
{
    auto *widget = App::getThreedWindow();
    auto *viewer = widget ? widget->getViewer() : nullptr;
    if (!viewer) {
        qWarning() << "Cannot update the environment: the 3D viewer is not ready.";
        return false;
    }
    osg::observer_ptr<osgViewer::Viewer> targetViewer = viewer;
    viewer->addUpdateOperation(new OneTimer([targetViewer, operation] {
        if (targetViewer.valid())
            operation(*targetViewer);
    }));
    return true;
}

void EnvironmentGUI::changeSky(const std::function<void(SkyNode&)> &change)
{
    osg::ref_ptr<MapNode> map = currentMapNode();
    osg::ref_ptr<SkyNode> sky = currentSkyNode();
    if (!map.valid() || !sky.valid()) {
        qWarning() << "Cannot change environment settings: no sky is installed.";
        return;
    }
    QPointer<EnvironmentGUI> panel(this);
    queueUpdate([map, sky, change, panel](osgViewer::Viewer& viewer) {
        if (MapNode::findMapNode(viewer.getSceneData()) != map.get() ||
            findFirstParentOfType<SkyNode>(map.get()) != sky.get())
            return;
        change(*sky);
        if (panel)
            panel->syncFromSceneLater();
    });
}

void EnvironmentGUI::toggleEnvironment()
{
    if (currentSkyNode()) {
        detachEnvironment();
        return;
    }
#ifdef OSGEARTH_HAVE_PRESTIGE_NODEKIT
    osg::ref_ptr<MapNode> map = currentMapNode();
    if (!map.valid()) {
        qWarning() << "Cannot attach the environment: no map is loaded.";
        return;
    }
    QPointer<EnvironmentGUI> panel(this);
    queueUpdate([map, panel](osgViewer::Viewer& viewer) {
        if (MapNode::findMapNode(viewer.getSceneData()) != map.get() ||
            findFirstParentOfType<SkyNode>(map.get()))
            return;

        osg::ref_ptr<osgEarthPrestige::SkyNode> sky = new osgEarthPrestige::SkyNode;
        if (map->getMapSRS()->isProjected())
            sky->setReferencePoint(map->getMap()->getProfile()->getExtent().getCentroid());
        sky->setSimulationTimeTracksDateTime(true);
        if (viewer.getSceneData() == map.get()) {
            sky->addChild(map.get());
            viewer.setSceneData(sky.get());
        }
        else {
            insertParent(sky.get(), map.get());
        }
        sky->attach(&viewer);
        if (panel)
            panel->syncFromSceneLater();
    });
#else
    qWarning() << "Cannot attach the environment: osgEarth was built without the Prestige NodeKit.";
#endif
}

void EnvironmentGUI::detachEnvironment()
{
    osg::ref_ptr<MapNode> map = currentMapNode();
    osg::ref_ptr<SkyNode> sky = currentSkyNode();
    if (!map.valid() || !sky.valid())
        return;

    const bool wasCyclePlaying = _timeCycle && _timeCycle->playing.load();
    const bool removeWindDebug = _windDebug;
    if (_timeCycle)
        _timeCycle->playing = false;
    if (_cycleUiTimer)
        _cycleUiTimer->stop();
    _updatingControls = true;
    ui->checkBoxCycleTime->setChecked(false);

    osg::ref_ptr<TimeCycle> cycle = _timeCycle.get();
    _windDebug = false;
    ui->chkWindDebug->setChecked(false);
    QPointer<EnvironmentGUI> panel(this);
    const bool queued = queueUpdate([map, sky, cycle, removeWindDebug, panel](osgViewer::Viewer& viewer) {
        if (MapNode::findMapNode(viewer.getSceneData()) != map.get() ||
            findFirstParentOfType<SkyNode>(map.get()) != sky.get())
            return;

        if (cycle.valid())
            sky->removeUpdateCallback(cycle.get());
        if (removeWindDebug)
            ShaderLoader::unload(VirtualProgram::getOrCreate(map->getOrCreateStateSet()), render_wind);
        if (viewer.getSceneData() == sky.get()) {
            sky->removeChild(map.get());
            viewer.setSceneData(map.get());
        }
        else {
            removeGroup(sky.get());
        }
#ifdef OSGEARTH_HAVE_PRESTIGE_NODEKIT
        if (dynamic_cast<osgEarthPrestige::SkyNode*>(sky.get())) {
            viewer.setLight(nullptr);
            viewer.setLightingMode(osg::View::HEADLIGHT);
            if (auto* widget = App::getThreedWindow())
                widget->setViewerColor(widget->getViewerColor());
        }
#endif
        if (panel)
            panel->syncFromSceneLater();
    });
    if (!queued) {
        _windDebug = removeWindDebug;
        if (_timeCycle)
            _timeCycle->playing = wasCyclePlaying;
        ui->checkBoxCycleTime->setChecked(wasCyclePlaying);
        ui->chkWindDebug->setChecked(removeWindDebug);
        _updatingControls = false;
        if (wasCyclePlaying && _cycleUiTimer)
            _cycleUiTimer->start();
        return;
    }
    _updatingControls = false;
    _timeCycle = nullptr;
}

void EnvironmentGUI::resetEnvironment()
{
    osg::ref_ptr<MapNode> map = currentMapNode();
    osg::ref_ptr<SkyNode> sky = currentSkyNode();
    osg::ref_ptr<ShadowCaster> caster = _shadowCaster.get();
    osg::ref_ptr<WindLayer> wind = _windLayer.get();
    if (!map.valid() || !sky.valid()) {
        qWarning() << "Cannot reset environment settings: no sky is attached.";
        return;
    }

    const bool wasCyclePlaying = _timeCycle && _timeCycle->playing.load();
    const float previousCycleSpeed = _timeCycleSpeed;
    const bool previousDetailsVisible = ui->checkBox_2->isChecked();
    const bool previousWindVisible = ui->checkBox->isChecked();
    const bool removeWindDebug = _windDebug;
    if (_timeCycle) {
        _timeCycle->playing = false;
        _timeCycle->hoursPerSecond = 0.25f;
    }
    _timeCycleSpeed = 0.25f;
    _windDebug = false;
    if (_cycleUiTimer)
        _cycleUiTimer->stop();

    _updatingControls = true;
    ui->checkBoxCycleTime->setChecked(false);
    ui->checkBox_2->setChecked(false);
    ui->checkBox->setChecked(false);
    ui->chkWindDebug->setChecked(false);
    setSliderValue(ui->horizontalSlider_spped, _timeCycleSpeed);
    _updatingControls = false;

    QPointer<EnvironmentGUI> panel(this);
    const osg::Vec4 defaultSunColor(1.0f, 1.0f, 1.0f, 1.0f);
    const bool queued = queueUpdate([map, sky, caster, wind, panel](osgViewer::Viewer& viewer) {
        if (MapNode::findMapNode(viewer.getSceneData()) != map.get() ||
            findFirstParentOfType<SkyNode>(map.get()) != sky.get())
            return;

        sky->setDateTime(DateTime());
        sky->setSimulationTimeTracksDateTime(true);
        sky->setLighting(osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);
        sky->setAtmosphereVisible(true);
        sky->setSunVisible(true);
        sky->setMoonVisible(true);
        sky->setStarsVisible(true);
        if (sky->getSunLight()) {
            sky->getSunLight()->setDiffuse(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
            sky->getSunLight()->setAmbient(osg::Vec4(0.033f, 0.033f, 0.033f, 1.0f));
        }

        osg::StateSet* stateSet = sky->getOrCreateStateSet();
        stateSet->getOrCreateUniform("oe_sky_exposure", osg::Uniform::FLOAT)->set(3.3f);
        stateSet->getOrCreateUniform("oe_normal_boost", osg::Uniform::FLOAT)->set(1.0f);
        stateSet->getOrCreateUniform("atmos_haze_cutoff", osg::Uniform::FLOAT)->set(0.0f);
        stateSet->getOrCreateUniform("atmos_haze_strength", osg::Uniform::FLOAT)->set(16.0f);
        if (auto* uniform = stateSet->getUniform("oe_sky_maxAmbientIntensity"))
            uniform->set(0.15f);
        if (auto* uniform = stateSet->getUniform("oe_sky_iblStrength"))
            uniform->set(1.0f);
        if (auto* uniform = stateSet->getUniform("oe_sky_groundReflectance"))
            uniform->set(0.15f);

#ifdef OSGEARTH_HAVE_PRESTIGE_NODEKIT
        if (auto* prestige = dynamic_cast<osgEarthPrestige::SkyNode*>(sky.get())) {
            const osgEarthPrestige::SkyNode::Options defaults;
            prestige->setExposure(defaults.exposure);
            prestige->setSunIntensity(defaults.sunIntensity);
            prestige->setEnvironmentIntensity(defaults.environmentIntensity);
            prestige->setAmbientIntensity(defaults.ambient().get());
            prestige->setCloudLayer(nullptr);
        }
#endif

        if (caster.valid()) {
            caster->setEnabled(true);
            caster->setSubmission(ShadowCaster::AUTOMATIC);
            caster->setCacheEnabled(false);
            caster->setCascades(3, 2500.0f);
            caster->setTextureSize(2048);
            caster->setFilter(ShadowCaster::PCF);
            caster->setFilterRadius(1.5f);
            caster->setShadowColor(0.0f);
            caster->setDepthBias(1.5f);
            caster->setCasterDistance(1000.0f);
            caster->setCascadeBlend(0.1f);
        }
        if (wind.valid())
            wind->setSpeedFactor(1.0f);
        if (panel) {
            panel->syncFromSceneLater();
        }
    });
    if (!queued) {
        _timeCycleSpeed = previousCycleSpeed;
        _windDebug = removeWindDebug;
        if (_timeCycle) {
            _timeCycle->playing = wasCyclePlaying;
            _timeCycle->hoursPerSecond = previousCycleSpeed;
        }
        _updatingControls = true;
        ui->checkBoxCycleTime->setChecked(wasCyclePlaying);
        ui->checkBox_2->setChecked(previousDetailsVisible);
        ui->checkBox->setChecked(previousWindVisible);
        ui->chkWindDebug->setChecked(removeWindDebug);
        setSliderValue(ui->horizontalSlider_spped, previousCycleSpeed);
        _updatingControls = false;
        if (wasCyclePlaying && _cycleUiTimer)
            _cycleUiTimer->start();
        return;
    }
    _sunColor = defaultSunColor;
    refreshSunColorButton(defaultSunColor, false);

    if (removeWindDebug && map.valid()) {
        queueUpdate([map](osgViewer::Viewer&) {
            ShaderLoader::unload(VirtualProgram::getOrCreate(map->getOrCreateStateSet()), render_wind);
        });
    }
}

void EnvironmentGUI::installShadows()
{
    osg::ref_ptr<MapNode> map = currentMapNode();
    osg::ref_ptr<SkyNode> sky = currentSkyNode();
    if (!map.valid() || !sky.valid()) {
        qWarning() << "Cannot install shadows without a map and sky.";
        return;
    }
    QPointer<EnvironmentGUI> panel(this);
    queueUpdate([map, sky, panel](osgViewer::Viewer& viewer) {
        if (MapNode::findMapNode(viewer.getSceneData()) != map.get() ||
            findFirstParentOfType<SkyNode>(map.get()) != sky.get() ||
            findFirstParentOfType<ShadowCaster>(map.get()))
            return;
        auto* terrain = map->getTerrainEngine();
        if (!terrain || !terrain->getResources()) {
            qWarning() << "Cannot install shadows: terrain resources are unavailable.";
            return;
        }
        int textureUnit = -1;
        if (!terrain->getResources()->reserveTextureImageUnit(textureUnit, "ShadowCaster")) {
            qWarning() << "Cannot install shadows: no texture image unit is available.";
            return;
        }
        osg::ref_ptr<ShadowCaster> caster = new ShadowCaster;
        caster->setTextureImageUnit(textureUnit);
        caster->setLight(sky->getSunLight());
        caster->getShadowCastingGroup()->addChild(map->getLayerNodeGroup());
        caster->getShadowCastingGroup()->addChild(terrain->getNode());
        insertParent(caster.get(), map.get());
        if (panel)
            panel->syncFromSceneLater();
    });
}

void EnvironmentGUI::applyCloudOptions()
{
#ifdef OSGEARTH_HAVE_PRESTIGE_NODEKIT
    osg::ref_ptr<MapNode> map = currentMapNode();
    osg::ref_ptr<SkyNode> sky = currentSkyNode();
    auto* prestige = dynamic_cast<osgEarthPrestige::SkyNode*>(sky.get());
    if (!map.valid() || !prestige)
        return;

    osgEarthPrestige::CloudLayer::Options options;
    if (auto* layer = prestige->getCloudLayer())
        options = layer->getOptions();
    options.enabled = ui->checkBox_clouds->isChecked();
    options.quality = static_cast<osgEarthPrestige::CloudLayer::Quality>(
        std::clamp(ui->comboCloudQuality->currentIndex(), 0, 2));
    options.coverage = sliderValue(ui->horizontalSlider_cloudCoverage);
    options.density = sliderValue(ui->horizontalSlider_cloudDensity);
    options.baseAltitude = sliderValue(ui->horizontalSlider_cloudBase);
    options.topAltitude = sliderValue(ui->horizontalSlider_cloudTop);
    options.fadeStartAltitude = sliderValue(ui->horizontalSlider_cloudFadeStart);
    options.fadeEndAltitude = sliderValue(ui->horizontalSlider_cloudHiddenAbove);
    options.size = sliderValue(ui->horizontalSlider_cloudSize);
    options.erosion = sliderValue(ui->horizontalSlider_cloudErosion);
    options.detailFiltering = ui->_farDensityDetails->isChecked();
    options.shadowStrength = sliderValue(ui->horizontalSlider_cloudShadows);
    options.shadowCoverage = sliderValue(ui->horizontalSlider_shadowCoverage) * 1000.0f;
    options.farShadows = ui->checkBox_farCloudShadow->isChecked();
    options.farShadowCoverage = sliderValue(ui->horizontalSlider_farShadowCoverage) * 1000.0f;
    options.crepuscularRays = ui->checkBox_crepussularRays->isChecked();
    options.rayQuality = static_cast<osgEarthPrestige::CloudLayer::Quality>(
        qRound(sliderValue(ui->horizontalSlider_rayQuality)));
    options.rayStrength = sliderValue(ui->horizontalSlider_rayStrength);
    options.rayIntensity = sliderValue(ui->horizontalSlider_rayIntensity);
    options.rayHaze = sliderValue(ui->horizontalSlider_rayHaze);
    options.rayDistance = sliderValue(ui->horizontalSlider_rayDistance);

    QPointer<EnvironmentGUI> panel(this);
    queueUpdate([map, sky, options, panel](osgViewer::Viewer& viewer) {
        if (MapNode::findMapNode(viewer.getSceneData()) != map.get() ||
            findFirstParentOfType<SkyNode>(map.get()) != sky.get())
            return;
        auto* prestigeSky = dynamic_cast<osgEarthPrestige::SkyNode*>(sky.get());
        if (!prestigeSky)
            return;
        if (auto* layer = prestigeSky->getCloudLayer())
            layer->setOptions(options);
        else
            prestigeSky->setCloudLayer(new osgEarthPrestige::CloudLayer(options));
        if (panel)
            panel->syncFromSceneLater();
    });
#endif
}

void EnvironmentGUI::syncFromScene()
{
    _updatingControls = true;
    auto* map = currentMapNode();
    auto* sky = currentSkyNode();
    if (_skyNode.get() != sky) {
        if (_timeCycle) {
            _timeCycle->playing = false;
            osg::ref_ptr<SkyNode> previous = _skyNode.get();
            osg::ref_ptr<TimeCycle> previousCycle = _timeCycle.get();
            if (previous.valid()) {
                queueUpdate([previous, previousCycle](osgViewer::Viewer&) {
                    if (previous.valid())
                        previous->removeUpdateCallback(previousCycle.get());
                });
            }
            _timeCycle = nullptr;
        }
        if (sky) {
            _timeCycle = new TimeCycle;
            _timeCycle->hoursPerSecond = _timeCycleSpeed;
            osg::ref_ptr<SkyNode> currentSky = sky;
            osg::ref_ptr<MapNode> currentMap = map;
            osg::ref_ptr<TimeCycle> cycle = _timeCycle.get();
            queueUpdate([currentSky, currentMap, cycle](osgViewer::Viewer& viewer) {
                if (currentSky.valid() && currentMap.valid() &&
                    MapNode::findMapNode(viewer.getSceneData()) == currentMap.get() &&
                    findFirstParentOfType<SkyNode>(currentMap.get()) == currentSky.get()) {
                    currentSky->setSimulationTimeTracksDateTime(true);
                    currentSky->addUpdateCallback(cycle.get());
                }
            });
        }
    }
    _mapNode = map;
    _skyNode = sky;
    _shadowCaster = map ? findFirstParentOfType<ShadowCaster>(map) : nullptr;

    _windLayer = nullptr;
    if (map && map->getMap()) {
        std::vector<osg::ref_ptr<WindLayer>> winds;
        map->getMap()->getLayers(winds);
        if (!winds.empty())
            _windLayer = winds.front().get();
    }

    const bool hasSky = sky != nullptr;
    bool isPrestigeSky = false;
#ifdef OSGEARTH_HAVE_PRESTIGE_NODEKIT
    auto* prestigeSky = dynamic_cast<osgEarthPrestige::SkyNode*>(sky);
    isPrestigeSky = prestigeSky != nullptr;
    ui->grpPrestige->setVisible(isPrestigeSky);
    ui->grpPrestige->setEnabled(isPrestigeSky);
    ui->horizontalSlider_sunIntensity->setEnabled(isPrestigeSky);
    ui->horizontalSlider_skyEnv->setEnabled(isPrestigeSky);
    ui->horizontalSlider_nightFill->setEnabled(isPrestigeSky);
    if (prestigeSky) {
        const auto& options = prestigeSky->getOptions();
        const char* presetName = options.preset == osgEarthPrestige::SkyNode::FLAT ? "Flat" :
            options.preset == osgEarthPrestige::SkyNode::HIGH ? "High" : "Balanced";
        ui->lblPrestigeInfo->setText(tr("Prestige sky / %1").arg(QString::fromLatin1(presetName)));
        setSliderValue(ui->horizontalSlider_Exposure, options.exposure);
        setSliderValue(ui->horizontalSlider_sunIntensity, options.sunIntensity);
        setSliderValue(ui->horizontalSlider_skyEnv, options.environmentIntensity);
        setSliderValue(ui->horizontalSlider_AmbientMin, options.ambient().get());

        auto* clouds = prestigeSky->getCloudLayer();
        const bool hasClouds = clouds != nullptr;
        const osgEarthPrestige::CloudLayer::Options cloudOptions =
            clouds ? clouds->getOptions() : osgEarthPrestige::CloudLayer::Options();
        ui->checkBox_clouds->setChecked(clouds && cloudOptions.enabled);
        ui->comboCloudQuality->setCurrentIndex(int(cloudOptions.quality));
        setSliderValue(ui->horizontalSlider_cloudCoverage, cloudOptions.coverage);
        setSliderValue(ui->horizontalSlider_cloudDensity, cloudOptions.density);
        setSliderValue(ui->horizontalSlider_cloudBase, cloudOptions.baseAltitude);
        setSliderValue(ui->horizontalSlider_cloudTop, cloudOptions.topAltitude);
        setSliderValue(ui->horizontalSlider_cloudFadeStart, cloudOptions.fadeStartAltitude);
        setSliderValue(ui->horizontalSlider_cloudHiddenAbove, cloudOptions.fadeEndAltitude);
        setSliderValue(ui->horizontalSlider_cloudSize, cloudOptions.size);
        setSliderValue(ui->horizontalSlider_cloudErosion, cloudOptions.erosion);
        ui->_farDensityDetails->setChecked(cloudOptions.detailFiltering);
        setSliderValue(ui->horizontalSlider_cloudShadows, cloudOptions.shadowStrength);
        setSliderValue(ui->horizontalSlider_shadowCoverage, cloudOptions.shadowCoverage * 0.001f);
        ui->checkBox_farCloudShadow->setChecked(cloudOptions.farShadows);
        setSliderValue(ui->horizontalSlider_farShadowCoverage, cloudOptions.farShadowCoverage * 0.001f);
        ui->checkBox_crepussularRays->setChecked(cloudOptions.crepuscularRays);
        setSliderValue(ui->horizontalSlider_rayQuality, float(cloudOptions.rayQuality));
        setSliderValue(ui->horizontalSlider_rayStrength, cloudOptions.rayStrength);
        setSliderValue(ui->horizontalSlider_rayIntensity, cloudOptions.rayIntensity);
        setSliderValue(ui->horizontalSlider_rayHaze, cloudOptions.rayHaze);
        setSliderValue(ui->horizontalSlider_rayDistance, cloudOptions.rayDistance);

        configureSlider(ui->horizontalSlider_cloudBase, 1, 0.0f, 10000.0f);
        configureSlider(ui->horizontalSlider_cloudTop, 1,
            100.0f, 20000.0f);
        configureSlider(ui->horizontalSlider_cloudFadeStart, 1,
            cloudOptions.topAltitude, 100000.0f);
        configureSlider(ui->horizontalSlider_cloudHiddenAbove, 1,
            cloudOptions.fadeStartAltitude + 100.0f, 200000.0f);
        configureSlider(ui->horizontalSlider_farShadowCoverage, 1000,
            cloudOptions.shadowCoverage * 0.001f * 1.25f, 1000.0f);

        const auto setCloudFieldVisible = [hasClouds](QWidget* widget) {
            widget->setVisible(hasClouds);
        };
        QWidget* cloudWidgets[] = {
            ui->lblCloudQuality, ui->comboCloudQuality,
            ui->lblCloudCoverage, ui->lblCloudDensity, ui->label_6, ui->label_7,
            ui->label_9, ui->label_10, ui->label_11, ui->label_12, ui->label_13,
            ui->label_14, ui->label_15, ui->horizontalSlider_cloudCoverage,
            ui->horizontalSlider_cloudDensity, ui->horizontalSlider_cloudBase,
            ui->horizontalSlider_cloudTop, ui->horizontalSlider_cloudFadeStart,
            ui->horizontalSlider_cloudHiddenAbove, ui->horizontalSlider_cloudSize,
            ui->horizontalSlider_cloudErosion, ui->_farDensityDetails,
            ui->horizontalSlider_cloudShadows, ui->horizontalSlider_shadowCoverage
        };
        for (QWidget* widget : cloudWidgets)
            setCloudFieldVisible(widget);
        ui->label_16->setVisible(hasClouds && cloudOptions.farShadows);
        ui->checkBox_farCloudShadow->setVisible(hasClouds);
        ui->lblFarShadowCoverage->setVisible(hasClouds && cloudOptions.farShadows);
        ui->horizontalSlider_farShadowCoverage->setVisible(hasClouds && cloudOptions.farShadows);
        ui->label_17->setVisible(hasClouds);
        ui->checkBox_crepussularRays->setVisible(hasClouds);
        const bool showRays = hasClouds && cloudOptions.crepuscularRays;
        QWidget* rayWidgets[] = {
            ui->lblRayQuality, ui->horizontalSlider_rayQuality,
            ui->lblRayStrength, ui->horizontalSlider_rayStrength, ui->lblRayIntensity,
            ui->horizontalSlider_rayIntensity, ui->lblRayHaze, ui->horizontalSlider_rayHaze,
            ui->lblRayDistance, ui->horizontalSlider_rayDistance
        };
        for (QWidget* widget : rayWidgets)
            widget->setVisible(showRays);
    }
#else
    ui->grpPrestige->hide();
    ui->horizontalSlider_sunIntensity->setEnabled(false);
    ui->horizontalSlider_skyEnv->setEnabled(false);
    ui->horizontalSlider_nightFill->setEnabled(false);
#endif
    configureSlider(ui->horizontalSlider_Exposure, 1000,
        isPrestigeSky ? 0.01f : 1.0f, isPrestigeSky ? 16.0f : 10.0f);
    ui->horizontalSlider_AmbientMin->setEnabled(!isPrestigeSky);
    ui->horizontalSlider_AmbientMax->setEnabled(!isPrestigeSky);
    ui->horizontalSlider_NormalBoost->setEnabled(!isPrestigeSky);
    ui->lblAmbientMin->setVisible(!isPrestigeSky);
    ui->horizontalSlider_AmbientMin->setVisible(!isPrestigeSky);
    ui->lblAmbientMax->setVisible(false);
    ui->horizontalSlider_AmbientMax->setVisible(false);
    ui->lblSunIntensity->setVisible(isPrestigeSky);
    ui->horizontalSlider_sunIntensity->setVisible(isPrestigeSky);
    ui->lblSkyEnvironment->setVisible(isPrestigeSky);
    ui->horizontalSlider_skyEnv->setVisible(isPrestigeSky);
    ui->label_8->setVisible(isPrestigeSky);
    ui->horizontalSlider_nightFill->setVisible(isPrestigeSky);
    ui->lblGroundReflectance->setVisible(false);
    ui->horizontalSlider_groundReflectance->setVisible(false);
    ui->lblNormalBoost->setVisible(!isPrestigeSky);
    ui->horizontalSlider_NormalBoost->setVisible(!isPrestigeSky);
    ui->lblHazeCutoff->setVisible(!isPrestigeSky && hasSky && ui->checkBox_2->isChecked());
    ui->horizontalSlider_hazeCutoff->setVisible(!isPrestigeSky && hasSky && ui->checkBox_2->isChecked());
    ui->lblHazeStrength->setVisible(!isPrestigeSky && hasSky && ui->checkBox_2->isChecked());
    ui->horizontalSlider_hazeStrength->setVisible(!isPrestigeSky && hasSky && ui->checkBox_2->isChecked());

    auto* widget = App::getThreedWindow();
    auto* viewer = widget ? widget->getViewer() : nullptr;
    const bool hasActiveMap = map && viewer && viewer->getSceneData() &&
        MapNode::findMapNode(viewer->getSceneData()) == map;
#ifdef OSGEARTH_HAVE_PRESTIGE_NODEKIT
    ui->pushButton_toggleEnvironment->setEnabled(hasActiveMap);
    ui->pushButton_toggleEnvironment->setText(hasSky ? tr("Detach Environment") : tr("Attach Environment"));
#else
    ui->pushButton_toggleEnvironment->setEnabled(hasSky);
    ui->pushButton_toggleEnvironment->setText(hasSky ? tr("Detach Environment") : tr("Prestige unavailable"));
#endif
    ui->pushButton_toggleEnvironment->setToolTip(hasSky
        ? tr("Detach the current sky environment from the map.")
        : tr("Attach a Prestige environment to the map."));
    ui->pushButton_reset->setEnabled(hasSky);
    ui->btnNow->setEnabled(hasSky);
    ui->chkLighting->setEnabled(hasSky);
    ui->chkShadows->setEnabled(hasSky);
    ui->grpDateTime->setEnabled(hasSky);
    ui->grpLighting->setEnabled(hasSky);
    ui->grpWind->setEnabled(hasSky && _windLayer.valid());
    ui->grpDetails->setEnabled(hasSky);
    ui->checkBox->setEnabled(_windLayer.valid());
    ui->checkBox_2->setEnabled(hasSky);
    ui->checkBoxCycleTime->setEnabled(hasSky);
    ui->spinWindPower->setEnabled(_windLayer.valid());
    ui->chkWindDebug->setEnabled(map != nullptr);
    ui->horizontalSlider_cascades->setEnabled(_shadowCaster.valid());
    ui->horizontalSlider_distance->setEnabled(_shadowCaster.valid());
    ui->comboBox->setEnabled(_shadowCaster.valid());
    ui->checkBox_cacheStaticCaster->setEnabled(_shadowCaster.valid());
    ui->comboBox_mapResolution->setEnabled(_shadowCaster.valid());
    ui->comboBox_filtering->setEnabled(_shadowCaster.valid());
    ui->horizontalSlider_softness->setEnabled(_shadowCaster.valid());
    ui->horizontalSlider_sunlightInShadow->setEnabled(_shadowCaster.valid());
    ui->horizontalSlider_deothBias->setEnabled(_shadowCaster.valid());
    ui->horizontalSlider_casterReach->setEnabled(_shadowCaster.valid());
    ui->horizontalSlider_casterBlend->setEnabled(_shadowCaster.valid());

    if (sky) {
        const DateTime dateTime = sky->getDateTime();
        ui->horizontalSlider_year->setValue(dateTime.year());
        ui->horizontalSlider_month->setValue(dateTime.month());
        ui->horizontalSlider_day->setValue(dateTime.day());
        setSliderValue(ui->horizontalSlider_Hour, float(dateTime.hours()));
        ui->chkLighting->setChecked((sky->getLighting() & osg::StateAttribute::ON) != 0);
        ui->chkShadows->setChecked(_shadowCaster.valid() && _shadowCaster->getEnabled());
        ui->chkAtmosphere->setChecked(sky->getAtmosphereVisible());
        ui->chkSun->setChecked(sky->getSunVisible());
        ui->chkMoon->setChecked(sky->getMoonVisible());
        ui->chkStars->setChecked(sky->getStarsVisible());

        if (sky->getSunLight()) {
            const osg::Vec4 ambient = sky->getSunLight()->getAmbient();
            const osg::Vec4 diffuse = sky->getSunLight()->getDiffuse();
            refreshSunColorButton(diffuse);
            if (!isPrestigeSky)
                setSliderValue(ui->horizontalSlider_AmbientMin, ambient.r());
        }

        osg::StateSet* stateSet = sky->getOrCreateStateSet();
        if (!isPrestigeSky) {
            if (auto* uniform = stateSet->getUniform("oe_sky_exposure")) {
                float value = 3.3f;
                uniform->get(value);
                setSliderValue(ui->horizontalSlider_Exposure, value);
            }
            if (auto* uniform = stateSet->getUniform("oe_sky_maxAmbientIntensity")) {
                float value = 0.15f;
                uniform->get(value);
                setSliderValue(ui->horizontalSlider_AmbientMax, value);
                ui->lblAmbientMax->setVisible(true);
                ui->horizontalSlider_AmbientMax->setVisible(true);
            }
            else {
                ui->lblAmbientMax->setVisible(false);
                ui->horizontalSlider_AmbientMax->setVisible(false);
            }
            if (auto* uniform = stateSet->getUniform("oe_normal_boost")) {
                float value = 1.0f;
                uniform->get(value);
                setSliderValue(ui->horizontalSlider_NormalBoost, value);
            }
            if (auto* uniform = stateSet->getUniform("atmos_haze_cutoff")) {
                uniform->get(_hazeCutoff);
                setSliderValue(ui->horizontalSlider_hazeCutoff, _hazeCutoff);
            }
            else {
                _hazeCutoff = 0.0f;
                setSliderValue(ui->horizontalSlider_hazeCutoff, _hazeCutoff);
            }
            if (auto* uniform = stateSet->getUniform("atmos_haze_strength")) {
                uniform->get(_hazeStrength);
                setSliderValue(ui->horizontalSlider_hazeStrength, _hazeStrength);
            }
            else {
                _hazeStrength = 16.0f;
                setSliderValue(ui->horizontalSlider_hazeStrength, _hazeStrength);
            }

            auto* environmentStrength = stateSet->getUniform("oe_sky_iblStrength");
            auto* environmentDefine = stateSet->getDefinePair("OE_SKY_ENVIRONMENT");
            const bool environmentActive = environmentStrength && environmentDefine &&
                (environmentDefine->second & osg::StateAttribute::ON) != 0;
            ui->lblSkyEnvironment->setVisible(environmentActive);
            ui->horizontalSlider_skyEnv->setVisible(environmentActive);
            ui->horizontalSlider_skyEnv->setEnabled(environmentActive);
            if (environmentActive) {
                float value = 1.0f;
                environmentStrength->get(value);
                setSliderValue(ui->horizontalSlider_skyEnv, value);
            }

            auto* groundReflectance = stateSet->getUniform("oe_sky_groundReflectance");
            ui->lblGroundReflectance->setVisible(environmentActive && groundReflectance);
            ui->horizontalSlider_groundReflectance->setVisible(environmentActive && groundReflectance);
            ui->horizontalSlider_groundReflectance->setEnabled(environmentActive && groundReflectance);
            if (environmentActive && groundReflectance) {
                groundReflectance->get(_groundReflectance);
                setSliderValue(ui->horizontalSlider_groundReflectance, _groundReflectance);
            }
        }
        else {
            ui->lblSkyEnvironment->setVisible(true);
            ui->horizontalSlider_skyEnv->setVisible(true);
            ui->lblAmbientMax->setVisible(false);
            ui->horizontalSlider_AmbientMax->setVisible(false);
            ui->lblGroundReflectance->setVisible(false);
            ui->horizontalSlider_groundReflectance->setVisible(false);
            ui->lblNormalBoost->setVisible(false);
            ui->horizontalSlider_NormalBoost->setVisible(false);
            ui->lblHazeCutoff->setVisible(false);
            ui->horizontalSlider_hazeCutoff->setVisible(false);
            ui->lblHazeStrength->setVisible(false);
            ui->horizontalSlider_hazeStrength->setVisible(false);
        }

        if (auto* ephemeris = sky->getEphemeris()) {
            const auto sun = ephemeris->getSunPosition(dateTime);
            ui->lblSunPosValue->setText(tr("RA (%1) Decl (%2)")
                .arg(sun.rightAscension.as(Units::DEGREES), 0, 'f', 2)
                .arg(sun.declination.as(Units::DEGREES), 0, 'f', 2));
            const auto moon = ephemeris->getMoonPosition(dateTime);
            ui->lblMoonPosValue->setText(tr("RA (%1) Decl (%2)")
                .arg(moon.rightAscension.as(Units::DEGREES), 0, 'f', 2)
                .arg(moon.declination.as(Units::DEGREES), 0, 'f', 2));
        }
    }
    else {
        ui->chkShadows->setChecked(false);
    }

    if (_timeCycle) {
        ui->checkBoxCycleTime->setChecked(_timeCycle->playing.load());
        _timeCycleSpeed = _timeCycle->hoursPerSecond.load();
        if (_timeCycle->playing.load())
            _cycleUiTimer->start();
        else
            _cycleUiTimer->stop();
    }
    else {
        ui->checkBoxCycleTime->setChecked(false);
        _cycleUiTimer->stop();
    }
    setSliderValue(ui->horizontalSlider_spped, _timeCycleSpeed);
    ui->label_4->setText(tr("%1 seconds").arg(24.0f / _timeCycleSpeed, 0, 'f', 1));

#ifdef OSGEARTH_HAVE_PRESTIGE_NODEKIT
    if (isPrestigeSky) {
        auto* prestigeSky = dynamic_cast<osgEarthPrestige::SkyNode*>(sky);
        if (prestigeSky && prestigeSky->getCloudLayer() && prestigeSky->getCloudLayer()->getWindLayer())
            _windLayer = prestigeSky->getCloudLayer()->getWindLayer();
    }
#endif
    if (_windLayer.valid()) {
        ui->spinWindPower->setValue(_windLayer->getSpeedFactor());
    }

    if (_shadowCaster.valid()) {
        const auto& ranges = _shadowCaster->getRanges();
        const int count = std::max(1, int(ranges.size()) - 1);
        setSliderValue(ui->horizontalSlider_cascades, float(count));
        if (!ranges.empty())
            setSliderValue(ui->horizontalSlider_distance, ranges.back());
        ui->comboBox->setCurrentIndex(int(_shadowCaster->getSubmission()));
        ui->checkBox_cacheStaticCaster->setChecked(_shadowCaster->getCacheEnabled());
        const int resolutionIndex = ui->comboBox_mapResolution->findText(
            QString::number(_shadowCaster->getTextureSize()));
        if (resolutionIndex >= 0)
            ui->comboBox_mapResolution->setCurrentIndex(resolutionIndex);
        ui->comboBox_filtering->setCurrentIndex(int(_shadowCaster->getFilter()));
        setSliderValue(ui->horizontalSlider_softness, _shadowCaster->getFilterRadius());
        setSliderValue(ui->horizontalSlider_sunlightInShadow, _shadowCaster->getShadowColor());
        setSliderValue(ui->horizontalSlider_deothBias, _shadowCaster->getDepthBias());
        setSliderValue(ui->horizontalSlider_casterReach, _shadowCaster->getCasterDistance());
        setSliderValue(ui->horizontalSlider_casterBlend, _shadowCaster->getCascadeBlend());
        ui->label_31->setText(tr("%1 rendered, %2 reused")
            .arg(_shadowCaster->getRenderedCascades())
            .arg(_shadowCaster->getReusedCascades()));
    }
    else {
        ui->label_31->setText(tr("No shadow caster"));
    }

    ui->grpDetails->setVisible(ui->checkBox_2->isChecked());
    ui->grpWind->setVisible(ui->checkBox->isChecked());
    ui->lblDay->setVisible(ui->checkBox_2->isChecked());
    ui->lblMonth->setVisible(ui->checkBox_2->isChecked());
    ui->lblYear->setVisible(ui->checkBox_2->isChecked());
    ui->horizontalSlider_day->setVisible(ui->checkBox_2->isChecked());
    ui->horizontalSlider_month->setVisible(ui->checkBox_2->isChecked());
    ui->horizontalSlider_year->setVisible(ui->checkBox_2->isChecked());
    const bool showShadowDetails = ui->checkBox_2->isChecked() &&
        ui->chkShadows->isChecked() && _shadowCaster.valid();
    QWidget* shadowWidgets[] = {
        ui->label_18, ui->label_19, ui->comboBox,
        ui->label_20, ui->checkBox_cacheStaticCaster, ui->label_21, ui->label_31,
        ui->label_22, ui->horizontalSlider_cascades, ui->label_23, ui->horizontalSlider_distance,
        ui->label_24, ui->comboBox_mapResolution, ui->label_25, ui->comboBox_filtering,
        ui->label_26, ui->horizontalSlider_softness, ui->label_27,
        ui->horizontalSlider_sunlightInShadow, ui->label_28, ui->horizontalSlider_deothBias,
        ui->label_29, ui->horizontalSlider_casterReach, ui->label_30, ui->horizontalSlider_casterBlend
    };
    for (QWidget* widget : shadowWidgets)
        widget->setVisible(showShadowDetails);
    ui->chkWindDebug->setChecked(_windDebug);
    _updatingControls = false;
}

void EnvironmentGUI::syncFromSceneLater()
{
    QPointer<EnvironmentGUI> panel(this);
    QMetaObject::invokeMethod(this, [panel] {
        if (panel)
            panel->syncFromScene();
    }, Qt::QueuedConnection);
}

void EnvironmentGUI::refreshSunColorButton(const osg::Vec4& color, bool remember)
{
    if (remember)
        _sunColor = color;
    const QColor qcolor = QColor::fromRgbF(color.r(), color.g(), color.b());
    ui->pushButton_sunColor->setPalette(QPalette(qcolor));
    ui->pushButton_sunColor->setAutoFillBackground(true);
    ui->label_sunColor->setText(QStringLiteral("[%1, %2, %3]")
        .arg(color.r(), 0, 'f', 2)
        .arg(color.g(), 0, 'f', 2)
        .arg(color.b(), 0, 'f', 2));
}
