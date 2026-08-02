#include "EnvironmentGUI.h"
#include "ui_EnvironmentGUI.h"
#include "OSGViewerWidget.h"

#include <QColorDialog>
#include <osgEarth/NodeUtils>
#include <osgEarth/ShaderLoader>
#include <osgEarth/VirtualProgram>

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

    const float oe_wind_animation_speed = 2.8 * MAX_WIND_SPEED * speed; // 32.0
    int cx = int(coordProj.x - int(osg_FrameTime*oe_wind_animation_speed));
    int ci = cx % 32;
    const int stipple = 0x00000001;
    int pattern32 = 0xffffffff & (stipple & (1 << ci));
    if (pattern32 != 0)
        frag_out = vec4(0.5,0.5,1,0.5);
  #endif
}
)";
}


EnvironmentGUI::EnvironmentGUI(QWidget *parent)
    : DockWidgetPanels(parent)
    , ui(new Ui::EnvironmentGUI)
{
    ui->setupUi(this);
    setObjectName("EnvironmentDock");
    DateTime now;
    _hour = now.hours(), _day = now.day(), _month = now.month(), _year = now.year();

}

EnvironmentGUI::~EnvironmentGUI()
{

}