#include "VisibilityTestArea.h"


#include <osg/LineWidth>
#include <osg/ShapeDrawable>
#include <osg/TextureCubeMap>
#include <osg/BoundingSphere>
#include <osg/PositionAttitudeTransform>
#include <osgDB/ReadFile>
#include <osgDB/Registry>

#include <osgUtil/SmoothingVisitor>
#include <osgSim/OverlayNode>

#include <osg/AutoTransform>

#include "ViewshedShaders.h"

#define USE_OSG_PROGRAM  0

// 1. Add these Shader Strings at the top of your file
static const char* debugVert =
    "#version 330\n"
    "layout(location = 0) in vec4 osg_Vertex;\n"
    "layout(location = 8) in vec4 osg_MultiTexCoord0;\n"
    "uniform mat4 osg_ModelViewProjectionMatrix;\n"
    "out vec2 texCoord;\n"
    "void main() {\n"
    "  gl_Position = osg_ModelViewProjectionMatrix * osg_Vertex;\n"
    "  texCoord = osg_MultiTexCoord0.xy;\n"
    "}\n";

static const char* debugFrag =
    "#version 330\n"
    "uniform samplerCube depthMap;\n"
    "in vec2 texCoord;\n"
    "out vec4 color;\n"
    "void main() {\n"
    "  vec3 dir = vec3(texCoord.x * 2.0 - 1.0, texCoord.y * 2.0 - 1.0, 1.0);\n"
    "  float rawDepth = texture(depthMap, dir).r;\n"
    "  \n"
    "  // If it's all white, try raising it to a power to see the contrast\n"
    "  float visualized = pow(rawDepth, 50.0);\n"
    "  color = vec4(vec3(visualized), 1.0);\n"
    "}\n";


// 2. Add this helper function to your class
void VisibilityTestArea::setupDebugHUD() {
    osg::Camera* hud = new osg::Camera;
    hud->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    hud->setProjectionMatrixAsOrtho2D(0, 1280, 0, 720);
    hud->setViewMatrix(osg::Matrix::identity());
    hud->setRenderOrder(osg::Camera::POST_RENDER);
    hud->setAllowEventFocus(false);
    hud->setClearMask(GL_DEPTH_BUFFER_BIT);

    // Create a quad in the bottom-left corner
    osg::Geometry* quad = osg::createTexturedQuadGeometry(
        osg::Vec3(10, 10, 0), osg::Vec3(300, 0, 0), osg::Vec3(0, 300, 0));
    osg::Geode* geode = new osg::Geode;
    geode->addDrawable(quad);
    hud->addChild(geode);

    osg::StateSet* ss = geode->getOrCreateStateSet();
    ss->setTextureAttributeAndModes(0, depthMap.get(), osg::StateAttribute::ON);

    osg::Program* prog = new osg::Program;
    prog->addShader(new osg::Shader(osg::Shader::VERTEX, debugVert));
    prog->addShader(new osg::Shader(osg::Shader::FRAGMENT, debugFrag));
    ss->setAttributeAndModes(prog, osg::StateAttribute::ON);

    ss->addUniform(new osg::Uniform("depthMap", 0));
    ss->addUniform(_farPlaneUniform);
    ss->addUniform(_nearPlaneUniform);

    _parentScene->addChild(hud);
}

enum TraversalOption
{
    INTERSECT_IGNORE = 0x00000004,
    TRAVERSAL_IGNORE = 0x00000010
};

static const int   SM_TEXTURE_WIDTH  = 1024;



VisibilityTestArea::VisibilityTestArea(osg::Group* sceneRoot, osgViewer::Viewer* viewer,osg::Vec3 lightSource, int radius):
    _shadowedScene(sceneRoot),
    _mainViewer(viewer),
    _lightSource(lightSource),
    _viweingRadius(radius)
{
        _parentScene = _shadowedScene->getParent(0);
}

osg::Camera * VisibilityTestArea::generateCubeCamera(osg::ref_ptr<osg::TextureCubeMap> cubeMap, unsigned face, osg::Camera::BufferComponent component)
{
    osg::ref_ptr<osg::Camera>  camera = new osg::Camera;

    camera->setClearMask(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

    camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    camera->setRenderOrder(osg::Camera::PRE_RENDER);
    camera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
    camera->setViewport(0, 0, SM_TEXTURE_WIDTH, SM_TEXTURE_WIDTH);
    camera->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
    camera->attach(component, cubeMap, 0, face);

    camera->setNodeMask(0xffffffff & (~INTERSECT_IGNORE));
    return camera.release();
}

void  VisibilityTestArea::updateAttributes()
{
    // Light source info
    osg::Vec3  lightPos = _lightSource;

    std::vector<osg::Matrix>  shadowViews;
    shadowViews.push_back(
        osg::Matrix::lookAt(lightPos, lightPos + osg::Vec3(1.0, 0.0, 0.0), osg::Vec3(0.0, -1.0, 0.0)));
    shadowViews.push_back(
        osg::Matrix::lookAt(lightPos, lightPos + osg::Vec3(-1.0, 0.0, 0.0), osg::Vec3(0.0, -1.0, 0.0)));
    shadowViews.push_back(
        osg::Matrix::lookAt(lightPos, lightPos + osg::Vec3(0.0, 1.0, 0.0), osg::Vec3(0.0, 0.0, 1.0)));
    shadowViews.push_back(
        osg::Matrix::lookAt(lightPos, lightPos + osg::Vec3(0.0, -1.0, 0.0), osg::Vec3(0.0, 0.0, -1.0)));
    shadowViews.push_back(
        osg::Matrix::lookAt(lightPos, lightPos + osg::Vec3(0.0, 0.0, 1.0), osg::Vec3(0.0, -1.0, 0.0)));
    shadowViews.push_back(
        osg::Matrix::lookAt(lightPos, lightPos + osg::Vec3(0.0, 0.0, -1.0), osg::Vec3(0.0, -1.0, 0.0)));

    // Update light source info for shadow map
    for (int i = 0; i < 6; i++)
    {
        auto  depthCamera = _depthCameras[i];
        depthCamera->setViewMatrix(shadowViews[i]);
    }

    if(!_inverseViewUniform[0].valid())
    {
        for (int i = 0; i < 6; i++){
            _inverseViewUniform[i] = new osg::Uniform("inverse_view", osg::Matrixf::inverse(shadowViews[i]));
        }
        for (int i = 0; i < 6; i++)
        {
            auto  depthCamera = _depthCameras[i];
            depthCamera->getOrCreateStateSet()->addUniform(_inverseViewUniform[i]);
        }
    }
    else{
        for (int i = 0; i < 6; i++){
            _inverseViewUniform[i]->set(osg::Matrixf::inverse(shadowViews[i]));
        }
    }
}

void  VisibilityTestArea::buildModel()
{

    _mainViewer->getCamera()->getGraphicsContext()->getState()->setUseModelViewAndProjectionUniforms(true);

    depthMap = new osg::TextureCubeMap;
    depthMap->setTextureSize(SM_TEXTURE_WIDTH, SM_TEXTURE_WIDTH);
    depthMap->setInternalFormat(GL_DEPTH_COMPONENT);
    depthMap->setSourceFormat(GL_DEPTH_COMPONENT);
    depthMap->setSourceType(GL_FLOAT);
    depthMap->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
    depthMap->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
    depthMap->setWrap(osg::Texture::WRAP_R, osg::Texture::CLAMP_TO_EDGE);
    depthMap->setFilter(osg::Texture::MIN_FILTER, osg::Texture::NEAREST);
    depthMap->setFilter(osg::Texture::MAG_FILTER, osg::Texture::NEAREST);


    // Light source in shader
    far_plane  = (float)_viweingRadius + _farPlanOffSet;

    _lightPosUniform = new osg::Uniform("lightPos", _lightSource);
    _viewRadiusUniform = new osg::Uniform("user_area", (float)_viweingRadius);
    _farPlaneUniform = new osg::Uniform("far_plane",far_plane);
    _nearPlaneUniform = new osg::Uniform("near_plane", near_plane);

    _visibleColorUniform = new osg::Uniform("visibleColor", visibleColor);
    _invisibleColorUniform = new osg::Uniform("invisibleColor", invisibleColor);

    _baseTextureUniform = new osg::Uniform("baseTexture", 0);
    _shadowMapUniform = new osg::Uniform("shadowMap", 1);

     // Generate shadow map cameras and corresponding textures
    osg::Matrix  shadowProj = osg::Matrix::perspective(_verticalFOV, SM_TEXTURE_WIDTH / SM_TEXTURE_WIDTH, near_plane, far_plane);


#if USE_OSG_PROGRAM
    osg::Program* _depthProgram = new osg::Program;
    _depthProgram->addShader(new osg::Shader(osg::Shader::VERTEX, depthMapVert));
    _depthProgram->addShader(new osg::Shader(osg::Shader::FRAGMENT, depthMapFrag));

    osg::Program* _visibilityProgram = new osg::Program;
    _visibilityProgram->addShader(new osg::Shader(osg::Shader::VERTEX, visibilityShaderVert));
    _visibilityProgram->addShader(new osg::Shader(osg::Shader::FRAGMENT, visibilityShaderFrag));

#else

    osgEarth::VirtualProgram* _depthCamerasVP = new osgEarth::VirtualProgram();
    _depthCamerasVP->setFunction("depthMapVertex",depthMapVertVP,osgEarth::VirtualProgram::LOCATION_VERTEX_VIEW);
    _depthCamerasVP->setFunction("depthMapFragment",depthMapFragVP,osgEarth::VirtualProgram::LOCATION_FRAGMENT_COLORING);

    osgEarth::VirtualProgram* _visibilityShaderVP = new osgEarth::VirtualProgram();
    _visibilityShaderVP->setFunction("visibilityVertex",visibilityShaderVertVP,osgEarth::VirtualProgram::LOCATION_VERTEX_VIEW);
    _visibilityShaderVP->setFunction("visibilityFragment",visibilityShaderFragVP,osgEarth::VirtualProgram::LOCATION_FRAGMENT_COLORING);

#endif



    // Generate one camera for each side of the shadow cubemap
    for (int i = 0; i < 6; i++)
    {
        _depthCameras[i] = generateCubeCamera(depthMap, i, osg::Camera::DEPTH_BUFFER);
        _depthCameras[i]->setProjectionMatrix(shadowProj);
#if USE_OSG_PROGRAM
    _depthCameras[i]->getOrCreateStateSet()->setAttributeAndModes(_depthProgram, osg::StateAttribute::ON);
#else
        _depthCameras[i]->getOrCreateStateSet()->setAttributeAndModes(_depthCamerasVP, osg::StateAttribute::ON);
#endif
        _depthCameras[i]->getOrCreateStateSet()->addUniform(_lightPosUniform);
        _depthCameras[i]->getOrCreateStateSet()->addUniform(_farPlaneUniform);
        _depthCameras[i]->getOrCreateStateSet()->addUniform(_nearPlaneUniform);

        _depthCameras[i]->addChild(_shadowedScene);
        _parentScene->addChild(_depthCameras[i]);
    }


    _parentScene->getOrCreateStateSet()->setTextureAttributeAndModes(1, depthMap, osg::StateAttribute::ON);
#if USE_OSG_PROGRAM
    _parentScene->getOrCreateStateSet()->setAttributeAndModes(_visibilityProgram, osg::StateAttribute::ON);
#else
    _parentScene->getOrCreateStateSet()->setAttributeAndModes(_visibilityShaderVP, osg::StateAttribute::ON);
#endif
    _parentScene->getOrCreateStateSet()->addUniform(_baseTextureUniform);
    _parentScene->getOrCreateStateSet()->addUniform(_shadowMapUniform);
    _parentScene->getOrCreateStateSet()->addUniform(_visibleColorUniform);
    _parentScene->getOrCreateStateSet()->addUniform(_invisibleColorUniform);

    // Update light source info for shadowing scene
    _parentScene->getOrCreateStateSet()->addUniform(_lightPosUniform);
    _parentScene->getOrCreateStateSet()->addUniform(_farPlaneUniform);
    _parentScene->getOrCreateStateSet()->addUniform(_nearPlaneUniform);
    _parentScene->getOrCreateStateSet()->addUniform(_viewRadiusUniform);

    _lightIndicator = makeIndicator(_lightSource);
    _parentScene->getParent(0)->addChild(_lightIndicator);


    // Call the visualizer
        setupDebugHUD();
    updateAttributes();
}


VisibilityTestArea::~VisibilityTestArea()
{
    clear();
}

osg::AutoTransform* VisibilityTestArea::makeIndicator(osg::Vec3 eye)
{
    osg::ref_ptr<osg::Sphere> sphere = new osg::Sphere(osg::Vec3(0.0f, 0.0f, 0.0f), 5.0f);

    osg::ref_ptr<osg::ShapeDrawable> drawable = new osg::ShapeDrawable(sphere.get());

    drawable->setColor(osg::Vec4(0, 0, 1, 1));

    osg::ref_ptr<osg::Geode> geode = new osg::Geode();
    geode->addDrawable(drawable.get());

    geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    osg::ref_ptr<osg::AutoTransform> xform = new osg::AutoTransform();
    xform->setAutoRotateMode(osg::AutoTransform::NO_ROTATION);
    xform->setAutoScaleToScreen(true);

    xform->addChild(geode.get());

    xform->setPosition(eye);

    return xform.release();
}

void VisibilityTestArea::setViwerPosition(const osg::Vec3 position)
{
    _lightSource = position;
    _lightPosUniform->set(_lightSource);
    updateAttributes();

    if (_lightIndicator.valid())
       {
           _lightIndicator->setPosition(position);
       }
}

void VisibilityTestArea::setRadius(int radius)
{
    if(_viewRadiusUniform.valid()){
        _viewRadiusUniform->set((float)radius);
        _farPlaneUniform->set((float)radius+ _farPlanOffSet);
    }
    else _viweingRadius = radius;
}

void VisibilityTestArea::setVisibleAreaColor(const osg::Vec4 color)
{
    if(_visibleColorUniform.valid()){
        _visibleColorUniform->set(color);
    }
    else visibleColor = color;
}

void VisibilityTestArea::setInvisibleAreaColor(const osg::Vec4 color)
{
    if(_invisibleColorUniform.valid()){
        _invisibleColorUniform->set(color);
    }
    else invisibleColor = color;
}

void VisibilityTestArea::setVerticalFOV(int fov)
{
    _verticalFOV = fov;
    osg::Matrix               shadowProj = osg::Matrix::perspective(_verticalFOV, SM_TEXTURE_WIDTH / SM_TEXTURE_WIDTH, near_plane, far_plane);
    for (int i = 0; i < 6; i++)
    {
        _depthCameras[i]->setProjectionMatrix(shadowProj);
    }

}


void  VisibilityTestArea::clear()
{
    for (int i = 0; i < 6; i++)
       {
           // _depthCameras[i]->getOrCreateStateSet()->setAttribute(depthShader, osg::StateAttribute::OFF);
           _depthCameras[i]->getOrCreateStateSet()->removeUniform(_lightPosUniform);
           _depthCameras[i]->getOrCreateStateSet()->removeUniform(_farPlaneUniform);
           _depthCameras[i]->getOrCreateStateSet()->removeUniform(_nearPlaneUniform);
           _depthCameras[i]->getOrCreateStateSet()->removeUniform(_inverseViewUniform[i]);
           _depthCameras[i]->removeChild(_shadowedScene);
           _parentScene->removeChild(_depthCameras[i]);
           _depthCameras[i] = nullptr;
       }

       _parentScene->getOrCreateStateSet()->removeUniform(_visibleColorUniform);
       _parentScene->getOrCreateStateSet()->removeUniform(_invisibleColorUniform);
       _parentScene->getOrCreateStateSet()->removeUniform(_lightPosUniform);
       _parentScene->getOrCreateStateSet()->removeUniform(_farPlaneUniform);
       _parentScene->getOrCreateStateSet()->removeUniform(_nearPlaneUniform);
       _parentScene->getOrCreateStateSet()->removeUniform(_viewRadiusUniform);
       _parentScene->getOrCreateStateSet()->removeUniform(_baseTextureUniform);
       _parentScene->getOrCreateStateSet()->removeUniform(_shadowMapUniform);

       _parentScene->getOrCreateStateSet()->setAttribute(_visibilityShader, osg::StateAttribute::OFF);
       _parentScene->getOrCreateStateSet()->setTextureAttributeAndModes(1, depthMap, osg::StateAttribute::OFF);
       _parentScene->getOrCreateStateSet()->removeAttribute(osg::StateAttribute::PROGRAM);
       _parentScene->getOrCreateStateSet()->removeTextureAttribute(1, osg::StateAttribute::TEXTURE);

       // _parentScene->removeChild(_shadowedScene);
       _parentScene->getParent(0)->removeChild(_lightIndicator);

       _lightIndicator = nullptr;
       _visibilityShader = nullptr;




}


