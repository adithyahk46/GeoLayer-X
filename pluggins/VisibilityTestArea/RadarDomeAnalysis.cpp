#include "RadarDomeAnalysis.h"


const char* depthMapVertVP1 = R"(
    #version 330 core

    //in vec4 gl_Vertex;
    out float lightDistance;

    uniform mat4 osg_ModelViewProjectionMatrix;
    uniform mat4 osg_ViewMatrixInverse;
    uniform mat4 osg_ModelViewMatrix;
    uniform vec3 lightPos;
    uniform mat4 inverse_view;

    uniform float near_plane;
    uniform float far_plane;

    void depthMapVertex(inout vec4 vertex)
    {
        //IN Model space
        // vec3 worldPos = (inverse_view * osg_ModelViewMatrix * vertex).xyz;

        //in view space
        vec4 w = inverse_view * vertex;
        vec3 worldPos = w.xyz / w.w;

        lightDistance = length(worldPos - lightPos);
        lightDistance = ((1 / lightDistance) - (1 / near_plane)) / (1 / far_plane - 1 / near_plane);

        // gl_Position = osg_ModelViewProjectionMatrix * vertex;
    }
)";

const char* depthMapFragVP1 = R"(
    #version 330 core
    in float lightDistance;
    uniform float far_plane;

    void depthMapFragment(inout vec4 color)
    {
        // Mapping to [0, 1]
        // float depth = clamp(lightDistance, 0.0, 1.0);

        // color = vec4(vec3(depth), 1.0);
        gl_FragDepth = lightDistance;
    }
)";

const char* visibilityShaderVertVP1 = R"(
    #version 330 core

in vec3 osg_Normal;

    uniform mat4 osg_ModelViewProjectionMatrix;
    uniform mat4 osg_ViewMatrixInverse;
    uniform mat4 osg_ModelViewMatrix;

    uniform mat4 inverse_view;


    uniform vec3 lightPos;

    out vec3 worldPos;
    out vec3 normal;
    out float lightDistance;

    void visibilityVertex(inout vec4 vertex)
    {
        //in Model space
        // worldPos = (osg_ViewMatrixInverse * osg_ModelViewMatrix * vertex).xyz;

        //IN view space
        vec4 w = osg_ViewMatrixInverse * vertex;
        worldPos = w.xyz / w.w;

        lightDistance = length(worldPos - lightPos);

        normal = normalize( osg_Normal );
        // gl_Position = osg_ModelViewProjectionMatrix * vertex;
    }
)";


const char* visibilityShaderFragVP1 = R"(
    #version 330 core

    in vec3 worldPos;
    in vec3 normal;
    in float lightDistance;

    uniform vec3 lightPos;
    uniform vec4 visibleColor;
    uniform vec4 invisibleColor;

    uniform samplerCube shadowMap;

    uniform float near_plane;
    uniform float far_plane;
    uniform float user_area;

    float linearizeDepth(float z)
    {
        float z_n = 2.0 * z - 1.0;
        return 2.0 * near_plane * far_plane / (far_plane + near_plane - z_n * (far_plane - near_plane));
    };

    // Return 1 for shadowed, 0 visible
    bool isShadowed(vec3 lightDir)
    {
        float bias = max(0.01 * (1.0 - dot(normal, lightDir)), 0.001) * far_plane;

        float z = linearizeDepth(texture(shadowMap, lightDir).r);
        return lightDistance - bias > z;
    }

    void visibilityFragment(inout vec4 FragColor)
    {
        if (length(lightPos - worldPos) < user_area)
        {

            vec3 lightDir = normalize(lightPos - worldPos);
            float normDif = max(dot(lightDir, normal), 0.0);

            // if (normDif > 0.0 && isShadowed(-lightDir) == false)
            if (isShadowed(-lightDir) == false)
            {
                // Blends the current FragColor with visibleColor based on its alpha
                FragColor.rgb = mix(FragColor.rgb, visibleColor.rgb, visibleColor.a);
            }
            else
            {
                // Blends the current FragColor with invisibleColor based on its alpha
                FragColor.rgb = mix(FragColor.rgb, invisibleColor.rgb, invisibleColor.a);
            }
        }
    }
)";



enum TraversalOption
{
    INTERSECT_IGNORE = 0x00000004,
    TRAVERSAL_IGNORE = 0x00000010
};

static const int   SM_TEXTURE_WIDTH  = 1024;


RadarDomeAnalysis::RadarDomeAnalysis(osg::Group* sceneRoot, osgViewer::Viewer* viewer,osg::Vec3 lightSource, int radius):
    _shadowedScene(sceneRoot),
    _mainViewer(viewer),
    _lightSource(lightSource),
    _viweingRadius(radius)
{

        _parentScene = new osg::Group;
        _mainViewer->getSceneData()->asGroup()->addChild(_parentScene);

        //create a sperical geod at the light soure position with _viweingRadius as radius, and add it to the scene for visualization

}

osg::Camera * RadarDomeAnalysis::generateCubeCamera(osg::ref_ptr<osg::TextureCubeMap> cubeMap, unsigned face, osg::Camera::BufferComponent component)
{
    osg::ref_ptr<osg::Camera>  camera = new osg::Camera;

    camera->setClearMask(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

    camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    camera->setRenderOrder(osg::Camera::PRE_RENDER);
    camera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
    camera->setViewport(0, 0, SM_TEXTURE_WIDTH, SM_TEXTURE_WIDTH);
    camera->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
    camera->attach(component, cubeMap, 0, face);

    // camera->setNodeMask(0xffffffff & (~INTERSECT_IGNORE));
    return camera.release();
}

void  RadarDomeAnalysis::updateAttributes()
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
        _depthCameras[i]->setViewMatrix(shadowViews[i]);
    }

    if(!_inverseViewUniform[0].valid())
    {
        for (int i = 0; i < 6; i++){
            _inverseViewUniform[i] = new osg::Uniform("inverse_view", osg::Matrixf::inverse(shadowViews[i]));
            _depthCameras[i]->getOrCreateStateSet()->addUniform(_inverseViewUniform[i]);
        }
    }
    else{
        for (int i = 0; i < 6; i++){
            _inverseViewUniform[i]->set(osg::Matrixf::inverse(shadowViews[i]));
        }
    }
}

void  RadarDomeAnalysis::buildModel()
{

    // _mainViewer->getCamera()->getGraphicsContext()->getState()->setUseModelViewAndProjectionUniforms(true);

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

    _shadowMapUniform = new osg::Uniform("shadowMap", 10);

     // Generate shadow map cameras and corresponding textures
    osg::Matrix  shadowProj = osg::Matrix::perspective(90, SM_TEXTURE_WIDTH / SM_TEXTURE_WIDTH, near_plane, far_plane);


    _depthCamerasVP = new osgEarth::VirtualProgram();
    _depthCamerasVP->setFunction("depthMapVertex",depthMapVertVP1,osgEarth::VirtualProgram::LOCATION_VERTEX_VIEW);
    _depthCamerasVP->setFunction("depthMapFragment",depthMapFragVP1,osgEarth::VirtualProgram::LOCATION_FRAGMENT_COLORING);


    // Generate one camera for each side of the shadow cubemap
    for (int i = 0; i < 6; i++)
    {
        _depthCameras[i] = generateCubeCamera(depthMap, i, osg::Camera::DEPTH_BUFFER);
        _depthCameras[i]->setProjectionMatrix(shadowProj);

        _depthCameras[i]->getOrCreateStateSet()->setAttributeAndModes(_depthCamerasVP, osg::StateAttribute::ON);

        _depthCameras[i]->getOrCreateStateSet()->addUniform(_lightPosUniform);
        _depthCameras[i]->getOrCreateStateSet()->addUniform(_farPlaneUniform);
        _depthCameras[i]->getOrCreateStateSet()->addUniform(_nearPlaneUniform);

        _depthCameras[i]->addChild(_shadowedScene);
        _parentScene->addChild(_depthCameras[i]);
    }


    _visibilityShaderVP = new osgEarth::VirtualProgram();
    _visibilityShaderVP->setFunction("visibilityVertex",visibilityShaderVertVP1,osgEarth::VirtualProgram::LOCATION_VERTEX_VIEW);
    _visibilityShaderVP->setFunction("visibilityFragment",visibilityShaderFragVP1,osgEarth::VirtualProgram::LOCATION_FRAGMENT_COLORING);

    _parentScene->getOrCreateStateSet()->setTextureAttributeAndModes(10, depthMap, osg::StateAttribute::ON);

    _parentScene->getOrCreateStateSet()->setAttributeAndModes(_visibilityShaderVP, osg::StateAttribute::ON);

    _parentScene->getOrCreateStateSet()->addUniform(_shadowMapUniform);
    _parentScene->getOrCreateStateSet()->addUniform(_visibleColorUniform);
    _parentScene->getOrCreateStateSet()->addUniform(_invisibleColorUniform);

    _parentScene->getOrCreateStateSet()->addUniform(_lightPosUniform);
    _parentScene->getOrCreateStateSet()->addUniform(_farPlaneUniform);
    _parentScene->getOrCreateStateSet()->addUniform(_nearPlaneUniform);
    _parentScene->getOrCreateStateSet()->addUniform(_viewRadiusUniform);

    _lightIndicator = makeIndicator(_lightSource);
    _parentScene->addChild(_lightIndicator);

    updateAttributes();
}


RadarDomeAnalysis::~RadarDomeAnalysis()
{
    clear();
}

osg::AutoTransform* RadarDomeAnalysis::makeIndicator(osg::Vec3 eye)
{
    osg::ref_ptr<osg::Sphere> sphere = new osg::Sphere(osg::Vec3(0.0f, 0.0f, 0.0f), (float)_viweingRadius);

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

void RadarDomeAnalysis::setViwerPosition(const osg::Vec3 position)
{
    _lightSource = position;

    if (_lightIndicator.valid())
    {
       _lightIndicator->setPosition(position);
       _lightPosUniform->set(_lightSource);
       updateAttributes();
    }
}

void RadarDomeAnalysis::setRadius(int radius)
{
    if(_viewRadiusUniform.valid()){
        _viewRadiusUniform->set((float)radius);
        _farPlaneUniform->set((float)radius+ _farPlanOffSet);
    }

    _viweingRadius = radius;
}

void RadarDomeAnalysis::setVisibleAreaColor(const osg::Vec4 color)
{
    if(_visibleColorUniform.valid()){
        _visibleColorUniform->set(color);
    }

    visibleColor = color;
}

void RadarDomeAnalysis::setInvisibleAreaColor(const osg::Vec4 color)
{
    if(_invisibleColorUniform.valid()){
        _invisibleColorUniform->set(color);
    }

    invisibleColor = color;
}

void  RadarDomeAnalysis::clear()
{
    // _mainViewer->getCamera()->getGraphicsContext()->getState()->setUseModelViewAndProjectionUniforms(false);

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
       _parentScene->getOrCreateStateSet()->removeUniform(_shadowMapUniform);

       _parentScene->getOrCreateStateSet()->setAttributeAndModes(_visibilityShaderVP, osg::StateAttribute::OFF);
       _parentScene->getOrCreateStateSet()->setTextureAttributeAndModes(10, depthMap, osg::StateAttribute::OFF);
       _parentScene->getOrCreateStateSet()->removeAttribute(osg::StateAttribute::PROGRAM);
       // _parentScene->getOrCreateStateSet()->removeAttribute(_visibilityShaderVP);
       _parentScene->getOrCreateStateSet()->removeTextureAttribute(10, osg::StateAttribute::TEXTURE);

       // _parentScene->removeChild(_shadowedScene);
       _parentScene->getParent(0)->removeChild(_lightIndicator);

       _lightIndicator = nullptr;
       // _visibilityShader = nullptr;



}


