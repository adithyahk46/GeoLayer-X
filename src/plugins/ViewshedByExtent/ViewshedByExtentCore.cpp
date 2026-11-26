#include "ViewshedByExtentCore.h"

#include <algorithm>
#include <array>
#include <cmath>

#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/StateSet>

namespace
{
constexpr unsigned int ShadowMapSize = 1024;
constexpr float NearPlane = 1.0f;
constexpr double FarPlanePadding = 100.0;
constexpr int ShadowTextureUnit = 10;

const char* DepthVertexShader = R"(
    #version 330 core
    out float lightDistance;
    uniform mat4 osg_ViewMatrixInverse;
    uniform mat4 inverse_view;
    uniform vec3 lightPos;
    uniform float near_plane;
    uniform float far_plane;

    void depthMapVertex(inout vec4 vertex)
    {
        vec4 world = inverse_view * vertex;
        vec3 worldPos = world.xyz / world.w;
        float distanceToViewer = length(worldPos - lightPos);
        lightDistance = ((1.0 / distanceToViewer) - (1.0 / near_plane))
                      / ((1.0 / far_plane) - (1.0 / near_plane));
    }
)";

const char* DepthFragmentShader = R"(
    #version 330 core
    in float lightDistance;

    void depthMapFragment(inout vec4 color)
    {
        gl_FragDepth = lightDistance;
    }
)";

const char* VisibilityVertexShader = R"(
    #version 330 core
    in vec3 osg_Normal;
    uniform mat4 osg_ViewMatrixInverse;
    uniform vec3 lightPos;
    out vec3 worldPos;
    out vec3 normal;
    out float lightDistance;

    void visibilityVertex(inout vec4 vertex)
    {
        vec4 world = osg_ViewMatrixInverse * vertex;
        worldPos = world.xyz / world.w;
        lightDistance = length(worldPos - lightPos);
        normal = normalize(osg_Normal);
    }
)";

const char* VisibilityFragmentShader = R"(
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
    uniform float extentWest;
    uniform float extentEast;
    uniform float extentSouth;
    uniform float extentNorth;

    const float WGS84_A = 6378137.0;
    const float WGS84_B = 6356752.314245;
    const float WGS84_E2 = 0.00669437999014;
    const float WGS84_EP2 = 0.00673949674228;
    const float RAD_TO_DEG = 57.29577951308232;

    bool insideSelectedExtent(vec3 position)
    {
        float p = length(position.xy);
        if (p < 0.001)
        {
            float latitudeAtPole = position.z > 0.0 ? 90.0 : -90.0;
            return latitudeAtPole >= extentSouth && latitudeAtPole <= extentNorth;
        }

        float theta = atan(position.z * WGS84_A, p * WGS84_B);
        float sinTheta = sin(theta);
        float cosTheta = cos(theta);
        float latitude = atan(
            position.z + WGS84_EP2 * WGS84_B * sinTheta * sinTheta * sinTheta,
            p - WGS84_E2 * WGS84_A * cosTheta * cosTheta * cosTheta) * RAD_TO_DEG;
        float longitude = atan(position.y, position.x) * RAD_TO_DEG;

        bool longitudeInside = extentWest <= extentEast
            ? longitude >= extentWest && longitude <= extentEast
            : longitude >= extentWest || longitude <= extentEast;
        return longitudeInside && latitude >= extentSouth && latitude <= extentNorth;
    }

    float linearizeDepth(float depth)
    {
        float z = 2.0 * depth - 1.0;
        return 2.0 * near_plane * far_plane
             / (far_plane + near_plane - z * (far_plane - near_plane));
    }

    bool isShadowed(vec3 lightDirection)
    {
        float bias = max(0.01 * (1.0 - dot(normal, lightDirection)), 0.001) * far_plane;
        float storedDistance = linearizeDepth(texture(shadowMap, lightDirection).r);
        return lightDistance - bias > storedDistance;
    }

    void visibilityFragment(inout vec4 color)
    {
        if (!insideSelectedExtent(worldPos) || lightDistance > far_plane)
            return;

        vec3 lightDirection = normalize(lightPos - worldPos);
        if (isShadowed(-lightDirection))
            color.rgb = mix(color.rgb, invisibleColor.rgb, invisibleColor.a);
        else
            color.rgb = mix(color.rgb, visibleColor.rgb, visibleColor.a);
    }
)";
}

ViewshedByExtentCore::ViewshedByExtentCore(
    osg::Group* sceneRoot,
    const osgEarth::GeoExtent& extent,
    const osgEarth::GeoPoint& viewerLocation)
    : _sceneRoot(sceneRoot)
    , _viewerLocation(viewerLocation)
{
    setExtent(extent);
}

ViewshedByExtentCore::~ViewshedByExtentCore()
{
    clear();
}

bool ViewshedByExtentCore::setExtent(const osgEarth::GeoExtent& extent)
{
    const osgEarth::SpatialReference* geographicSrs =
        osgEarth::SpatialReference::get("epsg:4326");
    if (!extent.isValid() || !geographicSrs)
        return false;

    osgEarth::GeoExtent geographicExtent;
    if (!extent.transform(geographicSrs, geographicExtent) || !geographicExtent.isValid())
        return false;

    const osgEarth::GeoExtent previousExtent = _extent;
    _extent = geographicExtent;
    if (!_built)
        return true;

    if (!updateExtentBounds())
    {
        _extent = previousExtent;
        updateExtentBounds();
        return false;
    }

    _extentWestUniform->set(static_cast<float>(_extent.west()));
    _extentEastUniform->set(static_cast<float>(_extent.east()));
    _extentSouthUniform->set(static_cast<float>(_extent.south()));
    _extentNorthUniform->set(static_cast<float>(_extent.north()));
    updateCameraProjection();
    return true;
}

bool ViewshedByExtentCore::setViewerLocation(const osgEarth::GeoPoint& viewerLocation)
{
    _viewerLocation = viewerLocation;
    if (!updateViewerWorldPosition())
        return false;

    if (_built)
    {
        _lightPosUniform->set(osg::Vec3(_viewerWorldPosition));
        _viewerIndicator->setPosition(_viewerWorldPosition);
        updateCameraViews();
        updateCameraProjection();
    }
    return true;
}

void ViewshedByExtentCore::setVisibleAreaColor(const osg::Vec4& color)
{
    if (_visibleColorUniform.valid())
        _visibleColorUniform->set(color);
}

void ViewshedByExtentCore::setInvisibleAreaColor(const osg::Vec4& color)
{
    if (_invisibleColorUniform.valid())
        _invisibleColorUniform->set(color);
}

bool ViewshedByExtentCore::buildModel(std::string* errorMessage)
{
    if (_built)
        return true;

    if (!_sceneRoot.valid())
    {
        if (errorMessage)
            *errorMessage = "The map scene root is not available.";
        return false;
    }

    if (_sceneRoot->getNumParents() == 0)
    {
        if (errorMessage)
            *errorMessage = "The map scene root has no parent scene.";
        return false;
    }
    _parentScene = _sceneRoot->getParent(0);

    if (!_extent.isValid() || !updateViewerWorldPosition())
    {
        if (errorMessage)
            *errorMessage = "A valid geographic extent and viewer location are required.";
        _parentScene = nullptr;
        return false;
    }

    _depthMap = new osg::TextureCubeMap;
    _depthMap->setTextureSize(ShadowMapSize, ShadowMapSize);
    _depthMap->setInternalFormat(GL_DEPTH_COMPONENT);
    _depthMap->setSourceFormat(GL_DEPTH_COMPONENT);
    _depthMap->setSourceType(GL_FLOAT);
    _depthMap->setWrap(osg::Texture::WRAP_S, osg::Texture::CLAMP_TO_EDGE);
    _depthMap->setWrap(osg::Texture::WRAP_T, osg::Texture::CLAMP_TO_EDGE);
    _depthMap->setWrap(osg::Texture::WRAP_R, osg::Texture::CLAMP_TO_EDGE);
    _depthMap->setFilter(osg::Texture::MIN_FILTER, osg::Texture::NEAREST);
    _depthMap->setFilter(osg::Texture::MAG_FILTER, osg::Texture::NEAREST);

    _lightPosUniform = new osg::Uniform("lightPos", osg::Vec3(_viewerWorldPosition));
    _farPlaneUniform = new osg::Uniform("far_plane", static_cast<float>(_farPlane));
    _nearPlaneUniform = new osg::Uniform("near_plane", NearPlane);
    _shadowMapUniform = new osg::Uniform("shadowMap", ShadowTextureUnit);
    _visibleColorUniform = new osg::Uniform("visibleColor", osg::Vec4(0.62f, 1.0f, 0.24f, 0.8f));
    _invisibleColorUniform = new osg::Uniform("invisibleColor", osg::Vec4(1.0f, 0.34f, 0.24f, 0.8f));
    _extentWestUniform = new osg::Uniform("extentWest", static_cast<float>(_extent.west()));
    _extentEastUniform = new osg::Uniform("extentEast", static_cast<float>(_extent.east()));
    _extentSouthUniform = new osg::Uniform("extentSouth", static_cast<float>(_extent.south()));
    _extentNorthUniform = new osg::Uniform("extentNorth", static_cast<float>(_extent.north()));

    _depthProgram = new osgEarth::VirtualProgram;
    _depthProgram->setFunction(
        "depthMapVertex", DepthVertexShader, osgEarth::VirtualProgram::LOCATION_VERTEX_VIEW);
    _depthProgram->setFunction(
        "depthMapFragment", DepthFragmentShader, osgEarth::VirtualProgram::LOCATION_FRAGMENT_COLORING);

    const osg::Matrix projection =
        osg::Matrix::perspective(90.0, 1.0, NearPlane, _farPlane);
    for (unsigned int face = 0; face < _depthCameras.size(); ++face)
    {
        _depthCameras[face] = generateCubeCamera(face);
        _depthCameras[face]->setProjectionMatrix(projection);
        osg::StateSet* cameraState = _depthCameras[face]->getOrCreateStateSet();
        cameraState->setAttributeAndModes(_depthProgram, osg::StateAttribute::ON);
        cameraState->addUniform(_lightPosUniform);
        cameraState->addUniform(_farPlaneUniform);
        cameraState->addUniform(_nearPlaneUniform);
        _inverseViewUniforms[face] =
            new osg::Uniform("inverse_view", osg::Matrixf::identity());
        cameraState->addUniform(_inverseViewUniforms[face]);
        _depthCameras[face]->addChild(_sceneRoot);
        _parentScene->addChild(_depthCameras[face]);
    }

    _visibilityProgram = new osgEarth::VirtualProgram;
    _visibilityProgram->setFunction(
        "visibilityVertex", VisibilityVertexShader, osgEarth::VirtualProgram::LOCATION_VERTEX_VIEW);
    _visibilityProgram->setFunction(
        "visibilityFragment", VisibilityFragmentShader, osgEarth::VirtualProgram::LOCATION_FRAGMENT_COLORING);

    osg::StateSet* sceneState = _parentScene->getOrCreateStateSet();
    sceneState->setTextureAttributeAndModes(
        ShadowTextureUnit, _depthMap, osg::StateAttribute::ON);
    sceneState->setAttributeAndModes(_visibilityProgram, osg::StateAttribute::ON);
    sceneState->addUniform(_shadowMapUniform);
    sceneState->addUniform(_visibleColorUniform);
    sceneState->addUniform(_invisibleColorUniform);
    sceneState->addUniform(_lightPosUniform);
    sceneState->addUniform(_farPlaneUniform);
    sceneState->addUniform(_nearPlaneUniform);
    sceneState->addUniform(_extentWestUniform);
    sceneState->addUniform(_extentEastUniform);
    sceneState->addUniform(_extentSouthUniform);
    sceneState->addUniform(_extentNorthUniform);

    osg::ref_ptr<osg::Geode> indicatorGeode = new osg::Geode;
    osg::ref_ptr<osg::ShapeDrawable> indicatorDrawable =
        new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(), 5.0f));
    indicatorDrawable->setColor(osg::Vec4(0.0f, 0.2f, 1.0f, 1.0f));
    indicatorGeode->addDrawable(indicatorDrawable);
    indicatorGeode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    _viewerIndicator = new osg::AutoTransform;
    _viewerIndicator->setAutoRotateMode(osg::AutoTransform::NO_ROTATION);
    _viewerIndicator->setAutoScaleToScreen(true);
    _viewerIndicator->setPosition(_viewerWorldPosition);
    _viewerIndicator->addChild(indicatorGeode);
    _parentScene->addChild(_viewerIndicator);

    _built = true;
    updateCameraViews();
    updateCameraProjection();
    return true;
}

void ViewshedByExtentCore::clear()
{
    if (!_built || !_parentScene.valid())
        return;

    for (std::size_t face = 0; face < _depthCameras.size(); ++face)
    {
        osg::ref_ptr<osg::Camera>& camera = _depthCameras[face];
        if (camera.valid())
        {
            camera->removeChild(_sceneRoot);
            camera->getOrCreateStateSet()->removeUniform(_inverseViewUniforms[face]);
            _parentScene->removeChild(camera);
            camera = nullptr;
        }
    }

    osg::StateSet* sceneState = _parentScene->getStateSet();
    if (sceneState)
    {
        sceneState->removeUniform(_shadowMapUniform);
        sceneState->removeUniform(_visibleColorUniform);
        sceneState->removeUniform(_invisibleColorUniform);
        sceneState->removeUniform(_lightPosUniform);
        sceneState->removeUniform(_farPlaneUniform);
        sceneState->removeUniform(_nearPlaneUniform);
        sceneState->removeUniform(_extentWestUniform);
        sceneState->removeUniform(_extentEastUniform);
        sceneState->removeUniform(_extentSouthUniform);
        sceneState->removeUniform(_extentNorthUniform);
        sceneState->removeAttribute(_visibilityProgram.get());
        if (sceneState->getTextureAttribute(
                ShadowTextureUnit, osg::StateAttribute::TEXTURE) == _depthMap.get())
        {
            sceneState->removeTextureAttribute(
                ShadowTextureUnit, osg::StateAttribute::TEXTURE);
        }
    }

    _parentScene->removeChild(_viewerIndicator);
    _viewerIndicator = nullptr;
    _depthMap = nullptr;
    _visibilityProgram = nullptr;
    _depthProgram = nullptr;
    _built = false;
}

bool ViewshedByExtentCore::updateExtentBounds()
{
    if (!_extent.isValid())
        return false;

    const osgEarth::SpatialReference* geographicSrs = _extent.getSRS();
    if (!geographicSrs)
        return false;

    const std::array<std::pair<double, double>, 4> corners{{
        {_extent.west(), _extent.south()},
        {_extent.east(), _extent.south()},
        {_extent.east(), _extent.north()},
        {_extent.west(), _extent.north()}
    }};
    double maxDistance = 0.0;
    for (const auto& corner : corners)
    {
        osgEarth::GeoPoint point(
            geographicSrs, corner.first, corner.second,
            _viewerLocation.isValid() ? _viewerLocation.z() : 0.0,
            osgEarth::ALTMODE_ABSOLUTE);
        osg::Vec3d worldPoint;
        if (!point.toWorld(worldPoint))
            return false;
        maxDistance = std::max(maxDistance, (worldPoint - _viewerWorldPosition).length());
    }
    _farPlane = std::max(maxDistance + FarPlanePadding, 1000.0);
    return true;
}

bool ViewshedByExtentCore::updateViewerWorldPosition()
{
    if (!_viewerLocation.isValid() || !_viewerLocation.toWorld(_viewerWorldPosition))
        return false;
    return updateExtentBounds();
}

void ViewshedByExtentCore::updateCameraViews()
{
    if (!_built)
        return;

    const osg::Vec3 light(_viewerWorldPosition);
    const std::array<osg::Matrix, 6> views{{
        osg::Matrix::lookAt(light, light + osg::Vec3(1.0f, 0.0f, 0.0f), osg::Vec3(0.0f, -1.0f, 0.0f)),
        osg::Matrix::lookAt(light, light + osg::Vec3(-1.0f, 0.0f, 0.0f), osg::Vec3(0.0f, -1.0f, 0.0f)),
        osg::Matrix::lookAt(light, light + osg::Vec3(0.0f, 1.0f, 0.0f), osg::Vec3(0.0f, 0.0f, 1.0f)),
        osg::Matrix::lookAt(light, light + osg::Vec3(0.0f, -1.0f, 0.0f), osg::Vec3(0.0f, 0.0f, -1.0f)),
        osg::Matrix::lookAt(light, light + osg::Vec3(0.0f, 0.0f, 1.0f), osg::Vec3(0.0f, -1.0f, 0.0f)),
        osg::Matrix::lookAt(light, light + osg::Vec3(0.0f, 0.0f, -1.0f), osg::Vec3(0.0f, -1.0f, 0.0f))
    }};

    for (std::size_t face = 0; face < _depthCameras.size(); ++face)
    {
        if (_depthCameras[face].valid())
        {
            _depthCameras[face]->setViewMatrix(views[face]);
            _inverseViewUniforms[face]->set(osg::Matrixf::inverse(views[face]));
        }
    }
}

void ViewshedByExtentCore::updateCameraProjection()
{
    if (!_built)
        return;

    _farPlaneUniform->set(static_cast<float>(_farPlane));
    const osg::Matrix projection =
        osg::Matrix::perspective(90.0, 1.0, NearPlane, _farPlane);
    for (auto& camera : _depthCameras)
    {
        if (camera.valid())
            camera->setProjectionMatrix(projection);
    }
}

osg::Camera* ViewshedByExtentCore::generateCubeCamera(unsigned int face)
{
    osg::ref_ptr<osg::Camera> camera = new osg::Camera;
    camera->setClearMask(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
    camera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    camera->setRenderOrder(osg::Camera::PRE_RENDER);
    camera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
    camera->setViewport(0, 0, ShadowMapSize, ShadowMapSize);
    camera->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
    camera->attach(osg::Camera::DEPTH_BUFFER, _depthMap, 0, face);
    return camera.release();
}