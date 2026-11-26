#ifndef VIEWSHED_BY_EXTENT_CORE_H
#define VIEWSHED_BY_EXTENT_CORE_H

#include <array>
#include <string>

#include <osg/AutoTransform>
#include <osg/Camera>
#include <osg/Group>
#include <osg/TextureCubeMap>
#include <osg/Uniform>
#include <osgEarth/GeoData>
#include <osgEarth/VirtualProgram>

class ViewshedByExtentCore
{
public:
    ViewshedByExtentCore(
        osg::Group* sceneRoot,
        const osgEarth::GeoExtent& extent,
        const osgEarth::GeoPoint& viewerLocation);
    ~ViewshedByExtentCore();

    bool buildModel(std::string* errorMessage = nullptr);
    bool setExtent(const osgEarth::GeoExtent& extent);
    bool setViewerLocation(const osgEarth::GeoPoint& viewerLocation);
    void setVisibleAreaColor(const osg::Vec4& color);
    void setInvisibleAreaColor(const osg::Vec4& color);
    void clear();

private:
    bool updateExtentBounds();
    bool updateViewerWorldPosition();
    void updateCameraViews();
    void updateCameraProjection();
    osg::Camera* generateCubeCamera(unsigned face);

    osg::ref_ptr<osg::Group> _sceneRoot;
    osg::ref_ptr<osg::Group> _parentScene;
    osgEarth::GeoExtent _extent;
    osgEarth::GeoPoint _viewerLocation;
    osg::Vec3d _viewerWorldPosition;
    double _farPlane = 1000.0;
    bool _built = false;

    osg::ref_ptr<osg::TextureCubeMap> _depthMap;
    std::array<osg::ref_ptr<osg::Camera>, 6> _depthCameras;
    osg::ref_ptr<osg::Uniform> _lightPosUniform;
    osg::ref_ptr<osg::Uniform> _farPlaneUniform;
    osg::ref_ptr<osg::Uniform> _nearPlaneUniform;
    osg::ref_ptr<osg::Uniform> _shadowMapUniform;
    osg::ref_ptr<osg::Uniform> _visibleColorUniform;
    osg::ref_ptr<osg::Uniform> _invisibleColorUniform;
    osg::ref_ptr<osg::Uniform> _extentWestUniform;
    osg::ref_ptr<osg::Uniform> _extentEastUniform;
    osg::ref_ptr<osg::Uniform> _extentSouthUniform;
    osg::ref_ptr<osg::Uniform> _extentNorthUniform;
    osg::ref_ptr<osgEarth::VirtualProgram> _depthProgram;
    osg::ref_ptr<osgEarth::VirtualProgram> _visibilityProgram;
    osg::ref_ptr<osg::AutoTransform> _viewerIndicator;
    std::array<osg::ref_ptr<osg::Uniform>, 6> _inverseViewUniforms;
};

#endif // VIEWSHED_BY_EXTENT_CORE_H