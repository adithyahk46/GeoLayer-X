#ifndef VIEWSHEDANALYSIS_H
#define VIEWSHEDANALYSIS_H


class ViewshedAnalysis
{

public:
    ViewshedAnalysis(osgEarth::MapNode* mapNode, osgEarth::GeoPoint camPos, float distance);
    ~ViewshedAnalysis();

    void buildModel();

    void clear();

    void setCameraPosition(osgEarth::GeoPoint position);

    void setDistance(int distance);

    void setVerticalFOV(int angle);

    void setHorizontalFOV(int angle);

    /**
     * @brief Rotation of Camera.
     *
     * rotate the camera  .
     *
     * @param angle
     * @param axis
     *
     * @note
     */
    void setRotation(double angle,osg::Vec3 axis);

    void setVisibleAreaColor(const osg::Vec4 color);

    void setHiddenAreaColor(const osg::Vec4 color);


protected:
    osg::Vec3 geoPointsToVev3(osgEarth::GeoPoint GeoPoint);

    bool setUpCamera();

    void updateAttributes();

    osg::AutoTransform *makeIndicator(osg::Vec3 eye);


    void updateProjectionMatrix();
    void setupDebugHUD();
private:
    osg::ref_ptr<osgEarth::MapNode> _mapNode;
    osg::ref_ptr<osg::Group> _shadowedScene;
    osg::ref_ptr<osg::Group> _parentScene;

    osg::ref_ptr<osg::Texture2D>  depthMap;
    osg::ref_ptr<osg::Camera> _depthCamera;
    osg::ref_ptr<osg::AutoTransform> _cameraIndicator;
    osg::ref_ptr<osg::MatrixTransform> frustumVisual;

    osgEarth::GeoPoint _camGeoPos;
    osg::Vec3 _cameraPos;
    float _viewDistance;
    double _horizontalFOV = 90.0;
    double _verticalFOV   = 60.0;

    float near_plane = 10.0f;
    float far_plane ;
    float _farPlanOffSet = 100.0f;

    osg::Vec4  visibleColor   = osg::Vec4(159.0f / 255.0f, 255.0f / 255.0f, 61.0f / 255.0f, 0.5f);
    osg::Vec4  invisibleColor = osg::Vec4(255.0f / 255.0f, 87.0f / 255.0f, 61.0f / 255.0f, 0.5f);



    osg::ref_ptr<osg::Uniform> _cameraPosUniform;
    osg::ref_ptr<osg::Uniform> _viewDistanceUniform;
    osg::ref_ptr<osg::Uniform> _farPlaneUniform;
    osg::ref_ptr<osg::Uniform> _nearPlaneUniform;
    osg::ref_ptr<osg::Uniform> _inverseViewUniform ;

    osg::ref_ptr<osg::Uniform> _shadowMapUniform;
    osg::ref_ptr<osg::Uniform> _cameraVPUniform;

    osg::ref_ptr<osg::Uniform> _visibleColorUniform;
    osg::ref_ptr<osg::Uniform> _invisibleColorUniform;

    osg::ref_ptr<osgEarth::VirtualProgram> _visibilityShaderVP;
    osg::ref_ptr<osgEarth::VirtualProgram> _depthCamerasVP;



};

#endif // VIEWSHEDANALYSIS_H
