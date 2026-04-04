#include "ViewshedAnalysis.h"

const std::string MODULE_ERROR = "ERROR: VIEWSHED: ";

const char* depthMapVert = R"(
    #version 330 core

    //in vec4 gl_Vertex;
    out float lightDistance;

    uniform mat4 osg_ModelViewProjectionMatrix;
    uniform mat4 osg_ViewMatrixInverse;
    uniform mat4 osg_ModelViewMatrix;
    uniform vec3 eyePos;
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

        lightDistance = length(worldPos - eyePos);
        lightDistance = ((1 / lightDistance) - (1 / near_plane)) / (1 / far_plane - 1 / near_plane);

        // gl_Position = osg_ModelViewProjectionMatrix * vertex;
    }
)";

const char* depthMapFrag = R"(
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

const char* visibilityShaderVert = R"(
    #version 330 core

in vec3 osg_Normal;

    uniform mat4 osg_ModelViewProjectionMatrix;
    uniform mat4 osg_ViewMatrixInverse;
    uniform mat4 osg_ModelViewMatrix;
uniform mat3 osg_NormalMatrix;

    uniform mat4 inverse_view;


    uniform vec3 eyePos;

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

        lightDistance = length(worldPos - eyePos);

// Convert normal from view → world
mat3 normalMatrixWorld = mat3(osg_ViewMatrixInverse) * mat3(osg_NormalMatrix);
normal = normalize(/*normalMatrixWorld **/ osg_Normal);
        // gl_Position = osg_ModelViewProjectionMatrix * vertex;
    }
)";


const char* visibilityShaderFrag = R"(
    #version 330 core

    in vec3 worldPos;
    in vec3 normal;
    in float lightDistance;

    uniform vec3 eyePos;
    uniform vec4 visibleColor;
    uniform vec4 invisibleColor;

    uniform sampler2D shadowMap;
    uniform mat4 cameraVP;       // The new Uniform

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

        float z = linearizeDepth(texture(shadowMap, lightDir.xy).r);
        return lightDistance - bias > z;
    }

    void visibilityFragment(inout vec4 FragColor)
    {
        // 1. Project world position into Camera's Screen Space
        vec4 projPos = cameraVP * vec4(worldPos, 1.0);
        vec3 projCoords = projPos.xyz / projPos.w; // Perspective divide

        // 2. Check if the point is inside the Camera's Frustum (FOV)
        // Since we used a bias matrix, the valid range is [0.0, 1.0]
        bool inFrustum = projCoords.x >= 0.0 && projCoords.x <= 1.0 &&
                         projCoords.y >= 0.0 && projCoords.y <= 1.0 &&
                         projCoords.z >= 0.0 && projCoords.z <= 1.0;

        float shadow = 0.0;
        if (inFrustum)
        {

           float shadow = 0.0;

            float closestDepth = texture(shadowMap, projCoords.xy).r;
            float currentDepth = projCoords.z;

            // Bias to prevent shadow acne
            float bias = 0.0005;
            shadow = (currentDepth - bias > closestDepth) ? 1.0 : 0.0;

            // Lighting calculation
            vec3 lightDir = normalize(eyePos - worldPos);
            float normDif = max(dot(normal, lightDir), 0.0);

            if (shadow < 0.5 /*&& normDif > 0.1*/)
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

osg::Vec3 ViewshedAnalysis::geoPointsToVev3(osgEarth::GeoPoint GeoPoint)
{
    osg::Vec3d worldPos;
    GeoPoint.toWorld(worldPos,_mapNode->getTerrain());
    osg::Vec3 worldPosition = osg::Vec3(worldPos);
    return worldPosition;
}

osg::AutoTransform* ViewshedAnalysis::makeIndicator(osg::Vec3 eye)
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

osg::ref_ptr<osg::MatrixTransform> createFrustumNode(osg::Camera* camera, float near_p, float far_p) {
    // 1. Extract 'top' and 'right' from the camera's existing projection matrix
    // Since you set it using osg::Matrix::frustum(-right, right, -top, top, near, far)
    osg::Matrix proj = camera->getProjectionMatrix();

    // In a standard frustum matrix:
    // proj(0,0) = near / right
    // proj(1,1) = near / top
    double right = near_p / proj(0,0);
    double top = near_p / proj(1,1);
    double ratio = (double)far_p / (double)near_p;
    double fTop = top * ratio;
    double fRight = right * ratio;

    // 2. Create Vertices in Local Space (Camera at 0,0,0 looking at -Z)
    osg::ref_ptr<osg::Vec3Array> vertices = new osg::Vec3Array(8);
    // Near Plane
    (*vertices)[0].set(-right, -top, -near_p);
    (*vertices)[1].set( right, -top, -near_p);
    (*vertices)[2].set( right,  top, -near_p);
    (*vertices)[3].set(-right,  top, -near_p);
    // Far Plane
    (*vertices)[4].set(-fRight, -fTop, -far_p);
    (*vertices)[5].set( fRight, -fTop, -far_p);
    (*vertices)[6].set( fRight,  fTop, -far_p);
    (*vertices)[7].set(-fRight,  fTop, -far_p);

    // 3. Create Geometry
    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
    geom->setVertexArray(vertices);

    osg::ref_ptr<osg::DrawElementsUInt> indices = new osg::DrawElementsUInt(GL_LINES);
    // Near Square
    indices->push_back(0); indices->push_back(1); indices->push_back(1); indices->push_back(2);
    indices->push_back(2); indices->push_back(3); indices->push_back(3); indices->push_back(0);
    // Far Square
    indices->push_back(4); indices->push_back(5); indices->push_back(5); indices->push_back(6);
    indices->push_back(6); indices->push_back(7); indices->push_back(7); indices->push_back(4);
    // Connecting Lines
    indices->push_back(0); indices->push_back(4); indices->push_back(1); indices->push_back(5);
    indices->push_back(2); indices->push_back(6); indices->push_back(3); indices->push_back(7);
    geom->addPrimitiveSet(indices);

    // 4. Wrap in Geode and MatrixTransform
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable(geom);
    geode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

    osg::ref_ptr<osg::MatrixTransform> xform = new osg::MatrixTransform;
    xform->addChild(geode);

    // Set initial position based on camera's current view
    xform->setMatrix(osg::Matrix::inverse(camera->getViewMatrix()));

    return xform;
}

bool ViewshedAnalysis::setUpCamera()
{
    _depthCamera = new osg::Camera;

    _depthCamera->setClearMask(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT);

    _depthCamera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    _depthCamera->setRenderOrder(osg::Camera::PRE_RENDER);
    _depthCamera->setRenderTargetImplementation(osg::Camera::FRAME_BUFFER_OBJECT);
    _depthCamera->setViewport(0, 0, SM_TEXTURE_WIDTH, SM_TEXTURE_WIDTH);
    _depthCamera->getOrCreateStateSet()->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
    _depthCamera->attach(osg::Camera::DEPTH_BUFFER, depthMap.get());
    // _depthCamera->attach(osg::Camera::COLOR_BUFFER, colorTexture.get());

    // camera->setNodeMask(0xffffffff & (~INTERSECT_IGNORE));

    // osg::Matrix  shadowProj = osg::Matrix::perspective(90, SM_TEXTURE_WIDTH / SM_TEXTURE_WIDTH, near_plane, far_plane);

    //Updating Projection matrix of camera
    {
        double vFovRad = osg::DegreesToRadians(_verticalFOV);
        double hFovRad = osg::DegreesToRadians(_horizontalFOV);

        double top = near_plane * tan(vFovRad / 2.0);
        double right = near_plane * tan(hFovRad / 2.0);

        osg::Matrix shadowProj = osg::Matrix::frustum(-right, right,-top, top, near_plane, far_plane);

        _depthCamera->setProjectionMatrix(shadowProj);
    }

    _depthCamerasVP = new osgEarth::VirtualProgram();
    _depthCamerasVP->setFunction("depthMapVertex",depthMapVert,osgEarth::VirtualProgram::LOCATION_VERTEX_VIEW);
    _depthCamerasVP->setFunction("depthMapFragment",depthMapFrag,osgEarth::VirtualProgram::LOCATION_FRAGMENT_COLORING);

    _depthCamera->getOrCreateStateSet()->setAttributeAndModes(_depthCamerasVP, osg::StateAttribute::ON);

    _depthCamera->getOrCreateStateSet()->addUniform(_cameraPosUniform);
    _depthCamera->getOrCreateStateSet()->addUniform(_farPlaneUniform);
    _depthCamera->getOrCreateStateSet()->addUniform(_nearPlaneUniform);

    _depthCamera->addChild(_shadowedScene);
    _parentScene->addChild(_depthCamera);

    _cameraIndicator = makeIndicator(_cameraPos);
    _parentScene->getParent(0)->addChild(_cameraIndicator);

    frustumVisual = createFrustumNode(_depthCamera.get(), near_plane, far_plane);
    _parentScene->getParent(0)->addChild(frustumVisual);


    //setting view matrix to camera
    osgEarth::GeoPoint dir (_mapNode->getMapSRS(), _camGeoPos.x()+ 1.0, _camGeoPos.y(), _camGeoPos.z(), osgEarth::ALTMODE_ABSOLUTE);
    osgEarth::GeoPoint up (_mapNode->getMapSRS(), _camGeoPos.x(), _camGeoPos.y(), _camGeoPos.z() + 1.0, osgEarth::ALTMODE_ABSOLUTE);

    osg::Matrix view = osg::Matrix::lookAt(_cameraPos, geoPointsToVev3(dir), geoPointsToVev3(up));
    _depthCamera->setViewMatrix(view);

    //setting inverse view matrix of shader uniform
    _inverseViewUniform->set(osg::Matrixf::inverse(view));
    _depthCamera->getOrCreateStateSet()->addUniform(_inverseViewUniform);

    osg::Matrix worldMatrix = osg::Matrix::inverse(_depthCamera->getViewMatrix());
    frustumVisual->setMatrix(worldMatrix);

    //setting camera view matrix to shader uniform
    {
        // Create a Bias Matrix (maps -1->1 to 0->1)
        osg::Matrix biasMatrix = osg::Matrix::translate(1.0, 1.0, 1.0) * osg::Matrix::scale(0.5, 0.5, 0.5);

        // Get the Camera's View and Projection
        osg::Matrix view1 = _depthCamera->getViewMatrix();
        osg::Matrix proj = _depthCamera->getProjectionMatrix();

        // Combine them: VP = View * Projection * Bias
        osg::Matrix viewProjectionBias = view1 * proj * biasMatrix;

        _cameraVPUniform->set(viewProjectionBias);
    }

}

void  ViewshedAnalysis::buildModel()
{
    if (!_shadowedScene.valid())
    {
        osg::notify(osg::WARN) << MODULE_ERROR << "shadow scene is not valid" << std::endl;
        return;
    }

    // _mainViewer->getCamera()->getGraphicsContext()->getState()->setUseModelViewAndProjectionUniforms(true);

    depthMap = new osg::Texture2D;
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
    far_plane  = _viewDistance + _farPlanOffSet;

    _cameraPosUniform = new osg::Uniform("eyePos", _cameraPos);
    _viewDistanceUniform = new osg::Uniform("user_area", _viewDistance);
    _farPlaneUniform = new osg::Uniform("far_plane",far_plane);
    _nearPlaneUniform = new osg::Uniform("near_plane", near_plane);

    _inverseViewUniform = new osg::Uniform("inverse_view", osg::Matrixf());

    _shadowMapUniform = new osg::Uniform("shadowMap", 10);
    _cameraVPUniform = new osg::Uniform("cameraVP", osg::Matrixf());

    _visibleColorUniform = new osg::Uniform("visibleColor", visibleColor);
    _invisibleColorUniform = new osg::Uniform("invisibleColor", invisibleColor);



    setUpCamera();


    _visibilityShaderVP = new osgEarth::VirtualProgram();
    _visibilityShaderVP->setFunction("visibilityVertex",visibilityShaderVert,osgEarth::VirtualProgram::LOCATION_VERTEX_VIEW);
    _visibilityShaderVP->setFunction("visibilityFragment",visibilityShaderFrag,osgEarth::VirtualProgram::LOCATION_FRAGMENT_COLORING);

    _parentScene->getOrCreateStateSet()->setTextureAttributeAndModes(10, depthMap, osg::StateAttribute::ON);
    _parentScene->getOrCreateStateSet()->setAttributeAndModes(_visibilityShaderVP, osg::StateAttribute::ON);
    _parentScene->getOrCreateStateSet()->addUniform(_shadowMapUniform);
    _parentScene->getOrCreateStateSet()->addUniform(_visibleColorUniform);
    _parentScene->getOrCreateStateSet()->addUniform(_invisibleColorUniform);

    _parentScene->getOrCreateStateSet()->addUniform(_cameraPosUniform);
    _parentScene->getOrCreateStateSet()->addUniform(_farPlaneUniform);
    _parentScene->getOrCreateStateSet()->addUniform(_nearPlaneUniform);
    _parentScene->getOrCreateStateSet()->addUniform(_viewDistanceUniform);
    _parentScene->getOrCreateStateSet()->addUniform(_cameraVPUniform);

    setupDebugHUD();
}

ViewshedAnalysis::ViewshedAnalysis(osgEarth::MapNode* mapNode, osgEarth::GeoPoint camPos, float distance):
    _mapNode(mapNode),
    _viewDistance(distance)
{
    _shadowedScene = _mapNode->asGroup();
    _parentScene = _shadowedScene->getParent(0);
    _cameraPos = geoPointsToVev3(camPos);
    _camGeoPos = camPos;
}

ViewshedAnalysis::~ViewshedAnalysis()
{
    clear();
}


void ViewshedAnalysis::updateProjectionMatrix()
{
    if(!_depthCamera.valid()) return;

    //Updating Projection matrix of camera
    {
        double vFovRad = osg::DegreesToRadians(_verticalFOV);
        double hFovRad = osg::DegreesToRadians(_horizontalFOV);

        double top = near_plane * tan(vFovRad / 2.0);
        double right = near_plane * tan(hFovRad / 2.0);

        osg::Matrix shadowProj = osg::Matrix::frustum(-right, right,-top, top, near_plane, far_plane);

        _depthCamera->setProjectionMatrix(shadowProj);
    }

    //updating view matrix of uniform
    {
        // Create a Bias Matrix (maps -1->1 to 0->1)
        osg::Matrix biasMatrix = osg::Matrix::translate(1.0, 1.0, 1.0) * osg::Matrix::scale(0.5, 0.5, 0.5);

        // Combine them: VP = View * Projection * Bias
        osg::Matrix viewProjectionBias = _depthCamera->getViewMatrix() * _depthCamera->getProjectionMatrix() * biasMatrix;

        _cameraVPUniform->set(viewProjectionBias);
    }

    //updating projection of frustrum outline
    _parentScene->getParent(0)->removeChild(frustumVisual);

    frustumVisual = createFrustumNode(_depthCamera.get(), near_plane, far_plane);
    _parentScene->getParent(0)->addChild(frustumVisual);


}

void ViewshedAnalysis::updateViewMatrix(){

    {
        osg::Vec3 up = geoPointsToVev3(osgEarth::GeoPoint(_mapNode->getMapSRS(), _camGeoPos.x(), _camGeoPos.y(), _camGeoPos.z()+1,osgEarth::ALTMODE_ABSOLUTE ));
        osg::Vec3f pos, dir, up1;
        _depthCamera->getViewMatrix().getLookAt(pos,dir,up1);
        osg::Matrix view =
            osg::Matrix::lookAt(_cameraPos, _cameraPos + (dir-pos), up);

        _depthCamera->setViewMatrix(view);

        _inverseViewUniform->set(osg::Matrixf::inverse(view));
    }

    osg::Matrix worldMatrix = osg::Matrix::inverse(_depthCamera->getViewMatrix());
    frustumVisual->setMatrix(worldMatrix);

    {
        // Create a Bias Matrix (maps -1->1 to 0->1)
        osg::Matrix biasMatrix = osg::Matrix::translate(1.0, 1.0, 1.0) * osg::Matrix::scale(0.5, 0.5, 0.5);

        // Get the Camera's View and Projection
        osg::Matrix view1 = _depthCamera->getViewMatrix();
        osg::Matrix proj = _depthCamera->getProjectionMatrix();

        // Combine them: VP = View * Projection * Bias
        osg::Matrix viewProjectionBias = view1 * proj * biasMatrix;

        _cameraVPUniform->set(viewProjectionBias);
    }
}

void ViewshedAnalysis::setCameraPosition(osgEarth::GeoPoint pos)
{
    if(!_depthCamera.valid()) return;

    _cameraPos = geoPointsToVev3(pos);
    _camGeoPos = pos;
    _cameraPosUniform->set(_cameraPos);
    if (_cameraIndicator.valid()) _cameraIndicator->setPosition(_cameraPos);
    updateViewMatrix();

}

void ViewshedAnalysis::setRotation(double angle, ViewshedAnalysis::Rotation dir)
{
    if (!_depthCamera) return;

        double radians = osg::DegreesToRadians(angle);

        osg::Vec3d eye, center, up;
        _depthCamera->getViewMatrix().getLookAt(eye, center, up);

        // 1. Get current Local Directions
        osg::Vec3d lookDir = center - eye;
        lookDir.normalize();

        // Normalize current up just in case
        up.normalize();

        // 2. Calculate the "Right" vector (Side axis)
        // This is perpendicular to both where we look and where 'up' is.
        osg::Vec3d side = lookDir ^ up; // Cross product
        side.normalize();

        osg::Matrixd rotationMatrix;

        if (dir == Rotation::HORIZANTAL) {
            // Rotate around the Up vector (Panning left/right)
            rotationMatrix = osg::Matrixd::rotate(radians, up);
        }
        else if (dir == Rotation::VERTICAL) {
            // Rotate around the Side vector (Tilting up/down)
            rotationMatrix = osg::Matrixd::rotate(radians, side);
        }

        // 3. Apply rotation to the Look direction and Up vector
        osg::Vec3d newLookDir = rotationMatrix.preMult(lookDir);
        osg::Vec3d newUp = rotationMatrix.preMult(up);

        // 4. Update the View Matrix
        // We keep 'eye' the same, and look at (eye + newLookDir)
        _depthCamera->setViewMatrixAsLookAt(eye, eye + newLookDir, newUp);

    // 6. Update the frustum visualizer
    osg::Matrix worldMatrix = osg::Matrix::inverse(_depthCamera->getViewMatrix());
    frustumVisual->setMatrix(worldMatrix);

    // 7. Sync the shader uniform
    if(_inverseViewUniform.valid()) {
        _inverseViewUniform->set(osg::Matrixf(worldMatrix));
    }

    // Create a Bias Matrix (maps -1->1 to 0->1)
    osg::Matrix biasMatrix = osg::Matrix::translate(1.0, 1.0, 1.0) * osg::Matrix::scale(0.5, 0.5, 0.5);

    // Get the Camera's View and Projection
    osg::Matrix view1 = _depthCamera->getViewMatrix();
    osg::Matrix proj = _depthCamera->getProjectionMatrix();

    // Combine them: VP = View * Projection * Bias
    osg::Matrix viewProjectionBias = view1 * proj * biasMatrix;

    _cameraVPUniform->set(viewProjectionBias);

}

void ViewshedAnalysis::setDistance(int distance)
{
    _viewDistance = distance;
    far_plane = _viewDistance + _farPlanOffSet;

    if(_viewDistanceUniform.valid()){
        _viewDistanceUniform->set((float)_viewDistance);
        _farPlaneUniform->set(far_plane);
    }

    updateProjectionMatrix();
}

void ViewshedAnalysis::setVerticalFOV(int angle)
{
    _verticalFOV = angle;
    updateProjectionMatrix();
}

void ViewshedAnalysis::setHorizontalFOV(int angle)
{
    _horizontalFOV = angle;
    updateProjectionMatrix();
}


void ViewshedAnalysis::setVisibleAreaColor(const osg::Vec4 color)
{
    if(_visibleColorUniform.valid()){
        _visibleColorUniform->set(color);
    }

    visibleColor = color;
}

void ViewshedAnalysis::setHiddenAreaColor(const osg::Vec4 color)
{
    if(_invisibleColorUniform.valid()){
        _invisibleColorUniform->set(color);
    }

    invisibleColor = color;
}


void  ViewshedAnalysis::clear()
{
    // _mainViewer->getCamera()->getGraphicsContext()->getState()->setUseModelViewAndProjectionUniforms(false);

           // _depthCamera->getOrCreateStateSet()->setAttribute(depthShader, osg::StateAttribute::OFF);
           _depthCamera->getOrCreateStateSet()->removeUniform(_cameraPosUniform);
           _depthCamera->getOrCreateStateSet()->removeUniform(_farPlaneUniform);
           _depthCamera->getOrCreateStateSet()->removeUniform(_nearPlaneUniform);
           _depthCamera->getOrCreateStateSet()->removeUniform(_inverseViewUniform);
           _depthCamera->removeChild(_shadowedScene);
           _parentScene->removeChild(_depthCamera);
           _depthCamera = nullptr;

       _parentScene->getOrCreateStateSet()->removeUniform(_visibleColorUniform);
       _parentScene->getOrCreateStateSet()->removeUniform(_invisibleColorUniform);
       _parentScene->getOrCreateStateSet()->removeUniform(_cameraPosUniform);
       _parentScene->getOrCreateStateSet()->removeUniform(_farPlaneUniform);
       _parentScene->getOrCreateStateSet()->removeUniform(_nearPlaneUniform);
       _parentScene->getOrCreateStateSet()->removeUniform(_viewDistanceUniform);
       _parentScene->getOrCreateStateSet()->removeUniform(_shadowMapUniform);
       _parentScene->getOrCreateStateSet()->removeUniform(_cameraVPUniform);

       _parentScene->getOrCreateStateSet()->setAttributeAndModes(_visibilityShaderVP, osg::StateAttribute::OFF);
       _parentScene->getOrCreateStateSet()->setTextureAttributeAndModes(10, depthMap, osg::StateAttribute::OFF);
       _parentScene->getOrCreateStateSet()->removeAttribute(osg::StateAttribute::PROGRAM);
       // _parentScene->getOrCreateStateSet()->removeAttribute(_visibilityShaderVP);
       _parentScene->getOrCreateStateSet()->removeTextureAttribute(10, osg::StateAttribute::TEXTURE);

       // _parentScene->removeChild(_shadowedScene);
       _parentScene->getParent(0)->removeChild(_cameraIndicator);
       _parentScene->getParent(0)->removeChild(frustumVisual);

       _cameraIndicator = nullptr;
       frustumVisual = nullptr;


}

void ViewshedAnalysis::setupDebugHUD()
{
    osg::ref_ptr<osg::Camera> hudCamera = new osg::Camera;
    hudCamera->setProjectionMatrix(osg::Matrix::ortho2D(0, 1280, 0, 720));
    hudCamera->setReferenceFrame(osg::Transform::ABSOLUTE_RF);
    hudCamera->setViewMatrix(osg::Matrix::identity());
    hudCamera->setClearMask(GL_DEPTH_BUFFER_BIT);
    hudCamera->setRenderOrder(osg::Camera::POST_RENDER);
    hudCamera->setAllowEventFocus(false);
    hudCamera->setNodeMask(0xffffffff);

    osg::ref_ptr<osg::Geode> quadGeode = new osg::Geode;
    osg::ref_ptr<osg::Geometry> quadBoard = osg::createTexturedQuadGeometry(
        osg::Vec3(50.0f, 50.0f, 0.0f),
        osg::Vec3(400.0f, 0.0f, 0.0f),
        osg::Vec3(0.0f, 400.0f, 0.0f)
    );
    quadGeode->addDrawable(quadBoard);
    hudCamera->addChild(quadGeode);

    osg::StateSet* stateset = quadBoard->getOrCreateStateSet();
    stateset->setTextureAttributeAndModes(0, depthMap.get(), osg::StateAttribute::ON);
    stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    stateset->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);

    osg::ref_ptr<osg::Program> visualProgram = new osg::Program;

    // Vertex Shader (Core Profile)
    // Note: osg_Vertex and osg_MultiTexCoord0 are automatically
    // provided by OSG when using setUseModelViewAndProjectionUniforms(true)
    const char* vertSource =
        "#version 150\n"
        "in vec4 osg_Vertex;\n"
        "in vec4 osg_MultiTexCoord0;\n"
        "uniform mat4 osg_ModelViewProjectionMatrix;\n"
        "out vec2 texCoord;\n"
        "void main() {\n"
        "  texCoord = osg_MultiTexCoord0.xy;\n"
        "  gl_Position = osg_ModelViewProjectionMatrix * osg_Vertex;\n"
        "}\n";

    // Fragment Shader (Core Profile)
    const char* fragSource =
        "#version 150\n"
        "uniform sampler2D depthMap;\n"
        "uniform float near;\n"
        "uniform float far;\n"
        "in vec2 texCoord;\n"
        "out vec4 fragColor;\n"
        "void main() {\n"
        "  float z = texture(depthMap, texCoord).r;\n"
        "  // Linearization for better visibility\n"
        "  float lz = (2.0 * near) / (far + near - z * (far - near));\n"
        "  fragColor = vec4(vec3(lz), 1.0);\n"
        "}\n";

    visualProgram->addShader(new osg::Shader(osg::Shader::VERTEX, vertSource));
    visualProgram->addShader(new osg::Shader(osg::Shader::FRAGMENT, fragSource));

    stateset->setAttributeAndModes(visualProgram, osg::StateAttribute::ON);
    stateset->addUniform(new osg::Uniform("depthMap", 0));
    stateset->addUniform(new osg::Uniform("near", (float)near_plane));
    stateset->addUniform(new osg::Uniform("far", (float)far_plane));

    _parentScene->getParent(0)->addChild(hudCamera);
}




