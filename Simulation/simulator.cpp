#include "simulator.h"

/**
* A simple simulator that moves an object around the Earth. We use this to
* demonstrate/test tethering.
*/

Simulator::Simulator(EarthManipulator* manip, MapNode* mapnode,
        osg::Group* annoGroup,
        QString name)
    :  _manip(manip), _mapNode(mapnode) ,_annoGroup(annoGroup), _name(name)
{
    Simulator::_instance = this;
    this->_followCamera = true;
    _simTime = 0.0;
    _lastUpdateTime = osg::Timer::instance()->time_s();
    _haveLastPosition = false;
}

const GeoPoint& Simulator::getPosition() const
{
    return _geo->getPosition();
}

bool Simulator::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
    if (!isActive) return false;
    if (ea.getEventType() == ea.FRAME)
    {
        if (_isPaused) return false;
        for (auto& id : _trackMap.keys())
        {
            TrackData& data = _trackMap[id];
            if (!data.hasTarget || data.interpolationProgress >= 1.0)
                continue;

            //BELOW FOR REAL TIME DATA:::
            // double now = osg::Timer::instance()->time_s();
            // double delta = data.nextUpdateTimeSec - data.lastUpdateTimeSec;

            // if (delta > 0.0) {
            //     data.interpolationProgress = (now - data.lastUpdateTimeSec) / delta;
            //     if (data.interpolationProgress > 1.0)
            //         data.interpolationProgress = 1.0;
            // }

            double now = osg::Timer::instance()->time_s();
            double duration = data.interpolationEndTime - data.interpolationStartTime;
            if (duration <= 0.0) duration = 1.0;

            data.interpolationProgress = (now - data.interpolationStartTime) / duration;
            if (data.interpolationProgress > 1.0)
                data.interpolationProgress = 1.0;

            osg::Vec3d interpolated = data.lastPosition * (1.0 - data.interpolationProgress)
                                      + data.currentTarget * data.interpolationProgress;

            GeoPoint geo(_mapNode->getMapSRS(), interpolated.x(), interpolated.y(), interpolated.z());
            if (data.label.valid())
                data.label->setPosition(geo);

            if (data.geo.valid())
                data.geo->setPosition(geo);

            if (data.planeTail.valid())
                data.planeTail->addTailPoint(geo.vec3d());

            if (_followCamera && id == _followTargetID && _manip) {
                osgViewer::View* view = dynamic_cast<osgViewer::View*>(&aa);
                if (view)
                {
                    osgEarth::Util::EarthManipulator* manip =
                        dynamic_cast<osgEarth::Util::EarthManipulator*>(view->getCameraManipulator());
                    GeoPoint gp = data.geo->getPosition();
                    if (manip)
                    {
                        manip->setViewpoint(osgEarth::Viewpoint("FollowTarget", gp.x(), gp.y(), gp.z() + 150, _heading, _pitch, _range),
                            0.0);
                    }
                }
            }
        }
    }
    else if (ea.getKey())
    {

    }
    return false;
}

Simulator* Simulator::_instance = nullptr;

Simulator* Simulator::instance()
{
    return _instance;
}

void Simulator::setInstance(Simulator* sim)
{
    _instance = sim;
}

void Simulator::setCameraFocus(bool follow, int targetID, double heading, double pitch, double range, int viewMode)
{
    _followCamera = follow;
    _followTargetID = targetID;
    _heading = heading;
    _pitch = pitch;
    _range = range;
    _currentViewMode = (enum ViewMode) viewMode;
}
void Simulator::setFollowCamera(bool follow) { _followCamera = follow; }
void Simulator::setFollowTargetID(int id) { _followTargetID = id; }
void Simulator::setHeading(double h) { _heading = h; }
void Simulator::setPitch(double p) { _pitch = p; }
void Simulator::setRange(double r) { _range = r; }
void Simulator::setViewMode(int mode) { _currentViewMode = (enum ViewMode) mode; }

bool Simulator::isFollowing() const
{
    return _followCamera;
}

void Simulator::adjustPitch(double delta)
{
    _pitch += delta;
}

void Simulator::adjustHeading(double delta)
{
    _heading += delta;
}

void Simulator::adjustRange(double factor)
{
    _range *= factor;
}

void Simulator::highlightModel(osg::ref_ptr<ModelNode> model)
{
    if (!model) return;

    Style highlightStyle;
    highlightStyle.getOrCreate<ModelSymbol>()->autoScale() = true;
    highlightStyle.getOrCreate<ModelSymbol>()->url()->setLiteral("D:/Models/su27_highlighted.ive");
    model->setStyle(highlightStyle);
}

void Simulator::restoreModelStyle(osg::ref_ptr<ModelNode> model)
{
    if (!model) return;

    Style normalStyle;
    normalStyle.getOrCreate<ModelSymbol>()->autoScale() = true;
    normalStyle.getOrCreate<ModelSymbol>()->url()->setLiteral("D:/Models/su27.ive");
    model->setStyle(normalStyle);
}

void Simulator::updatePositionFromData(int id, QString tracktype, double lat, double lon, double alt)
{
    std::lock_guard<std::mutex> lock(_dataMutex);
    osg::Vec3d newTarget(lon, lat, alt);
    if (!_trackMap.contains(id))
        initializeTrack(id, tracktype.toStdString());
    TrackData& data = _trackMap[id];
    if (!data.hasTarget) {
        GeoPoint gp(_mapNode->getMapSRS(), newTarget);
        data.geo->setPosition(gp);
        data.label->setPosition(gp);
        data.planeTail->addTailPoint(newTarget);

        data.lastPosition = newTarget;
        data.currentTarget = newTarget;
        data.interpolationProgress = 1.0;
        data.hasTarget = true;
    } else {
        osg::Vec3d lastPos = data.geo->getPosition().vec3d();
        osg::Vec3d direction = newTarget - lastPos;
        double heading = atan2(direction.x(), direction.y());

        osg::Quat orientationQuat;
        orientationQuat.makeRotate(-heading, osg::Vec3(0, 0, 1));
        data.geo->getPositionAttitudeTransform()->setAttitude(orientationQuat);
        data.lastPosition = data.geo->getPosition().vec3d();
        data.currentTarget = newTarget;

        // === Compute Speed ===
        osg::Vec3d deltaVec = data.currentTarget - data.lastPosition;
        double distance = deltaVec.length();
        double time_sec = 1.0;
        data.calculated_speed_mps = distance / time_sec;
        data.interpolationProgress = 0.0;
        data.hasTarget = true;
    }
}

void Simulator::updatePositionFromDataUDP(int id, QString tracktype, double lat, double lon, double alt, double speed, double rollDeg, double pitchDeg, double yawDeg)
{
    std::lock_guard<std::mutex> lock(_dataMutex);
    osg::Vec3d newTarget(lon, lat, alt);

    if (!_trackMap.contains(id))
        initializeTrack(id, tracktype.toStdString());  // customize this

    TrackData& data = _trackMap[id];
    double currentTime = osg::Timer::instance()->time_s();
    if (!data.hasTarget) {
        // GeoPoint gp(_mapNode->getMapSRS(), newTarget, ALTMODE_RELATIVE);
        // data.geo->setPosition(gp);
        // data.label->setPosition(gp);
        // data.planeTail->addTailPoint(newTarget);

        // data.lastPosition = newTarget;
        // data.currentTarget = newTarget;
        // data.interpolationProgress = 1.0;
        // data.hasTarget = true;

        // //BELOW FOR REALISTIC SPEED::
        // // data.lastUpdateTimeSec = currentTime;
        // // data.nextUpdateTimeSec = currentTime + 1.0;  // If server updates every second
        // // data.speed_mps = speed;   // Store speed from server
        // // data.interpolationProgress = 0.0;
        // // data.lastPosition = data.geo->getPosition().vec3d();
        // // data.currentTarget = osg::Vec3d(lon, lat, alt);
        // // data.hasTarget = true;

        // First time initialization
        data.lastPosition = newTarget;
        data.currentTarget = newTarget;
        data.interpolationProgress = 1.0;
        data.hasTarget = true;

        data.interpolationStartTime = currentTime;
        data.interpolationEndTime = currentTime + 1.0;
        data.lastUpdateTimeSec = currentTime;

        GeoPoint gp(_mapNode->getMapSRS(), newTarget, AltitudeMode());
        data.geo->setPosition(gp);
        data.label->setPosition(gp);
        data.planeTail->addTailPoint(newTarget);

    } else {
        // // === Compute Speed === Internal SPEED
        // osg::Vec3d deltaVec = data.currentTarget - data.lastPosition;
        // double distance = deltaVec.length();       // in meters

        // double time_sec = 1.0;                     // assuming 1 second between updates
        // data.calculated_speed_mps = distance / time_sec;

        // emit speedUpdated(id, data.calculated_speed_mps);  // Emit to notify observers

        // qDebug() << "[EMIT] Speed update emitted for ID:" << id << "Speed:" << data.calculated_speed_mps;

        // Compute speed
        osg::Vec3d deltaVec = newTarget - data.currentTarget;
        double distance = deltaVec.length();
        double deltaTime = currentTime - data.lastUpdateTimeSec;
        if (deltaTime <= 0.0) deltaTime = 1.0;

        data.calculated_speed_mps = distance / deltaTime;
        emit speedUpdated(id, data.calculated_speed_mps);
        qDebug() << "[SPEED] ID:" << id
                 << "Dist:" << distance
                 << "Time:" << deltaTime
                 << "Speed:" << data.calculated_speed_mps;

        data.lastPosition = data.currentTarget;
        data.currentTarget = newTarget;
        data.interpolationProgress = 0.0;
        data.interpolationStartTime = currentTime;
        data.interpolationEndTime = currentTime + deltaTime;
        data.lastUpdateTimeSec = currentTime;

        osg::Vec3d direction = newTarget - data.lastPosition;
        double heading = atan2(direction.x(), direction.y());
        osg::Quat yawQuat;
        yawQuat.makeRotate(-heading, osg::Vec3(0, 0, 1));

        osg::Quat modelCorrection;
        modelCorrection.makeRotate(osg::DegreesToRadians(-90.0), osg::Vec3(0, 0, 1));

        osg::Quat finalQuat = yawQuat * modelCorrection;
        data.geo->getPositionAttitudeTransform()->setAttitude(finalQuat);
    }
}

double Simulator::haversineDistance(double lat1, double lon1, double lat2, double lon2)
{
    const double R = 6371000.0; // Earth radius in meters
    double dLat = osg::DegreesToRadians(lat2 - lat1);
    double dLon = osg::DegreesToRadians(lon2 - lon1);
    double a = sin(dLat / 2) * sin(dLat / 2) +
               cos(osg::DegreesToRadians(lat1)) * cos(osg::DegreesToRadians(lat2)) *
                   sin(dLon / 2) * sin(dLon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return R * c;
}

void Simulator::initializeTrack(int id, const std::string& labelText)
{
    if (_trackMap.contains(id)) return;
    Style labelStyle;
    TextSymbol* symbol = labelStyle.getOrCreateSymbol<TextSymbol>();
    symbol->fill() = Color::White;
    symbol->halo()->color() = Color::Black;
    symbol->size() = 12.0f;
    symbol->alignment() = TextSymbol::ALIGN_CENTER_BOTTOM;

    data.label = new osgEarth::LabelNode(labelText, labelStyle);
    data.label->setDynamic(true);
    data.label->setHorizonCulling(false);
    data.geo = new GeoPositionNode();

    Style modelStyle;
    QString modelPath = QCoreApplication::applicationDirPath() + "/MapData/missile_2obj.obj";
    std::string pathStr = modelPath.toStdString();
    modelStyle.getOrCreate<ModelSymbol>()->autoScale() = true;
    modelStyle.getOrCreate<ModelSymbol>()->url()->setLiteral("C:/Users/pnmt1054/Adithya-working-directory/QT_PROJECTS/GeoLayer-X/Data/simulator/missile_2obj.obj");

    data.model = new ModelNode(_mapNode, modelStyle);
    data.attachPoint = new osg::Group();
    data.attachPoint->addChild(data.model);
    data.attachPoint->addChild(data.label.get());
    data.geo->getPositionAttitudeTransform()->addChild(data.attachPoint);
    _annoGroup->addChild(data.attachPoint);
    data.geo->addChild(data.model);
    data.planeTail = new PlaneTail(_mapNode, _annoGroup);
    data.geo->addChild(data.planeTail.get());
    _annoGroup->addChild(data.geo.get());
    _trackMap[id] = data;
}

void Simulator::adjustSpeed(double delta)
{
    _interpolationSpeed += delta;

    if (_interpolationSpeed < 0.01)
        _interpolationSpeed = 0.01;
    else if (_interpolationSpeed > 0.01)
        _interpolationSpeed = 0.01;
}

void Simulator::setPaused(bool paused)
{
    _isPaused = paused;
}

void Simulator::setPlaneMatrix(float latStart, float lonStart, float latEnd, float lonEnd) //CONVERT DEGRESS to RADIANS
{
    double bearing = GeoMath::bearing(D2R*latStart, D2R*lonStart, D2R*latEnd, D2R*lonEnd);
    _geo->setLocalRotation(osg::Quat(-bearing, osg::Vec3d(0, 0, 1)));
}

PlaneTail::PlaneTail(MapNode* mapnode, osg::Group* parentGroup) : _mapnode(mapnode), _parentGroup(parentGroup)
{
    _geode = new osg::Geode();
    _parentGroup->addChild(_geode);  // Add to simulator group

}

PlaneTail::PlaneTail()
{ }

PlaneTail::PlaneTail(const PlaneTail& rhs, const osg::CopyOp& copyop)
    : osg::Group(rhs, copyop)
{ }

void PlaneTail::addTailPoint(osg::Vec3d pos)
{
    PlaneTailPoint *planeTailPoint = new PlaneTailPoint(_mapnode, pos);
    _parentGroup->addChild(planeTailPoint);
    planeTailPoint->createLineToEarth(pos, _parentGroup);
    _tailPoints.push_back(planeTailPoint);
}

void PlaneTail::createLine(osg::Vec3d startline, osg::Vec3d endline)
{
    linesGeom = new osg::Geometry();
    osg::Vec3d startWorld;
    osg::Vec3d endWorld;
    vertices = new osg::Vec3dArray(2);
    GeoPoint(_mapnode->getMapSRS(), startline).toWorld(startWorld);
    GeoPoint(_mapnode->getMapSRS(), endline).toWorld(endWorld);
    (*vertices)[0] = startWorld;
    (*vertices)[1] = endWorld;
    linesGeom->setVertexArray(vertices);
    osg::ref_ptr<osg::Vec4Array> colors = new osg::Vec4Array;
    colors->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
    linesGeom->setColorArray(colors);
    linesGeom->setColorBinding(osg::Geometry::BIND_OVERALL);

    osg::ref_ptr<osg::Vec3Array> normals = new osg::Vec3Array;
    normals->push_back(osg::Vec3(0.0f, -1.0f, 0.0f));
    linesGeom->setNormalArray(normals);
    linesGeom->setNormalBinding(osg::Geometry::BIND_OVERALL);

    linesGeom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 2));
    _geode->addDrawable(linesGeom);
}

void PlaneTail::cleanup()
{
    for (auto& point : _tailPoints)
    {
        if (point.valid())
            point->cleanup(_parentGroup);
        _parentGroup->removeChild(point);
    }
    _tailPoints.clear();
    _parentGroup->removeChild(_geode);
    _geode = nullptr;
}

PlaneTailPoint::PlaneTailPoint(MapNode* mapnode, osg::Vec3d pos) :  GeoPositionNode(), _mapnode(mapnode)
{
    setNumChildrenRequiringEventTraversal(1);
    this->getOrCreateStateSet()->setRenderBinDetails(50, "DepthSortedBin");
    setMapNode(_mapnode);
    setCullingActive(false);
    osg::Geode* geode = new osg::Geode();
    osg::Sphere* shape = new osg::Sphere(osg::Vec3(0, 0, 0), 3);
    _shapeDrawable = new osg::ShapeDrawable(shape);
    _shapeDrawable->setColor(osg::Vec4f(0.0f, 1.0f, 0.0f, 1.0f));
    _shapeDrawable->setDataVariance(osg::Object::DYNAMIC);
    geode->addDrawable(_shapeDrawable);
    getPositionAttitudeTransform()->addChild(geode);
    this->addCullCallback(new GeoPositionNodeAutoScaler());
    setPosition(GeoPoint(_mapnode->getMapSRS(), pos));
}

//creating point on earth
void PlaneTailPoint::createLineToEarth(osg::Vec3d pos, osg::Group *annoGroup)
{
    path = new LineString();
    path->push_back(pos);
    // path->push_back(pos);    //THIS IS COMMENTED FOR NOW
    path->push_back(osg::Vec3d(pos.x(), pos.y(), 0));
    _height = pos.z();
    pathFeature = new Feature(path, _mapnode->getMapSRS());
    pathFeature->geoInterp() = GEOINTERP_RHUMB_LINE;
    _lineToEarth = new FeatureNode(pathFeature, createStyle());
    _lineToEarth->setNodeMask(false);
    annoGroup->addChild(_lineToEarth);
}

Style PlaneTailPoint::createStyle()
{
    Style pathStyle;
    pathStyle.getOrCreate<LineSymbol>()->stroke()->color() = Color(Color::Red, 0.75f);
    // pathStyle.getOrCreate<LineSymbol>()->stroke()->width() = 2.0f;
    // pathStyle.getOrCreate<LineSymbol>()->stroke()->stipple() = 255;
    pathStyle.getOrCreate<AltitudeSymbol>()->clamping() = AltitudeSymbol::CLAMP_TO_TERRAIN;
    pathStyle.getOrCreate<AltitudeSymbol>()->technique() = AltitudeSymbol::TECHNIQUE_GPU;
    pathStyle.getOrCreate<ExtrusionSymbol>()->height() = _height;// 250000.0f;
    pathStyle.getOrCreate<PolygonSymbol>()->fill()->color() = Color(Color::Red, 0);
    return pathStyle;
}

void PlaneTailPoint::traverse(osg::NodeVisitor& nv)
{
    if (nv.getVisitorType() == osg::NodeVisitor::EVENT_VISITOR)
    {
        osgGA::EventVisitor* ev = static_cast<osgGA::EventVisitor*>(&nv);
        for (osgGA::EventQueue::Events::iterator itr = ev->getEvents().begin();
             itr != ev->getEvents().end();
             ++itr)
        {
            osgGA::GUIEventAdapter* ea = dynamic_cast<osgGA::GUIEventAdapter*>(itr->get());
            if (ea && handle(*ea, *(ev->getActionAdapter())))
                ea->setHandled(true);
        }
    }
    GeoPositionNode::traverse(nv);
}

bool PlaneTailPoint::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
    if (ea.getHandled()) return false;
    osgViewer::View* view = dynamic_cast<osgViewer::View*>(&aa);

    if (!view) return false;

    if (!_mapnode) return false;

    if (ea.getEventType() == osgGA::GUIEventAdapter::MOVE)
    {
        IntersectionPicker picker(view, this);
        IntersectionPicker::Hits hits;

        if (picker.pick(ea.getX(), ea.getY(), hits))
        {
            setHover(true);
        }
        else
        {
            setHover(false);
        }
    }
    return false;
}

void PlaneTailPoint::setHover(bool hovered)
{
    if (_hovered != hovered)
    {
        bool wasHovered = _hovered;
        _hovered = hovered;
        if (wasHovered)
        {
            updateColor();
        }
        else
        {
            updateColor();
        }
    }
}

void PlaneTailPoint::updateColor()
{
    if (_hovered)
    {
        _shapeDrawable->setColor(osg::Vec4f(1.0f, 1.0f, 0.0f, 1.0f));
        _lineToEarth->setNodeMask(true);
    }
    else
    {
        _shapeDrawable->setColor(osg::Vec4f(0.0f, 1.0f, 0.0f, 1.0f));
        _lineToEarth->setNodeMask(false);
    }
}

void PlaneTailPoint::cleanup(osg::Group* parent)
{
    if (parent)
        parent->removeChild(_lineToEarth);
    _lineToEarth = nullptr;
    osg::PositionAttitudeTransform* pat = getPositionAttitudeTransform();
    if (pat && pat->getNumChildren() > 0)
        pat->removeChildren(0, pat->getNumChildren());
}

Simulator::~Simulator()
{
    for (auto id : _trackMap.keys())
    {
        TrackData& data = _trackMap[id];

        if (_annoGroup && data.geo.valid())
        {
            _annoGroup->removeChild(data.geo);
        }

        if (data.planeTail.valid())
        {
            data.planeTail->cleanup();
        }

        if (_annoGroup && data.attachPoint.valid())
        {
            _annoGroup->removeChild(data.attachPoint.get());
        }
        data.attachPoint = nullptr;

        data.label = nullptr;
        data.model = nullptr;
        data.geo = nullptr;
        data.planeTail = nullptr;
    }
    _trackMap.clear();
    _mapNode = nullptr;
    _manip = nullptr;
    _annoGroup = nullptr;
}

PlaneTail::~PlaneTail()
{
    for (auto& pt : _tailPoints)
    {
        if (pt.valid() && _parentGroup)
        {
            pt->cleanup(_parentGroup);
            _parentGroup->removeChild(pt);
        }
    }
    _tailPoints.clear();
    if (_geode && _parentGroup)
    {
        _parentGroup->removeChild(_geode);
    }
    _geode = nullptr;
    _parentGroup = nullptr;
}

PlaneTailPoint::~PlaneTailPoint()
{
    if (_lineToEarth && _lineToEarth->getNumParents() > 0)
    {
        osg::Group* parent = _lineToEarth->getParent(0)->asGroup();
        if (parent)
        {
            parent->removeChild(_lineToEarth);
        }
    }
    _lineToEarth = nullptr;
    pathFeature = nullptr;
    path = nullptr;
    _shapeDrawable = nullptr;
}

void Simulator::cleanup(osgViewer::Viewer* viewer)
{
    if (_geo.valid())
    {
        if (_geo->getNumParents() > 0)
            _geo->getParent(0)->removeChild(_geo.get());
    }
    _mapNode->removeChild(_geo.get());
    _simGroup->removeChild(_planeTail->_geode);

    _planeTail = nullptr;
    _model = nullptr;
    _label = nullptr;
    modelTransform = nullptr;
    _geo = nullptr;
    _simGroup = nullptr;
    _haveLastPosition = false;
    _dataCount = 0;
    _simTime = 0.0;
}
