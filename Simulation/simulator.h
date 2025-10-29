#ifndef SIMULATOR_H
#define SIMULATOR_H

//checking
#include <osgDB/ReadFile>
#include <osgUtil/Optimizer>
#include <osg/CoordinateSystemNode>
#include <osg/ShapeDrawable>
#include <osg/Switch>
#include <osg/Types>
#include <osgText/Text>
#include <osgViewer/Viewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgGA/TrackballManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/DriveManipulator>
#include <osgGA/KeySwitchMatrixManipulator>
#include <osgGA/StateSetManipulator>
#include <osgGA/AnimationPathManipulator>
#include <osgGA/TerrainManipulator>
#include <osgGA/SphericalManipulator>
#include <osgGA/Device>
#include <osgEarth/GeoMath>
#include <osgEarth/GeoTransform>
#include <osgEarth/MapNode>
#include <osgEarth/TerrainEngineNode>
#include <osgEarth/Viewpoint>
#include <osgEarth/EarthManipulator>
#include <osgEarth/ExampleResources>
#include <osgEarth/LogarithmicDepthBuffer>
#include <osgEarth/ViewFitter>
#include <osgEarth/AnnotationUtils>
#include <osgEarth/LabelNode>
#include <osgEarth/Style>
#include <osgEarth/ScreenSpaceLayout>
#include <osgEarth/ModelNode>
#include <osgEarth/LineDrawable>
#include <osgUtil/Optimizer>
#include <osgEarth/FeatureNode>
#include <osgEarth/FeatureIndex>
#include <osgEarth/IntersectionPicker>
#include <osgEarth/Registry>
#include <osgEarth/RTTPicker>
#include <osgEarth/GLUtils>
#include <osgEarth/GeoPositionNodeAutoScaler>
#include <osgEarth/ElevationQuery>
#include <osgEarth/FeatureModelLayer>
#include <osgEarth/OGRFeatureSource>
#include <QApplication>
#include <QSurfaceFormat>
#include <QTextCodec>
#include <osgEarth/LabelNode>
#include <QFile>
#include <QDebug>

using namespace osgEarth;
#define D2R (osg::PI/180.0)
#define R2D (180.0/osg::PI)

#include <QObject>
#include <osgEarth/Controls>
using namespace osgEarth::Controls;

class Simulator : public QObject, public osgGA::GUIEventHandler
{
    Q_OBJECT
public:
    Simulator(EarthManipulator* manip, MapNode* mapnode, osg::Group* annoGroup, QString name);
    static Simulator* instance();
    static void setInstance(Simulator* sim);
    const GeoPoint& getPosition() const;
    void cleanup(osgViewer::Viewer* viewer);
    double haversineDistance(double lat1, double lon1, double lat2, double lon2);
    ~Simulator();
    osg::ref_ptr<osgEarth::ModelNode> _selectedModel;
    int _selectedId = -1;
    bool isActive = true;
    std::mutex _dataMutex;
    osg::Vec3d _latestPosition;
    bool _hasNewPosition = false;
    void updatePositionFromData(int id, QString, double latitude, double longitude, double altitude);
    void updatePositionFromDataUDP(int id, QString tracktype, double lat, double lon, double alt, double speed,  double rollDeg, double                 pitchDeg, double yawDeg);
    void setPaused(bool paused);
    bool isPaused() const { return _isPaused; }
    bool _isPaused = false;
    osg::Vec3d _lastPosition;
    osg::Vec3d _currentTarget;
    double _interpolationProgress = 1.0;
    double _interpolationSpeed = 0.02; // 0.02 = approx. 50 frames to complete (1 sec at 50 fps)
    double getSpeed() const { return _interpolationSpeed; }
    void adjustSpeed(double delta);    bool _hasTarget = false;
    bool _isFirstFocus = true;
    void initializeTrack(int id, const std::string& labelText);
    void setCameraFocus(bool follow, int targetID, double heading, double pitch, double range, int viewMode);
    void setFollowCamera(bool follow);
    void setFollowTargetID(int id);
    void setHeading(double h);
    void setPitch(double p);
    void setRange(double r);
    void setViewMode(int mode);
    bool _followCamera = false;
    int _followTargetID = 1;
    enum ViewMode { VIEW_NONE, VIEW_REAR, VIEW_FRONT, VIEW_TOP, VIEW_SIDE };
    ViewMode _currentViewMode = VIEW_NONE;
    bool isFollowing() const;
    void adjustPitch(double delta);
    void adjustHeading(double delta);
    void adjustRange(double factor);
    struct TrackData {
        osg::Vec3d lastPosition;
        osg::ref_ptr<osgEarth::GeoPositionNode> geo;
        osg::ref_ptr<struct PlaneTail> planeTail;  // Assuming PlaneTail is your custom class
        osg::Vec3d currentTarget;
        osg::ref_ptr<osgEarth::LabelNode> label;
        double interpolationProgress = 1.0;
        double interpolationStartTime = 0.0;
        double interpolationEndTime = 0.0;
        bool hasTarget = false;
        osg::ref_ptr<osg::Group> attachPoint;
        osg::ref_ptr<osgEarth::ModelNode> model;
        osg::Quat lastOrientation;
        osg::Quat currentOrientation;
        double lastUpdateTimeSec = 0.0;
        double nextUpdateTimeSec = 0.0;
        double speed_mps = 0.0;  // From server
        double calculated_speed_mps = 0.0;
    };
    QMap<int, TrackData> _trackMap;
    TrackData data;
    void enable()
    { _isEnable = true; }
    void disable()
    { _isEnable = false; }
    bool isEnabled()
    { return _isEnable; }
    void setActive(bool enabled)
    { isActive = enabled; }

private:
    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa) override;
    void setPlaneMatrix(float latStart, float lonStart, float latEnd, float lonEnd);
    void highlightModel(osg::ref_ptr<ModelNode> model);
    void restoreModelStyle(osg::ref_ptr<ModelNode> model);
    static Simulator* _instance;
    struct PlaneTail*                         _planeTail;
    struct PlaneTailPoint*                     _ptp;
    ModelNode*                         _model;
    EarthManipulator*                  _manip;
    LabelNode*                         _label;
    osg::ref_ptr<GeoPositionNode>      _geo;
    int                                _dataNum = 0;
    int                                _dataCount;
    float                              _hiArray[10000];
    float                              _longArray[10000];
    float                              _latArray[10000];
    float                              _heArray[10000];
    float                              _pArray[10000];
    float                              _rArray[10000];
    float                              _vArray[10000];
    double                             _speed = 1.0;  // 1.0 = normal, 2.0 = 2x faster, 0.5 = half speed
    double                             _pitch = -30.0;  // default pitch
    double                             _range = 30000.0; // Default zoom range
    double                             _heading = 0.0;  // default heading angle in degrees
    bool _isEnable;
    double _simTime;
    double _lastUpdateTime;
    bool _haveLastPosition;
    MapNode* _mapNode;
    osg::PositionAttitudeTransform* modelTransform;
    osg::Group*                        _annoGroup;
    osg::Group*                         _simGroup = nullptr;
    QString                         _name;

signals:
    void speedUpdated(int flight_id, double speed_mps);
};

struct PlaneTail :public osg::Group
{
public:
    META_Node(osg, PlaneTail);
    PlaneTail();
    PlaneTail(const PlaneTail& rhs, const osg::CopyOp& copyop = osg::CopyOp::SHALLOW_COPY);  // Copy constructor
    PlaneTail(MapNode* mapnode, osg::Group* annoGroup);
    void addTailPoint(osg::Vec3d pos);
    void createLine(osg::Vec3d startline, osg::Vec3d endline);
    void cleanup();
    ~PlaneTail();
    osg::ref_ptr<osg::Geometry> getLineGeo()
    { return linesGeom; }
    osg::ref_ptr<osg::Vec3dArray> getVertices()
    { return vertices; }
    MapNode*                           _mapnode;
    osg::Geode*                        _geode;
    osg::Group*                        _annoGroup;
    osg::ref_ptr<osg::Geometry>         linesGeom;
    osg::ref_ptr<osg::Vec3dArray>       vertices;
    osg::Group*                         _parentGroup = nullptr;
    std::vector<PlaneTailPoint*>        tailPoints;
    std::vector<osg::ref_ptr<PlaneTailPoint>> _tailPoints;  //TO TRACK THE CREATED POINTS::
};

struct PlaneTailPoint : public GeoPositionNode
{
    std::vector<FeatureNode*>           createdLines;
    PlaneTailPoint(MapNode* mapnode, osg::Vec3d pos);
    void createLineToEarth(osg::Vec3d pos, osg::Group *annoGroup);
    Style createStyle();
    void traverse(osg::NodeVisitor& nv) override;
    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
    void setHover(bool hovered);
    void updateColor();
    void cleanup(osg::Group* parent);
    ~PlaneTailPoint();
    bool                               _hovered;
    double                             _height;
    FeatureNode*                       _lineToEarth;
    MapNode*                           _mapnode;
    osg::ShapeDrawable*                _shapeDrawable;
    Feature*                            pathFeature = nullptr;
    Geometry*                           path = nullptr;

};

#endif // SIMULATOR_H
