#ifndef FLIGHTSIMULATOR_H
#define FLIGHTSIMULATOR_H

#include <QDialog>
#include <QJsonObject>
#include <QJsonDocument>
#include <QUdpSocket>

#include <osg/Group>
#include <osg/Node>
#include <osgDB/ReadFile>
#include <osgEarth/MapNode>
#include <osgEarth/Feature>
#include <osgEarth/FeatureNode>
#include <osgEarth/Geometry>
#include <osgEarth/Style>
#include <osgEarth/LineSymbol>
#include <osgEarth/AltitudeSymbol>
#include <osgEarth/ExtrusionSymbol>
#include <osgEarth/PolygonSymbol>
#include <osgEarth/PointSymbol>
#include <osgEarth/GeoData>
#include <osgEarth/GeoTransform>
#include <osgEarth/ModelNode>

#include <osgViewer/Viewer>
#include <osgEarth/EarthManipulator>


#include <osgEarth/Controls>
#include <osgQOpenGL/osgQOpenGLWidget>

using namespace osgEarth::Controls;

typedef struct CAMERAFocus{
    int flight_id;
    QString tracktype;
    QString trackname;
    QString timestamp;
    double speed_mps = 0.0;
    double lon   ;
    double lat;
    double alt;
    float heading;
    float pitch;
    float range;
    double roll;
    double pitchAngle;
    double yaw;

}CAMERAFocus;


namespace Ui {
class FlightSimulator;
}

class FlightSimulator : public QDialog
{
    Q_OBJECT

public:
    explicit FlightSimulator(osgViewer::Viewer* viewer,osgQOpenGLWidget* osgWidget, osgEarth::EarthManipulator* manip,osg::Group *annoGroup = NULL,osgEarth::MapNode* mapnode = NULL, QWidget *parent = nullptr);
    ~FlightSimulator();

    bool startUdpListener(const QString UDP_IP_ADDR, const QString UDP_PORT_NUMBER);

signals:
    void messageReceived(const QString &message, const QString &senderAddress, quint16 senderPort);

private slots:
    void handleReceivedMessage(const QString &message, const QString &senderAddress, quint16 senderPort);
    void on_pb_Replay_clicked();

private:
    Ui::FlightSimulator *ui;

    QUdpSocket* udpSocket =NULL;
    osgViewer::Viewer* _viewer = NULL;
    osgQOpenGLWidget* _osgWidget = NULL;
    osgEarth::EarthManipulator* _manip =NULL ;
    osg::Group *_annoGroup = NULL;
    osgEarth::MapNode* _mapNode = NULL;
    QList<QJsonObject> udpJsonMessages;

    osg::ref_ptr<osgEarth::LineString> _flightPath;
    osg::ref_ptr<osgEarth::FeatureNode> _flightPathNode;
    osgEarth::ModelNode* modelNode = nullptr;

    QWidget* controlWidget = nullptr;
    ControlCanvas* canvas = nullptr;
    osgEarth::Util::Controls::LabelControl* infolabel;
    osgEarth::Util::Controls::VBox* infoPanel;


    bool simulateTrake = false;
    bool followTrack = false;
    double timeInterval = 1000;
    CAMERAFocus cameraFocus;

    void _initConnection();
    void initFlightPath();
    void createControlGUI();
    void createFlightPath();
};

#endif // FLIGHTSIMULATOR_H
