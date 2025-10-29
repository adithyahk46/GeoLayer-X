#ifndef TCPCLIENT_H
#define TCPCLIENT_H

#include <QDialog>
#include <QTcpSocket>
#include <QPushButton>
#include <QVBoxLayout>
#include <QTextEdit>
#include <QLineEdit>
#include <QHostAddress>
#include <QString>
#include <QCheckBox>
#include <QRadioButton>
#include <QButtonGroup>
#include <QTabWidget>
#include <QLabel>
#include <QUdpSocket>
#include <QOpenGLFunctions>

// OSG and osgEarth
#include "osgQOpenGL/osgQOpenGLWidget"
#include <osgViewer/Viewer>
#include <osg/Group>
#include <osgEarth/MapNode>
#include <osgEarth/EarthManipulator>
#include <QTimer>
#include <nlohmann/json.hpp>
#include <string>
#include <QJsonDocument>
#include <QJsonObject>
#include <QFileDialog>
#include <osgEarth/Controls>
#include <osgEarth/AnnotationNode>
#include <osgEarth/Style>
#include <osgEarth/IconSymbol>
#include <osgEarth/PlaceNode>
#include <QCloseEvent>

#include "Simulation/simulator.h"

using namespace osgEarth::Controls;

struct TrackingData {
    int id;
    QString tracktype;
    QString trackname;
    double latitude;
    double longitude;
    double altitude;
    QString timestamp;
    double lastUpdateTimeSec = 0.0;
    double nextUpdateTimeSec = 0.0;
    double speed_mps = 0.0;  // From server
};

struct TrackingDataUDP {
    int flight_id;
    double timestamp;
    double latitude;
    double longitude;
    double altitude;
    double speed_mps;
    double heading_deg;
    double reference_altitude;
    double roll;
    double pitch;
    double yaw;
};

class UdpClient : public QDialog
{
    Q_OBJECT

public:
    explicit UdpClient(osgViewer::Viewer* viewer, osgQOpenGLWidget* osgWidget, osgEarth::EarthManipulator* manip, osgEarth::MapNode* mapNode, osg::Group* annoGroup, QWidget* parent = nullptr);
    const QHash<int, QList<TrackingData>>& getData() const;
    QHash<int, QList<TrackingData>> dataMap;
    QHash<int, QList<TrackingDataUDP>> udpDataMap;
    QString buffer;
    QMap<int, Simulator*> simulators;
    Simulator* _simulator;
    osg::ref_ptr<AnnotationNode> addMarker(MapNode* mapNode, double lat, double lon, const std::string& iconPath, const std::string& labelText);
    QMap<int, TrackingDataUDP> firstPositionMap;
    QCheckBox* simulationCheckBox;
    void addPlane();
    void replayFromJson(const QString& filePath);
    osgViewer::Viewer* _viewer = nullptr;
    osgQOpenGLWidget* _osgWidget = nullptr;
    osgEarth::EarthManipulator* _manip = nullptr;
    osgEarth::MapNode* _mapNode = nullptr;
    osg::Group* _annoGroup;
    void stopSimulation();
    void onDisconnected();
    ~UdpClient();
    QRadioButton *cargoRadio, *navalRadio, *jetRadio, *droneRadio, *tankRadio;
    QPushButton* simulateAllButton;
    QPushButton* simulateAllButtonUDP;
    QPushButton* replayButton;
    bool simulateAll = false;
    QSet<int> registeredSimIds;
    bool isShuttingDown = false;
    QLineEdit* udpIpInput;
    QLineEdit* udpPortInput;
    QPushButton* udpConnectButton;
    QTextEdit* udpOutputBox;
    QUdpSocket* udpSocket = nullptr;
    osgEarth::Util::Controls::VBox* _flightInfoPanel;
    QMap<int, osgEarth::Util::Controls::LabelControl*> flightHUDMap;
    VBox* vboxLeft = nullptr;
    ControlCanvas* canvas = nullptr;
    QVector<QString> replayLines;
    int currentReplayIndex = 0;
    QTimer* replayTimer = nullptr;
    osg::ref_ptr<osgEarth::AnnotationNode> _startMarker;
    QMap<int, osg::ref_ptr<AnnotationNode>> flightStartMarkers;
    bool isPlaying = true;
    QMap<int, TrackingDataUDP> lastPositionMap;
    QMap<int, osg::ref_ptr<AnnotationNode>> flightEndMarkers;
    QMap<int, double> simulationStartTimestamps; // Stores the first timestamp per flight
    QMap<int, double> simulationDurations;       // Stores simulation duration per flight
    QSet<int> endMarkersPlaced;                  // Tracks which flights have end markers
    QMap<int, qint64> simulationStartWallTime; // Stores wall time per flight ID


signals:
    void messageReceived(const QString &message, const QString &senderAddress, quint16 senderPort);

private slots:
    void connectToServer();
    void readData();
    void onError(QAbstractSocket::SocketError socketError);
    void readUdpData();

    void onSpeedUpdated(int flight_id, double speed_mps);

    void handleReceivedMessage(const QString &message, const QString &senderAddress, quint16 senderPort);


protected:
    void closeEvent(QCloseEvent* event) override;

public:
    bool startUdpListener(const QString UDP_IP_ADDR , const QString UDP_PORT_NUMBER);

private:
    void _initUi();


private:
    QTcpSocket* socket;
    QPushButton* connectButton;
    QPushButton* disconnectButton;
    QPushButton* disconnectButtonUDP;
    QLineEdit* ipInput;
    QTextEdit* outputBox;
    void processLine(const QString &line);
    void processLineUDP(const QString &line);


    QList<QJsonObject> udpJsonMessages;  // Store parsed messages


};

class MyButtonHandler : public osgEarth::Controls::ControlEventHandler
{
public:
    MyButtonHandler(std::function<void()> callback) : _callback(callback) {}

    void onClick(osgEarth::Controls::Control* control) override {
        if (_callback) _callback();
    }

private:
    std::function<void()> _callback;
};


#endif // TCPCLIENT_H
