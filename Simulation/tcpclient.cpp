#include "TcpClient.h"
#include "qjsonarray.h"
#include <QDebug>

#include <QGroupBox>
#include <QFormLayout>

UdpClient::UdpClient(osgViewer::Viewer* viewer, osgQOpenGLWidget* osgWidget, osgEarth::EarthManipulator* manip, osgEarth::MapNode* mapNode, osg::Group* annoGroup, QWidget* parent)
    : QDialog(parent), _viewer(viewer), _osgWidget(osgWidget), _manip(manip), _mapNode(mapNode), _annoGroup(annoGroup)
{
    setWindowTitle("Client");
    resize(500, 400);






    connect(this, &UdpClient::messageReceived,
                this, &UdpClient::handleReceivedMessage);

    // socket = new QTcpSocket(this);
    // udpSocket = new QUdpSocket(this);
    //UI - PART
    QTabWidget* tabWidget = new QTabWidget(this);
    QWidget* udpTab = new QWidget();
    QVBoxLayout* udpLayout = new QVBoxLayout(udpTab);

    udpIpInput = new QLineEdit("127.0.0.1", this);  //("0.0.0.0", this);    //LISTEN TO ALL INPUTS
    udpPortInput = new QLineEdit("12345", this);
    udpConnectButton = new QPushButton("Start Listening", this);
    udpOutputBox = new QTextEdit(this);
    udpOutputBox->setReadOnly(true);
    simulateAllButtonUDP = new QPushButton("SIMULATE ALL TRACKS", this);
    replayButton = new QPushButton("REPLAY", this);
    disconnectButtonUDP = new QPushButton("STOP SIMULATION", this);

    udpLayout->addWidget(new QLabel("UDP IP Address:"));
    udpLayout->addWidget(udpIpInput);
    udpLayout->addWidget(new QLabel("UDP Port:"));
    udpLayout->addWidget(udpPortInput);
    udpLayout->addWidget(udpConnectButton);
    udpLayout->addWidget(udpOutputBox);
    udpLayout->addWidget(simulateAllButtonUDP);
    udpLayout->addWidget(replayButton);
    udpLayout->addWidget(disconnectButtonUDP);
    udpTab->setLayout(udpLayout);
    tabWidget->addTab(udpTab, "UDP Listener");

    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    mainLayout->addWidget(tabWidget);
    setLayout(mainLayout);

    // connect(socket, &QTcpSocket::readyRead, this, &UdpClient::readData);
    // connect(socket, &QTcpSocket::errorOccurred, this, &UdpClient::onError);
    connect(udpConnectButton, &QPushButton::clicked, this,[=](){
        startUdpListener(udpIpInput->text(), udpPortInput->text());
    });
    // connect(udpSocket, &QUdpSocket::readyRead, this, &UdpClient::readUdpData);
    connect(disconnectButtonUDP, &QPushButton::clicked, this, &UdpClient::stopSimulation);

    connect(simulateAllButtonUDP, &QPushButton::clicked, this, [this]()
    {
        simulateAll = true;
        canvas = ControlCanvas::get(_viewer);
        _viewer->getCamera()->addChild(canvas);
        vboxLeft = new VBox();
        vboxLeft->setMargin(10);
        vboxLeft->setBackColor(osgEarth::Color(0.1f, 0.1f, 0.1f, 0.2f));  // R, G, B, A

        ButtonControl* btnTopView = new ButtonControl("Top View");
        ButtonControl* btnFrontView = new ButtonControl("Front View");
        ButtonControl* btnRearView = new ButtonControl("Rear View");
        ButtonControl* btnSideView = new ButtonControl("Side View");
        ButtonControl* btnStopFollow = new ButtonControl("Stop Follow");
        ButtonControl* btnSpeedUp = new ButtonControl("Speed[+1.2x]");
        ButtonControl* btnSpeedDown = new ButtonControl("Speed[-1.2x]");
        ButtonControl* btnPlayPause = new ButtonControl("Pause");

        ButtonControl* btnPitchUp     = new ButtonControl("Pitch +");
        ButtonControl* btnPitchDown   = new ButtonControl("Pitch -");
        ButtonControl* btnHeadingLeft = new ButtonControl("Heading -");
        ButtonControl* btnHeadingRight= new ButtonControl("Heading +");
        ButtonControl* btnZoomIn      = new ButtonControl("Zoom In");
        ButtonControl* btnZoomOut     = new ButtonControl("Zoom Out");

        btnTopView->setWidth(110);
        btnTopView->setHeight(25);
        btnTopView->setFontSize(15.0f);

        btnFrontView->setWidth(110);
        btnFrontView->setHeight(25);
        btnFrontView->setFontSize(15.0f);

        btnRearView->setWidth(110);
        btnRearView->setHeight(25);
        btnRearView->setFontSize(15.0f);

        btnSideView->setWidth(110);
        btnSideView->setHeight(25);
        btnSideView->setFontSize(15.0f);

        btnStopFollow->setWidth(110);
        btnStopFollow->setHeight(25);
        btnStopFollow->setFontSize(15.0f);

        btnSpeedUp->setWidth(110);
        btnSpeedUp->setHeight(25);
        btnSpeedUp->setFontSize(15.0f);

        btnSpeedDown->setWidth(110);
        btnSpeedDown->setHeight(25);
        btnSpeedDown->setFontSize(15.0f);

        btnPitchUp->setWidth(110);
        btnPitchUp->setHeight(25);
        btnPitchUp->setFontSize(15.0f);

        btnPlayPause->setWidth(110);
        btnPlayPause->setHeight(25);
        btnPlayPause->setFontSize(15.0f);

        btnPitchDown->setWidth(110);
        btnPitchDown->setHeight(25);
        btnPitchDown->setFontSize(15.0f);

        btnHeadingLeft->setWidth(110);
        btnHeadingLeft->setHeight(25);
        btnHeadingLeft->setFontSize(15.0f);

        btnHeadingRight->setWidth(110);
        btnHeadingRight->setHeight(25);
        btnHeadingRight->setFontSize(15.0f);

        btnZoomIn->setWidth(110);
        btnZoomIn->setHeight(25);
        btnZoomIn->setFontSize(15.0f);

        btnZoomOut->setWidth(110);
        btnZoomOut->setHeight(25);
        btnZoomOut->setFontSize(15.0f);

        vboxLeft->addControl(btnTopView);
        vboxLeft->addControl(btnFrontView);
        vboxLeft->addControl(btnRearView);
        vboxLeft->addControl(btnSideView);
        vboxLeft->addControl(btnStopFollow);
        vboxLeft->addControl(btnSpeedUp);
        vboxLeft->addControl(btnSpeedDown);
        vboxLeft->addControl(btnPlayPause);
        vboxLeft->addControl(btnPitchUp);
        vboxLeft->addControl(btnPitchDown);
        vboxLeft->addControl(btnHeadingLeft);
        vboxLeft->addControl(btnHeadingRight);
        vboxLeft->addControl(btnZoomIn);
        vboxLeft->addControl(btnZoomOut);

        canvas->addControl(vboxLeft);

        btnTopView->addEventHandler(new MyButtonHandler([]() {
            auto sim = Simulator::instance();
            if (sim) {
                sim->setCameraFocus(true, 1, 0.0, -90.0, 15000.0, Simulator::VIEW_TOP);
            }
        }));

        btnFrontView->addEventHandler(new MyButtonHandler([]() {
            auto sim = Simulator::instance();
            if (sim) {
                sim->setCameraFocus(true, 1, 90.0, -10.0, 3500.0, Simulator::VIEW_FRONT);
            }
        }));

        btnRearView->addEventHandler(new MyButtonHandler([]() {
            auto sim = Simulator::instance();
            if (sim) {
                sim->setCameraFocus(true, 1, -90.0, -10.0, 3500.0, Simulator::VIEW_REAR);
            }
        }));

        btnSideView->addEventHandler(new MyButtonHandler([]() {
            auto sim = Simulator::instance();
            if (sim) {
                sim->setCameraFocus(true, 1, 50.0, 10.0, 3500.0, Simulator::VIEW_SIDE);
            }
        }));

        btnStopFollow->addEventHandler(new MyButtonHandler([]() {
            auto sim = Simulator::instance();
            if (sim) {
                sim->setCameraFocus(false, -1, 0.0, 0.0, 0.0, Simulator::VIEW_NONE);
            }
        }));

        btnSpeedUp->addEventHandler(new MyButtonHandler([]() {
            auto sim = Simulator::instance();
            if (sim) {
                sim->adjustSpeed(0.01);
            }
        }));

        btnSpeedDown->addEventHandler(new MyButtonHandler([]() {
            auto sim = Simulator::instance();
            if (sim) {
                sim->adjustSpeed(-0.01);
            }
        }));

        btnPlayPause->addEventHandler(new MyButtonHandler([btnPlayPause]() mutable {
            auto sim = Simulator::instance();
            if (sim) {
                bool current = sim->isPaused();
                sim->setPaused(!current);

                btnPlayPause->setText(current ? "Pause" : "Play");
            }
        }));

        btnPitchUp->addEventHandler(new MyButtonHandler([]() {
            auto sim = Simulator::instance();
            if (sim && sim->isFollowing()) {
                sim->adjustPitch(+5.0);
            }
        }));

        btnPitchDown->addEventHandler(new MyButtonHandler([]() {
            auto sim = Simulator::instance();
            if (sim && sim->isFollowing()) {
                sim->adjustPitch(-5.0);
            }
        }));

        btnHeadingLeft->addEventHandler(new MyButtonHandler([]() {
            auto sim = Simulator::instance();
            if (sim && sim->isFollowing()) {
                sim->adjustHeading(-10.0);
            }
        }));

        btnHeadingRight->addEventHandler(new MyButtonHandler([]() {
            auto sim = Simulator::instance();
            if (sim && sim->isFollowing()) {
                sim->adjustHeading(+10.0);
            }
        }));

        btnZoomIn->addEventHandler(new MyButtonHandler([]() {
            auto sim = Simulator::instance();
            if (sim && sim->isFollowing()) {
                sim->adjustRange(0.9);  // Zoom in = 0.9x
            }
        }));

        btnZoomOut->addEventHandler(new MyButtonHandler([]() {
            auto sim = Simulator::instance();
            if (sim && sim->isFollowing()) {
                sim->adjustRange(1.1);  // Zoom out = 1.1x
            }
        }));

        canvas = ControlCanvas::get(_viewer);
        _viewer->getCamera()->addChild(canvas);

        osgEarth::Util::Controls::VBox* infoPanel = new osgEarth::Util::Controls::VBox();
        infoPanel->setBackColor(osgEarth::Color(0, 0, 0, 0.5));
        infoPanel->setPadding(10);
        infoPanel->setMargin(5);
        infoPanel->setHorizAlign(osgEarth::Util::Controls::Control::ALIGN_LEFT);
        infoPanel->setVertAlign(osgEarth::Util::Controls::Control::ALIGN_BOTTOM);
        canvas->addControl(infoPanel);  // Add panel to screen

        _flightInfoPanel = infoPanel;
        simulateAll = true;
        addPlane();
        qDebug() << "[SIMULATION] Simulating all track types.";
    });

    connect(replayButton, &QPushButton::clicked, this, [this]() {
        QString filePath = QFileDialog::getOpenFileName(this, "Select JSON Replay File", ".", "*.json");
        if (!filePath.isEmpty()) {
            replayFromJson(filePath);
        }
    });
}

void UdpClient::_initUi()
{
    //UI - PART
    QTabWidget* tabWidget = new QTabWidget(this);

    // ==========================
    // UDP Listener Tab
    // ==========================
    QWidget* udpTab = new QWidget();
    QVBoxLayout* udpLayout = new QVBoxLayout(udpTab);

    // --- Connection Settings Group ---
    QGroupBox* connectionGroup = new QGroupBox("Connection Settings", this);
    QFormLayout* connectionForm = new QFormLayout();

    udpIpInput = new QLineEdit("127.0.0.1", this);
    udpPortInput = new QLineEdit("12345", this);
    udpConnectButton = new QPushButton("Start Listening", this);

    connectionForm->addRow("IP Address:", udpIpInput);
    connectionForm->addRow("Port:", udpPortInput);
    connectionForm->addRow(udpConnectButton);
    connectionGroup->setLayout(connectionForm);

    // --- Output Display Group ---
    QGroupBox* outputGroup = new QGroupBox("Received Data", this);
    QVBoxLayout* outputLayout = new QVBoxLayout();
    udpOutputBox = new QTextEdit(this);
    udpOutputBox->setReadOnly(true);
    outputLayout->addWidget(udpOutputBox);
    outputGroup->setLayout(outputLayout);

    // --- Control Buttons Group ---
    QGroupBox* controlGroup = new QGroupBox("Simulation Controls", this);
    QHBoxLayout* controlLayout = new QHBoxLayout();
    simulateAllButtonUDP = new QPushButton("Simulate All Tracks", this);
    replayButton = new QPushButton("Replay", this);
    disconnectButtonUDP = new QPushButton("Stop Simulation", this);

    controlLayout->addWidget(simulateAllButtonUDP);
    controlLayout->addWidget(replayButton);
    controlLayout->addWidget(disconnectButtonUDP);
    controlGroup->setLayout(controlLayout);

    // Add groups to UDP Layout
    udpLayout->addWidget(connectionGroup);
    udpLayout->addWidget(outputGroup);
    udpLayout->addWidget(controlGroup);

    udpTab->setLayout(udpLayout);
    tabWidget->addTab(udpTab, "UDP Listener");

    // ==========================
    // Main Layout
    // ==========================
    QVBoxLayout* mainLayout = new QVBoxLayout(this);
    mainLayout->addWidget(tabWidget);
    setLayout(mainLayout);
}

void UdpClient::closeEvent(QCloseEvent* event)
{
    if(simulateAll == true)
        stopSimulation();
    QDialog::closeEvent(event);
    event->accept();
}

void UdpClient::addPlane()
{
    _osgWidget->setFocusPolicy(Qt::StrongFocus);
    _osgWidget->setFocus();
}

const QHash<int, QList<TrackingData>>& UdpClient::getData() const
{
    return dataMap;
}

void UdpClient::replayFromJson(const QString& filePath)
{
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qWarning() << "[REPLAY] Failed to open file:" << filePath;
        return;
    }

    QByteArray jsonData = file.readAll();
    QJsonDocument doc = QJsonDocument::fromJson(jsonData);
    if (!doc.isArray())
    {
        qWarning() << "[REPLAY] JSON is not an array.";
        return;
    }
    QJsonArray jsonArray = doc.array();
    replayLines.clear();
    for (const QJsonValue& value : jsonArray)
    {
        if (value.isObject())
        {
            QJsonDocument singleDoc(value.toObject());
            QString jsonString = singleDoc.toJson(QJsonDocument::Compact);
            replayLines.append(jsonString);
        }
    }
    currentReplayIndex = 0;
    if (!replayTimer)
    {
        replayTimer = new QTimer(this);
    }
    connect(replayTimer, &QTimer::timeout, this, [=]() mutable
    {
        if (currentReplayIndex < replayLines.size())
        {
            QString jsonLine = replayLines[currentReplayIndex++];
            processLineUDP(jsonLine);
        } else
        {
            replayTimer->stop();
            replayTimer->deleteLater();
            replayTimer = nullptr;
            qDebug() << "[REPLAY] Finished replaying" << replayLines.size() << "entries.";
        }
    });
    replayTimer->start(1000); // 1 second per data point
}

void UdpClient::stopSimulation()
{    
    isShuttingDown = true;
    simulateAll = false;
    onDisconnected();
    QList<int> keys = simulators.keys();
    for (int id : keys)
    {
        Simulator* sim = simulators.value(id, nullptr);
        if (!sim) continue;
        sim->setActive(false);
        try
        {
            _viewer->removeEventHandler(sim);
        } catch (...)
        {
            qCritical() << "[STOP SIMULATE] EXCEPTION removing event handler for ID:" << id;
        }
        simulators.remove(id);
        sim = nullptr;
    }
    registeredSimIds.clear();
    for (auto marker : flightStartMarkers)
    {
        if (_mapNode && marker.valid())
        {
            _mapNode->removeChild(marker);
        }
    }
    flightStartMarkers.clear();
    if (_flightInfoPanel)
    {
        _flightInfoPanel->clearControls();
    }
    flightHUDMap.clear();
    if (vboxLeft && canvas)
    {
        canvas->removeControl(vboxLeft);
        vboxLeft = nullptr;
    }
    udpOutputBox->append("Disconnected and simulators cleaned up.");
    for (auto marker : flightEndMarkers)
    {
        if (_mapNode && marker.valid())
        {
            _mapNode->removeChild(marker);
        }
    }
    flightEndMarkers.clear();
    endMarkersPlaced.clear();
}

void UdpClient::connectToServer()
{
    QString ip = ipInput->text();
    quint16 port = 12345;
    socket->connectToHost(ip, port);
    if (!socket->waitForConnected(3000))
    {
        udpOutputBox->append("Connection failed.");
        return;
    }
    udpOutputBox->append("Connected to server.");
}

void UdpClient::readData()
{
    QByteArray data = socket->readAll();
    QString message = QString::fromUtf8(data);
    udpOutputBox->append("[Server] " + message);

    buffer += data;
    while (buffer.contains('\n'))
    {
        int index = buffer.indexOf('\n');
        QString line = buffer.left(index).trimmed();
        buffer.remove(0, index + 1);
        if (!line.isEmpty()) {
            processLine(line);
        }
    }
}

void UdpClient::onError(QAbstractSocket::SocketError socketError)
{
    Q_UNUSED(socketError)
    udpOutputBox->append("Socket error: " + socket->errorString());

    for (auto id : dataMap.keys())
    {
        qDebug() << "ID:" << id;
        for (const TrackingData &data : dataMap[id])
        {
            qDebug() << " " << data.tracktype << data.trackname
                     << data.latitude << data.longitude
                     << data.altitude << data.timestamp;
        }
    }
}

void UdpClient::processLine(const QString &line)
{
    if (isShuttingDown)
    {
        qDebug() << "[PROCESS LINE] Skipping because shutting down.";
        return;
    }
    QStringList parts = line.split(',');
    if (parts.size() != 7)
    {
        qDebug() << "[IGNORED] Malformed line:" << line;
        return;
    }
    TrackingData data;
    data.id = parts[0].toInt();
    data.tracktype = parts[1];
    data.trackname = parts[2];
    data.latitude = parts[3].toDouble();
    data.longitude = parts[4].toDouble();
    data.altitude = parts[5].toDouble();
    data.timestamp = parts[6];
    dataMap[data.id].append(data);
    if (simulateAll)
    {
        if (!simulators.contains(data.id))
        {    //IF NOT, CREATE to register event handlers
            try
            {
                Simulator* newSim = new Simulator(_manip, _mapNode, _annoGroup, data.tracktype);
                simulators[data.id] = newSim;
                if (!_viewer)
                {
                    _viewer = _osgWidget->getOsgViewer();
                }
                if (_viewer && !registeredSimIds.contains(data.id)) {
                    _viewer->addEventHandler(newSim);
                    _osgWidget->setFocusPolicy(Qt::StrongFocus);
                    _osgWidget->setFocus();
                    registeredSimIds.insert(data.id);
                }
            } catch (...)
            {
                qWarning() << "[ERROR] Failed to create Simulator for ID:" << data.id;
                return;
            }
        }
        Simulator* sim = simulators.value(data.id, nullptr);
        if (sim)
        {
            sim->updatePositionFromData(data.id, data.tracktype, data.latitude, data.longitude, data.altitude);
        } else
        {
            qWarning() << "[ERROR] Simulator is null for ID:" << data.id;
        }
    }
}

void UdpClient::processLineUDP(const QString &line)
{
    QJsonParseError parseError;
    QJsonDocument doc = QJsonDocument::fromJson(line.toUtf8(), &parseError);
    if (parseError.error != QJsonParseError::NoError || !doc.isObject())
    {
        return;
    }
    QJsonObject obj = doc.object();
    QStringList requiredFields =
    {
        "flight_id", "latitude", "longitude", "altitude",
        "timestamp", "speed_mps", "heading_deg", "reference_altitude",
        "roll", "pitch", "yaw"
    };
    for (const QString &key : requiredFields)
    {
        if (!obj.contains(key))
        {
            return;
        }
    }
    TrackingDataUDP data;
    data.flight_id = obj["flight_id"].toInt();
    QString tracktype = "Missile";  // Assign default if "tracktype" not in JSON
    data.latitude = obj["latitude"].toDouble();
    data.longitude = obj["longitude"].toDouble();
    data.altitude = obj["altitude"].toDouble();
    data.timestamp = obj["timestamp"].toDouble();
    data.speed_mps = obj["speed_mps"].toDouble();
    data.heading_deg = obj["heading_deg"].toDouble();
    data.reference_altitude = obj["reference_altitude"].toDouble();
    data.roll = obj["roll"].toDouble();
    data.pitch = obj["pitch"].toDouble();
    data.yaw = obj["yaw"].toDouble();


    udpDataMap[data.flight_id].append(data);
    lastPositionMap[data.flight_id] = data;  // Track the latest position

    // for (auto it = udpDataMap.begin(); it != udpDataMap.end(); ++it) {
    //     int id = it.key();
    //     const QList<TrackingDataUDP> &entries = it.value();
    // }

    // if (!firstPositionMap.contains(data.flight_id)) {
    //     firstPositionMap[data.flight_id] = data;
    //     QString iconPath = QCoreApplication::applicationDirPath() + "/MapData/icon/location-pin.png";
    //     flightStartMarkers[data.flight_id] = addMarker(_mapNode, data.latitude, data.longitude, iconPath.toStdString(), "START");
    // }

    // QDateTime now = QDateTime::currentDateTimeUtc();
    // qint64 currentTimeSec = now.toSecsSinceEpoch();

    // if (!simulationStartWallTime.contains(data.flight_id)) {
    //     simulationStartWallTime[data.flight_id] = currentTimeSec;
    // }

    // qint64 elapsedSec = currentTimeSec - simulationStartWallTime[data.flight_id];

    // if (elapsedSec >= 120 && !endMarkersPlaced.contains(data.flight_id)) {
    //     QString iconPath = QCoreApplication::applicationDirPath() + "/MapData/icon/location-pin.png";
    //     flightEndMarkers[data.flight_id] = addMarker(_mapNode, data.latitude, data.longitude, iconPath.toStdString(), "END");

    //     endMarkersPlaced.insert(data.flight_id);
    // }

    if (simulateAll)
    {
        if (!simulators.contains(data.flight_id))
        {
            try {
                Simulator* newSim = new Simulator(_manip, _mapNode, _annoGroup, tracktype);
                simulators[data.flight_id] = newSim;
                qDebug() << "[SIM] Created Simulator for ID:" << data.flight_id;

                connect(newSim, &Simulator::speedUpdated, this, &UdpClient::onSpeedUpdated);
                if (!_viewer) {
                    _viewer = _osgWidget->getOsgViewer();
                }
                if (_viewer && !registeredSimIds.contains(data.flight_id)) {
                    _viewer->addEventHandler(newSim);
                    registeredSimIds.insert(data.flight_id);
                }

            } catch (...)
            {
                qWarning() << "[ERROR] Failed to create Simulator for ID:" << data.flight_id;
                return;
            }
        }

        Simulator* sim = simulators.value(data.flight_id, nullptr);
        if (sim) {
            sim->updatePositionFromDataUDP(
                data.flight_id,
                tracktype,
                data.latitude,
                data.longitude,
                data.altitude,
                data.speed_mps,
                data.roll,
                data.pitch,
                data.yaw
                );

        } else
        {
            qWarning() << "[ERROR] Simulator is null for ID:" << data.flight_id;
        }
    }
}

void UdpClient::onSpeedUpdated(int flight_id, double speed_mps)
{
    if (lastPositionMap.contains(flight_id))
    {
        lastPositionMap[flight_id].speed_mps = speed_mps;
        const auto& data = lastPositionMap[flight_id];
        QString labelText = QString(
                                "ID: %1\nLat: %2\nLon: %3\nAlt: %4 m\nPitch: %5\nHeading: %6\nYaw: %7\nSpeed: %8 m/s")
                                .arg(data.flight_id)
                                .arg(data.latitude, 0, 'f', 6)
                                .arg(data.longitude, 0, 'f', 6)
                                .arg(data.altitude, 0, 'f', 1)
                                .arg(data.pitch, 0, 'f', 1)
                                .arg(data.heading_deg, 0, 'f', 1)
                                .arg(data.yaw, 0, 'f', 1)
                                .arg(speed_mps, 0, 'f', 1);

        if (!flightHUDMap.contains(flight_id)) {
            osgEarth::Util::Controls::LabelControl* label =
                new osgEarth::Util::Controls::LabelControl(labelText.toStdString(), 12.0f);
            label->setFontSize(20.0f);
            label->setForeColor(osg::Vec4f(1.0f, 1.0f, 1.0f, 1.0f)); // White
            label->setBackColor(osg::Vec4f(0.0f, 0.0f, 0.0f, 0.4f));   // semi-transparent black
            label->setBorderColor(osg::Vec4f(1.0f, 1.0f, 1.0f, 0.8f)); // white border
            label->setBorderWidth(1.0f);
            label->setOpacity(1.0f);  // fully opaque

            _flightInfoPanel->addControl(label);
            flightHUDMap[flight_id] = label;
        } else {
            flightHUDMap[flight_id]->setText(labelText.toStdString());
        }
    }
}

osg::ref_ptr<AnnotationNode> UdpClient::addMarker(MapNode* mapNode, double lat, double lon, const std::string& iconPath, const std::string& labelText)
{
    if (!mapNode)
    {
        qWarning() << "[ERROR] MapNode is null, cannot add marker.";
        return nullptr;
    }
    Style style;
    IconSymbol* icon = style.getOrCreateSymbol<IconSymbol>();
    icon->url()->setLiteral(iconPath);
    icon->scale() = 0.1;
    icon->alignment() = IconSymbol::ALIGN_CENTER_CENTER;  // Center the icon on the location
    GeoPoint point(mapNode->getMapSRS(), lon, lat, 0.0, ALTMODE_ABSOLUTE);
    osg::ref_ptr<PlaceNode> marker = new PlaceNode(point, labelText, style);
    mapNode->addChild(marker);
    _startMarker = marker;
    return marker;
}

void UdpClient::readUdpData()
{
    while (udpSocket->hasPendingDatagrams())
    {
        QByteArray datagram;
        datagram.resize(int(udpSocket->pendingDatagramSize()));
        QHostAddress sender;
        quint16 senderPort;
        udpSocket->readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);
        QString message = QString::fromUtf8(datagram);
        udpOutputBox->append("[UDP] " + message);
        processLineUDP(message);
    }
}

#include <QNetworkDatagram>

void UdpClient::handleReceivedMessage(const QString &message, const QString &senderAddress, quint16 senderPort)
{
    qDebug() << "Received from" << senderAddress << ":" << senderPort;
    qDebug() << "Raw message:" << message;

    QJsonDocument doc = QJsonDocument::fromJson(message.toUtf8());
    if (doc.isNull()) {
        qDebug() << "Failed to parse JSON:" << message;
        return;
    }

    if (!doc.isObject()) {
        qDebug() << "JSON is not an object";
        return;
    }

    QJsonObject jsonObj = doc.object();
    udpJsonMessages.append(jsonObj);


    // for (auto it = jsonObj.constBegin(); it != jsonObj.constEnd(); ++it) {
    //     QString key = it.key();
    //     QJsonValue value = it.value();

    //     QString valueStr;
    //     if (value.isDouble()) {
    //         valueStr = QString::number(value.toDouble());
    //     } else if (value.isString()) {
    //         valueStr = value.toString();
    //     } else if (value.isBool()) {
    //         valueStr = value.toBool() ? "true" : "false";
    //     } else if (value.isNull()) {
    //         valueStr = "null";
    //     } else {
    //         valueStr = "[complex value]";
    //     }

    //     qDebug().noquote() << QString("%1: %2").arg(key, -20).arg(valueStr);
    // }

    // qDebug() << "---------------------";

    // // Example of accessing specific values
    // if (jsonObj.contains("flight_id")) {
    //     int flightId = jsonObj["flight_id"].toInt();
    //     qDebug() << "Flight ID (extracted):" << flightId;
    // }

    // if (jsonObj.contains("timestamp")) {
    //     double timestamp = jsonObj["timestamp"].toDouble();
    //     QDateTime dt = QDateTime::fromSecsSinceEpoch((qint64)timestamp);
    //     qDebug() << "Timestamp (converted):" << dt.toString(Qt::ISODate);
    // }
}


bool UdpClient::startUdpListener(const QString UDP_IP_ADDR , const QString UDP_PORT_NUMBER)
{

    if (udpSocket) {
        udpSocket->close();
        udpSocket->deleteLater();
    }
    udpSocket = new QUdpSocket(this);
    QHostAddress address(UDP_IP_ADDR);
    if (!udpSocket->bind(address, UDP_PORT_NUMBER.toUShort()))
    {
        return false;
    }

    connect(udpSocket, &QUdpSocket::readyRead, this, &UdpClient::readUdpData);
       // udpOutputBox->append(QString("Listening on %1:%2 (UDP)").arg(ip).arg(port));

    // Connect the readyRead signal to a slot that will handle incoming data
   // connect(udpSocket, &QUdpSocket::readyRead, this, [=]() {
   //     while (udpSocket->hasPendingDatagrams()) {
   //         // Read the datagram
   //         QNetworkDatagram datagram = udpSocket->receiveDatagram();
   //         if (!datagram.isValid()) {
   //             continue;
   //         }

   //         // Get the data and convert to QString
   //         QByteArray data = datagram.data();
   //         QString message = QString::fromUtf8(data);

   //         // Get sender address and port
   //         QString senderAddress = datagram.senderAddress().toString();
   //         quint16 senderPort = datagram.senderPort();

   //         // You can emit a signal here with the received data if needed
   //         emit messageReceived(message, senderAddress, senderPort);
   //     }
   // });

   return true;
}


void UdpClient::onDisconnected()
{
    if (socket && socket->isOpen())
    {
        socket->disconnectFromHost();
        socket->close();
        qDebug() << "[TCP] Disconnected from server.";
    }
}

UdpClient::~UdpClient()
{
    isShuttingDown = true;
    if (socket)
    {
        socket->disconnectFromHost();
        socket->close();
    }
    if (udpSocket)
    {
        udpSocket->close();
        udpSocket->deleteLater();
        udpSocket = nullptr;
    }
    for (Simulator* sim : simulators.values())
    {
        delete sim;
    }
    simulators.clear();
    registeredSimIds.clear();
}
