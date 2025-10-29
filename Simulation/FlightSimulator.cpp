#include "FlightSimulator.h"
#include "ui_FlightSimulator.h"

#include <QAbstractSocket>
#include <QUdpSocket>
#include <QHostAddress>
#include <QNetworkDatagram>
#include <QByteArray>
#include <QDebug>
#include <qlogging.h>

#include <QWidget>
#include <QPushButton>
#include <QVBoxLayout>
#include <QGraphicsOpacityEffect>

#include <QFileDialog>
#include <QDir>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QMessageBox>

#include <QWidget>
#include <QPushButton>
#include <QVBoxLayout>
#include <QGraphicsOpacityEffect>

#include <osg/MatrixTransform>
#include <osg/Quat>

#include <QTimer>
#include <QEventLoop>


FlightSimulator::FlightSimulator(osgViewer::Viewer* viewer,osgQOpenGLWidget* osgWidget, osgEarth::EarthManipulator* manip,osg::Group *annoGroup, osgEarth::MapNode* mapnode,QWidget *parent) :
    QDialog(parent)
  ,_viewer(viewer)
  ,_osgWidget(osgWidget)
  ,_manip(manip)
  ,_annoGroup(annoGroup)
  ,_mapNode(mapnode)
   , ui(new Ui::FlightSimulator)
{
    ui->setupUi(this);

   _initConnection();
   initFlightPath();

   createControlGUI();
}

FlightSimulator::~FlightSimulator()
{
    delete ui;
    if (controlWidget) {
        if (_osgWidget && _osgWidget->layout()) {
            _osgWidget->layout()->removeWidget(controlWidget);
        }
        controlWidget->deleteLater();  // Prefer this in Qt to avoid immediate deletion issues
        controlWidget = nullptr;
    }

    if (udpSocket) {
        delete udpSocket;
        udpSocket = nullptr;
    }
    if (_annoGroup && _flightPathNode.valid()) {
        _annoGroup->removeChild(_flightPathNode.get());
    }
    if (_annoGroup && modelNode) {
        _annoGroup->removeChild(modelNode);
    }

    udpJsonMessages.clear();

    infoPanel->clearControls();
    canvas->removeControl(infoPanel);

    _viewer->getCamera()->removeChild(canvas);

}

void FlightSimulator::_initConnection()
{
    QObject::connect(ui->cb_simulateTrack, &QCheckBox::toggled, this, [=](bool checked){
        ui->pb_SimulateAllTracks->setEnabled(!checked);
        simulateTrake = checked;
    });
    ui->cb_simulateTrack->setChecked(true);

    QObject::connect(ui->pb_SimulateAllTracks, &QPushButton::clicked, this, [=](){

        simulateTrake? simulateTrake = false :simulateTrake = true;
        cameraFocus.pitch = -30.0;  // default pitch
        cameraFocus.range = 30000.0;

    });

    // QObject::connect(ui->pb_SimulateAllTracks)

    QObject::connect(ui->pb_DisconnectUDP, &QPushButton::clicked, this, [=]() {
        if (udpSocket) {
            udpSocket->close();        // Close the socket
            udpSocket->disconnect();   // Disconnect any connected signals
            udpSocket->deleteLater();  // Schedule deletion
            udpSocket = nullptr;       // Prevent dangling pointer
            ui->messageDispalay->append("----------------------------------------------"
                                        "UDP socket disconnected and cleaned up."
                                        "------------------------------------------------");
        }
    });

    QObject::connect(ui->pb_Stop, &QPushButton::clicked,this, [=](){
        simulateTrake = false;
        ui->cb_simulateTrack->setChecked(false);
    });


    QObject::connect(this, &FlightSimulator::messageReceived,
                         this, &FlightSimulator::handleReceivedMessage);

    QObject::connect(ui->pb_StartListening, &QPushButton::clicked, this ,[=](){
        ui->messageDispalay->clear();
        const QString UDP_IP_ADDR = ui->le_IpAddr->text().trimmed();
        const QString UDP_PORT_NUMBER = ui->le_portNum->text().trimmed();
        if(startUdpListener(UDP_IP_ADDR ,UDP_PORT_NUMBER)){
            ui->messageDispalay->append(QString("Listening on %1:%2 (UDP)").arg(UDP_IP_ADDR).arg(UDP_PORT_NUMBER));
        }
        else ui->messageDispalay->append(QString("failed to connect %1:%2 (UDP)").arg(UDP_IP_ADDR).arg(UDP_PORT_NUMBER));
    });

    QObject::connect(ui->pb_SaveData, &QPushButton::clicked, this , [=]() {
        // Open file dialog
        QString fileName = QFileDialog::getSaveFileName(
            nullptr,
            "Save UDP Data",
            QDir::homePath(),                   // Default path
            "JSON Files (*.json);;All Files (*)"
        );

        // If user canceled, do nothing
        if (fileName.isEmpty())
            return;

        // Convert QList<QJsonObject> to QJsonArray
        QJsonArray jsonArray;
        for (const QJsonObject& obj : udpJsonMessages) {
            jsonArray.append(obj);
        }

        // Write to file
        QFile file(fileName);
        if (!file.open(QIODevice::WriteOnly)) {
            QMessageBox::warning(nullptr, "Error", "Failed to open file for writing.");
            return;
        }

        QJsonDocument doc(jsonArray);
        file.write(doc.toJson(QJsonDocument::Indented));
        file.close();

        QMessageBox::information(nullptr, "Success", "Data saved successfully.");
    });

}

void FlightSimulator::initFlightPath()
{
        _flightPath = new osgEarth::LineString();

        osg::ref_ptr<osgEarth::Feature> feature = new osgEarth::Feature(_flightPath.get(), _mapNode->getMapSRS());
        feature->geoInterp() = osgEarth::GEOINTERP_DEFAULT;

        osgEarth::Style style;
        auto* line = style.getOrCreate<osgEarth::LineSymbol>();
        line->stroke()->color() = osgEarth::Color::Cyan;
        // line->stroke()->width() = 2.0f;
        line->stroke().mutable_value().width() = osgEarth::Distance(5.0,osgEarth::Units::PIXELS);


        auto* alt = style.getOrCreate<osgEarth::AltitudeSymbol>();
        alt->clamping() = osgEarth::AltitudeSymbol::CLAMP_NONE;
        alt->technique() = osgEarth::AltitudeSymbol::TECHNIQUE_GPU;

        _flightPathNode = new osgEarth::FeatureNode(feature.get(), style);
        _flightPathNode->setNodeMask(0xffffffff);

        _annoGroup->addChild(_flightPathNode.get());


        //////////////////////////////////////////////to add Flight Object///////////////////////////////

        QString flightModel = QCoreApplication::applicationDirPath() + "/Data/simulator/missile_2obj.obj";
        // a model node with auto scaling.
            {

                // style.getOrCreate<ModelSymbol>()->url()->setLiteral("C:/Users/pnmt1054/Adithya-working-directory/Data/3D-models/akm__free_lowpoly/scene.gltf");
                // osg::ref_ptr<osgEarth::ModelNode> modelNode = new osgEarth::ModelNode(_mapNode, style);
                // modelNode->setName("threed model");
                // //modelNode->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);
                // //mViewer->getCamera()->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF | osg::StateAttribute::OVERRIDE);

                // modelNode->setPosition(osgEarth::GeoPoint(_mapNode->getMapSRS(), 0, 0));
                // _annoGroup->addChild(modelNode);
                // osg::Vec3d Scale = modelNode->getScale();
                // Scale *= 2;
                // modelNode->setScale(Scale);
                // osg::Quat quata = modelNode->getLocalRotation();
                // // quata.z() = sin(z);
                // // quata.w() = cos(z);
                // // modelNode->setLocalRotation(quata);

            // osgEarth::Style style;
            // style.getOrCreate<osgEarth::ModelSymbol>()->autoScale() = true;
            // style.getOrCreate<osgEarth::ModelSymbol>()->url().mutable_value().setLiteral("C:/Users/pnmt1054/Adithya-working-directory/Data/3D-models/akm__free_lowpoly/scene.gltf");
            // style.getOrCreate<osgEarth::ModelSymbol>()->scale() = 2000.f;
            // style.getOrCreate<osgEarth::ModelSymbol>()->minAutoScale() = 2000.0f;
            // style.getOrCreate<osgEarth::ModelSymbol>()->maxAutoScale() = 2000.0f;
            // modelNode = new osgEarth::ModelNode(_mapNode, style);
            // // modelNode->setPosition(GeoPoint(geoSRS, -100, 52));
            // modelNode->setPosition(osgEarth::GeoPoint(_mapNode->getMapSRS(), 0, 0,0));
            // _annoGroup->addChild(modelNode);


            //////////////////////////////////////////////////////////////////////////////////////////////////
                // osgEarth::Style style;
                // style.getOrCreate<osgEarth::ModelSymbol>()->autoScale() = true;
                // style.getOrCreate<osgEarth::ModelSymbol>()->url().mutable_value().setLiteral("C:/Users/pnmt1054/Adithya-working-directory/QT_PROJECTS/GeoLayer-X/Data/simulator/missile_2obj.obj");
                // style.getOrCreate<osgEarth::ModelSymbol>()->scale() = 2000.f;
                // style.getOrCreate<osgEarth::ModelSymbol>()->minAutoScale() = 2000.0f;
                // style.getOrCreate<osgEarth::ModelSymbol>()->maxAutoScale() = 2000.0f;
                // modelNode = new osgEarth::ModelNode(_mapNode, style);
                // // modelNode->setPosition(GeoPoint(geoSRS, -100, 52));
                // modelNode->setPosition(osgEarth::GeoPoint(_mapNode->getMapSRS(), 0, 0,0));
                // _annoGroup->addChild(modelNode);
            //////////////////////////////////////////////////////////////////////////////////////////////


            // Create and configure the style
            osgEarth::Style style;
            auto* modelSymbol = style.getOrCreate<osgEarth::ModelSymbol>();

            // Configure the model properties
            modelSymbol->autoScale() = false;  // Disable auto-scaling for better control
            modelSymbol->url().mutable_value().setLiteral(
"C:/Users/pnmt1054/Downloads/Untitled.obj");
            modelSymbol->scale() = 10.0f;  // Adjust if too small or too big

            // Create the ModelNode with this style
            modelNode = new osgEarth::ModelNode(_mapNode, style);

            // Set the model position (example: Bangalore)
            modelNode->setPosition(osgEarth::GeoPoint(
                _mapNode->getMapSRS(),
                77.59,    // Longitude
                12.97,    // Latitude
                0.0,      // Elevation in meters
                osgEarth::ALTMODE_RELATIVE));  // Place relative to terrain

            // Disable face culling so reversed faces are visible
            // modelNode->getOrCreateStateSet()->setMode(GL_CULL_FACE, osg::StateAttribute::OFF);

            // Add the model to your annotation group
            _annoGroup->addChild(modelNode);

            // OPTIONAL: Move the camera to focus on this model
            osgEarth::Viewpoint vp("Model View", 77.59, 12.97, 10000.0, 0.0, -90.0, 1000.0);
            _manip->setViewpoint(vp);


            }
    // }



}

bool FlightSimulator::startUdpListener(const QString UDP_IP_ADDR , const QString UDP_PORT_NUMBER)
{

    // Clean up existing socket if any
   if (udpSocket) {
       udpSocket->close();
       udpSocket->deleteLater();
       udpSocket = nullptr;
   }

   // Create new socket
   udpSocket = new QUdpSocket(this);

   // Validate IP address
   QHostAddress address;
   if (!address.setAddress(UDP_IP_ADDR)) {
       qWarning() << "Invalid IP address:" << UDP_IP_ADDR;
       udpSocket->deleteLater();
       udpSocket = nullptr;
       return false;
   }

   // Validate port number
   bool ok;
   quint16 port = UDP_PORT_NUMBER.toUShort(&ok);
   if (!ok || port == 0) {
       qWarning() << "Invalid port number:" << UDP_PORT_NUMBER;
       udpSocket->deleteLater();
       udpSocket = nullptr;
       return false;
   }

   // Attempt to bind
   if (!udpSocket->bind(address, port)) {
       qWarning() << "Failed to bind UDP socket to" << UDP_IP_ADDR << ":" << port
                  << "Error:" << udpSocket->errorString();
       udpSocket->deleteLater();
       udpSocket = nullptr;
       return false;
   }

   // Connect error signals for runtime error handling
   connect(udpSocket, &QUdpSocket::errorOccurred, this, [this](QAbstractSocket::SocketError socketError) {
       qWarning() << "UDP Socket error occurred:" << socketError << udpSocket->errorString();
       // You might want to emit a signal here to notify other parts of your application
   });

    // connect(udpSocket, &QUdpSocket::readyRead, this, &UdpClient::readUdpData);

    // Connect the readyRead signal to a slot that will handle incoming data
   connect(udpSocket, &QUdpSocket::readyRead, this, [=]() {
       while (udpSocket->hasPendingDatagrams()) {
           // Read the datagram
           QNetworkDatagram datagram = udpSocket->receiveDatagram();
           if (!datagram.isValid()) {
               continue;
           }

           // Get the data and convert to QString
           QByteArray data = datagram.data();
           QString message = QString::fromUtf8(data);

           // Get sender address and port
           QString senderAddress = datagram.senderAddress().toString();
           quint16 senderPort = datagram.senderPort();

           // You can emit a signal here with the received data if needed
           emit messageReceived(message, senderAddress, senderPort);
       }
   });

   return true;
}

void FlightSimulator::handleReceivedMessage(const QString &message, const QString &senderAddress, quint16 senderPort)
{

    if(!message.isEmpty()){
        ui->messageDispalay->append(message);
        ui->messageDispalay->append(" ");
    }

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

    cameraFocus.flight_id = jsonObj.value("flight_id").toInt();
    cameraFocus.lon = jsonObj.value("longitude").toDouble(0.0);
    cameraFocus.lat = jsonObj.value("latitude").toDouble(0.0);
    cameraFocus.alt = jsonObj.value("altitude").toDouble(0.0);
    cameraFocus.roll = jsonObj.value("roll").toDouble(0.0);
    cameraFocus.pitchAngle = jsonObj.value("pitch").toDouble(0.0);
    cameraFocus.yaw = jsonObj.value("yaw").toDouble(0.0);
    cameraFocus.speed_mps = jsonObj.value("speed_mps").toDouble();

    createFlightPath();


}

void FlightSimulator::createFlightPath()
{



    if(simulateTrake){

        QString labelText = QString(
                                        "ID: %1\nLat: %2\nLon: %3\nAlt: %4 m\nPitch: %5\nHeading: %6\nYaw: %7\nSpeed: %8 m/s")
                                        .arg(cameraFocus.flight_id)
                                        .arg(cameraFocus.lat, 0, 'f', 6)
                                        .arg(cameraFocus.lon, 0, 'f', 6)
                                        .arg(cameraFocus.alt, 0, 'f', 1)
                                        .arg(cameraFocus.pitchAngle, 0, 'f', 1)
                                        .arg(cameraFocus.roll, 0, 'f', 1)
                                        .arg(cameraFocus.yaw, 0, 'f', 1)
                                        .arg(cameraFocus.speed_mps, 0, 'f', 1);

        infolabel->setText(labelText.toStdString());

        osgEarth::GeoPoint point(_mapNode->getMapSRS(), cameraFocus.lon, cameraFocus.lat, cameraFocus.alt, osgEarth::ALTMODE_RELATIVE);
        /// Append to path
        _flightPath->push_back(point.vec3d());

        // Update feature geometry and mark dirty
            auto feature = _flightPathNode->getFeature();
            if (feature)
            {
                feature->setGeometry(_flightPath);
                _flightPathNode->dirty();
            }

        modelNode->setPosition(point);


        if (followTrack)
        {
            osgEarth::GeoPoint geoPoint(_mapNode->getMapSRS(), cameraFocus.lon, cameraFocus.lat, cameraFocus.alt, osgEarth::ALTMODE_ABSOLUTE);
            osgEarth::Viewpoint vp;
            vp.setName("SimView");
            vp.setFocalPoint(geoPoint);  // Only setting the focal point — nothing else
            _manip->setViewpoint(vp);
        }

    }

}

void FlightSimulator::createControlGUI() {
    // Create parent widget for controls
    controlWidget = new QWidget(this);
    controlWidget->setAttribute(Qt::WA_DeleteOnClose);
    controlWidget->setAttribute(Qt::WA_TranslucentBackground);
    controlWidget->setStyleSheet("background-color: rgba(0, 0, 0, 50);");

    // Layout for buttons
    QVBoxLayout* vbox = new QVBoxLayout(controlWidget);
    vbox->setContentsMargins(10, 10, 10, 10);
    vbox->setSpacing(8);

    // Helper: Button creator with style
    auto createButton = [](const QString& text) -> QPushButton* {
        QPushButton* btn = new QPushButton(text);
        btn->setFixedSize(110, 30);
        btn->setStyleSheet(R"(
            QPushButton {
                background-color: rgba(0, 122, 204, 180);
                color: white;
                font-size: 14px;
                border: none;
                border-radius: 4px;
            }
            QPushButton:hover {
                background-color: rgba(0, 122, 204, 230);
            }
        )");

        // Optional: Apply opacity (e.g., 0.8)
        QGraphicsOpacityEffect* opacity = new QGraphicsOpacityEffect;
        opacity->setOpacity(0.8);
        btn->setGraphicsEffect(opacity);

        return btn;
    };

    // Action list
    struct ButtonAction {
        QString label;
        std::function<void()> handler;
    };

    std::vector<ButtonAction> actions = {
        {"Top View", [=]() {
             _manip->setViewpoint(osgEarth::Viewpoint("SimView", cameraFocus.lon, cameraFocus.lat, cameraFocus.alt, 0.0, -90.0, 15000.0));
         }},
        {"Front View", [=]() {
             _manip->setViewpoint(osgEarth::Viewpoint("SimView", cameraFocus.lon, cameraFocus.lat, cameraFocus.alt, 90, -10.0, 3500.0));
         }},
        {"Rear View", [=]() {
             _manip->setViewpoint(osgEarth::Viewpoint("SimView", cameraFocus.lon, cameraFocus.lat, cameraFocus.alt, -90, -10.0, 3500.0));
         }},
        {"Side View", [=]() {
             _manip->setViewpoint(osgEarth::Viewpoint("SimView", cameraFocus.lon, cameraFocus.lat, cameraFocus.alt, 50, 10.0, 3500.0));
         }},
        {"Follow Track", [=]() {
             followTrack = true;
            _manip->setViewpoint(osgEarth::Viewpoint("SimView", cameraFocus.lon, cameraFocus.lat, cameraFocus.alt, 0.0, -30.0, 30000.0));
         }},
        {"Stop Follow", [=]() {
            followTrack = false;
        }},
        {"Speed[+1.2x]", [=]() {
            timeInterval *= 0.8;
        }},
        {"Speed[-1.2x]", [=]() {
            timeInterval *= 1.2;
        }},
        {"Pitch +", [=]() {
             _manip->rotate(0.0, +0.1);  // Adjust value as needed (0.1 radians ~ 5.7 degrees)
         }},
        {"Pitch -", [=]() {
             _manip->rotate(0.0, -0.1);
         }},
        {"Heading -", [=]() {
             _manip->rotate(-0.1, 0.0);
         }},
        {"Heading +", [=]() {
             _manip->rotate(+0.1, 0.0);
         }},
        {"Zoom In", [=]() {
            _manip->setDistance( _manip->getDistance() * 0.8);
         }},
        {"Zoom Out", [=]() {
             _manip->setDistance( _manip->getDistance() * 1.2);
         }},
    };

    // Add each button and connect it
    for (const auto& action : actions) {
        QPushButton* btn = createButton(action.label);
        QObject::connect(btn, &QPushButton::clicked, action.handler);
        vbox->addWidget(btn);
    }

      _osgWidget->layout()->addWidget(controlWidget);

    /////////////////////////////////////////////////adding Info Panel on left-bottom side /////////////////////////////////////

       canvas = ControlCanvas::get(_viewer);
      _viewer->getCamera()->addChild(canvas);

      infoPanel = new osgEarth::Util::Controls::VBox();
      infoPanel->setBackColor(osgEarth::Color(0, 0, 0, 0.5));
      infoPanel->setPadding(10);
      infoPanel->setMargin(5);
      infoPanel->setHorizAlign(osgEarth::Util::Controls::Control::ALIGN_LEFT);
      infoPanel->setVertAlign(osgEarth::Util::Controls::Control::ALIGN_BOTTOM);
      canvas->addControl(infoPanel);  // Add panel to screen

      QString labelText = QString(
                                      "ID: %1\nLat: %2\nLon: %3\nAlt: %4 m\nPitch: %5\nHeading: %6\nYaw: %7\nSpeed: %8 m/s")
                                      .arg(cameraFocus.flight_id)
                                      .arg(cameraFocus.lat, 0, 'f', 6)
                                      .arg(cameraFocus.lon, 0, 'f', 6)
                                      .arg(cameraFocus.alt, 0, 'f', 1)
                                      .arg(cameraFocus.pitchAngle, 0, 'f', 1)
                                      .arg(cameraFocus.roll, 0, 'f', 1)
                                      .arg(cameraFocus.yaw, 0, 'f', 1)
                                      .arg(cameraFocus.speed_mps, 0, 'f', 1);

                  infolabel = new osgEarth::Util::Controls::LabelControl(" ", 12.0f);
                    infolabel->setFontSize(20.0f);
                  infolabel->setForeColor(osg::Vec4f(1.0f, 1.0f, 1.0f, 1.0f)); // White
                  infolabel->setBackColor(osg::Vec4f(0.0f, 0.0f, 0.0f, 0.4f));   // semi-transparent black
                  infolabel->setBorderColor(osg::Vec4f(1.0f, 1.0f, 1.0f, 0.8f)); // white border
                  infolabel->setBorderWidth(1.0f);
                  infolabel->setOpacity(1.0f);  // fully opaqu

                  infoPanel->addControl(infolabel);

}

void FlightSimulator::on_pb_Replay_clicked()
{
    if (udpSocket) {
        delete udpSocket;
        udpSocket = nullptr;
    }
    if (_annoGroup && _flightPathNode.valid()) {
        _annoGroup->removeChild(_flightPathNode.get());
    }
    if (_annoGroup && modelNode) {
        _annoGroup->removeChild(modelNode);
    }

    initFlightPath();
    simulateTrake = true;
    timeInterval = 1000;

    for (const QJsonObject& jsonObj : udpJsonMessages) {

        cameraFocus.flight_id = jsonObj.value("flight_id").toInt();
        cameraFocus.lon = jsonObj.value("longitude").toDouble(0.0);
        cameraFocus.lat = jsonObj.value("latitude").toDouble(0.0);
        cameraFocus.alt = jsonObj.value("altitude").toDouble(0.0);
        cameraFocus.roll = jsonObj.value("roll").toDouble(0.0);
        cameraFocus.pitchAngle = jsonObj.value("pitch").toDouble(0.0);
        cameraFocus.yaw = jsonObj.value("yaw").toDouble(0.0);
        cameraFocus.speed_mps = jsonObj.value("speed_mps").toDouble();

        QEventLoop loop;
           QTimer::singleShot(timeInterval, &loop, SLOT(quit()));
           loop.exec();

        createFlightPath();  // Optional: move outside loop if it's meant to process the whole list
    }


}
