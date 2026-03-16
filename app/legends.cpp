#include "legends.h"
#include <cstdint>

#include <QMenu>
#include <QMessageBox>
#include <QDebug>

#include <osgEarth/Notify>
#include <osgEarth/GeoMath>

using namespace osgEarth;

Legends::Legends(osgEarth::Layer* mapLayer, MapNode* mapNode, EarthManipulator* manip, QWidget* parent)
    : QCheckBox(parent), _manip(manip), _mapNode(mapNode), m_mapLayer(nullptr)
{

    setChecked(true);

    if (mapLayer)
    {
        setText(QString::fromStdString(mapLayer->getName()));
        qDebug() << QString::fromStdString(mapLayer->getTypeName());

        // Cast to specific layer type and assign to m_mapLayer
        if (auto imageLayer = dynamic_cast<osgEarth::ImageLayer*>(mapLayer)) {
            connect(this, &QCheckBox::toggled, this, [this,imageLayer]() {
                if (imageLayer) imageLayer->setVisible(isChecked());
            });
            m_mapLayer = imageLayer;
        }
        else if (auto elevationLayer = dynamic_cast<osgEarth::ElevationLayer*>(mapLayer)) {
            connect(this, &QCheckBox::toggled, this, [this,elevationLayer]() {
                if (elevationLayer) elevationLayer->setVisible(isChecked());
            });
            m_mapLayer = elevationLayer;
        }
        else if (auto modelLayer = dynamic_cast<osgEarth::FeatureModelLayer*>(mapLayer)) {
            connect(this, &QCheckBox::toggled, this, [this,modelLayer]() {
                if (modelLayer) modelLayer->setVisible(isChecked());
            });
            m_mapLayer = modelLayer;
        }
        else if (auto xyzLayer = dynamic_cast<osgEarth::XYZImageLayer*>(mapLayer)) {
            connect(this, &QCheckBox::toggled, this, [this,xyzLayer]() {
                if (xyzLayer) xyzLayer->setVisible(isChecked());
            });
            m_mapLayer = xyzLayer;
        }
        else if (auto AnnotationLayer = dynamic_cast<osgEarth::AnnotationLayer*>(mapLayer)) {
            connect(this, &QCheckBox::toggled, this, [this,AnnotationLayer]() {
                if (AnnotationLayer) AnnotationLayer->setVisible(isChecked());
            });
            m_mapLayer = AnnotationLayer;
        }
        else if (auto dmodelLayer = dynamic_cast<osgEarth::ModelLayer*>(mapLayer)) {
            connect(this, &QCheckBox::toggled, this, [this,dmodelLayer]() {
                if (dmodelLayer) dmodelLayer->setVisible(isChecked());
            });
            m_mapLayer = dmodelLayer;
        }
        else {
            std::cout << "Unknown layer type: " << mapLayer->getName() << std::endl;
        }
    }
    else
    {
        setText("Unnamed Layer");
    }

    // connect(this, &QCheckBox::toggled, this, [this]() {
    //     if (m_mapLayer) m_mapLayer->setVisible(isChecked());
    // });

    // Context menu
    menu.addAction("Zoom to Layer", [=]() {

        if(QString::fromStdString(m_mapLayer->getTypeName()) == "class osgEarth::ModelLayer" )
        {
            osg::ComputeBoundsVisitor cbv;
              m_mapLayer->getNode()->accept(cbv);
            osg::BoundingBox bbox = cbv.getBoundingBox();
            if (!bbox.valid()) {
                qDebug() << "Invalid bounding box!";
                return;
            }
            qDebug() << "Model layer extent (local):"
                     << bbox.xMin() << bbox.yMin() << bbox.zMin()
                     << bbox.xMax() << bbox.yMax() << bbox.zMax();

            // Transform to world coordinates
            osg::Matrix localToWorld;
            osg::NodePathList paths = m_mapLayer->getNode()->getParentalNodePaths();
            if (!paths.empty())
                localToWorld = osg::computeLocalToWorld(paths.front());

            osg::Vec3d minCorner = bbox.corner(0) * localToWorld;
            osg::Vec3d maxCorner = bbox.corner(7) * localToWorld;

            // Convert to geographic coordinates
            const osgEarth::SpatialReference* srs = mapNode->getMapSRS();
            osgEarth::GeoPoint geoMin, geoMax;
            geoMin.fromWorld(srs, minCorner);
            geoMax.fromWorld(srs, maxCorner);

            // Build extent
            osgEarth::GeoExtent extent(
                srs,
                std::min(geoMin.x(), geoMax.x()), std::min(geoMin.y(), geoMax.y()),
                std::max(geoMin.x(), geoMax.x()), std::max(geoMin.y(), geoMax.y()));

            qDebug() << "Extent (geographic):"
                     << extent.xMin() << extent.yMin()
                     << extent.xMax() << extent.yMax();

            // Compute the geographic center of the extent
            double centerX = (extent.xMin() + extent.xMax()) / 2.0;
            double centerY = (extent.yMin() + extent.yMax()) / 2.0;

            // Compute diagonal distance (rough size of model on map)
            double dx = extent.width();
            double dy = extent.height();
            double distance = sqrt(dx*dx + dy*dy) * 1000.0; // scale factor (tune as needed)

            // Build viewpoint
            osgEarth::Viewpoint vp(
                "",                 // name (optional)
                centerX,            // longitude
                centerY,            // latitude
                0.0,                // altitude (meters above ellipsoid)
                0.0,                // heading
                -90.0,              // pitch (looking straight down)
                distance   // range (meters from target)
            );

            // Animate to viewpoint
            _manip->setViewpoint(vp, 1.0); // 1-second animation

        }
        else{
            const SpatialReference* wgs84 = SpatialReference::get("wgs84");
            if (!m_mapLayer) return;
            GeoExtent extent = m_mapLayer->getExtent().transform(wgs84);
            if (!extent.isValid())
            {
                qDebug() << "Error: Layer extent is invalid.";
                return;
            }

            GeoPoint center(extent.getSRS(), extent.getCentroid().x(), extent.getCentroid().y(), 0.0);
            GeoPoint p1(extent.getSRS(), extent.xMin(), extent.yMin(), 0.0);
            GeoPoint p2(extent.getSRS(), extent.xMax(), extent.yMax(), 0.0);

            double distance = p1.distanceTo(p2);
            Viewpoint vp("", center.x(), center.y(), 0.0, 0.0, -90.0, distance < 1000 ? 1000 : distance);
            _manip->setViewpoint(vp, 1.0); // 1 second animation
        }

    });

    menu.addAction("Remove Layer", [=]() {
        if (m_mapLayer)
        {
            _mapNode->getMap()->removeLayer(m_mapLayer);
            QMessageBox::information(this, "Success", "Layer removed successfully!");
            this->deleteLater();
        }
    });

    menu.addAction("Move to Top", [=]() {
        if (m_mapLayer)
        {
            _mapNode->getMap()->moveLayer(m_mapLayer, _mapNode->getMap()->getNumLayers() - 1);
        }
    });

    menu.addAction("Move to Bottom", [=]() {
        if (m_mapLayer)
        {
            _mapNode->getMap()->moveLayer(m_mapLayer, 1);
        }
    });

    if(QString::fromStdString(m_mapLayer->getTypeName()) == "class osgEarth::ModelLayer" )
    {
        menu.addAction("Edit Properties",[=](){
            osgEarth::ModelLayer* modelLayer = dynamic_cast<osgEarth::ModelLayer*>(m_mapLayer);

                        modelLayerPropertyDialog(modelLayer, _mapNode->getMapSRS());
        });
    }
}

Legends::~Legends()
{
    // Optional: Cleanup logic if needed
}

void Legends::mousePressEvent(QMouseEvent* event)
{
    if (event->button() == Qt::RightButton)
    {
        menu.exec(this->mapToGlobal(event->pos()));
    }
    else
    {
        QCheckBox::mousePressEvent(event);  // Preserve default behavior
    }
}

void Legends::modelLayerPropertyDialog(osgEarth::ModelLayer* modelLayer, const osgEarth::SpatialReference* srs)
{
    if (!modelLayer || !modelLayer->getNode())
        return;
    qDebug() << "dailog 1";


    osgEarth::GeoTransform* geoRaw = dynamic_cast<osgEarth::GeoTransform*>(modelLayer->getNode());
    if (!geoRaw) {
        qDebug() << "GeoTransform not found!";
        return;
    }

    // Wrap in ref_ptr safely if needed
    osg::ref_ptr<osgEarth::GeoTransform> geoXform = geoRaw;

    qDebug() << "GeoTransform found!";


    QString name = QString::fromStdString(modelLayer->getName());

    // Position
    osgEarth::GeoPoint pos = geoXform->getPosition();
    double lon = pos.x();
    double lat = pos.y();
    double alt = pos.z();

    // Rotation (from MatrixTransform)
    // osg::Quat q = geoXform->getRotation();
    // double h, p, r;
    // q.getEuler(h, p, r);

    // // Scale
    // osg::Vec3d scale = geoXform->getScale();


    // --- Build dialog ---
    QDialog dialog;
    dialog.setWindowTitle("Model Layer Properties");

    QFormLayout* form = new QFormLayout(&dialog);

    QLineEdit* nameEdit = new QLineEdit(name);
    QDoubleSpinBox *lonEdit = new QDoubleSpinBox(), *latEdit = new QDoubleSpinBox(), *altEdit = new QDoubleSpinBox();
    QDoubleSpinBox *rotX = new QDoubleSpinBox(), *rotY = new QDoubleSpinBox(), *rotZ = new QDoubleSpinBox();
    QDoubleSpinBox *scaleX = new QDoubleSpinBox(), *scaleY = new QDoubleSpinBox(), *scaleZ = new QDoubleSpinBox();

    // Configure spin boxes
    lonEdit->setRange(-180,180); lonEdit->setValue(lon);
    latEdit->setRange(-90,90);   latEdit->setValue(lat);
    altEdit->setRange(-10000,10000); altEdit->setValue(alt);

    rotX->setRange(-360,360); rotY->setRange(-360,360); rotZ->setRange(-360,360);
    scaleX->setRange(0.01,1000); scaleY->setRange(0.01,1000); scaleZ->setRange(0.01,1000);
    // scaleX->setValue(scale.x()); scaleY->setValue(scale.y()); scaleZ->setValue(scale.z());

    // Add fields
    form->addRow("Name:", nameEdit);
    form->addRow("Longitude:", lonEdit);
    form->addRow("Latitude:", latEdit);
    form->addRow("Altitude:", altEdit);
    form->addRow("Rotation X:", rotX);
    form->addRow("Rotation Y:", rotY);
    form->addRow("Rotation Z:", rotZ);
    form->addRow("Scale X:", scaleX);
    form->addRow("Scale Y:", scaleY);
    form->addRow("Scale Z:", scaleZ);

    QDialogButtonBox* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    form->addRow(buttons);

    QObject::connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
    QObject::connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);

    // --- Run dialog ---
    if (dialog.exec() == QDialog::Accepted)
    {
        // Apply changes
        modelLayer->setName(nameEdit->text().toStdString());

        // osgEarth::GeoPoint newLoc(srs,
        //                           lonEdit->value(), latEdit->value(), altEdit->value());
        // modelLayer->setLocation(newLoc);

        // if (geoXform)
        // {
        //     // Location
        //     osgEarth::GeoPoint newLoc(srs,
        //                               lonEdit->value(),
        //                               latEdit->value(),
        //                               altEdit->value());
        //     geoXform->setPosition(newLoc);

        //     // Rotation
        //     osg::Quat q;
        //     q.makeRotate(
        //         osg::DegreesToRadians(rotX->value()), osg::Vec3(1,0,0),
        //         osg::DegreesToRadians(rotY->value()), osg::Vec3(0,1,0),
        //         osg::DegreesToRadians(rotZ->value()), osg::Vec3(0,0,1));
        //     geoXform->setRotation(q);

        //     // Scale
        //     geoXform->setScale(osg::Vec3d(scaleX->value(),
        //                                   scaleY->value(),
        //                                   scaleZ->value()));

        //     // Layer name
        //     modelLayer->setName(nameEdit->text().toStdString());

        //     // Mark layer dirty so map refreshes
        //     modelLayer->dirty();


        // Example: move to lon=77.6, lat=12.9, altitude=1000m (Bangalore-ish)
          osgEarth::GeoPoint newPos(srs, lonEdit->value(), latEdit->value(), altEdit->value(), osgEarth::ALTMODE_ABSOLUTE);
          geoXform->setPosition(newPos);

          // --- 2. Apply scale ---
          osg::Matrix scaleMat = osg::Matrix::scale(scaleX->value(),
                                                    scaleY->value(),
                                                    scaleZ->value()); // uniform scaling ×2

          // --- 3. Apply rotation ---
          osg::Matrix rotMat = osg::Matrix::rotate(
              osg::DegreesToRadians(45.0), osg::Z_AXIS // rotate 45° around Z axis
          );

          // --- 4. Combine transforms ---
          // Order: translate (from GeoPoint) is already applied in setPosition,
          // so we multiply the extra scale/rotation on top.
          osg::Matrix finalMat = scaleMat * rotMat * geoXform->getMatrix();

          // Apply back
          geoXform->setMatrix(finalMat);

    }
}
