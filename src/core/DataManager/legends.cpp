#include "legends.h"
#include <cstdint>

#include <QMenu>
#include <QDebug>

#include <osgEarth/Notify>
#include <osgEarth/GeoMath>
#include <osgEarth/FeatureImageLayer>
#include <osgEarth/VisibleLayer>

using namespace osgEarth;

#include "SymbologyEditorDialog.h"

Legends::Legends(osgEarth::Layer* mapLayer, MapNode* mapNode, EarthManipulator* manip, QWidget* parent)
    : QCheckBox(parent), _manip(manip), _mapNode(mapNode), m_mapLayer(mapLayer)
{

    auto* visibleLayer = dynamic_cast<osgEarth::VisibleLayer*>(mapLayer);
    setChecked(visibleLayer ? visibleLayer->getVisible() : true);
    if (visibleLayer)
    {
        connect(this, &QCheckBox::toggled, this, [this, visibleLayer]() {
            visibleLayer->setVisible(isChecked());
        });
    }

    if (mapLayer)
    {
        setText(QString::fromStdString(mapLayer->getName()));
        qDebug() << QString::fromStdString(mapLayer->getTypeName());
    }
    else
    {
        setText("Unnamed Layer");
    }

    // Context menu
    menu.addAction("Zoom to Layer", [=]() {
        if (!m_mapLayer || !_mapNode || !_manip)
            return;

        if (auto* modelLayer = dynamic_cast<osgEarth::ModelLayer*>(m_mapLayer))
        {
            if (!modelLayer->getNode())
                return;

            osg::ComputeBoundsVisitor cbv;
            modelLayer->getNode()->accept(cbv);
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
            osg::NodePathList paths = modelLayer->getNode()->getParentalNodePaths();
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

    menu.addAction("Remove Layer", [this]() {
        if (!m_mapLayer || !_mapNode || !_mapNode->getMap())
            return;

        osg::ref_ptr<osgEarth::Layer> layerToRemove = m_mapLayer;
        osgEarth::Map* map = _mapNode->getMap();
        menu.close();
        map->removeLayer(layerToRemove.get());
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

    auto* featureModelLayer = dynamic_cast<osgEarth::FeatureModelLayer*>(mapLayer);
    auto* featureImageLayer = dynamic_cast<osgEarth::FeatureImageLayer*>(mapLayer);
    if (featureModelLayer || featureImageLayer)
    {
        menu.addAction("Edit Symbology", [this, featureModelLayer, featureImageLayer]() {
            if (featureModelLayer)
            {
                SymbologyEditorDialog dialog(featureModelLayer, this);
                dialog.exec();
            }
            else if (featureImageLayer)
            {
                SymbologyEditorDialog dialog(featureImageLayer, this, _mapNode);
                dialog.exec();
            }
        });
    }

    if (auto* modelLayer = dynamic_cast<osgEarth::ModelLayer*>(m_mapLayer))
    {
        menu.addAction("Edit Properties", [this, modelLayer]() {
            if (_mapNode)
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
