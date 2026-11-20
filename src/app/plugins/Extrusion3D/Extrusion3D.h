#ifndef EXTRUSION3D_H
#define EXTRUSION3D_H
/**
 * @file Extrusion3D.h
 * @brief It is the inbuilt app plugin for 3D extrusion functionality.
 *
 * This file defines the Extrusion3D class, which provides a user interface
 * for configuring 3D extrusion using opened feature layers/browse shapefiles.
 * it loads new FeatureModelLayer to the map on apply changes. if its already loaded, it updates the existing layer with new extrusion settings.
 * on combo box selection, it updates the ui extrusion settings of the selected layer.
 *
 * @author Adithya
 * @version 1.0
 * @date 2024-06-10
 */

#include <QDialog>
#include <QColor>
#include <QString>

#include <osg/ref_ptr>

#include <osgEarth/OGRFeatureSource>
#include <osgEarth/MapNode>

class QPushButton;
class QShowEvent;

QT_BEGIN_NAMESPACE
namespace Ui {
class Extrusion3D;
}
QT_END_NAMESPACE

class Extrusion3D : public QDialog
{
    Q_OBJECT

public:
    explicit Extrusion3D(osgEarth::MapNode *mapNode, QWidget *parent = nullptr);
    ~Extrusion3D() override;

protected:
    void showEvent(QShowEvent *event) override;

private:
    void refreshFeatureLayers(osgEarth::UID selectedLayerUID = 0);
    void updateHeightControls();
    void updateColorButton(QPushButton *button, const QColor& color);
    bool loadAttributeFields(osgEarth::FeatureSource *source);
    void applyChanges();

    Ui::Extrusion3D *ui;
    osg::ref_ptr<osgEarth::MapNode> _mapNode;
    osg::ref_ptr<osgEarth::OGRFeatureSource> _pendingFeatureSource;
    QString _pendingFilePath;
    QColor _fillColor;
    QColor _outlineColor;
};
#endif // EXTRUSION3D_H
