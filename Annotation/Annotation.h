#ifndef ANNOTATION_H
#define ANNOTATION_H

#include <QWidget>

#include <osgEarth/AnnotationUtils>
#include <osgEarth/MapNode>
#include <osgEarth/LabelNode>
#include <osgEarth/Style>

#include <osg/Group>
#include <osg/Node>
#include <QDialog>

#include "MouseEventHandler.h"





namespace Ui {
class Annotation;
}

class Annotation : public QDialog
{
    Q_OBJECT

public:
    explicit Annotation(osgEarth::MapNode* mapNode,MouseEventHandler* mouseControlle,QWidget *parent = nullptr);
    ~Annotation();

private slots:
    void on_pb_Add_clicked();
    void mouseClickEvent(const double lon, const double lat,const double alt,const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
    // void mouseDoubleClickEvent(const double lon, const double lat,const double alt,const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
    // void mouseMoveEvent(const double lon, const double lat,const double alt,const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);
    // void mouseScrollEvent(const double lon, const double lat,const double alt,const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);


private:
    Ui::Annotation *ui;

    osgEarth::MapNode* _mapNode = nullptr;
    osg::Group* annoGroup = nullptr;
    osg::Group* labelGroup= nullptr;
    MouseEventHandler* _mouseControlle = nullptr;


    bool _addText = false;
};



#include <QDialog>
#include <QTableWidget>
#include <QVariantMap>
#include <QVector>

#include <osgEarth/Feature>
#include <osgEarth/FeatureNode>

class AttributeDialog : public QDialog
{
    Q_OBJECT

public:
    explicit AttributeDialog(osgEarth::FeatureNode* featureNode,const QList<QVariantMap>& fields, QWidget* parent = nullptr);
    ~AttributeDialog();

    // Retrieve the attribute values entered by the user
    QMap<QString, QString> getAttributes() const;

private:

    osgEarth::FeatureNode* m_featureNode;
    QTableWidget* m_table;
    QList<QVariantMap> m_fields;

    void setupUI();
    void populateTable();
    void applyAttributesToFeature();
};

#endif // ANNOTATION_H
