#ifndef LABELINGDIALOG_H
#define LABELINGDIALOG_H

#include <QDialog>
#include <osgearth/MapNode>
#include <osg/Group>
#include "MouseEventHandler.h"

#include <QColorDialog>
#include <osgEarth/LabelNode>
#include <osgEarth/Style>
#include <osgEarth/TextSymbol>
#include <osgEarth/BBoxSymbol>

using namespace osgEarth;


namespace Ui {
class LabelingDialog;
}

class LabelingDialog : public QDialog
{
    Q_OBJECT

public:
    explicit LabelingDialog(osgEarth::MapNode* mapNode, MouseEventHandler* mousehandler,QWidget *parent = nullptr);
    ~LabelingDialog();

    void OpenTOGetStyle();

    osgEarth::Style getStyle();
private slots:
    void onMapClicked(const double lon, const double lat, const double alt, const osgGA::GUIEventAdapter &, osgGA::GUIActionAdapter &);
signals:
    void LabelCreated( LabelNode* _measureLabel);
private:
    Ui::LabelingDialog *ui;

    osgEarth::MapNode* _mapNode{nullptr};
    MouseEventHandler* _mousehandler{nullptr};

    void onReset();
    void onAddTextClicked();
    QString getColor();
};

#endif // LABELINGDIALOG_H
