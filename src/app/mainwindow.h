#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <memory>


#include "MapController/MouseEventHandler.h"

#include "osgviewerwidget.h"
#include "MapController.h"

#include "DrawingTool/DrawingManager.h"

#include "MapLoadModuleDialog.h"
#include "LayerManagerWidget.h"
#include "StatusbarHandlerWidget.h"

class QActionGroup;
class QKeyEvent;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void initAll();

    void setApplicationTheme(bool dark);

    void saveAllSettings();

    QMenu* dockPanelsMenu() const;


    void restoreAllSettings();
private slots:
    void osgViewerSetup();

private:
    void keyPressEvent(QKeyEvent* event) override;

    Ui::MainWindow *ui;
    std::ofstream *_log;


    OSGViewerWidget* osgwidget= nullptr;
    StatusbarHandlerWidget* mStatusBarWidget = nullptr;
    osg::ref_ptr<MouseEventHandler> _mousePicker = nullptr;
    LayerManagerWidget* layerManager = nullptr;
    
    std::unique_ptr<DrawingManager> m_drawingManager;
    QActionGroup* m_drawingActionGroup = nullptr;


//declare private functions here
    void initCore();
    void initSignalsAndSlots();
    void initLogs();
};

#endif // MAINWINDOW_H
