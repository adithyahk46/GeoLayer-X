#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "../core/osgEarthViewerWidget.h"
#include <PluginManager/PluginManager.h>

namespace Ui {
class MainWindow;
}


class MainWindow : public QMainWindow
{
	Q_OBJECT

public:
	explicit MainWindow(QWidget* parent = nullptr);
	~MainWindow() override;

protected:
    void _initConnections();
	void _refreshPluginsDock();
private:
	Ui::MainWindow* ui = nullptr;

    OsgEarthViewerWidget* osgWidget = nullptr;
	PluginManager* _pluginManager = nullptr;

};

#endif // MAINWINDOW_H
