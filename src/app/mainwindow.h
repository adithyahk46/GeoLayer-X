#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "osgviewerwidget.h"


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    void setApplicationTheme(bool dark);

private:
    Ui::MainWindow *ui;

    OSGViewerWidget* osgwidget= nullptr;


//declare private functions here
    void initCore();
    void initSignalsAndSlots();
};

#endif // MAINWINDOW_H
