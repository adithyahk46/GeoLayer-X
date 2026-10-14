#ifndef DOCKWIDGETPANELS_H
#define DOCKWIDGETPANELS_H

/**
 * @file DockWidgetPanels.h
 * @author your name (
 *
 * @brief This is the header file for the DockWidgetPanels class, which is a subclass of QDockWidget.
 *  It provides a base class for creating dockable panels in the application.
 * It allows for customization of the window title and object name of the dock widget.
 * if any class inherits from this class, it must implement the setWindowTitle and setObjectName methods to set the title and object name of the dock widget.
*/

#include <QDockWidget>
#include <QObject>

#include "App.h"

class DockWidgetPanels : public QDockWidget
{
    Q_OBJECT

public:
    explicit DockWidgetPanels(QWidget *parent = nullptr);

    //! override to set custom values in the .ini file
    // virtual void load(const osgEarth::Config&) { }

    // //! override to get custom values from the .ini file
    // virtual void save(osgEarth::Config&) { }

    virtual void setWindowTitle(const QString &title) = 0;
    virtual void setObjectName(const QString &name) = 0;
};

#endif // DOCKWIDGETPANELS_H
