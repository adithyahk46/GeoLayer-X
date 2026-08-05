#ifndef DOCKWIDGETPANELS_H
#define DOCKWIDGETPANELS_H

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
};

#endif // DOCKWIDGETPANELS_H
