#include "DockWidgetPanels.h"

#include "mainwindow.h"
#include "QMenu"

DockWidgetPanels::DockWidgetPanels(QWidget *parent)
    : QDockWidget(parent)
{
    setAllowedAreas(Qt::LeftDockWidgetArea |
                          Qt::RightDockWidgetArea );

    auto* mainWindow = qobject_cast<MainWindow*>(parent);

    if (mainWindow)
    {
        mainWindow->dockPanelsMenu()->addAction(toggleViewAction());
    }
}