#include "DockWidgetPanels.h"

DockWidgetPanels::DockWidgetPanels(QWidget *parent)
    : QDockWidget(parent)
{
    setAllowedAreas(Qt::LeftDockWidgetArea |
                          Qt::RightDockWidgetArea );
}