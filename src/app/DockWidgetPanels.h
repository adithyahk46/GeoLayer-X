#ifndef DOCKWIDGETPANELS_H
#define DOCKWIDGETPANELS_H

#include <QDockWidget>

class DockWidgetPanels : public QDockWidget
{
    Q_OBJECT

public:
    explicit DockWidgetPanels(QWidget *parent = nullptr);

    virtual void setWindowTitle(const QString &title) = 0;
    virtual void setObjectName(const QString &name) = 0;
};

#endif // DOCKWIDGETPANELS_H
