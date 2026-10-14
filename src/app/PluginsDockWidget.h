#ifndef PLUGINSDOCKWIDGET_H
#define PLUGINSDOCKWIDGET_H

#include "DockWidgetPanels.h"
#include <PluginManager/PluginManager.h>

class QListWidget;
class QLineEdit;
class QPushButton;
class QVBoxLayout;
class QHBoxLayout;

class PluginsDockWidget : public DockWidgetPanels
{
    Q_OBJECT

public:
    explicit PluginsDockWidget(QWidget* parent = nullptr);

    void setUpPluginManager(PluginManager* pluginManager){_pluginManager = pluginManager;}
    void refreshPluginsList();

protected:
    void setWindowTitle(const QString& title) override { QDockWidget::setWindowTitle(title); }
    void setObjectName(const QString& name) override { QDockWidget::setObjectName(name); }

private:
    PluginManager* _pluginManager = nullptr;
    QLineEdit* _searchEdit = nullptr;
    QListWidget* _pluginList = nullptr;
    QPushButton* _refreshPlugins = nullptr;

    void setUpDockWidget();
};

#endif
