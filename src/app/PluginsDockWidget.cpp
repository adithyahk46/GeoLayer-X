#include "PluginsDockWidget.h"

#include <QLineEdit>
#include <QListWidget>
#include <QListWidgetItem>
#include <QPushButton>
#include <QVBoxLayout>
#include <QMessageBox>

PluginsDockWidget::PluginsDockWidget(QWidget* parent)
    : DockWidgetPanels(parent)

{
    setWindowTitle("Plugins");
    setObjectName("PluginsDockWdget");

    setUpDockWidget();

}

void PluginsDockWidget::refreshPluginsList()
{
    _pluginList->clear();

    const QList<PluginManager::PluginInfo> plugins = _pluginManager->plugins();
    for (const PluginManager::PluginInfo& plugin : plugins)
    {
        QString label = plugin.name.isEmpty() ? plugin.id : plugin.name;
        if (plugin.loaded)
        {
            label += " [loaded]";
        }
        else if (!plugin.valid)
        {
            label += " [invalid]";
        }

        QListWidgetItem* item = new QListWidgetItem(label, _pluginList);
        item->setData(Qt::UserRole, plugin.id);

        if (!plugin.valid)
        {
            item->setForeground(Qt::red);
            item->setToolTip(plugin.error);
        }
        else
        {
            item->setToolTip(QStringLiteral("Double-click to open plugin dialog."));
        }
    }

}

void PluginsDockWidget::setUpDockWidget()
{
    //write a code to setup docwidget by lineedit and refresh button in horizantal layout and list widget in vertical

    // Search field
    _searchEdit = new QLineEdit(this);
    _searchEdit->setPlaceholderText("Search plugins...");

    // Refresh button
    _refreshPlugins = new QPushButton("Refresh", this);

    // Plugin list
    _pluginList = new QListWidget(this);

    // Horizontal layout: Search field + Refresh button
    QHBoxLayout* topLayout = new QHBoxLayout();
    topLayout->addWidget(_searchEdit, 1);
    topLayout->addWidget(_refreshPlugins);

    // Vertical layout: Top layout + Plugin list
    QVBoxLayout* mainLayout = new QVBoxLayout();
    mainLayout->addLayout(topLayout);
    mainLayout->addWidget(_pluginList, 1);

    // Set layout on dock widget content
    QWidget* contentWidget = new QWidget(this);
    contentWidget->setLayout(mainLayout);

    setWidget(contentWidget);


    connect(_pluginList, &QListWidget::itemDoubleClicked, this, [this](QListWidgetItem* item) {
        if (!item)
        {
            return;
        }

        const QString pluginId = item->data(Qt::UserRole).toString();
        if (pluginId.isEmpty())
        {
            return;
        }

        QString errorMessage;
        if (!_pluginManager->executePluginCommand(pluginId, QStringLiteral("showDialog"), {}, nullptr, &errorMessage))
        {
            QMessageBox::warning(this, QStringLiteral("Plugin"), errorMessage);
        }
    });
}
