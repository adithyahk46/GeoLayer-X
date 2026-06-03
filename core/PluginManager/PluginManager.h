#ifndef PLUGIN_MANAGER_H
#define PLUGIN_MANAGER_H

#include "PluginInterface.h"
#include "PluginManager_global.h"

#include <QHash>
#include <QObject>
#include <QPointer>
#include <QSet>

class QDialog;
class QPluginLoader;

class PLUGINMANAGER_EXPORT PluginManager : public QObject
{
    Q_OBJECT

public:
    enum class FilterCategory
    {
        All = 0,
        Installed,
        NotInstalled,
        Upgradeable,
        Invalid
    };
    Q_ENUM(FilterCategory)

    struct PluginInfo
    {
        QString id;
        QString name;
        QString version;
        QString description;
        QString category;
        QString vendor;
        QString filePath;
        QString error;
        QStringList tags;
        QStringList dependencies;

        bool installed = false;
        bool available = false;
        bool enabled = false;
        bool loaded = false;
        bool valid = false;
        bool hasUpdate = false;
    };

    static PluginManager* instance();

    ~PluginManager() override;

    // Host context APIs removed; plugins should use App singleton instead.

    void setPluginSearchPaths(const QStringList& paths);
    QStringList pluginSearchPaths() const;
    void addPluginSearchPath(const QString& path);

    void scanPlugins();
    QList<PluginInfo> plugins(FilterCategory filter = FilterCategory::All, const QString& searchText = QString()) const;
    PluginInfo plugin(const QString& pluginId) const;

    bool setEnabled(const QString& pluginId, bool enabled, QString* errorMessage = nullptr);
    bool loadPlugin(const QString& pluginId, QString* errorMessage = nullptr);
    bool unloadPlugin(const QString& pluginId, QString* errorMessage = nullptr);
    bool executePluginCommand(
        const QString& pluginId,
        const QString& commandId,
        const QVariantMap& arguments = {},
        QVariantMap* result = nullptr,
        QString* errorMessage = nullptr);

    bool installFromZip(const QString& zipPath, QString* errorMessage = nullptr);

    void showManagerDialog(QWidget* parent = nullptr);

signals:
    void pluginsChanged();
    void pluginStateChanged(const QString& pluginId, bool loaded, bool enabled);
    void pluginError(const QString& pluginId, const QString& message);

private:
    explicit PluginManager(QObject* parent = nullptr);
    Q_DISABLE_COPY_MOVE(PluginManager)

    struct RuntimePlugin
    {
        PluginInfo info;
        QPluginLoader* loader = nullptr;
        PluginInterface* iface = nullptr;
    };

    void clearRuntimeLoaders();
    void clearDiscoveredPlugins();
    void discoverFromFilesystem();
    void discoverFromCatalog();

    void saveEnabledState();
    void loadEnabledState();

    QString normalizePluginId(const QString& value) const;
    QString defaultPluginDir() const;

    static PluginManager* _instance;

    // PluginHostContext removed.
    QStringList _pluginSearchPaths;
    QSet<QString> _enabledIds;
    QHash<QString, RuntimePlugin> _plugins;
    QPointer<QDialog> _dialog;
};

#endif // PLUGIN_MANAGER_H
