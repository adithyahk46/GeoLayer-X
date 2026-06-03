#ifndef PLUGIN_INTERFACE_H
#define PLUGIN_INTERFACE_H

#include "PluginManager_global.h"

#include <QFlags>
#include <QObject>
#include <QStringList>
#include <QVariantMap>
#include <QVersionNumber>
#include <QMainWindow>

#include "app.h"

class QAction;
class QDockWidget;
class QMainWindow;
class QMenu;
class QToolBar;
class QWidget;

namespace osgEarth {
class MapNode;
}

// PluginHostContext removed: plugins access host state via `App::getInstance()`.

struct PLUGINMANAGER_EXPORT PluginDescriptor
{
    QString id;
    QString name;
    QString version;
    QString description;
    QString category;
    QString vendor;
    QString homepageUrl;
    QString iconPath;
    QString license;
    QString minimumHostVersion;
    QString maximumTestedHostVersion;
    QStringList tags;
    QStringList dependencies;
    QVariantMap custom;
};

class PLUGINMANAGER_EXPORT PluginInterface : public QObject
{
    Q_OBJECT

public:
    enum class PluginKind
    {
        Unknown = 0,
        Core,
        Ui,
        DataProvider,
        Analysis,
        Rendering,
        Automation,
        Integration
    };
    Q_ENUM(PluginKind)

    enum CapabilityFlag
    {
        NoCapability = 0x0000,
        AddsMenu = 0x0001,
        AddsToolbar = 0x0002,
        AddsDockWidget = 0x0004,
        AddsMapLayer = 0x0008,
        AddsRenderer = 0x0010,
        SupportsBackgroundTasks = 0x0020,
        SupportsSettingsUi = 0x0040,
        SupportsHotReload = 0x0080,
        SupportsCommands = 0x0100,
        SupportsScripting = 0x0200,
        Experimental = 0x0400
    };
    Q_DECLARE_FLAGS(Capabilities, CapabilityFlag)
    Q_FLAG(Capabilities)

    enum class LifecycleState
    {
        Constructed = 0,
        Loaded,
        Initialized,
        Active,
        Suspended,
        Stopped,
        Error
    };
    Q_ENUM(LifecycleState)

    static constexpr int HostApiMajor = 1;
    static constexpr int HostApiMinor = 0;

    explicit PluginInterface(QObject* parent = nullptr);
    virtual ~PluginInterface() = default;

    // Legacy compatibility surface.
    virtual QString pluginName() const = 0;
    virtual void initialize() = 0;

    // Stable identity and descriptor metadata.
    virtual QString pluginId() const;
    virtual QString pluginVersion() const;
    virtual QString pluginDescription() const;
    virtual QString pluginCategory() const;
    virtual PluginKind pluginKind() const;
    virtual Capabilities capabilities() const;
    virtual PluginDescriptor descriptor() const;

    // Compatibility and dependency model.
    virtual QVersionNumber pluginApiVersion() const;
    virtual QVersionNumber minimumHostApiVersion() const;
    virtual QVersionNumber maximumHostApiVersion() const;
    virtual QStringList hardDependencies() const;
    virtual QStringList optionalDependencies() const;

    // Lifecycle hooks.
    virtual bool load(QString* errorMessage = nullptr);
    virtual bool activate(QString* errorMessage = nullptr);
    virtual bool deactivate(QString* errorMessage = nullptr);
    virtual bool unload(QString* errorMessage = nullptr);
    virtual bool canUnload(QString* reason = nullptr) const;
    virtual LifecycleState state() const;

    // Optional extension points.
    virtual QList<QAction*> globalActions(QObject* parent = nullptr);
    virtual QList<QMenu*> menus(QWidget* parent = nullptr);
    virtual QList<QToolBar*> toolbars(QWidget* parent = nullptr);
    virtual QList<QDockWidget*> dockWidgets(QWidget* parent = nullptr);
    virtual QWidget* settingsPage(QWidget* parent = nullptr);

    // Command and message hooks.
    virtual QStringList commandIds() const;
    virtual bool executeCommand(
        const QString& commandId,
        const QVariantMap& arguments,
        QVariantMap* result = nullptr,
        QString* errorMessage = nullptr);

    virtual bool onHostEvent(const QString& eventName, const QVariantMap& payload);


    osgEarth::MapNode* getMapNode(){return App::getInstance()->getMapNode();}

    QMainWindow* getMainWindow(){ return App::getInstance()->getMainwindow();}




protected:
    void setState(LifecycleState newState);

    // Host context access removed; use `App::getInstance()` instead.

private:
    // Host context removed; plugins should use App singleton.
    LifecycleState _state = LifecycleState::Constructed;

signals:
    void lifecycleStateChanged(PluginInterface::LifecycleState previous, PluginInterface::LifecycleState current);
};

Q_DECLARE_OPERATORS_FOR_FLAGS(PluginInterface::Capabilities)

#define GEOLAYER_PLUGIN_INTERFACE_IID "com.geolayerx.plugins.PluginInterface/1.0"
Q_DECLARE_INTERFACE(PluginInterface, GEOLAYER_PLUGIN_INTERFACE_IID)

#endif // PLUGIN_INTERFACE_H
