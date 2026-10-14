#ifndef PLUGIN_INTERFACE_H
#define PLUGIN_INTERFACE_H

#include "osgearthcore_global.h"

#include <QObject>
#include <QVariantMap>

class OSGEARTHCORE_EXPORT PluginInterface : public QObject
{
    Q_OBJECT

public:
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

    explicit PluginInterface(QObject* parent = nullptr);
    virtual ~PluginInterface() = default;

    // Legacy compatibility surface.
    virtual QString pluginName() const = 0;
    virtual void initialize() = 0;

    // Lifecycle hooks.
    virtual bool load(QString* errorMessage = nullptr);
    virtual bool activate(QString* errorMessage = nullptr);
    virtual bool deactivate(QString* errorMessage = nullptr);
    virtual bool unload(QString* errorMessage = nullptr);
    virtual bool canUnload(QString* reason = nullptr) const;

    virtual bool executeCommand(
        const QString& commandId,
        const QVariantMap& arguments,
        QVariantMap* result = nullptr,
        QString* errorMessage = nullptr);

protected:
    void setState(LifecycleState newState);

private:
    LifecycleState _state = LifecycleState::Constructed;

signals:
    void lifecycleStateChanged(PluginInterface::LifecycleState previous, PluginInterface::LifecycleState current);
};

#define GEOLAYER_PLUGIN_INTERFACE_IID "com.geolayerx.plugins.PluginInterface/1.0"
Q_DECLARE_INTERFACE(PluginInterface, "com.geolayerx.plugins.PluginInterface/1.0")

#endif // PLUGIN_INTERFACE_H