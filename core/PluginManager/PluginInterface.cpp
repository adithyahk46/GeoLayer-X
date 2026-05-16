#include "PluginInterface.h"

PluginInterface::PluginInterface(QObject* parent)
	: QObject(parent)
{
}

QString PluginInterface::pluginId() const
{
	const QString fallbackName = pluginName().trimmed();
	if (fallbackName.isEmpty())
	{
		return metaObject()->className();
	}

	QString id = fallbackName;
	id = id.toLower();
	id.replace(' ', '.');
	return id;
}

QString PluginInterface::pluginVersion() const
{
	return QStringLiteral("0.0.0-dev");
}

QString PluginInterface::pluginDescription() const
{
	return QString();
}

QString PluginInterface::pluginCategory() const
{
	return QStringLiteral("general");
}

PluginInterface::PluginKind PluginInterface::pluginKind() const
{
	return PluginKind::Unknown;
}

PluginInterface::Capabilities PluginInterface::capabilities() const
{
	return NoCapability;
}

PluginDescriptor PluginInterface::descriptor() const
{
	PluginDescriptor info;
	info.id = pluginId();
	info.name = pluginName();
	info.version = pluginVersion();
	info.description = pluginDescription();
	info.category = pluginCategory();
	info.dependencies = hardDependencies();
	return info;
}

QVersionNumber PluginInterface::pluginApiVersion() const
{
	return QVersionNumber(HostApiMajor, HostApiMinor);
}

QVersionNumber PluginInterface::minimumHostApiVersion() const
{
	return QVersionNumber(1, 0);
}

QVersionNumber PluginInterface::maximumHostApiVersion() const
{
	return QVersionNumber(999, 0);
}

QStringList PluginInterface::hardDependencies() const
{
	return {};
}

QStringList PluginInterface::optionalDependencies() const
{
	return {};
}

bool PluginInterface::load(const PluginHostContext& context, QString* errorMessage)
{
	Q_UNUSED(errorMessage)
	_hostContext = context;
	initialize();
	setState(LifecycleState::Initialized);
	return true;
}

bool PluginInterface::activate(QString* errorMessage)
{
	Q_UNUSED(errorMessage)
	if (_state == LifecycleState::Constructed)
	{
		if (!load(_hostContext, errorMessage))
		{
			setState(LifecycleState::Error);
			return false;
		}
	}

	setState(LifecycleState::Active);
	return true;
}

bool PluginInterface::deactivate(QString* errorMessage)
{
	Q_UNUSED(errorMessage)
	if (_state == LifecycleState::Active)
	{
		setState(LifecycleState::Suspended);
	}
	return true;
}

bool PluginInterface::unload(QString* errorMessage)
{
	Q_UNUSED(errorMessage)
	setState(LifecycleState::Stopped);
	return true;
}

bool PluginInterface::canUnload(QString* reason) const
{
	if (reason)
	{
		reason->clear();
	}
	return true;
}

PluginInterface::LifecycleState PluginInterface::state() const
{
	return _state;
}

QList<QAction*> PluginInterface::globalActions(QObject* parent)
{
	Q_UNUSED(parent)
	return {};
}

QList<QMenu*> PluginInterface::menus(QWidget* parent)
{
	Q_UNUSED(parent)
	return {};
}

QList<QToolBar*> PluginInterface::toolbars(QWidget* parent)
{
	Q_UNUSED(parent)
	return {};
}

QList<QDockWidget*> PluginInterface::dockWidgets(QWidget* parent)
{
	Q_UNUSED(parent)
	return {};
}

QWidget* PluginInterface::settingsPage(QWidget* parent)
{
	Q_UNUSED(parent)
	return nullptr;
}

QStringList PluginInterface::commandIds() const
{
	return {};
}

bool PluginInterface::executeCommand(
	const QString& commandId,
	const QVariantMap& arguments,
	QVariantMap* result,
	QString* errorMessage)
{
	Q_UNUSED(commandId)
	Q_UNUSED(arguments)

	if (result)
	{
		result->clear();
	}

	if (errorMessage)
	{
		*errorMessage = QStringLiteral("Command not implemented by plugin.");
	}

	return false;
}

bool PluginInterface::onHostEvent(const QString& eventName, const QVariantMap& payload)
{
	Q_UNUSED(eventName)
	Q_UNUSED(payload)
	return false;
}

void PluginInterface::setState(LifecycleState newState)
{
	if (_state == newState)
	{
		return;
	}

	const LifecycleState previousState = _state;
	_state = newState;
	emit lifecycleStateChanged(previousState, _state);
}

const PluginHostContext& PluginInterface::hostContext() const
{
	return _hostContext;
}

PluginHostContext& PluginInterface::hostContext()
{
	return _hostContext;
}
