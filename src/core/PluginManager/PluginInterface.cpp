#include "PluginInterface.h"

PluginInterface::PluginInterface(QObject* parent)
	: QObject(parent)
{
}

bool PluginInterface::load(QString* errorMessage)
{
	Q_UNUSED(errorMessage)
	initialize();
	setState(LifecycleState::Initialized);
	return true;
}

bool PluginInterface::activate(QString* errorMessage)
{
	Q_UNUSED(errorMessage)
	if (_state == LifecycleState::Constructed)
	{
		if (!load(errorMessage))
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
