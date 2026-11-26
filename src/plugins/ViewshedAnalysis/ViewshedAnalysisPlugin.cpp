#include "ViewshedAnalysisPlugin.h"


#include <App.h>
#include <QMainWindow>


ViewshedAnalysisPlugin::ViewshedAnalysisPlugin(QObject* parent)
	: PluginInterface(parent)
{
}

ViewshedAnalysisPlugin::~ViewshedAnalysisPlugin()
{
	if (_dialog)
	{
		_dialog->close();
		delete _dialog;
		_dialog = nullptr;
	}
}

QString ViewshedAnalysisPlugin::pluginName() const
{
	return QStringLiteral("Viewshed Analysis");
}

void ViewshedAnalysisPlugin::initialize()
{
}

bool ViewshedAnalysisPlugin::executeCommand(
	const QString& commandId,
	const QVariantMap& arguments,
	QVariantMap* result,
	QString* errorMessage)
{
	Q_UNUSED(arguments)

	if (commandId.compare(QStringLiteral("showDialog"), Qt::CaseInsensitive) != 0)
	{
		if (errorMessage)
		{
			*errorMessage = QStringLiteral("Unsupported command: %1").arg(commandId);
		}
		return false;
	}

	QMainWindow* parent = App::getMainwindow();
	if (!parent)
	{
		if (errorMessage)
		{
			*errorMessage = QStringLiteral("Main window is not available.");
		}
		return false;
	}

	if (!_dialog)
	{
        _dialog = new ViewshedAnalysisWidget(parent);
		_dialog->setAttribute(Qt::WA_DeleteOnClose, true);
		QObject::connect(_dialog, &QObject::destroyed, this, [this]() {
			_dialog = nullptr;
		});
        _dialog->adjustSize();
	}
    _dialog->adjustSize();

	_dialog->show();
	_dialog->raise();
	_dialog->activateWindow();

	if (result)
	{
		(*result)[QStringLiteral("dialogVisible")] = true;
	}
	return true;
}
