#include "ViewshedAnalysis.h"

#include "ui_ViewshedAnalysis.h"

#include <QMainWindow>

ViewshedAnalysisDialog::ViewshedAnalysisDialog(QWidget* parent)
	: QDialog(parent)
	, ui(new Ui::ViewshedAnalysis)
{
	ui->setupUi(this);
}

ViewshedAnalysisDialog::~ViewshedAnalysisDialog()
{
	delete ui;
}

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

QString ViewshedAnalysisPlugin::pluginId() const
{
	return QStringLiteral("viewshed.analysis");
}

QString ViewshedAnalysisPlugin::pluginVersion() const
{
	return QStringLiteral("0.1.0");
}

QString ViewshedAnalysisPlugin::pluginDescription() const
{
	return QStringLiteral("Shows the Viewshed Analysis dialog shell.");
}

QString ViewshedAnalysisPlugin::pluginCategory() const
{
	return QStringLiteral("Analysis");
}

PluginInterface::PluginKind ViewshedAnalysisPlugin::pluginKind() const
{
	return PluginKind::Analysis;
}

PluginInterface::Capabilities ViewshedAnalysisPlugin::capabilities() const
{
	return AddsDockWidget | SupportsCommands;
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

	QWidget* parent = hostContext().mainWindow;
	if (!_dialog)
	{
		_dialog = new ViewshedAnalysisDialog(parent);
		_dialog->setAttribute(Qt::WA_DeleteOnClose, true);
		QObject::connect(_dialog, &QObject::destroyed, this, [this]() {
			_dialog = nullptr;
		});
	}

	_dialog->show();
	_dialog->raise();
	_dialog->activateWindow();

	if (result)
	{
		(*result)[QStringLiteral("dialogVisible")] = true;
	}
	return true;
}
