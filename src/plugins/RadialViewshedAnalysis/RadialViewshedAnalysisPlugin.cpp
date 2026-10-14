#include "RadialViewshedAnalysisPlugin.h"

#include <App.h>
#include <QMainWindow>

RadialViewshedAnalysisPlugin::RadialViewshedAnalysisPlugin(QObject* parent)
    : PluginInterface(parent)
{
}

RadialViewshedAnalysisPlugin::~RadialViewshedAnalysisPlugin()
{
    if (_dialog)
    {
        delete _dialog;
        _dialog = nullptr;
    }
}

QString RadialViewshedAnalysisPlugin::pluginName() const
{
    return QStringLiteral("Radial Viewshed Analysis");
}

void RadialViewshedAnalysisPlugin::initialize()
{
}

bool RadialViewshedAnalysisPlugin::executeCommand(
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
        _dialog = new RadialViewshedAnalysisWidget(parent);
        QObject::connect(_dialog, &QObject::destroyed, this, [this]() {
            _dialog = nullptr;
        });
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
