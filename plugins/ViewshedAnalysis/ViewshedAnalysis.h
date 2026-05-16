#ifndef VIEWSHED_ANALYSIS_PLUGIN_H
#define VIEWSHED_ANALYSIS_PLUGIN_H

#include <QDialog>

#include <PluginManager/PluginInterface.h>

namespace Ui {
class ViewshedAnalysis;
}

class ViewshedAnalysisDialog : public QDialog
{
	Q_OBJECT

public:
	explicit ViewshedAnalysisDialog(QWidget* parent = nullptr);
	~ViewshedAnalysisDialog() override;

private:
	Ui::ViewshedAnalysis* ui = nullptr;
};

class ViewshedAnalysisPlugin : public PluginInterface
{
	Q_OBJECT
	Q_PLUGIN_METADATA(IID GEOLAYER_PLUGIN_INTERFACE_IID FILE "ViewshedAnalysis.json")
	Q_INTERFACES(PluginInterface)

public:
	explicit ViewshedAnalysisPlugin(QObject* parent = nullptr);
	~ViewshedAnalysisPlugin() override;

	QString pluginName() const override;
	void initialize() override;

	QString pluginId() const override;
	QString pluginVersion() const override;
	QString pluginDescription() const override;
	QString pluginCategory() const override;
	PluginKind pluginKind() const override;
	Capabilities capabilities() const override;

	bool executeCommand(
		const QString& commandId,
		const QVariantMap& arguments,
		QVariantMap* result = nullptr,
		QString* errorMessage = nullptr) override;

private:
	ViewshedAnalysisDialog* _dialog = nullptr;
};

#endif // VIEWSHED_ANALYSIS_PLUGIN_H
