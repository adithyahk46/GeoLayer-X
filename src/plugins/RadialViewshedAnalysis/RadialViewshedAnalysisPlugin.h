#ifndef RADIAL_VIEWSHED_ANALYSIS_PLUGIN_H
#define RADIAL_VIEWSHED_ANALYSIS_PLUGIN_H

#include <PluginManager/PluginInterface.h>

#include "RadialViewshedAnalysisWidget.h"

class RadialViewshedAnalysisPlugin : public PluginInterface
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID "com.geolayerx.plugins.PluginInterface/1.0" FILE "RadialViewshedAnalysisPlugin.json")
    Q_INTERFACES(PluginInterface)

public:
    explicit RadialViewshedAnalysisPlugin(QObject* parent = nullptr);
    ~RadialViewshedAnalysisPlugin() override;

    QString pluginName() const override;
    void initialize() override;

    bool executeCommand(
        const QString& commandId,
        const QVariantMap& arguments,
        QVariantMap* result = nullptr,
        QString* errorMessage = nullptr) override;

private:
    RadialViewshedAnalysisWidget* _dialog = nullptr;
};

#endif
