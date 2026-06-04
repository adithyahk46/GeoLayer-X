#ifndef PLUGIN_INSTALLER_H
#define PLUGIN_INSTALLER_H

#include "../core_global.h"
#include <QString>

class GEOLAYERCORE_EXPORT PluginInstaller
{
public:
	static bool installPackage(const QString& sourcePath, const QString& destinationDir, QString* errorMessage = nullptr);
};

#endif // PLUGIN_INSTALLER_H
