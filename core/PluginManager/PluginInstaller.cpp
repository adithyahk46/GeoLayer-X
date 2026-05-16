#include "PluginInstaller.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QProcess>

namespace
{
bool copyRecursively(const QString& sourcePath, const QString& destinationPath)
{
	QFileInfo sourceInfo(sourcePath);
	if (!sourceInfo.exists())
	{
		return false;
	}

	if (sourceInfo.isDir())
	{
		QDir destinationDir(destinationPath);
		if (!destinationDir.exists() && !destinationDir.mkpath("."))
		{
			return false;
		}

		const QDir sourceDir(sourcePath);
		const QFileInfoList entries = sourceDir.entryInfoList(
			QDir::NoDotAndDotDot | QDir::AllEntries,
			QDir::Name | QDir::DirsFirst);

		for (const QFileInfo& entry : entries)
		{
			const QString targetPath = destinationDir.filePath(entry.fileName());
			if (!copyRecursively(entry.absoluteFilePath(), targetPath))
			{
				return false;
			}
		}

		return true;
	}

	QFile::remove(destinationPath);
	return QFile::copy(sourcePath, destinationPath);
}
}

bool PluginInstaller::installPackage(const QString& sourcePath, const QString& destinationDir, QString* errorMessage)
{
	const QFileInfo sourceInfo(sourcePath);
	if (!sourceInfo.exists())
	{
		if (errorMessage)
		{
			*errorMessage = QStringLiteral("Plugin package does not exist: %1").arg(sourcePath);
		}
		return false;
	}

	QDir().mkpath(destinationDir);

	if (sourceInfo.isDir())
	{
		const QString targetPath = QDir(destinationDir).filePath(sourceInfo.fileName());
		if (!copyRecursively(sourcePath, targetPath))
		{
			if (errorMessage)
			{
				*errorMessage = QStringLiteral("Failed to copy plugin directory to %1").arg(targetPath);
			}
			return false;
		}
		return true;
	}

	if (sourceInfo.suffix().compare(QStringLiteral("zip"), Qt::CaseInsensitive) == 0)
	{
#ifdef Q_OS_WIN
		const QString command = QStringLiteral("Expand-Archive -LiteralPath \"%1\" -DestinationPath \"%2\" -Force")
									.arg(QDir::toNativeSeparators(sourcePath), QDir::toNativeSeparators(destinationDir));

		QProcess process;
		process.start(QStringLiteral("powershell"), {QStringLiteral("-NoProfile"), QStringLiteral("-Command"), command});
		process.waitForFinished(-1);

		if (process.exitStatus() != QProcess::NormalExit || process.exitCode() != 0)
		{
			if (errorMessage)
			{
				const QString stderrText = QString::fromLocal8Bit(process.readAllStandardError());
				*errorMessage = stderrText.isEmpty()
									? QStringLiteral("Expand-Archive failed for package: %1").arg(sourcePath)
									: stderrText;
			}
			return false;
		}

		return true;
#else
		if (errorMessage)
		{
			*errorMessage = QStringLiteral("ZIP installation is currently implemented for Windows only.");
		}
		return false;
#endif
	}

	if (errorMessage)
	{
		*errorMessage = QStringLiteral("Unsupported package type: %1").arg(sourceInfo.suffix());
	}
	return false;
}
