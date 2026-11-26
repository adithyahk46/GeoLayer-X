#include "SettingsManager.h"

#include <QApplication>
#include <QMainWindow>
#include <QVariant>

SettingsManager& SettingsManager::instance()
{
    static SettingsManager instance;
    return instance;
}

SettingsManager::SettingsManager()
    : m_settings(
        QDir(QCoreApplication::applicationDirPath())
            .filePath("settings.ini"),
        QSettings::IniFormat)
{
}

void SettingsManager::setValue(
    const QString& key,
    const QVariant& value)
{
    m_settings.setValue(key, value);
}

QVariant SettingsManager::value(
    const QString& key,
    const QVariant& defaultValue) const
{
    return m_settings.value(key, defaultValue);
}

void SettingsManager::remove(const QString& key)
{
    m_settings.remove(key);
}

void SettingsManager::sync()
{
    m_settings.sync();
}

void SettingsManager::saveMainWindowState(QMainWindow* window)
{
    if (!window)
        return;

    m_settings.setValue(
        "MainWindow/Geometry",
        window->saveGeometry());

    m_settings.setValue(
        "MainWindow/State",
        window->saveState());
}

void SettingsManager::restoreMainWindowState(QMainWindow* window)
{
    if (!window)
        return;

    const QByteArray geometry =
        m_settings.value("MainWindow/Geometry").toByteArray();

    const QByteArray state =
        m_settings.value("MainWindow/State").toByteArray();

    if (!geometry.isEmpty())
        window->restoreGeometry(geometry);

    if (!state.isEmpty())
        window->restoreState(state);
}