#pragma once

#include <QSettings>
#include <QMainWindow>
#include <QSettings>
#include <QCoreApplication>
#include <QDir>

#include "osgearthcore_global.h"


class OSGEARTHCORE_EXPORT SettingsManager
{
public:
    static SettingsManager& instance();

    // Main window
    void saveMainWindowState(QMainWindow* window);
    void restoreMainWindowState(QMainWindow* window);

    // Generic settings
    void setValue(const QString& key, const QVariant& value);
    QVariant value(const QString& key,
                   const QVariant& defaultValue = QVariant()) const;

    void remove(const QString& key);

    void sync();

private:
    SettingsManager();

    SettingsManager(const SettingsManager&) = delete;
    SettingsManager& operator=(const SettingsManager&) = delete;

    QSettings m_settings;
};