#include "ProjectWelcomeScreen.h"

#include "ProjectManager/ProjectManager.h"
#include "SettingsManager/SettingsManager.h"

#include <QQmlApplicationEngine>
#include <QQuickWindow>
#include <QMessageBox>
#include <QWidget>
#include <QUrl>
#include <QDate>
#include <QDesktopServices>
#include <QDir>
#include <QFileInfo>
#include <QStandardPaths>
#include <QVariantMap>

ProjectWelcomeScreen::ProjectWelcomeScreen(ProjectManager* projectManager,
                                                   QObject* parent)
    : QObject(parent), m_projectManager(projectManager)
{
    if (m_projectManager) {
        connect(m_projectManager, &ProjectManager::projectOpened,
                this, &ProjectWelcomeScreen::projectsChanged);
        connect(m_projectManager, &ProjectManager::errorOccurred, this,
                [this](const QString& message) { reportError(tr("Project"), message); });
        connect(this, &ProjectWelcomeScreen::projectOpenRequested,
                this, &ProjectWelcomeScreen::handleProjectOpen);
        connect(this, &ProjectWelcomeScreen::confirmedProjectFileSelected,
                this, &ProjectWelcomeScreen::handleProjectOpen);
        connect(this, &ProjectWelcomeScreen::saveAsProjectRequested, this,
                [this](const QString& filePath) {
            m_projectManager->saveProjectAs(filePath);
        });
        connect(this, &ProjectWelcomeScreen::earthFileSelected, this,
                [this](const QString& filePath) {
            m_projectManager->openEarthFile(filePath);
        });
        connect(this, &ProjectWelcomeScreen::projectActivated,
                this, &ProjectWelcomeScreen::welcomeScreenDismissed);
    }
}

void ProjectWelcomeScreen::ensureWelcomeScreen()
{
    if (m_window)
        return;

    m_engine = new QQmlApplicationEngine(this);
    m_engine->setInitialProperties({
        {QStringLiteral("projectController"),
         QVariant::fromValue(static_cast<QObject*>(this))}
    });
    m_engine->load(QUrl(QStringLiteral("qrc:/qml/WelcomeScreen.qml")));
    if (m_engine->rootObjects().isEmpty()) {
        delete m_engine;
        m_engine = nullptr;
        reportError(tr("Welcome screen"),
                    tr("The GeoLayer-X welcome screen could not be loaded."));
        return;
    }

    m_window = qobject_cast<QQuickWindow*>(m_engine->rootObjects().constFirst());
    if (!m_window) {
        delete m_engine;
        m_engine = nullptr;
        reportError(tr("Welcome screen"),
                    tr("The welcome screen QML root is not a window."));
        return;
    }
}

void ProjectWelcomeScreen::showWelcomeScreen()
{
    ensureWelcomeScreen();
    if (!m_window)
        return;

    m_window->setProperty("closingForProject", false);
    if (auto* parentWindow = qobject_cast<QWidget*>(parent()))
        m_window->setTransientParent(parentWindow->windowHandle());
    m_window->show();
    m_window->raise();
    m_window->requestActivate();
}

void ProjectWelcomeScreen::welcomeScreenDismissed()
{
    if (!m_engine)
        return;
    m_engine->deleteLater();
    m_engine = nullptr;
    m_window = nullptr;
}

QVariantList ProjectWelcomeScreen::visibleProjects() const
{
    QVariantList projects;
    if (!m_projectManager)
        return projects;

    const QDate today = QDate::currentDate();
    for (const ProjectInfo& project : m_projectManager->recentProjects()) {
        if (m_activeSection == QStringLiteral("Favorites") && !project.favorite)
            continue;

        const QFileInfo fileInfo(project.filePath);
        QString lastOpened = tr("Recently opened");
        if (project.lastOpened.isValid()) {
            const qint64 daysAgo = project.lastOpened.date().daysTo(today);
            if (daysAgo <= 0)
                lastOpened = tr("Last opened · Today");
            else if (daysAgo == 1)
                lastOpened = tr("Last opened · Yesterday");
            else
                lastOpened = tr("Last opened · %1 days ago").arg(daysAgo);
        }

        QString iconType = QStringLiteral("map");
        if (project.templateId == QStringLiteral("terrain"))
            iconType = QStringLiteral("terrain");
        else if (project.templateId == QStringLiteral("satellite")
                 || project.templateId == QStringLiteral("hybrid")
                 || project.templateId == QStringLiteral("earth3d"))
            iconType = QStringLiteral("globe");

        projects.push_back(QVariantMap{
            {QStringLiteral("title"), project.name},
            {QStringLiteral("subtitle"), project.filePath},
            {QStringLiteral("lastOpened"), lastOpened},
            {QStringLiteral("filePath"), project.filePath},
            {QStringLiteral("iconType"), iconType},
            {QStringLiteral("favorite"), project.favorite},
            {QStringLiteral("exists"), fileInfo.isFile()}
        });
    }
    return projects;
}

QString ProjectWelcomeScreen::activeSection() const
{
    return m_activeSection;
}

QString ProjectWelcomeScreen::defaultProjectLocation() const
{
    const QString savedLocation = SettingsManager::instance()
        .value(QStringLiteral("Project/DefaultLocation")).toString();
    return savedLocation.isEmpty()
        ? QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation)
        : savedLocation;
}

QUrl ProjectWelcomeScreen::defaultProjectLocationUrl() const
{
    return QUrl::fromLocalFile(defaultProjectLocation());
}

QString ProjectWelcomeScreen::localFilePath(const QUrl& fileUrl) const
{
    return fileUrl.toLocalFile();
}

bool ProjectWelcomeScreen::hasSavedDefaultProjectLocation() const
{
    return !SettingsManager::instance()
        .value(QStringLiteral("Project/DefaultLocation")).toString().isEmpty();
}

QVariantList ProjectWelcomeScreen::recentBasemaps() const
{
    return SettingsManager::instance()
        .value(QStringLiteral("Project/RecentBasemaps")).toList();
}

QString ProjectWelcomeScreen::currentTheme() const
{
    return SettingsManager::instance()
        .value(QStringLiteral("theme"), QStringLiteral("light")).toString();
}

void ProjectWelcomeScreen::selectSection(const QString& section)
{
    if (section != QStringLiteral("Recent")
        && section != QStringLiteral("Favorites")
        && section != QStringLiteral("Manage"))
        return;

    if (m_activeSection == section)
        return;
    m_activeSection = section;
    emit activeSectionChanged();
    emit projectsChanged();
}

bool ProjectWelcomeScreen::openProject(const QString& filePath)
{
    if (!m_projectManager || filePath.isEmpty())
        return false;

    emit projectOpenRequested(filePath);
    return true;
}

void ProjectWelcomeScreen::handleProjectOpen(const QString& filePath)
{
    if (m_projectManager->hasOpenProject()) {
        requestTransition(QStringLiteral("openRecent"), filePath);
    } else if (m_projectManager->openProject(filePath)) {
        completeProjectOpen();
    }
}

void ProjectWelcomeScreen::completeProjectOpen()
{
    emit projectsChanged();
    emit projectActivated();
}

void ProjectWelcomeScreen::requestTransition(const QString& action,
                                             const QString& filePath)
{
    if (m_projectManager)
        m_projectManager->requestProjectTransition(action, filePath);
}

bool ProjectWelcomeScreen::createProject(const QString& templateId,
                                             const QString& name,
                                             const QString& filePath,
                                             bool useAsDefaultLocation,
                                             const QString& description,
                                             const QString& basemapUrl,
                                             bool overwriteExisting)
{
    if (!m_projectManager)
        return false;

    const QString projectName = name.trimmed();
    QString normalizedPath = filePath.trimmed();
    if (projectName.isEmpty() || normalizedPath.isEmpty()) {
        emit projectError(tr("New project"),
                          tr("Enter both a project name and a save location."));
        return false;
    }
    if (!normalizedPath.endsWith(QStringLiteral(".earth"), Qt::CaseInsensitive))
        normalizedPath += QStringLiteral(".earth");

    if (QFileInfo::exists(normalizedPath) && !overwriteExisting) {
        emit projectFileExists(normalizedPath);
        return false;
    }

    if (!m_projectManager->createNewProject(projectName, normalizedPath, templateId,
                                             QStringLiteral("EPSG:4326"), basemapUrl,
                                             description))
        return false;

    if (useAsDefaultLocation) {
        SettingsManager::instance().setValue(
            QStringLiteral("Project/DefaultLocation"),
            QFileInfo(normalizedPath).absolutePath());
        SettingsManager::instance().sync();
    }

    if (templateId.startsWith(QStringLiteral("custom:")) && !basemapUrl.isEmpty()) {
        QVariantList updatedTemplates = recentBasemaps();
        for (qsizetype i = updatedTemplates.size() - 1; i >= 0; --i) {
            if (updatedTemplates.at(i).toMap()
                    .value(QStringLiteral("url")).toString() == basemapUrl) {
                updatedTemplates.removeAt(i);
            }
        }
        updatedTemplates.prepend(QVariantMap{
            {QStringLiteral("name"), templateId.mid(QStringLiteral("custom:").size())},
            {QStringLiteral("url"), basemapUrl}
        });
        while (updatedTemplates.size() > 8)
            updatedTemplates.removeLast();
        SettingsManager::instance().setValue(
            QStringLiteral("Project/RecentBasemaps"), updatedTemplates);
        SettingsManager::instance().sync();
        emit recentBasemapsChanged();
    }

    emit projectsChanged();
    emit projectActivated();
    return true;
}

bool ProjectWelcomeScreen::isDefaultProjectLocation(const QString& projectFilePath) const
{
    return QDir::cleanPath(QFileInfo(projectFilePath).absoluteFilePath()).compare(
               QDir::cleanPath(defaultProjectLocation()), Qt::CaseInsensitive) == 0;
}

QString ProjectWelcomeScreen::suggestedProjectFilePath(const QString& name) const
{
    return QDir(defaultProjectLocation()).filePath(name + QStringLiteral(".earth"));
}

QString ProjectWelcomeScreen::projectFilePath(const QString& name,
                                                  const QString& directory) const
{
    return QDir(directory).filePath(name.trimmed() + QStringLiteral(".earth"));
}

void ProjectWelcomeScreen::saveTheme(const QString& theme)
{
    if (theme != QStringLiteral("light") && theme != QStringLiteral("dark"))
        return;
    SettingsManager::instance().setValue(QStringLiteral("theme"), theme);
    SettingsManager::instance().sync();
    emit currentThemeChanged();
}

void ProjectWelcomeScreen::requestApplicationExit()
{
    if (m_projectManager)
        m_projectManager->requestApplicationExit();
}

void ProjectWelcomeScreen::requestOpenProjectFileDialog(bool transitionAlreadyConfirmed)
{
    emit openProjectFileDialogRequested(transitionAlreadyConfirmed);
}

void ProjectWelcomeScreen::requestOpenEarthFileDialog()
{
    emit openEarthFileDialogRequested();
}

void ProjectWelcomeScreen::requestSaveAsFileDialog(const QUrl& initialFile)
{
    emit saveAsFileDialogRequested(initialFile);
}

void ProjectWelcomeScreen::openProjectFileSelected(const QString& filePath)
{
    emit confirmedProjectFileSelected(filePath);
}

void ProjectWelcomeScreen::submitSaveAsFile(const QString& filePath)
{
    emit saveAsProjectRequested(filePath);
}

void ProjectWelcomeScreen::selectEarthFile(const QString& filePath)
{
    emit earthFileSelected(filePath);
}

void ProjectWelcomeScreen::reportError(const QString& title, const QString& message)
{
    emit projectError(title, message);
    if (!m_window || !m_window->isVisible()) {
        if (auto* parentWidget = qobject_cast<QWidget*>(parent()))
            QMessageBox::warning(parentWidget, title, message);
    }
}

void ProjectWelcomeScreen::openProjectCenter()
{
    selectSection(QStringLiteral("Recent"));
}

void ProjectWelcomeScreen::toggleFavorite(const QString& filePath)
{
    if (!m_projectManager)
        return;

    for (const ProjectInfo& project : m_projectManager->recentProjects()) {
        if (project.filePath == filePath) {
            if (!m_projectManager->setFavorite(filePath, !project.favorite))
                emit projectError(tr("Favorites"),
                                  tr("Could not update the favorite status."));
            emit projectsChanged();
            return;
        }
    }
}

void ProjectWelcomeScreen::removeRecentProject(const QString& filePath)
{
    if (m_projectManager && !m_projectManager->removeFromRecent(filePath))
        emit projectError(tr("Recent projects"),
                          tr("Could not remove the project from recent projects."));
    emit projectsChanged();
}

void ProjectWelcomeScreen::showProjectLocation(const QString& filePath)
{
    if (!QDesktopServices::openUrl(QUrl::fromLocalFile(QFileInfo(filePath).absolutePath())))
        emit projectError(tr("Project location"),
                          tr("Could not open the project folder."));
}
