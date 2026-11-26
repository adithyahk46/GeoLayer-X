#pragma once

#include <QObject>
#include <QUrl>
#include <QVariantList>

class ProjectManager;
class QQmlApplicationEngine;
class QQuickWindow;

class ProjectWelcomeScreen : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QVariantList visibleProjects READ visibleProjects NOTIFY projectsChanged)
    Q_PROPERTY(QString activeSection READ activeSection NOTIFY activeSectionChanged)
    Q_PROPERTY(QString defaultProjectLocation READ defaultProjectLocation CONSTANT)
    Q_PROPERTY(QUrl defaultProjectLocationUrl READ defaultProjectLocationUrl CONSTANT)
    Q_PROPERTY(bool hasSavedDefaultProjectLocation READ hasSavedDefaultProjectLocation CONSTANT)
    Q_PROPERTY(QVariantList recentBasemaps READ recentBasemaps NOTIFY recentBasemapsChanged)
    Q_PROPERTY(QString currentTheme READ currentTheme NOTIFY currentThemeChanged)

public:
    explicit ProjectWelcomeScreen(ProjectManager* projectManager,
                                      QObject* parent = nullptr);
    void showWelcomeScreen();

    QVariantList visibleProjects() const;
    QString activeSection() const;
    QString defaultProjectLocation() const;
    QUrl defaultProjectLocationUrl() const;
    bool hasSavedDefaultProjectLocation() const;
    QVariantList recentBasemaps() const;
    QString currentTheme() const;

    Q_INVOKABLE void selectSection(const QString& section);
    Q_INVOKABLE bool openProject(const QString& filePath);
    Q_INVOKABLE bool createProject(const QString& templateId,
                                   const QString& name,
                                   const QString& filePath,
                                   bool useAsDefaultLocation,
                                   const QString& description = QString(),
                                   const QString& basemapUrl = QString(),
                                   bool overwriteExisting = false);
    Q_INVOKABLE bool isDefaultProjectLocation(const QString& projectFilePath) const;
    Q_INVOKABLE QString suggestedProjectFilePath(const QString& name) const;
    Q_INVOKABLE QString projectFilePath(const QString& name,
                                        const QString& directory) const;
    Q_INVOKABLE QString localFilePath(const QUrl& fileUrl) const;
    Q_INVOKABLE void saveTheme(const QString& theme);
    Q_INVOKABLE void requestApplicationExit();
    Q_INVOKABLE void openProjectCenter();
    Q_INVOKABLE void toggleFavorite(const QString& filePath);
    Q_INVOKABLE void removeRecentProject(const QString& filePath);
    Q_INVOKABLE void showProjectLocation(const QString& filePath);
    void requestOpenProjectFileDialog(bool transitionAlreadyConfirmed = false);
    void requestOpenEarthFileDialog();
    void requestSaveAsFileDialog(const QUrl& initialFile);
    Q_INVOKABLE void openProjectFileSelected(const QString& filePath);
    Q_INVOKABLE void submitSaveAsFile(const QString& filePath);
    Q_INVOKABLE void selectEarthFile(const QString& filePath);
    void reportError(const QString& title, const QString& message);
    void completeProjectOpen();
    void requestTransition(const QString& action, const QString& filePath = QString());
    Q_INVOKABLE void welcomeScreenDismissed();

signals:
    void projectsChanged();
    void activeSectionChanged();
    void recentBasemapsChanged();
    void projectActivated();
    void projectOpenRequested(const QString& filePath);
    void projectFileExists(const QString& filePath);
    void projectError(const QString& title, const QString& message);
    void openProjectFileDialogRequested(bool transitionAlreadyConfirmed);
    void openEarthFileDialogRequested();
    void saveAsFileDialogRequested(const QUrl& initialFile);
    void saveAsProjectRequested(const QString& filePath);
    void confirmedProjectFileSelected(const QString& filePath);
    void earthFileSelected(const QString& filePath);
    void currentThemeChanged();

private:
    void ensureWelcomeScreen();
    void handleProjectOpen(const QString& filePath);

    ProjectManager* m_projectManager = nullptr;
    QQmlApplicationEngine* m_engine = nullptr;
    QQuickWindow* m_window = nullptr;
    QString m_activeSection = QStringLiteral("Recent");
};
