#pragma once

#include "ProjectInfo.h"

#include <QObject>
#include <QList>
#include <memory>
#include <osg/ref_ptr>
#include <osgEarth/MapNode>
#include "osgearthcore_global.h"

class RecentProjectsRepository;

class OSGEARTHCORE_EXPORT ProjectManager : public QObject
{
    Q_OBJECT

public:
    explicit ProjectManager(osgEarth::MapNode* mapNode, QObject* parent = nullptr);
    ~ProjectManager() override;

    void setMapNode(osgEarth::MapNode* mapNode);

    bool createNewProject(const QString& name,
                          const QString& projectFilePath,
                          const QString& templateId = QStringLiteral("empty"),
                          const QString& crs = QStringLiteral("EPSG:4326"),
                          const QString& basemapUrl = QString(),
                          const QString& description = QString());

    bool openProject(const QString& projectFilePath);
    bool saveProjectAs(const QString& newFilePath);
    bool openEarthFile(const QString& filePath);
    void closeProject();

    bool hasOpenProject() const;
    QString currentProjectPath() const;
    QString currentProjectName() const;
    QString currentTemplateId() const;
    QList<ProjectInfo> recentProjects() const;
    bool removeFromRecent(const QString& projectFilePath);
    bool setFavorite(const QString& projectFilePath, bool favorite);

public slots:
    bool saveProject();
    void requestNewProject();
    void requestOpenProject();
    void requestProjectTransition(const QString& action,
                                 const QString& filePath = QString());
    void requestSaveAs();
    void requestCloseProject();
    void requestOpenEarthFile();
    void requestApplicationExit();

signals:
    void projectOpened(const QString& projectFilePath);
    void projectClosed();
    void projectSaved(const QString& projectFilePath);
    void errorOccurred(const QString& message);
    void projectTransitionRequested(const QString& action, const QString& filePath);
    void saveAsRequested();
    void openEarthFileRequested();

private:
    bool loadMap(const QString& filePath);
    bool writeProjectFile(const QString& filePath);
    bool writeMapNode(osgEarth::MapNode* mapNode, const QString& filePath);
    bool readProjectFile(const QString& filePath,
                         QString& name,
                         QString& templateId,
                         QString& basemapUrl,
                         QString& description) const;
    void updateRecentProjects();

    QString m_currentProjectName;
    QString m_currentTemplateId = QStringLiteral("empty");
    QString m_currentBasemapUrl;
    QString m_currentDescription;
    QString m_currentProjectPath;
    osg::ref_ptr<osgEarth::MapNode> m_mapNode;
    std::unique_ptr<RecentProjectsRepository> m_recentRepository;
};
