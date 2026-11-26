#pragma once

#include "ProjectInfo.h"

#include <QList>
#include <QString>

class RecentProjectsRepository
{
public:
    explicit RecentProjectsRepository(const QString& databasePath);
    ~RecentProjectsRepository();

    bool isValid() const;
    QList<ProjectInfo> recentProjects(int limit = 50) const;
    bool addOrUpdate(const ProjectInfo& project);
    bool remove(const QString& filePath);
    bool setFavorite(const QString& filePath, bool favorite);

private:
    QString m_connectionName;
    QString m_databasePath;
    bool m_valid = false;
};
