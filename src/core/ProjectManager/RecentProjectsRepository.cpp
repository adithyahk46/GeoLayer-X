#include "RecentProjectsRepository.h"

#include <QDir>
#include <QFileInfo>
#include <QSqlDatabase>
#include <QSqlError>
#include <QSqlQuery>
#include <QVariant>
#include <QUuid>

RecentProjectsRepository::RecentProjectsRepository(const QString& databasePath)
    : m_connectionName(QStringLiteral("glx_recent_%1")
                           .arg(QUuid::createUuid().toString(QUuid::WithoutBraces))),
      m_databasePath(databasePath)
{
    const QFileInfo info(databasePath);
    if (!QDir().mkpath(info.absolutePath()))
        return;

    QSqlDatabase db = QSqlDatabase::addDatabase(QStringLiteral("QSQLITE"), m_connectionName);
    db.setDatabaseName(databasePath);

    if (!db.open())
        return;

    QSqlQuery query(db);
    m_valid = query.exec(QStringLiteral(
        "CREATE TABLE IF NOT EXISTS recent_projects ("
        "file_path TEXT PRIMARY KEY NOT NULL,"
        "name TEXT NOT NULL,"
        "template_id TEXT NOT NULL DEFAULT 'empty',"
        "favorite INTEGER NOT NULL DEFAULT 0,"
        "last_opened TEXT NOT NULL)"));

    if (!m_valid)
        return;

    QSqlQuery columns(db);
    if (!columns.exec(QStringLiteral("PRAGMA table_info(recent_projects)"))) {
        m_valid = false;
        return;
    }

    bool hasFavoriteColumn = false;
    while (columns.next()) {
        if (columns.value(1).toString() == QStringLiteral("favorite")) {
            hasFavoriteColumn = true;
            break;
        }
    }
    if (!hasFavoriteColumn) {
        QSqlQuery migrate(db);
        m_valid = migrate.exec(QStringLiteral(
            "ALTER TABLE recent_projects ADD COLUMN favorite INTEGER NOT NULL DEFAULT 0"));
    }
}

RecentProjectsRepository::~RecentProjectsRepository()
{
    if (QSqlDatabase::contains(m_connectionName)) {
        {
            QSqlDatabase db = QSqlDatabase::database(m_connectionName);
            db.close();
        }
        QSqlDatabase::removeDatabase(m_connectionName);
    }
}

bool RecentProjectsRepository::isValid() const
{
    return m_valid;
}

QList<ProjectInfo> RecentProjectsRepository::recentProjects(int limit) const
{
    QList<ProjectInfo> projects;
    if (!m_valid)
        return projects;

    const QSqlDatabase db = QSqlDatabase::database(m_connectionName);
    QSqlQuery query(db);
    query.prepare(QStringLiteral(
        "SELECT name, file_path, template_id, last_opened, favorite "
        "FROM recent_projects ORDER BY last_opened DESC LIMIT :limit"));
    query.bindValue(QStringLiteral(":limit"), qMax(1, limit));

    if (!query.exec())
        return projects;

    while (query.next()) {
        ProjectInfo info;
        info.name = query.value(0).toString();
        info.filePath = query.value(1).toString();
        info.templateId = query.value(2).toString();
        info.lastOpened = QDateTime::fromString(query.value(3).toString(), Qt::ISODate);
        info.favorite = query.value(4).toBool();
        projects.push_back(info);
    }
    return projects;
}

bool RecentProjectsRepository::addOrUpdate(const ProjectInfo& project)
{
    if (!m_valid || project.filePath.isEmpty())
        return false;

    const QSqlDatabase db = QSqlDatabase::database(m_connectionName);
    QSqlQuery query(db);
    query.prepare(QStringLiteral(
        "INSERT INTO recent_projects(file_path, name, template_id, favorite, last_opened) "
        "VALUES(:path, :name, :template, :favorite, :opened) "
        "ON CONFLICT(file_path) DO UPDATE SET "
        "name=excluded.name, template_id=excluded.template_id, "
        "favorite=MAX(recent_projects.favorite, excluded.favorite), "
        "last_opened=excluded.last_opened"));
    query.bindValue(QStringLiteral(":path"), project.filePath);
    query.bindValue(QStringLiteral(":name"), project.name);
    query.bindValue(QStringLiteral(":template"), project.templateId);
    query.bindValue(QStringLiteral(":favorite"), project.favorite);
    query.bindValue(QStringLiteral(":opened"),
                    QDateTime::currentDateTimeUtc().toString(Qt::ISODate));
    return query.exec();
}

bool RecentProjectsRepository::setFavorite(const QString& filePath, bool favorite)
{
    if (!m_valid)
        return false;

    const QSqlDatabase db = QSqlDatabase::database(m_connectionName);
    QSqlQuery query(db);
    query.prepare(QStringLiteral(
        "UPDATE recent_projects SET favorite=:favorite WHERE file_path=:path"));
    query.bindValue(QStringLiteral(":favorite"), favorite);
    query.bindValue(QStringLiteral(":path"), filePath);
    return query.exec() && query.numRowsAffected() == 1;
}

bool RecentProjectsRepository::remove(const QString& filePath)
{
    if (!m_valid)
        return false;

    const QSqlDatabase db = QSqlDatabase::database(m_connectionName);
    QSqlQuery query(db);
    query.prepare(QStringLiteral("DELETE FROM recent_projects WHERE file_path=:path"));
    query.bindValue(QStringLiteral(":path"), filePath);
    return query.exec();
}
