#include "ProjectManager.h"
#include "RecentProjectsRepository.h"

#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QStandardPaths>
#include <QXmlStreamReader>
#include <QXmlStreamWriter>

#include <osgDB/WriteFile>
#include <osgDB/Registry>
#include <osgEarth/MapNode>
#include <osgEarth/Layer>
#include <osgDB/ReadFile>

namespace
{
struct BasemapTemplate
{
    const char* id;
    const char* name;
    const char* url;
};

const BasemapTemplate sBasemapTemplates[] = {
    {"openstreetmap", "OpenStreetMap",
     "https://tile.openstreetmap.org/{z}/{x}/{y}.png"},
    {"satellite", "Satellite",
     "https://mt1.google.com/vt/lyrs=s&x={x}&y={y}&z={z}"},
    {"hybrid", "Hybrid",
     "https://mt1.google.com/vt/lyrs=y&x={x}&y={y}&z={z}"},
    {"esri", "Esri World Imagery",
     "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"}
};

QString templateName(const QString& templateId)
{
    for (const BasemapTemplate& basemap : sBasemapTemplates) {
        if (templateId == QLatin1String(basemap.id))
            return QString::fromLatin1(basemap.name);
    }
    return templateId == QStringLiteral("empty")
        ? QStringLiteral("Empty")
        : QStringLiteral("Custom XYZ");
}

QString templateUrl(const QString& templateId)
{
    for (const BasemapTemplate& basemap : sBasemapTemplates) {
        if (templateId == QLatin1String(basemap.id))
            return QString::fromLatin1(basemap.url);
    }
    return QString();
}

QString normalizedEarthPath(QString filePath)
{
    if (!filePath.endsWith(QStringLiteral(".earth"), Qt::CaseInsensitive))
        filePath += QStringLiteral(".earth");
    return filePath;
}
}

ProjectManager::ProjectManager(osgEarth::MapNode* mapNode, QObject* parent)
    : QObject(parent), m_mapNode(mapNode)
{
    const QString appData = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
    const QString databasePath =
        QDir(appData).filePath(QStringLiteral("GeoLayer-X/recent_projects.sqlite"));
    m_recentRepository = std::make_unique<RecentProjectsRepository>(databasePath);

    if (!m_recentRepository->isValid())
        emit errorOccurred(tr("Could not initialize the recent-project database."));
}

ProjectManager::~ProjectManager() = default;

void ProjectManager::setMapNode(osgEarth::MapNode* mapNode)
{
    m_mapNode = mapNode;
}

bool ProjectManager::createNewProject(const QString& name,
                                      const QString& projectFilePath,
                                      const QString& templateId,
                                      const QString&,
                                      const QString& basemapUrl,
                                      const QString& description)
{
    if (name.trimmed().isEmpty() || projectFilePath.trimmed().isEmpty()) {
        emit errorOccurred(tr("Project name and file path are required."));
        return false;
    }

    const QString path = normalizedEarthPath(projectFilePath);
    const QFileInfo info(path);
    if (!QDir().mkpath(info.absolutePath())) {
        emit errorOccurred(tr("Could not create the project directory."));
        return false;
    }

    const QString oldName = m_currentProjectName;
    const QString oldTemplateId = m_currentTemplateId;
    const QString oldBasemapUrl = m_currentBasemapUrl;
    const QString oldDescription = m_currentDescription;
    const QString oldPath = m_currentProjectPath;

    m_currentProjectName = name.trimmed();
    m_currentTemplateId = templateId;
    m_currentBasemapUrl = basemapUrl.isEmpty() ? templateUrl(templateId) : basemapUrl;
    m_currentDescription = description.trimmed();
    m_currentProjectPath = path;

    if (!writeProjectFile(path)) {
        m_currentProjectName = oldName;
        m_currentTemplateId = oldTemplateId;
        m_currentBasemapUrl = oldBasemapUrl;
        m_currentDescription = oldDescription;
        m_currentProjectPath = oldPath;
        return false;
    }

    if (!loadMap(path)) {
        m_currentProjectName = oldName;
        m_currentTemplateId = oldTemplateId;
        m_currentBasemapUrl = oldBasemapUrl;
        m_currentDescription = oldDescription;
        m_currentProjectPath = oldPath;
        return false;
    }

    updateRecentProjects();
    emit projectOpened(m_currentProjectPath);
    return true;
}

bool ProjectManager::openProject(const QString& projectFilePath)
{
    QString name;
    QString templateId;
    QString basemapUrl;
    QString description;
    if (!readProjectFile(projectFilePath, name, templateId, basemapUrl, description)) {
        emit errorOccurred(tr("Could not read project metadata from:\n%1\n\n"
                              "Select a valid osgEarth .earth file.")
                               .arg(QDir::toNativeSeparators(projectFilePath)));
        return false;
    }

    if (!loadMap(projectFilePath))
        return false;

    m_currentProjectName = name.isEmpty()
        ? QFileInfo(projectFilePath).completeBaseName()
        : name;
    m_currentTemplateId = templateId;
    m_currentBasemapUrl = basemapUrl;
    m_currentDescription = description;
    m_currentProjectPath = QFileInfo(projectFilePath).absoluteFilePath();
    updateRecentProjects();
    emit projectOpened(m_currentProjectPath);
    return true;
}

bool ProjectManager::saveProject()
{
    if (m_currentProjectPath.isEmpty()) {
        emit errorOccurred(tr("There is no open project. Use Save As first."));
        return false;
    }

    if (!writeMapNode(m_mapNode.get(), m_currentProjectPath))
        return false;

    updateRecentProjects();
    emit projectSaved(m_currentProjectPath);
    return true;
}

bool ProjectManager::saveProjectAs(const QString& newFilePath)
{
    if (m_currentProjectName.isEmpty() || newFilePath.trimmed().isEmpty()
        || !m_mapNode || !m_mapNode->getMap()) {
        emit errorOccurred(tr("There is no open project or the destination is empty."));
        return false;
    }

    const QString path = normalizedEarthPath(newFilePath);
    if (!writeMapNode(m_mapNode.get(), path))
        return false;

    m_currentProjectPath = path;
    updateRecentProjects();
    emit projectSaved(m_currentProjectPath);
    return true;
}

void ProjectManager::closeProject()
{
    if (m_mapNode && m_mapNode->getMap())
        m_mapNode->getMap()->clear();
    if (m_mapNode)
        m_mapNode->externalConfig() = osgEarth::Config();
    m_currentProjectName.clear();
    m_currentTemplateId = QStringLiteral("empty");
    m_currentBasemapUrl.clear();
    m_currentDescription.clear();
    m_currentProjectPath.clear();
    emit projectClosed();
}

bool ProjectManager::openEarthFile(const QString& filePath)
{
    return loadMap(filePath);
}

void ProjectManager::requestNewProject()
{
    requestProjectTransition(QStringLiteral("new"));
}

void ProjectManager::requestOpenProject()
{
    requestProjectTransition(QStringLiteral("openFile"));
}

void ProjectManager::requestProjectTransition(const QString& action,
                                              const QString& filePath)
{
    emit projectTransitionRequested(action, filePath);
}

void ProjectManager::requestSaveAs()
{
    if (!hasOpenProject()) {
        emit errorOccurred(tr("There is no open project to save."));
        return;
    }
    emit saveAsRequested();
}

void ProjectManager::requestCloseProject()
{
    requestProjectTransition(QStringLiteral("close"));
}

void ProjectManager::requestOpenEarthFile()
{
    emit openEarthFileRequested();
}

void ProjectManager::requestApplicationExit()
{
    requestProjectTransition(QStringLiteral("exit"));
}

bool ProjectManager::loadMap(const QString& filePath)
{
    if (!m_mapNode || !m_mapNode->getMap()) {
        emit errorOccurred(tr("The map is unavailable."));
        return false;
    }

    const std::string nativePath = QFile::encodeName(filePath).toStdString();
    osgDB::ReaderWriter::ReadResult readResult =
        osgDB::Registry::instance()->readNode(nativePath, nullptr);
    osg::ref_ptr<osg::Node> node = readResult.getNode();
    osgEarth::MapNode* source = node ? osgEarth::MapNode::findMapNode(node) : nullptr;
    if (!source || !source->getMap()) {
        QString detail = QString::fromStdString(readResult.message());
        if (detail.isEmpty()) {
            if (readResult.notFound())
                detail = tr("The file could not be found or read.");
            else if (readResult.notHandled())
                detail = tr("No osgDB reader recognized this file format.");
            else
                detail = tr("The file did not contain a valid osgEarth map.");
        }
        emit errorOccurred(tr("Could not load the earth file:\n%1\n\n%2")
                               .arg(QDir::toNativeSeparators(filePath), detail));
        return false;
    }

    osgEarth::LayerVector layers;
    source->getMap()->getLayers(layers);
    osgEarth::Map* map = m_mapNode->getMap();
    // map->clear();
    for (osgEarth::Layer* layer : layers) {
        if (layer) {
            layer->open();
            map->addLayer(layer);
        }
    }

    osgEarth::Config externalConfig = source->externalConfig();
    const osgEarth::Config sourceConfig = source->getConfig();
    for (const osgEarth::Config& child : sourceConfig.children()) {
        if (child.key() == "viewpoints")
            externalConfig.set(child);
    }
    m_mapNode->externalConfig() = externalConfig;

    return true;
}

bool ProjectManager::hasOpenProject() const
{
    return !m_currentProjectName.isEmpty() && !m_currentProjectPath.isEmpty();
}

QString ProjectManager::currentProjectPath() const
{
    return m_currentProjectPath;
}

QString ProjectManager::currentProjectName() const
{
    return m_currentProjectName;
}

QString ProjectManager::currentTemplateId() const
{
    return m_currentTemplateId;
}

QList<ProjectInfo> ProjectManager::recentProjects() const
{
    return m_recentRepository ? m_recentRepository->recentProjects() : QList<ProjectInfo>();
}

bool ProjectManager::removeFromRecent(const QString& projectFilePath)
{
    return m_recentRepository && m_recentRepository->remove(projectFilePath);
}

bool ProjectManager::setFavorite(const QString& projectFilePath, bool favorite)
{
    return m_recentRepository && m_recentRepository->setFavorite(projectFilePath, favorite);
}

bool ProjectManager::writeProjectFile(const QString& filePath)
{
    const QFileInfo info(filePath);
    if (!QDir().mkpath(info.absolutePath())) {
        emit errorOccurred(tr("Could not create the project directory."));
        return false;
    }

    QByteArray contents;
    QXmlStreamWriter writer(&contents);
    writer.setAutoFormatting(true);
    writer.writeStartDocument();
    writer.writeStartElement(QStringLiteral("map"));
    writer.writeAttribute(QStringLiteral("name"), m_currentProjectName);
    if (!m_currentDescription.isEmpty())
        writer.writeAttribute(QStringLiteral("description"), m_currentDescription);

    if (!m_currentBasemapUrl.isEmpty()) {
        writer.writeStartElement(QStringLiteral("XYZImage"));
        writer.writeAttribute(QStringLiteral("name"), templateName(m_currentTemplateId));
        writer.writeTextElement(QStringLiteral("url"), m_currentBasemapUrl);
        writer.writeTextElement(QStringLiteral("profile"), QStringLiteral("spherical-mercator"));
        writer.writeEndElement();
    }

    writer.writeEndElement();
    writer.writeEndDocument();
    if (writer.hasError()) {
        emit errorOccurred(tr("Could not generate the project file."));
        return false;
    }

    QFile file(filePath);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
        emit errorOccurred(tr("Could not write project file: %1").arg(file.errorString()));
        return false;
    }
    if (file.write(contents) != contents.size() || file.error() != QFile::NoError) {
        emit errorOccurred(tr("Could not finish writing the project file."));
        return false;
    }
    return true;
}

bool ProjectManager::writeMapNode(osgEarth::MapNode* mapNode, const QString& filePath)
{
    if (!mapNode || !mapNode->getMap()) {
        emit errorOccurred(tr("Could not save the project because its map is unavailable."));
        return false;
    }

    const QFileInfo info(filePath);
    if (!QDir().mkpath(info.absolutePath())) {
        emit errorOccurred(tr("Could not create the project directory."));
        return false;
    }

    const std::string nativePath = QFile::encodeName(filePath).toStdString();
    if (!osgDB::writeNodeFile(*mapNode, nativePath)) {
        emit errorOccurred(tr("Could not serialize the current map to the project file."));
        return false;
    }
    return true;
}

bool ProjectManager::readProjectFile(const QString& filePath,
                                    QString& name,
                                    QString& templateId,
                                    QString& basemapUrl,
                                    QString& description) const
{
    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
        return false;

    QXmlStreamReader reader(&file);
    if (!reader.readNextStartElement() || reader.name() != QStringLiteral("map"))
        return false;

    name = reader.attributes().value(QStringLiteral("name")).toString();
    description = reader.attributes().value(QStringLiteral("description")).toString();
    templateId = QStringLiteral("empty");
    basemapUrl.clear();

    while (reader.readNextStartElement() && !reader.hasError()) {
        if (reader.name() == QStringLiteral("options")) {
            while (reader.readNextStartElement()) {
                if (reader.name() == QStringLiteral("name"))
                    name = reader.readElementText().trimmed();
                else
                    reader.skipCurrentElement();
            }
            continue;
        }

        if (reader.name() != QStringLiteral("XYZImage")) {
            reader.skipCurrentElement();
            continue;
        }

        while (reader.readNextStartElement()) {
            if (reader.name() == QStringLiteral("url"))
                basemapUrl = reader.readElementText().trimmed();
            else
                reader.skipCurrentElement();
        }

        if (!basemapUrl.isEmpty()) {
            templateId = QStringLiteral("custom");
            for (const BasemapTemplate& basemap : sBasemapTemplates) {
                if (basemapUrl == QLatin1String(basemap.url)) {
                    templateId = QString::fromLatin1(basemap.id);
                    break;
                }
            }
        }
    }

    // osgEarth's earth reader accepts constructs that a strict XML metadata
    // parser may reject, such as xi:include elements with undeclared prefixes.
    // The osgDB read in loadMap is the authoritative validation step.
    return true;
}

void ProjectManager::updateRecentProjects()
{
    if (!m_recentRepository || m_currentProjectPath.isEmpty())
        return;

    ProjectInfo info;
    info.name = currentProjectName();
    info.filePath = QFileInfo(m_currentProjectPath).absoluteFilePath();
    info.templateId = currentTemplateId();
    m_recentRepository->addOrUpdate(info);
}
