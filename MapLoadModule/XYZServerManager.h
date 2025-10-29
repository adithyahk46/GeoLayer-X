#ifndef XYZSERVERMANAGER_H
#define XYZSERVERMANAGER_H

#include <QObject>
#include <QProcess>
#include <QMap>
#include <osgEarth/ImageLayer>


class XYZServerManager: public QObject
{
    Q_OBJECT

public:
    struct ServerInfo {
        int port;
        QProcess* process;
        int refCount;
    };

    static XYZServerManager& instance();

    int getOrStartServer(const QString& directory);
    void releaseServer(const QString& directory);

private:
    XYZServerManager(QObject* parent = nullptr);
    QMap<QString, ServerInfo> m_servers;  // Key: directory path

};

#endif // XYZSERVERMANAGER_H
