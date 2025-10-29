#include "XYZServerManager.h"

#include <QTcpSocket>
#include <QDebug>

XYZServerManager::XYZServerManager(QObject* parent)
    : QObject(parent)
{
    //initialization code (if any)
}

XYZServerManager& XYZServerManager::instance() {
    static XYZServerManager instance;
    return instance;
}

int XYZServerManager::getOrStartServer(const QString& directory) {

    // Check for existing server
    if (m_servers.contains(directory)) {
        ServerInfo& info = m_servers[directory];
        info.refCount++;
        qDebug() << "[ServerManager] Reusing existing server. RefCount:" << info.refCount;
        return info.port;
    }

    // Find free port
    for (int port = 8000; port <= 8999; ++port) {
        // Skip ports in use by our servers
        bool portUsed = false;
        for (auto& server : m_servers) {
            if (server.port == port) {
                portUsed = true;
                break;
            }
        }
        if (portUsed) continue;

        // Check port availability
        QTcpSocket socket;
        socket.connectToHost("127.0.0.1", port);
        if (socket.waitForConnected(100)) {
            socket.disconnectFromHost();
            continue;
        }

        // Start new server
        QProcess* process = new QProcess(this);
        process->setWorkingDirectory(directory);
        process->start("python", QStringList() << "-m" << "http.server" << QString::number(port));

        if (!process->waitForStarted(3000)) {
            qDebug() << "[ServerManager] FAILED to start server! Error:" << process->errorString();
            delete process;
            continue;
        }

        qDebug() << "[ServerManager] Server started successfully! PID:" << process->processId();

        // Add to server list
        ServerInfo newServer{port, process, 1};
        m_servers[directory] = newServer;
        return port;
    }

    qDebug() << "[ServerManager] ERROR: No free ports available!";
    return -1;
}

void XYZServerManager::releaseServer(const QString& directory) {
    if (!m_servers.contains(directory)) {
        qDebug() << "[ServerManager] releaseServer: No server for directory:" << directory;
        return;
    }

    ServerInfo& server = m_servers[directory];
    server.refCount--;
    qDebug() << "[ServerManager] Releasing server. RefCount:" << server.refCount << "Directory:" << directory;

    if (server.refCount <= 0) {
        qDebug() << "[ServerManager] Stopping server process";
        if (server.process && server.process->state() == QProcess::Running) {
            server.process->terminate();
            if (!server.process->waitForFinished(3000)) {
                server.process->kill();
                server.process->waitForFinished();
            }
        }
        delete server.process;
        m_servers.remove(directory);
    }
}
