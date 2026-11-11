#pragma once

#include <QString>
#include <QDateTime>

struct ProjectInfo
{
    QString name;
    QString filePath;
    QString templateId;
    QDateTime lastOpened;
    bool favorite = false;
};
