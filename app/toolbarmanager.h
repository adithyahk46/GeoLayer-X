#ifndef TOOLBARMANAGER_H
#define TOOLBARMANAGER_H
\
#include <QObject>
#include <QToolBar>


class ToolBarManager : public QObject
{
    Q_OBJECT

public:
    static ToolBarManager *instance();

    ~ToolBarManager();


private:
    ToolBarManager();

    static ToolBarManager* _instance;

    QToolBar* _projectBar = nullptr;
};

#endif // TOOLBARMANAGER_H
