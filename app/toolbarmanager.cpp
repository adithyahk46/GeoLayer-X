#include "toolbarmanager.h"
#include"app/app.h"

#include <QAction>

ToolBarManager* ToolBarManager::_instance = nullptr;


ToolBarManager *ToolBarManager::instance()
{
    if(!_instance){
        _instance = new ToolBarManager;
    }
    return _instance;
}

ToolBarManager::~ToolBarManager()
{

}

ToolBarManager::ToolBarManager()
{
    _projectBar = new QToolBar( "Project Toolbar");
    App::getMainwindow()->addToolBar(Qt::TopToolBarArea, _projectBar);
    // QToolBar* existingToolbar = App::getMainwindow()->findChild<QToolBar*>("existingToolbarName");

    // if (existingToolbar)
    // {
    //     mainWindow->insertToolBar(existingToolbar, _projectBar);
    // }
    // else
    // {
    //     mainWindow->addToolBar(Qt::TopToolBarArea, _projectBar);
    // }

    QAction* _newProjectAction = new QAction(QIcon(":/new/prefix1/resources/icons/project/newfolder.png"),"New Project", this );
    QAction* openAction = new QAction(QIcon(":/new/prefix1/resources/icons/project/open_folder.png"), "Open Project",this);
    QAction* saveAction = new QAction(QIcon(":/new/prefix1/resources/icons/project/save_project.png"), "Save",this);
    QAction* _printAction = new QAction(QIcon(":/new/prefix1/resources/icons/project/print.png"), "Print Layout",this);

    connect(_newProjectAction, &QAction::triggered, this, [](){
        qDebug() << "New Project clicked";
    });

    connect(openAction, &QAction::triggered, this, [](){
        qDebug() << "Open project  clicked";
    });

    connect(saveAction, &QAction::triggered, this, [](){
        qDebug() << "Save clicked";
    });
    connect(_printAction, &QAction::triggered, this, [](){
        qDebug() << "print Action clicked";
    });



    _projectBar->addAction(openAction);
    _projectBar->addAction(_newProjectAction);
    // _projectBar->addSeparator();
    _projectBar->addAction(saveAction);
    _projectBar->addAction(_printAction);
}
