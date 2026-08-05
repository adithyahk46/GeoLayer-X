#include "App.h"


#include "App.h"

App* App::_app = nullptr;

QMainWindow* App::_mainWindow = nullptr;



App *App::getInstance(){
    if(!_app){
        _app = new App();
    }
    return _app;
}

App::~App()
{
    _mapNode = nullptr;
    _manip =nullptr;
    _viewer = nullptr;
}