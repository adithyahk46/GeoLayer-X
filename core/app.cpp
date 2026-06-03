#include "App.h"

App* App::_app = nullptr;

QMainWindow* App::_mainWindow = nullptr;

App::~App()
{
    _mapNode = nullptr;
    _manip =nullptr;
    _viewer = nullptr;
}
