#include "App.h"

App* App::_app = nullptr;
QMainWindow* App::_mainWindow = nullptr;
OSGViewerWidget* App::_threeDWindow = nullptr;
OSGViewerWidget* App::_2DWindow = nullptr;


App* App::getInstance()
{
    if (!_app)
    {
        _app = new App();
    }
    return _app;
}

App::~App() = default;