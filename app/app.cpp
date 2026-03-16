#include "App.h"

App* App::_app = nullptr;

App::~App()
{
    _mapNode = nullptr;
    _manip =nullptr;
    _viewer = nullptr;
}
