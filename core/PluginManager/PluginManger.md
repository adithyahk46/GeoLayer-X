all the files/classes in the PluginManager directory should export to PLUGINMANAGER_EXPORT
ex: 
```cpp
class PLUGINMANAGER_EXPORT PluginManager : public QObject
````
This is required for bundle the plugin manager directory into a single dll and use it in other projects.