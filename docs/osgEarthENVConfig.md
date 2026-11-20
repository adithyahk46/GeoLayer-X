This document guides the required or essensials or all the env configuration for osgEarth deployment

set "GLX_ROOT=C:\Users\pnmt1054\Adithya_working_directory\osgearth_dev\GeoLayer-X\deployment\Release"

set "GDAL_DATA=%GLX_ROOT%\share\gdal"
set "GDAL_DRIVER_PATH=%GLX_ROOT%\bin\gdalplugins"
set "GDAL_FILENAME_IS_UTF8=YES"
set "PROJ_LIB=%GLX_ROOT%\share\proj"
set "PROJ_DATA=%GLX_ROOT%\share\proj"
set "QT_PLUGIN_PATH=%GLX_ROOT%\apps\qt\plugins"
set "OSG_NOTIFY_LEVEL=DEBUG"
set "OSGEARTH_NOTIFY_LEVEL=INFO"
set "OSG_FILE_PATH="%GLX_ROOT%\data\osg;%GLX_ROOT%\data\osgearth;%GLX_ROOT%\share\osgearth\shaders"
set "OSGEARTH_USE_GL4="
set "OSGEARTH_USE_NVGL="
