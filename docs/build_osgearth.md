git clone https://github.com/microsoft/vcpkg.git
cd vcpkg
git pull
./bootstrap-vcpkg.bat
./vcpkg integrate install
./vcpkg install icu[*]:x64-windows
./vcpkg install doxygen[*]:x64-windows

set VCPKG_ROOT=C:\path\to\vcpkg

edit the x64-windows triplet file to add the following lines:

set(osg_OPENGL_PROFILE GL3)

rerun the bootstrap script to apply the changes:
./bootstrap-vcpkg.bat

install osg in vcpkg using the following command:

vcpkg install osg[*]:x64-windows
or
vcpkg install osg[collada,core,docs,examples,fontconfig,freetype,nvtt,openexr,packages,plugins,sdl,tools]:x64-windows-static

vcpkg.exe install basisu:x64-windows


#build cesium-native
cd ..
git clone https://github.com/CesiumGS/cesium-native.git --recurse-submodules
cd cesium-native
cp doc/cmake-presets/CMakeUserPresets.json .
cmake --preset=vcpkg-windows-vs -DICU_INCLUDE_DIR=C:\vcpkg\installed\x64-windows\include -DCMAKE_INSTALL_PREFIX=./install -DCMAKE_DEBUG_POSTFIX="d" --fresh
or
cmake -B build -S . -G "Visual Studio 17 2022" -DCMAKE_TOOLCHAIN_FILE=$${VCPKG_ROOT}/scripts/buildsystems/vcpkg.cmake -DCESIUM_USE_EZVCPG=OFF -DCMAKE_INSTALL_PREFIX=./install -DICU_INCLUDE_DIR=C:\vcpkg\installed\x64-windows\include -DVCPKG_TRIPLET=x64-windows -DVCPKG_TARGET_TRIPLET=x64-windows

cmake --build build --config Debug --parallel
cmake --build build --config Release --parallel

clone the osgEarth repository and build it using vcpkg:
git clone --recurse-submodules https://github.com/adithyahk46/osgearth.git
cd osgearth

set(CMAKE_PREFIX_PATH "C:/vcpkg/installed/x64-windows/share;C:/vcpkg/installed/x64-windows;C:/Users/pnmt1054/Adithya_working_directory/QT_PROJECTS/INDIGIS_DEPS/cesium-native/install/share;C:/Users/pnmt1054/Adithya_working_directory/QT_PROJECTS/INDIGIS_DEPS/cesium-native/install/Debug/share;C:/Users/pnmt1054/Adithya_working_directory/QT_PROJECTS/INDIGIS_DEPS/cesium-native/build/vcpkg_installed/x64-windows-static-md" ${CMAKE_PREFIX_PATH})

set(CESIUM_NATIVE_DIR "C:/Users/pnmt1054/Adithya_working_directory/QT_PROJECTS/INDIGIS_DEPS/cesium-native/install" CACHE PATH "Cesium Native install directory")

cmake -B build/Release -S . -G "Visual Studio 17 2022" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=./install/Release

OSGEARTH_CESIUMION_KEY
eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJqdGkiOiI3MjQwOGQyYS1mNzg0LTQ3YWMtOWNiZS05MmRlYzRkNDA5MGEiLCJpZCI6MzYzMjg5LCJpYXQiOjE3NjM5NzU1ODF9.JELgd7EG1Mq-HvZ5Y4XAuvFlppq4fZ275LuLC_EoEag


PROJ_LIB=D:\OSG1\vcpkg\installed\x64-windows\share\proj