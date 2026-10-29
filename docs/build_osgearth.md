git clone https://github.com/microsoft/vcpkg.git
cd vcpkg
./bootstrap-vcpkg.bat

edit the x64-windows triplet file to add the following lines:

set(osg_OPENGL_PROFILE GL3)

rerun the bootstrap script to apply the changes:
./bootstrap-vcpkg.bat

install osg in vcpkg using the following command:

vcpkg install osg[*]:x64-windows
or
vcpkg install osg[collada,core,docs,examples,fontconfig,freetype,nvtt,openexr,packages,plugins,sdl,tools]:x64-windows-static

this will install osg with all supported features and all its dependencies. You can also install osg with specific features by replacing the asterisk (*) with the desired features.

clone the osgEarth repository and build it using vcpkg:
git clone --recurse-submodules https://github.com/pelicanmapping/osgearth.git
cd osgearth