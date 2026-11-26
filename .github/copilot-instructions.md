# Copilot instructions

## Project overview

GeoLayer-X is a C++17 desktop mapping application built with Qt, OpenSceneGraph
(OSG), osgEarth, and GDAL. CMake configures Qt's automatic MOC, UIC, and RCC
steps. The top-level build produces the Windows launcher, the `glx_app` and
`glx_core` libraries, and plugin modules.

## Repository layout

- `src/app/` contains application startup, the main window, dock widgets, QML,
  and UI-facing integrations.
- `src/core/` contains shared map/viewer, data, project, settings, plugin
  management, and drawing logic.
- `src/plugins/` contains the Viewshed Analysis, Radial Viewshed Analysis, and
  Viewshed By Extent plugins. Each plugin has its own CMake target.
- `src/ui/` contains Qt Designer `.ui` forms; generated UI headers belong in the
  build directory, not in source.
- `src/customWidgets/` contains reusable Qt widgets.
- `resources/` contains Qt resources and application assets.
- `cmake/` contains CMake support files; `docs/` contains build and deployment
  notes.
- `build/` and `deployment/` are generated build/runtime output directories;
  do not commit generated files from them.

## Coding conventions

- Keep changes focused and follow the style of the surrounding file; formatting
  and naming are not completely uniform across the repository.
- Use the existing C++17, Qt, and OSG patterns. Prefer typed Qt signal/slot
  connections, Qt parent ownership for `QObject`s, and OSG `ref_ptr` or
  `observer_ptr` according to ownership.
- Preserve existing plugin interfaces and CMake target/link relationships.
- Edit source `.ui`, `.qml`, and resource files rather than generated build
  outputs.
- Keep machine-specific paths and credentials out of commits. The top-level
  `CMakeLists.txt` currently appends Windows-local dependency prefixes; on a
  different machine, configure valid package locations through CMake rather
  than committing local path changes.

## Build

Build on Windows with an x64 Visual Studio developer environment, CMake, Ninja,
and the required Qt, GDAL, OSG, and osgEarth development packages available.
The project prefers Qt 6 when available and falls back to Qt 5.

From the repository root, configure and build a Release tree:

```powershell
cmake -S . -B build\Release -G Ninja -DCMAKE_BUILD_TYPE=Release `
  -DCMAKE_PREFIX_PATH="<Qt-prefix>;<vcpkg-prefix>;<osgEarth-prefix>"
cmake --build build\Release --parallel 4
```

Set `CMAKE_PREFIX_PATH` to the prefixes containing the relevant CMake package
configurations for the installed dependencies. The optional 2D map view is
disabled by default; enable it at configure time with
`-DGLX_BUILD_2D_MAP=ON`.

## Run

The Windows launcher is written to `deployment\bin\GeoLayer-X.exe`. Run it from
the repository root with:

```powershell
.\deployment\bin\GeoLayer-X.exe
```

The launcher reads `glx.env` beside the executable and loads `glx_app.dll`;
keep the deployment runtime files and expected `deployment` directory layout
intact. Add `--debug` when launching from a console to enable console logging.

## Tests and validation

There is currently no configured CTest suite or Qt Test target in this
repository. Use the CMake build above as the primary compile/link validation.
If adding tests, follow the existing CMake target structure and document the
test command with the change.
