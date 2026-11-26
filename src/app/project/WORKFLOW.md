# GeoLayer-X project workflow

## Project files

GeoLayer-X projects are native osgEarth `.earth` files. They are XML map
configurations that osgEarth can open directly. Empty projects contain a named
`<map>` with no layers; basemap projects contain an `XYZImage` layer with its
tile URL and spherical-Mercator profile.

## Create a project

1. Choose **Empty Project**, a basemap tile, or **Add template** in the welcome
   screen.
2. Enter a project name and `.earth` save location in the separate
   `qml/newProject.qml` dialog.
   The dialog can remember the selected directory as the default for future
   project creation. It shows the chosen template and accepts an optional
   description stored with the project configuration.
3. GeoLayer-X writes the map configuration, records the file in Recent, and
   loads its layers into the `MapNode` registered with `ProjectManager`.
4. An empty map replaces the current map's layer stack with no layers. A basemap
   map replaces it with the selected XYZ image layer.

The four bundled basemap choices use the previews in `resources/images/tiles`.
The fifth basemap card opens a QML template picker for recent custom sources or
an XYZ URL using `{z}`, `{x}`, and `{y}` placeholders. Up to eight custom
sources are remembered.

The project center, transition dialogs, settings, and project file pickers are
implemented in QML. The welcome screen opens `qml/newProject.qml` for project
creation. `ProjectWelcomeScreen` in `app/project` coordinates those UI actions
with `ProjectManager`, which owns the registered map node and project
operations.

## Open and save

- Browse, Recent, and **Project > Open** accept `.earth` map files.
- Opening a project validates its XML map configuration, updates Recent, and
  loads its layers into the active map.
- **Save** serializes the live `MapNode` using osgEarth's `.earth` writer, so
  the current map options, extensions and layers are written as osgEarth XML.
- **Save As** writes that same live map to a new `.earth` path and makes the new
  path current.

The Recent list displays the full file path. Its menu can toggle favorites, show
the containing folder, or remove an entry from Recent without deleting the file.

## Project transitions

Before creating or opening another project, closing the current project, or
exiting the application, GeoLayer-X offers **Save**, **Discard**, and **Cancel**.
Cancel leaves the active project untouched. Save writes the live map before
continuing. Discard clears the current map's layer stack in place and closes its
project record; the `MapNode` instance itself remains attached to the viewer.
Closing a project returns to the modal project center. Canceling application exit
keeps the application open.

## Recent-project history

Recent-project history is stored separately from project files in SQLite:

`QStandardPaths::AppDataLocation/GeoLayer-X/recent_projects.sqlite`

The database stores each project path, name, template identifier, favorite
status, and last-opened timestamp. Qt SQL's SQLite driver (`QSQLITE`) must be
available at runtime.
