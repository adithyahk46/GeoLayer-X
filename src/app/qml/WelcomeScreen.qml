import QtQuick
import QtQuick.Window
import QtQuick.Controls
import QtQuick.Dialogs

pragma ComponentBehavior: Bound

Window {
    id: window

    property var projectController: null
    property bool closingForProject: false
    property bool fileDialogTransitionConfirmed: false
    property var newProjectDialog: null
    property bool darkTheme: projectController
                                 ? projectController.currentTheme === "dark" : false
    property color pageColor: darkTheme ? "#000000" : "#f5f5f5"
    property color surfaceColor: darkTheme ? "#101010" : "#ffffff"
    property color borderColor: darkTheme ? "#252525" : "#d0d0d0"
    property color primaryTextColor: darkTheme ? "#e5e5e5" : "#202020"
    property color secondaryTextColor: darkTheme ? "#b9b9b9" : "#555555"
    property color dividerColor: darkTheme ? "#191919" : "#d8d8d8"
    property color hoverColor: darkTheme ? "#171717" : "#eeeeee"
    property color accentColor: darkTheme ? "#00d66b" : "#08783e"

    flags: Qt.Dialog | Qt.CustomizeWindowHint | Qt.WindowTitleHint | Qt.WindowCloseButtonHint
    modality: Qt.ApplicationModal
    width: 960
    height: 640
    minimumWidth: 760
    minimumHeight: 520
    visible: false
    title: "GeoLayer-X"
    color: pageColor

    onClosing: function(close) {
        if (!closingForProject) {
            close.accepted = false
            projectController.requestApplicationExit()
        }
    }

    Connections {
        target: window.projectController
        function onProjectActivated() {
            window.closingForProject = true
            window.close()
        }
        function onProjectError(title, message) {
            errorDialog.title = title
            errorDialog.text = message
            errorDialog.open()
        }
    }

    function beginProjectCreation(templateId, basemapUrl) {
        if (!newProjectDialog) {
            const component = Qt.createComponent(
                        Qt.resolvedUrl("newProject.qml"), Component.PreferSynchronous)
            if (component.status !== Component.Ready) {
                errorDialog.title = "New project"
                errorDialog.text = component.errorString()
                errorDialog.open()
                return
            }
            newProjectDialog = component.createObject(null, {
                projectController: window.projectController
            })
            if (!newProjectDialog) {
                errorDialog.title = "New project"
                errorDialog.text = "Could not create the new-project dialog."
                errorDialog.open()
                return
            }
            newProjectDialog.transientParent = window
        }
        newProjectDialog.openForTemplate(templateId, basemapUrl)
    }

    function openCustomBasemap(name, url) {
        recentBasemapDialog.close()
        beginProjectCreation("custom:" + name, url)
    }

    function dismissProjectCenter() {
        closingForProject = true
        close()
        projectController.welcomeScreenDismissed()
    }

    FileDialog {
        id: openProjectFileDialog
        title: "Open GeoLayer-X Project"
        fileMode: FileDialog.OpenFile
        nameFilters: ["osgEarth Projects (*.earth)", "All files (*)"]
        currentFolder: window.projectController.defaultProjectLocationUrl
        onAccepted: {
            const filePath = window.projectController.localFilePath(selectedFile)
            if (window.fileDialogTransitionConfirmed)
                window.projectController.openProjectFileSelected(filePath)
            else
                window.projectController.openProject(filePath)
            window.fileDialogTransitionConfirmed = false
        }
        onRejected: {
            if (window.fileDialogTransitionConfirmed)
                window.dismissProjectCenter()
            window.fileDialogTransitionConfirmed = false
        }
    }

    FileDialog {
        id: saveAsProjectFileDialog
        title: "Save GeoLayer-X Project As"
        fileMode: FileDialog.SaveFile
        nameFilters: ["osgEarth Projects (*.earth)"]
        currentFolder: window.projectController.defaultProjectLocationUrl
        onAccepted: window.projectController.submitSaveAsFile(
                        window.projectController.localFilePath(selectedFile))
        onRejected: window.dismissProjectCenter()
    }

    FileDialog {
        id: openEarthFileDialog
        title: "Open Earth File"
        fileMode: FileDialog.OpenFile
        nameFilters: ["Earth Files (*.earth)", "All files (*)"]
        currentFolder: window.projectController.defaultProjectLocationUrl
        onAccepted: window.projectController.selectEarthFile(
                        window.projectController.localFilePath(selectedFile))
        onRejected: window.dismissProjectCenter()
    }

    Connections {
        target: window.projectController
        function onOpenProjectFileDialogRequested(transitionAlreadyConfirmed) {
            window.fileDialogTransitionConfirmed = transitionAlreadyConfirmed
            openProjectFileDialog.open()
        }
        function onOpenEarthFileDialogRequested() {
            openEarthFileDialog.open()
        }
        function onSaveAsFileDialogRequested(initialFile) {
            saveAsProjectFileDialog.currentFile = initialFile
            saveAsProjectFileDialog.open()
        }
    }

    Dialog {
        id: errorDialog
        property string text: ""
        title: "GeoLayer-X"
        modal: true
        width: 440
        parent: Overlay.overlay
        standardButtons: Dialog.Ok
        contentItem: Label {
            width: 400
            text: errorDialog.text
            wrapMode: Text.WordWrap
        }
    }

    Dialog {
        id: recentBasemapDialog
        title: "Basemap templates"
        modal: true
        standardButtons: Dialog.Cancel
        width: 440
        parent: Overlay.overlay

        contentItem: Column {
            width: 392
            spacing: 8
            Label { text: "Recently used XYZ templates" }
            Repeater {
                model: window.projectController.recentBasemaps
                delegate: Button {
                    required property var modelData
                    width: 392
                    text: modelData.name
                    onClicked: window.openCustomBasemap(modelData.name, modelData.url)
                }
            }
            Button {
                text: "Add XYZ tile source..."
                onClicked: {
                    recentBasemapDialog.close()
                    customBasemapDialog.open()
                }
            }
        }
    }

    Dialog {
        id: customBasemapDialog
        title: "Add XYZ tile source"
        modal: true
        standardButtons: Dialog.Ok | Dialog.Cancel
        width: 460
        parent: Overlay.overlay
        onAccepted: {
            const name = customBasemapName.text.trim()
            const url = customBasemapUrl.text.trim()
            if (name.length === 0 || !url.toLowerCase().startsWith("http")
                    || !url.includes("{z}") || !url.includes("{x}") || !url.includes("{y}")) {
                errorDialog.title = "XYZ tile source"
                errorDialog.text = "Enter a template name and an HTTP URL containing {z}, {x}, and {y}."
                errorDialog.open()
                customBasemapDialog.open()
                return
            }
            window.beginProjectCreation("custom:" + name, url)
        }
        contentItem: Column {
            width: 412
            spacing: 10
            Label { text: "Template name" }
            TextField {
                id: customBasemapName
                width: 412
                placeholderText: "For example, My imagery"
            }
            Label { text: "XYZ tile URL" }
            TextField {
                id: customBasemapUrl
                width: 412
                placeholderText: "https://server/{z}/{x}/{y}.png"
            }
        }
    }

    Dialog {
        id: settingsDialog
        title: "GeoLayer-X settings"
        modal: true
        standardButtons: Dialog.Save | Dialog.Cancel
        width: 440
        parent: Overlay.overlay
        onOpened: themeSelector.currentIndex =
                      window.projectController.currentTheme === "dark" ? 1 : 0
        onAccepted: window.projectController.saveTheme(
                        themeSelector.currentIndex === 1 ? "dark" : "light")
        contentItem: Column {
            width: 392
            spacing: 10
            Label { text: "Application theme" }
            ComboBox {
                id: themeSelector
                width: 392
                model: ["Light", "Dark"]
            }
            Label {
                text: "Use the Project menu to open or save GeoLayer-X projects."
                wrapMode: Text.WordWrap
            }
        }
    }

    component IconText: Text {
        property string iconName: "map"
        property color iconColor: window.darkTheme ? "#cccccc" : "#666666"
        text: {
            switch (iconName) {
            case "recent": return "◷"
            case "browse": return "▱"
            case "favorites": return "☆"
            case "manage": return "⌫"
            case "empty_project": return "⊞"
            case "globe": return "◎"
            case "terrain": return "⌁"
            case "map": return "▤"
            default: return "•"
            }
        }
        font.pixelSize: 18
        color: iconColor
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
    }

    Rectangle {
        anchors.fill: parent
        anchors.margins: 10
        color: window.surfaceColor
        border.color: window.borderColor
        border.width: 1
        radius: 14

        Column {
            anchors.fill: parent
            anchors.margins: 15
            spacing: 0

            Row {
                id: header
                width: parent.width
                height: 58

                Text {
                    id: headerTitle
                    text: "GeoLayer-X"
                    color: window.primaryTextColor
                    font.pixelSize: 20
                    font.bold: true
                    anchors.verticalCenter: parent.verticalCenter
                }

                Item {
                    width: Math.max(0, header.width - headerTitle.implicitWidth - centerButton.width)
                    height: 1
                }

                Rectangle {
                    id: centerButton
                    width: 118
                    height: 30
                    radius: 15
                    color: centerMouse.containsMouse
                           ? (window.darkTheme ? "#06321e" : "#d9f0e2")
                           : (window.darkTheme ? "#031b11" : "#e8f5ed")
                    anchors.verticalCenter: parent.verticalCenter

                    Text {
                        anchors.centerIn: parent
                        text: "Project Center"
                        color: window.accentColor
                        font.pixelSize: 14
                    }

                    MouseArea {
                        id: centerMouse
                        anchors.fill: parent
                        hoverEnabled: true
                        cursorShape: Qt.PointingHandCursor
                        onClicked: window.projectController.openProjectCenter()
                    }
                }
            }

            Rectangle {
                width: parent.width
                height: 1
                color: window.dividerColor
            }

            Row {
                id: body
                width: parent.width
                height: parent.height - header.height - footer.height - 2
                spacing: 24

                Column {
                    id: navigation
                    width: 245
                    y: 28
                    spacing: 14

                    Text {
                        text: "Navigation"
                        color: window.primaryTextColor
                        font.pixelSize: 19
                        font.bold: true
                        bottomPadding: 7
                    }

                    component NavigationItem: Rectangle {
                        id: navItem
                        property string label: ""
                        property string iconName: ""
                        property bool selected: false
                        signal activated()
                        width: navigation.width - 8
                        height: 42
                        radius: 6
                        color: navMouse.containsMouse ? window.hoverColor : "transparent"

                        Row {
                            anchors.fill: parent
                            anchors.leftMargin: 8
                            spacing: 12

                            IconText {
                                width: 20
                                height: parent.height
                                iconName: navItem.iconName
                                iconColor: navItem.selected
                                           ? window.primaryTextColor : window.secondaryTextColor
                            }

                            Text {
                                text: navItem.label
                                color: window.primaryTextColor
                                font.pixelSize: 16
                                anchors.verticalCenter: parent.verticalCenter
                            }
                        }

                        MouseArea {
                            id: navMouse
                            anchors.fill: parent
                            hoverEnabled: true
                            cursorShape: Qt.PointingHandCursor
                            onClicked: parent.activated()
                        }
                    }

                    NavigationItem {
                        label: "Recent"
                        iconName: "recent"
                        selected: window.projectController.activeSection === "Recent"
                        onActivated: window.projectController.selectSection("Recent")
                    }
                    NavigationItem {
                        label: "Browse"
                        iconName: "browse"
                        onActivated: openProjectFileDialog.open()
                    }
                    NavigationItem {
                        label: "Favorites"
                        iconName: "favorites"
                        selected: window.projectController.activeSection === "Favorites"
                        onActivated: window.projectController.selectSection("Favorites")
                    }
                    NavigationItem {
                        label: "Manage"
                        iconName: "manage"
                        selected: window.projectController.activeSection === "Manage"
                        onActivated: window.projectController.selectSection("Manage")
                    }
                }

                Rectangle {
                    width: 1
                    y: 28
                    height: parent.height - 54
                    color: window.dividerColor
                }

                Column {
                    id: projectArea
                    width: body.width - navigation.width - 49
                    y: 28
                    height: parent.height - 28
                    spacing: 20

                    Column {
                        id: startSection
                        width: parent.width
                        spacing: 8

                        Text {
                            text: "Start a project"
                            color: window.primaryTextColor
                            font.pixelSize: 19
                            font.bold: true
                        }

                        component ProjectAction: Rectangle {
                            id: projectAction
                            property string actionTitle: ""
                            property string description: ""
                            property string iconName: ""
                            signal activated()
                            width: projectArea.width
                            height: 54
                            radius: 7
                            color: actionMouse.containsMouse
                                   ? window.hoverColor : "transparent"

                            Row {
                                anchors.fill: parent
                                anchors.leftMargin: 10
                                spacing: 12

                                IconText {
                                    width: 24
                                    height: 32
                                    iconName: projectAction.iconName
                                    iconColor: window.primaryTextColor
                                    anchors.verticalCenter: parent.verticalCenter
                                }

                                Column {
                                    spacing: 3
                                    anchors.verticalCenter: parent.verticalCenter

                                    Text {
                                        text: projectAction.actionTitle
                                        color: window.primaryTextColor
                                        font.pixelSize: 18
                                        font.bold: true
                                    }
                                    Text {
                                        text: projectAction.description
                                        color: window.secondaryTextColor
                                        font.pixelSize: 14
                                    }
                                }
                            }

                            MouseArea {
                                id: actionMouse
                                anchors.fill: parent
                                hoverEnabled: true
                                cursorShape: Qt.PointingHandCursor
                                onClicked: parent.activated()
                            }
                        }

                        ProjectAction {
                            actionTitle: "Empty Project"
                            description: "Start with a blank GIS workspace."
                            iconName: "empty_project"
                            onActivated: window.beginProjectCreation("empty", "")
                        }

                        Text {
                            text: "Basemap Template"
                            color: window.primaryTextColor
                            font.pixelSize: 17
                            font.bold: true
                            topPadding: 4
                        }

                        Row {
                            id: basemapRow
                            width: parent.width
                            height: 112
                            spacing: 7

                            component BasemapCard: Rectangle {
                                id: tileCard
                                property string templateId: ""
                                property string label: ""
                                property string imageSource: ""
                                property bool addNew: false
                                width: (basemapRow.width - basemapRow.spacing * 4) / 5
                                height: basemapRow.height
                                radius: 6
                                color: tileMouse.containsMouse
                                       ? window.hoverColor : window.surfaceColor
                                border.color: tileMouse.containsMouse
                                              ? window.accentColor : window.borderColor

                                Column {
                                    anchors.fill: parent
                                    anchors.margins: 5
                                    spacing: 4

                                    Rectangle {
                                        width: parent.width
                                        height: 75
                                        radius: 4
                                        color: window.darkTheme ? "#202020" : "#e5e5e5"
                                        clip: true

                                        Image {
                                            anchors.fill: parent
                                            source: tileCard.imageSource
                                            fillMode: Image.PreserveAspectCrop
                                            visible: !tileCard.addNew
                                            asynchronous: true
                                        }
                                    }

                                    Text {
                                        width: parent.width
                                        text: tileCard.label
                                        color: window.primaryTextColor
                                        font.pixelSize: 12
                                        horizontalAlignment: Text.AlignHCenter
                                        elide: Text.ElideRight
                                    }
                                }

                                Text {
                                    anchors.centerIn: parent
                                    text: "+"
                                    color: window.primaryTextColor
                                    font.pixelSize: 40
                                    visible: tileCard.addNew
                                }

                                MouseArea {
                                    id: tileMouse
                                    anchors.fill: parent
                                    hoverEnabled: true
                                    cursorShape: Qt.PointingHandCursor
                                    onClicked: {
                                        if (tileCard.addNew)
                                            recentBasemapDialog.open()
                                        else
                                            window.beginProjectCreation(tileCard.templateId, "")
                                    }
                                }
                            }

                            BasemapCard {
                                templateId: "openstreetmap"
                                label: "OpenStreetMap"
                                imageSource: "qrc:/images/tiles/openstreetmap.png"
                            }
                            BasemapCard {
                                templateId: "satellite"
                                label: "Satellite"
                                imageSource: "qrc:/images/tiles/googlesatellite.png"
                            }
                            BasemapCard {
                                templateId: "hybrid"
                                label: "Hybrid"
                                imageSource: "qrc:/images/tiles/googlehybrid.png"
                            }
                            BasemapCard {
                                templateId: "esri"
                                label: "Esri Imagery"
                                imageSource: "qrc:/images/tiles/EsriWorldImagery.png"
                            }
                            BasemapCard {
                                label: "Add template"
                                addNew: true
                            }
                        }
                    }

                    Rectangle {
                        width: parent.width
                        height: 1
                        color: window.dividerColor
                    }

                    Column {
                        width: parent.width
                        height: projectArea.height - startSection.height
                                - projectArea.spacing * 2 - 1
                        spacing: 8

                        Text {
                            text: window.projectController.activeSection === "Favorites"
                                  ? "Favorite projects"
                                  : window.projectController.activeSection === "Manage"
                                    ? "Manage projects"
                                    : "Recent projects"
                            color: window.primaryTextColor
                            font.pixelSize: 19
                            font.bold: true
                        }

                        Item {
                            id: projectListArea
                            width: parent.width
                            height: parent.height - 28

                            ListView {
                            id: recentProjects
                            anchors.fill: parent
                            model: window.projectController.visibleProjects
                            clip: true
                            spacing: 2
                            boundsBehavior: Flickable.StopAtBounds

                            delegate: Rectangle {
                                id: projectRow
                                required property var modelData
                                width: recentProjects.width
                                height: 62
                                radius: 6
                                color: rowMouse.containsMouse ? window.hoverColor : "transparent"
                                opacity: modelData.exists ? 1 : 0.55
                                ToolTip.visible: rowMouse.containsMouse
                                ToolTip.text: modelData.filePath
                                ToolTip.delay: 500

                                Row {
                                    anchors.fill: parent
                                    anchors.leftMargin: 8
                                    anchors.rightMargin: 8
                                    spacing: 10

                                    IconText {
                                        width: 22
                                        height: parent.height
                                        iconName: projectRow.modelData.iconType
                                        iconColor: window.secondaryTextColor
                                    }

                                    Column {
                                        width: projectRow.width - 100
                                        anchors.verticalCenter: parent.verticalCenter
                                        spacing: 2

                                        Text {
                                            width: parent.width
                                            text: projectRow.modelData.title
                                            color: window.primaryTextColor
                                            font.pixelSize: 18
                                            font.bold: true
                                            elide: Text.ElideRight
                                        }
                                        Text {
                                            width: parent.width
                                            text: projectRow.modelData.exists
                                                  ? projectRow.modelData.subtitle
                                                  : projectRow.modelData.subtitle + " · File not found"
                                            color: window.secondaryTextColor
                                            font.pixelSize: 12
                                            elide: Text.ElideRight
                                        }
                                    }

                                    ToolButton {
                                        id: projectMenuButton
                                        width: 32
                                        height: 36
                                        anchors.verticalCenter: parent.verticalCenter
                                        text: "⋮"
                                        font.pixelSize: 20
                                        palette.buttonText: window.primaryTextColor
                                        onClicked: projectMenu.open()
                                    }
                                }

                                MouseArea {
                                    id: rowMouse
                                    anchors.fill: parent
                                    anchors.rightMargin: 44
                                    hoverEnabled: true
                                    cursorShape: Qt.PointingHandCursor
                                    onClicked: {
                                        if (projectRow.modelData.exists)
                                            window.projectController.openProject(projectRow.modelData.filePath)
                                    }
                                }

                                Menu {
                                    id: projectMenu
                                    parent: projectMenuButton
                                    x: projectMenuButton.width - width
                                    y: projectMenuButton.height
                                    background: Rectangle {
                                        implicitWidth: 190
                                        color: window.surfaceColor
                                        border.color: window.borderColor
                                        radius: 5
                                    }
                                    delegate: MenuItem {
                                        id: menuItem
                                        contentItem: Text {
                                            text: menuItem.text
                                            color: window.primaryTextColor
                                            font: menuItem.font
                                            verticalAlignment: Text.AlignVCenter
                                        }
                                        background: Rectangle {
                                            color: menuItem.highlighted
                                                   ? window.hoverColor : "transparent"
                                        }
                                    }
                                    MenuItem {
                                        text: projectRow.modelData.favorite
                                              ? "Remove from Favorites"
                                              : "Add to Favorites"
                                        onTriggered: window.projectController.toggleFavorite(
                                                         projectRow.modelData.filePath)
                                    }
                                    MenuItem {
                                        text: "Show in File Explorer"
                                        enabled: projectRow.modelData.exists
                                        onTriggered: window.projectController.showProjectLocation(
                                                         projectRow.modelData.filePath)
                                    }
                                    MenuItem {
                                        text: "Remove from Recent"
                                        onTriggered: window.projectController.removeRecentProject(
                                                         projectRow.modelData.filePath)
                                    }
                                }
                            }
                            }

                            Label {
                                anchors.centerIn: projectListArea
                                visible: recentProjects.count === 0
                                width: projectListArea.width - 24
                                horizontalAlignment: Text.AlignHCenter
                                wrapMode: Text.WordWrap
                                color: window.secondaryTextColor
                                text: window.projectController.activeSection === "Favorites"
                                      ? "No favorite projects yet. Use a project's menu to add it to Favorites."
                                      : "No recent projects yet. Create a project or browse for a .earth file."
                            }
                        }
                    }
                }
            }

            Rectangle {
                width: parent.width
                height: 1
                color: window.dividerColor
            }

            Row {
                id: footer
                width: parent.width
                height: 56

                Text {
                    id: footerText
                    text: "GeoLayer-X GIS Platform"
                    color: window.secondaryTextColor
                    font.pixelSize: 14
                    anchors.verticalCenter: parent.verticalCenter
                }

                Item {
                    width: Math.max(0, footer.width - footerText.implicitWidth - footerSettings.implicitWidth)
                    height: 1
                }

                Text {
                    id: footerSettings
                    text: "Settings · Help"
                    color: window.secondaryTextColor
                    font.pixelSize: 14
                    anchors.verticalCenter: parent.verticalCenter

                    MouseArea {
                        anchors.fill: parent
                        cursorShape: Qt.PointingHandCursor
                        onClicked: settingsDialog.open()
                    }
                }
            }
        }
    }
}
