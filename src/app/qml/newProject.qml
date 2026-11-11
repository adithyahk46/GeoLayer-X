import QtQuick 2.15
import QtQuick.Window 2.15
import QtQuick.Controls 2.15
import QtQuick.Layouts 1.15
import QtQuick.Dialogs

Window {
    id: root
    property var projectController
    property string templateId: "empty"
    property string basemapUrl: ""
    property string templateName: "Empty Project"
    property string templateImage: ""
    property string defaultLocationAtOpen: ""
    property bool darkTheme: projectController
                                 ? projectController.currentTheme === "dark" : false

    width: 680
    height: 480
    modality: Qt.ApplicationModal
    visible: false
    title: "Configure Your New Project"
    color: "transparent"
    flags: Qt.Dialog | Qt.FramelessWindowHint

    function openForTemplate(selectedTemplateId, selectedBasemapUrl) {
        templateId = selectedTemplateId
        basemapUrl = selectedBasemapUrl
        defaultLocationAtOpen = projectController.defaultProjectLocation
        nameInput.text = "Untitled Project"
        locationInput.text = defaultLocationAtOpen
        defaultCheck.checked = projectController.hasSavedDefaultProjectLocation
        descInput.text = ""
        updateTemplatePreview()
        if (transientParent) {
            x = transientParent.x + (transientParent.width - width) / 2
            y = transientParent.y + (transientParent.height - height) / 2
        }
        show()
        raise()
        requestActivate()
    }

    function updateTemplatePreview() {
        if (templateId === "openstreetmap") {
            templateName = "OpenStreetMap"
            templateImage = "qrc:/images/tiles/openstreetmap.png"
        } else if (templateId === "satellite") {
            templateName = "Satellite"
            templateImage = "qrc:/images/tiles/googlesatellite.png"
        } else if (templateId === "hybrid") {
            templateName = "Hybrid"
            templateImage = "qrc:/images/tiles/googlehybrid.png"
        } else if (templateId === "esri") {
            templateName = "Esri World Imagery"
            templateImage = "qrc:/images/tiles/EsriWorldImagery.png"
        } else if (templateId.startsWith("custom:")) {
            templateName = templateId.substring("custom:".length)
            templateImage = "qrc:/images/tiles/openstreetmap.png"
        } else {
            templateName = "Empty Project"
            templateImage = ""
        }
    }

    function submitProject(overwriteExisting) {
        if (nameInput.text.trim().length === 0 || locationInput.text.trim().length === 0) {
            validationDialog.open()
            return
        }
        projectController.createProject(
                    templateId,
                    nameInput.text,
                    projectController.projectFilePath(nameInput.text, locationInput.text),
                    defaultCheck.checked,
                    descInput.text,
                    basemapUrl,
                    overwriteExisting)
    }

    readonly property color bgBase: darkTheme ? "#070707" : "#f5f5f5"
    readonly property color bgCard: darkTheme ? "#1b1f24" : "#ffffff"
    readonly property color bgInput: darkTheme ? "#232830" : "#f4f4f4"
    readonly property color borderCard: darkTheme ? "#2e3540" : "#d0d0d0"
    readonly property color borderInput: darkTheme ? "#373e4b" : "#c7c7c7"
    readonly property color primaryAccent: darkTheme ? "#2563eb" : "#08783e"
    readonly property color primaryHover: darkTheme ? "#3b82f6" : "#0a8c4c"
    readonly property color textPrimary: darkTheme ? "#f4f4f5" : "#202020"
    readonly property color textSecondary: darkTheme ? "#93939f" : "#555555"

    Rectangle {
        anchors.fill: parent
        radius: 12
        color: root.bgBase
        border.color: root.borderCard
        border.width: 1

        // Window Dragging Support
        MouseArea {
            id: windowDragArea
            anchors.fill: parent
            property point clickPos: Qt.point(0, 0)
            onPressed: function(mouse) {
                windowDragArea.clickPos = Qt.point(mouse.x, mouse.y)
            }
            onPositionChanged: function(mouse) {
                var delta = Qt.point(mouse.x - windowDragArea.clickPos.x,
                                     mouse.y - windowDragArea.clickPos.y)
                root.x += delta.x
                root.y += delta.y
            }
        }

        ColumnLayout {
            anchors.fill: parent
            anchors.margins: 28
            spacing: 20

            // Title Header
            ColumnLayout {
                spacing: 4
                Layout.fillWidth: true

                RowLayout {
                    spacing: 8
                    Text {
                        text: "Configure Your New Project"
                        color: root.textPrimary
                        font.pixelSize: 22
                        font.bold: true
                        font.family: "Segoe UI"
                    }
                    Text {
                        text: "🌐"
                        font.pixelSize: 20
                    }
                }

                Text {
                    text: "Define your project details and default save location."
                    color: root.textSecondary
                    font.pixelSize: 13
                    font.family: "Segoe UI"
                }
            }

            // Main Two-Column Layout
            RowLayout {
                Layout.fillWidth: true
                Layout.fillHeight: true
                spacing: 20

                // Left Column: Project Details
                Rectangle {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    color: root.bgCard
                    radius: 10
                    border.color: root.borderCard
                    border.width: 1

                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: 16
                        spacing: 14

                        Text {
                            text: "Project Details"
                            color: root.textPrimary
                            font.pixelSize: 15
                            font.bold: true
                        }

                        // Project Name
                        ColumnLayout {
                            spacing: 6
                            Layout.fillWidth: true

                            Text {
                                text: "Project Name"
                                color: root.textSecondary
                                font.pixelSize: 12
                            }

                            TextField {
                                id: nameInput
                                Layout.fillWidth: true
                                Layout.preferredHeight: 38
                                text: "Untitled Project"
                                color: root.textPrimary
                                font.pixelSize: 13
                                leftPadding: 10
                                rightPadding: 10
                                selectByMouse: true

                                background: Rectangle {
                                    color: root.bgInput
                                    radius: 6
                                    border.color: nameInput.activeFocus ? root.primaryAccent : root.borderInput
                                    border.width: nameInput.activeFocus ? 2 : 1
                                    Behavior on border.color { ColorAnimation { duration: 120 } }
                                }
                            }
                        }

                        // Save Location
                        ColumnLayout {
                            spacing: 6
                            Layout.fillWidth: true

                            Text {
                                text: "Save Location"
                                color: root.textSecondary
                                font.pixelSize: 12
                            }

                            TextField {
                                id: locationInput
                                Layout.fillWidth: true
                                Layout.preferredHeight: 38
                                text: root.projectController
                                      ? root.projectController.defaultProjectLocation : ""
                                color: root.textPrimary
                                font.pixelSize: 12
                                leftPadding: 10
                                rightPadding: 10
                                selectByMouse: true
                                onTextChanged: {
                                    if (defaultCheck.checked
                                            && !root.projectController.isDefaultProjectLocation(text))
                                        defaultCheck.checked = false
                                }

                                background: Rectangle {
                                    color: root.bgInput
                                    radius: 6
                                    border.color: locationInput.activeFocus ? root.primaryAccent : root.borderInput
                                    border.width: locationInput.activeFocus ? 2 : 1
                                    Behavior on border.color { ColorAnimation { duration: 120 } }
                                }
                            }

                            Button {
                                id: browseBtn
                                text: "📂  Browse..."
                                Layout.fillWidth: true
                                Layout.preferredHeight: 36

                                contentItem: Text {
                                    text: browseBtn.text
                                    color: root.textPrimary
                                    font.pixelSize: 13
                                    horizontalAlignment: Text.AlignHCenter
                                    verticalAlignment: Text.AlignVCenter
                                }

                                background: Rectangle {
                                    color: browseBtn.hovered
                                           ? (root.darkTheme ? "#2c333e" : "#e8e8e8")
                                           : root.bgInput
                                    radius: 6
                                    border.color: root.borderInput
                                    border.width: 1
                                    Behavior on color { ColorAnimation { duration: 120 } }
                                }
                                onClicked: folderDialog.open()
                            }
                        }

                        // Default Save Checkbox
                        CheckBox {
                            id: defaultCheck
                            text: "Set/Save as Default Project Location"
                            checked: root.projectController
                                     ? root.projectController.hasSavedDefaultProjectLocation : false
                            Layout.topMargin: 4

                            contentItem: Text {
                                text: defaultCheck.text
                                color: root.textSecondary
                                font.pixelSize: 12
                                leftPadding: defaultCheck.indicator.width + 8
                                verticalAlignment: Text.AlignVCenter
                            }

                            indicator: Rectangle {
                                implicitWidth: 16
                                implicitHeight: 16
                                x: defaultCheck.leftPadding
                                y: parent.height / 2 - height / 2
                                radius: 4
                                color: defaultCheck.checked ? root.primaryAccent : root.bgInput
                                border.color: defaultCheck.checked ? root.primaryAccent : root.borderInput
                                border.width: 1

                                Text {
                                    anchors.centerIn: parent
                                    text: "✓"
                                    color: "white"
                                    font.pixelSize: 11
                                    visible: defaultCheck.checked
                                }
                            }
                        }

                        Item { Layout.fillHeight: true }
                    }
                }

                // Right Column: Template & Description
                Rectangle {
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    color: root.bgCard
                    radius: 10
                    border.color: root.borderCard
                    border.width: 1

                    ColumnLayout {
                        anchors.fill: parent
                        anchors.margins: 16
                        spacing: 14

                        Text {
                            text: "Template & Description"
                            color: root.textPrimary
                            font.pixelSize: 15
                            font.bold: true
                        }

                        // Selected Template Preview Card
                        Rectangle {
                            Layout.fillWidth: true
                            Layout.preferredHeight: 90
                            radius: 6
                            color: root.darkTheme ? "#121519" : "#f0f3f5"
                            border.color: root.primaryAccent
                            border.width: 1.5

                            Image {
                                anchors.fill: parent
                                anchors.margins: 2
                                source: root.templateImage
                                fillMode: Image.PreserveAspectCrop
                                opacity: 0.85
                                visible: root.templateImage.length > 0
                            }

                            Rectangle {
                                anchors.bottom: parent.bottom
                                anchors.left: parent.left
                                anchors.right: parent.right
                                height: 24
                                color: root.darkTheme ? "#cc000000" : "#ccffffff"
                                radius: 4

                                Text {
                                    anchors.centerIn: parent
                                    text: "Selected Template: " + root.templateName
                                    color: root.textPrimary
                                    font.pixelSize: 11
                                }
                            }
                        }

                        // Optional Project Description
                        ColumnLayout {
                            spacing: 6
                            Layout.fillWidth: true
                            Layout.fillHeight: true

                            Text {
                                text: "Project Description (optional)"
                                color: root.textSecondary
                                font.pixelSize: 12
                            }

                            ScrollView {
                                Layout.fillWidth: true
                                Layout.fillHeight: true
                                clip: true

                                TextArea {
                                    id: descInput
                                    placeholderText: "Enter optional project description or metadata..."
                                    placeholderTextColor: root.darkTheme ? "#555d6b" : "#777777"
                                    color: root.textPrimary
                                    font.pixelSize: 12
                                    wrapMode: TextEdit.Wrap

                                    background: Rectangle {
                                        color: root.bgInput
                                        radius: 6
                                        border.color: descInput.activeFocus ? root.primaryAccent : root.borderInput
                                        border.width: descInput.activeFocus ? 2 : 1
                                        Behavior on border.color { ColorAnimation { duration: 120 } }
                                    }
                                }
                            }
                        }
                    }
                }
            }

            // Bottom Dialog Action Buttons
            RowLayout {
                Layout.fillWidth: true
                spacing: 12

                Item { Layout.fillWidth: true }

                Button {
                    id: createBtn
                    text: "Create Project"
                    Layout.preferredHeight: 38
                    Layout.preferredWidth: 120

                    contentItem: Text {
                        text: createBtn.text
                        color: "#ffffff"
                        font.pixelSize: 13
                        font.weight: Font.DemiBold
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }

                    background: Rectangle {
                        color: createBtn.hovered ? root.primaryHover : root.primaryAccent
                        radius: 6
                        Behavior on color { ColorAnimation { duration: 120 } }
                    }

                    onClicked: {
                        root.submitProject(false)
                    }
                }

                Button {
                    id: cancelBtn
                    text: "Cancel"
                    Layout.preferredHeight: 38
                    Layout.preferredWidth: 90

                    contentItem: Text {
                        text: cancelBtn.text
                        color: root.textPrimary
                        font.pixelSize: 13
                        horizontalAlignment: Text.AlignHCenter
                        verticalAlignment: Text.AlignVCenter
                    }

                    background: Rectangle {
                        color: cancelBtn.hovered
                               ? (root.darkTheme ? "#2c333e" : "#e8e8e8")
                               : root.bgInput
                        radius: 6
                        border.color: root.borderInput
                        border.width: 1
                        Behavior on color { ColorAnimation { duration: 120 } }
                    }

                    onClicked: root.close()
                }

            }
        }
    }

    FolderDialog {
        id: folderDialog
        title: "Choose project location"
        currentFolder: root.projectController
                      ? root.projectController.defaultProjectLocationUrl : ""
        onAccepted: locationInput.text =
                        root.projectController.localFilePath(selectedFolder)
    }

    Connections {
        target: root.projectController
        function onProjectActivated() {
            root.close()
        }
        function onProjectFileExists(filePath) {
            overwriteDialog.message = "A project already exists at:\n"
                    + filePath + "\n\nReplace it?"
            overwriteDialog.open()
        }
    }

    Dialog {
        id: overwriteDialog
        property string message: ""
        title: "Replace project file?"
        modal: true
        width: 380
        standardButtons: Dialog.Yes | Dialog.No
        onAccepted: root.submitProject(true)
        contentItem: Label {
            text: overwriteDialog.message
            wrapMode: Text.WordWrap
        }
    }

    Dialog {
        id: validationDialog
        title: "New project"
        modal: true
        width: 380
        standardButtons: Dialog.Ok
        contentItem: Label {
            text: "Enter both a project name and a save location."
            wrapMode: Text.WordWrap
        }
    }
}