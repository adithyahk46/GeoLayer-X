#include "ViewpointsGUI.h"

#include "ui_ViewpointsGUI.h"

#include "App.h"
#include "MapController/GLXEarthManipulator.h"
#include "ProjectManager/ProjectManager.h"
#include "osgviewerwidget.h"

#include <QListWidgetItem>
#include <QHBoxLayout>
#include <QIcon>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QResource>
#include <QSize>
#include <QSizePolicy>
#include <QShowEvent>

#include <algorithm>
#include <functional>

namespace
{
/**
 * Displays one saved viewpoint and its individual zoom and delete actions.
 */
class ViewpointListItem final : public QWidget
{
public:
    /**
     * Builds a viewpoint row with icon actions.
     * @param name Display name of the viewpoint.
     * @param zoomAction Action invoked by the zoom button.
     * @param deleteAction Action invoked by the delete button.
     * @param parent Parent widget.
     */
    ViewpointListItem(const QString& name,
                      std::function<void()> zoomAction,
                      std::function<void()> deleteAction,
                      QWidget* parent = nullptr)
        : QWidget(parent)
    {
        auto* layout = new QHBoxLayout(this);
        layout->setContentsMargins(0, 0, 0, 0);
        layout->setSpacing(2);

        auto* nameLabel = new QLabel(name, this);
        nameLabel->setToolTip(name);
        nameLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Preferred);
        layout->addWidget(nameLabel, 1);

        auto* deleteButton = new QPushButton(this);
        deleteButton->setIcon(
            QIcon(QStringLiteral(":/images/themes/default/mIconDelete.svg")));
        deleteButton->setIconSize(QSize(18, 18));
        deleteButton->setFixedSize(18, 18);
        deleteButton->setToolTip(tr("Delete viewpoint"));
        deleteButton->setAccessibleName(tr("Delete viewpoint"));

        layout->addWidget(deleteButton);
        // setFixedHeight(32);

        connect(deleteButton, &QPushButton::clicked, this,
                [action = std::move(deleteAction)] { action(); });
    }
};
}

ViewpointsGUI::ViewpointsGUI(QWidget* parent)
    : DockWidgetPanels(parent)
    , ui(new Ui::ViewpointsGUI)
{
    Q_INIT_RESOURCE(images);

    ui->setupUi(this);
    setObjectName(QStringLiteral("ViewpointsDock"));

    connect(ui->lineEditViewpointName, &QLineEdit::textChanged,
            this, &ViewpointsGUI::updateButtonStates);
    connect(ui->listWidgetViewpoints, &QListWidget::currentRowChanged,
            this, &ViewpointsGUI::updateButtonStates);
    connect(ui->listWidgetViewpoints, &QListWidget::itemDoubleClicked,
            this, [this](QListWidgetItem*) { zoomToSelectedViewpoint(); });
    connect(ui->pushButtonAdd, &QPushButton::clicked,
            this, &ViewpointsGUI::addCurrentViewpoint);

    if (auto* projectManager = parent ? parent->findChild<ProjectManager*>() : nullptr)
    {
        connect(projectManager, &ProjectManager::projectOpened,
                this, [this] { reloadViewpoints(); });
        connect(projectManager, &ProjectManager::projectClosed,
                this, [this] { reloadViewpoints(); });
    }

    reloadViewpoints();
}

ViewpointsGUI::~ViewpointsGUI() = default;

void ViewpointsGUI::showEvent(QShowEvent* event)
{
    DockWidgetPanels::showEvent(event);
    reloadViewpoints();
}

osgEarth::MapNode* ViewpointsGUI::currentMapNode() const
{
    auto* widget = App::getThreedWindow();
    auto* viewer = widget ? widget->getViewer() : nullptr;
    auto* scene = viewer ? viewer->getSceneData() : nullptr;
    auto* mapNode = scene ? osgEarth::MapNode::findMapNode(scene) : nullptr;
    return mapNode ? mapNode : (widget ? widget->getMapNode() : nullptr);
}

GLXEarthManipulator* ViewpointsGUI::currentManipulator() const
{
    auto* widget = App::getThreedWindow();
    return widget ? widget->getManipulator() : nullptr;
}

void ViewpointsGUI::loadViewpointSection(const osgEarth::Config& config)
{
    _duration = std::max(0.0f, config.value("time", 1.0f));
    for (const osgEarth::Config& viewpointConfig : config.children("viewpoint"))
    {
        osgEarth::Viewpoint viewpoint(viewpointConfig);
        if (viewpoint.isValid())
            _viewpoints.push_back(viewpoint);
    }
}

void ViewpointsGUI::reloadViewpoints()
{
    _viewpoints.clear();
    _duration = 1.0f;

    if (osgEarth::MapNode* mapNode = currentMapNode())
    {
        const osgEarth::Config* savedViewpoints =
            mapNode->externalConfig().child_ptr("viewpoints");
        if (savedViewpoints)
        {
            loadViewpointSection(*savedViewpoints);
        }
        else
        {
            const osgEarth::Config mapConfig = mapNode->getConfig();
            for (const osgEarth::Config& child : mapConfig.children())
            {
                if (child.key() == "viewpoints")
                    loadViewpointSection(child);
            }
        }
    }

    ui->listWidgetViewpoints->clear();
    for (std::size_t index = 0; index < _viewpoints.size(); ++index)
    {
        const osgEarth::Viewpoint& viewpoint = _viewpoints[index];
        const std::string name = viewpoint.name().isSet()
            ? viewpoint.name().get()
            : std::string();
        const QString label = name.empty()
            ? tr("<Unnamed viewpoint>")
            : QString::fromStdString(name);

        auto* item = new QListWidgetItem;
        ui->listWidgetViewpoints->addItem(item);
        auto* itemWidget = new ViewpointListItem(
            label,
            [this, index] {
                ui->listWidgetViewpoints->setCurrentRow(static_cast<int>(index));
                zoomToSelectedViewpoint();
            },
            [this, index] {
                ui->listWidgetViewpoints->setCurrentRow(static_cast<int>(index));
                deleteSelectedViewpoint();
            });
        item->setSizeHint(itemWidget->sizeHint());
        ui->listWidgetViewpoints->setItemWidget(item, itemWidget);
    }
    ui->labelEmpty->setVisible(_viewpoints.empty());
    updateButtonStates();
}

void ViewpointsGUI::persistViewpoints(osgEarth::MapNode* mapNode)
{
    if (!mapNode)
        return;

    osgEarth::Config config("viewpoints");
    config.set("time", _duration);
    for (const osgEarth::Viewpoint& viewpoint : _viewpoints)
        config.add(viewpoint.getConfig());

    mapNode->externalConfig().set(config);
}

void ViewpointsGUI::addCurrentViewpoint()
{
    osgEarth::MapNode* mapNode = currentMapNode();
    GLXEarthManipulator* manipulator = currentManipulator();
    const QString name = ui->lineEditViewpointName->text().trimmed();
    if (!mapNode || !manipulator)
    {
        QMessageBox::warning(this, tr("Viewpoints"),
                             tr("Open a map before adding a viewpoint."));
        return;
    }
    if (name.isEmpty())
    {
        // QMessageBox::warning(this, tr("Viewpoints"),
        //                      tr("Enter a name for the viewpoint."));
        return;
    }

    const auto duplicate = std::find_if(
        _viewpoints.begin(), _viewpoints.end(),
        [&name](const osgEarth::Viewpoint& viewpoint) {
            return viewpoint.name().isSet() &&
                   QString::fromStdString(viewpoint.name().get()).compare(
                       name, Qt::CaseInsensitive) == 0;
        });
    if (duplicate != _viewpoints.end())
    {
        QMessageBox::warning(this, tr("Viewpoints"),
                             tr("A viewpoint with that name already exists."));
        return;
    }

    osgEarth::Viewpoint viewpoint = manipulator->currentViewpoint();
    if (!viewpoint.isValid())
    {
        QMessageBox::warning(this, tr("Viewpoints"),
                             tr("The current map view cannot be saved as a viewpoint."));
        return;
    }

    viewpoint.setName(name.toStdString());
    _viewpoints.push_back(viewpoint);
    persistViewpoints(mapNode);
    ui->lineEditViewpointName->clear();
    reloadViewpoints();
    ui->listWidgetViewpoints->setCurrentRow(static_cast<int>(_viewpoints.size()) - 1);
}

void ViewpointsGUI::zoomToSelectedViewpoint()
{
    const int row = ui->listWidgetViewpoints->currentRow();
    GLXEarthManipulator* manipulator = currentManipulator();
    if (row < 0 || row >= static_cast<int>(_viewpoints.size()) || !manipulator)
        return;

    manipulator->flyTo(_viewpoints[static_cast<std::size_t>(row)], _duration);
}

void ViewpointsGUI::deleteSelectedViewpoint()
{
    const int row = ui->listWidgetViewpoints->currentRow();
    if (row < 0 || row >= static_cast<int>(_viewpoints.size()))
        return;

    const osgEarth::Viewpoint& viewpoint = _viewpoints[static_cast<std::size_t>(row)];
    const QString name = viewpoint.name().isSet()
        ? QString::fromStdString(viewpoint.name().get())
        : tr("<Unnamed viewpoint>");

    _viewpoints.erase(_viewpoints.begin() + row);
    persistViewpoints(currentMapNode());
    reloadViewpoints();
    if (!_viewpoints.empty())
        ui->listWidgetViewpoints->setCurrentRow(
            std::min(row, static_cast<int>(_viewpoints.size()) - 1));
}

void ViewpointsGUI::updateButtonStates()
{
}