#include "GLXProjectsDialog.h"
#include "ui_GLXProjectsDialog.h"

#include <QFileDialog>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QMessageBox>
#include <QPushButton>
#include <QTabWidget>
#include <QVBoxLayout>

#include "SettingsManager/SettingsManager.h"

namespace
{
const char* RecentProjectsSetting = "Projects/RecentProjects";
constexpr int MaxRecentProjects = 10;
}

GLXProjectsDialog::GLXProjectsDialog(QWidget *parent)
    : QDialog(parent)
    , ui(new Ui::GLXProjectsDialog)
{
    ui->setupUi(this);
    setWindowTitle(tr("Welcome to GeoLayer-X"));
    resize(900, 560);

    auto* layout = new QVBoxLayout(this);
    auto* heading = new QLabel(tr("Welcome to GeoLayer-X"), this);
    QFont headingFont = heading->font();
    headingFont.setPointSize(headingFont.pointSize() + 8);
    headingFont.setBold(true);
    heading->setFont(headingFont);
    layout->addWidget(heading);

    m_tabs = new QTabWidget(this);
    layout->addWidget(m_tabs, 1);

    auto* recentPage = new QWidget(m_tabs);
    auto* recentLayout = new QVBoxLayout(recentPage);
    recentLayout->addWidget(new QLabel(tr("Open a recent project or browse for an .earth file."), recentPage));
    m_recentProjects = new QListWidget(recentPage);
    m_recentProjects->setSelectionMode(QAbstractItemView::SingleSelection);
    recentLayout->addWidget(m_recentProjects, 1);

    auto* recentButtons = new QHBoxLayout();
    auto* openSelectedButton = new QPushButton(tr("Open Selected"), recentPage);
    auto* browseButton = new QPushButton(tr("Open Project..."), recentPage);
    auto* removeButton = new QPushButton(tr("Remove from List"), recentPage);
    auto* clearButton = new QPushButton(tr("Clear List"), recentPage);
    recentButtons->addWidget(openSelectedButton);
    recentButtons->addWidget(browseButton);
    recentButtons->addStretch();
    recentButtons->addWidget(removeButton);
    recentButtons->addWidget(clearButton);
    recentLayout->addLayout(recentButtons);
    m_tabs->addTab(recentPage, tr("Recent Projects"));

    auto* templatesPage = new QWidget(m_tabs);
    auto* templatesLayout = new QVBoxLayout(templatesPage);
    templatesLayout->addWidget(new QLabel(tr("Start a new map from a template."), templatesPage));

    auto* blankButton = new QPushButton(tr("Blank Map\nStart with an empty map"), templatesPage);
    blankButton->setMinimumHeight(90);
    auto* basemapButton = new QPushButton(tr("Default Basemap\nStart with the GeoLayer-X online basemap"), templatesPage);
    basemapButton->setMinimumHeight(90);
    templatesLayout->addWidget(blankButton);
    templatesLayout->addWidget(basemapButton);
    templatesLayout->addStretch();
    m_tabs->addTab(templatesPage, tr("Templates"));

    auto* closeButton = new QPushButton(tr("Close"), this);
    auto* closeLayout = new QHBoxLayout();
    closeLayout->addStretch();
    closeLayout->addWidget(closeButton);
    layout->addLayout(closeLayout);

    connect(openSelectedButton, &QPushButton::clicked, this, &GLXProjectsDialog::openSelectedProject);
    connect(m_recentProjects, &QListWidget::itemDoubleClicked, this, [this](QListWidgetItem*) {
        openSelectedProject();
    });
    connect(browseButton, &QPushButton::clicked, this, &GLXProjectsDialog::browseForProject);
    connect(removeButton, &QPushButton::clicked, this, [this] {
        const int row = m_recentProjects->currentRow();
        if (row < 0)
            return;

        QStringList recent = SettingsManager::instance()
                                 .value(RecentProjectsSetting).toStringList();
        recent.removeAll(m_recentProjects->currentItem()->data(Qt::UserRole).toString());
        SettingsManager::instance().setValue(RecentProjectsSetting, recent);
        SettingsManager::instance().sync();
        updateRecentProjects();
    });
    connect(clearButton, &QPushButton::clicked, this, [this] {
        SettingsManager::instance().remove(RecentProjectsSetting);
        SettingsManager::instance().sync();
        updateRecentProjects();
    });
    connect(blankButton, &QPushButton::clicked, this, [this] {
        emit newProjectRequested(TemplateType::Blank);
        accept();
    });
    connect(basemapButton, &QPushButton::clicked, this, [this] {
        emit newProjectRequested(TemplateType::Basemap);
        accept();
    });
    connect(closeButton, &QPushButton::clicked, this, &QDialog::reject);

    updateRecentProjects();
}

GLXProjectsDialog::~GLXProjectsDialog()
{
    delete ui;
}

void GLXProjectsDialog::addRecentProject(const QString& projectPath)
{
    const QString absolutePath = QFileInfo(projectPath).absoluteFilePath();
    QStringList recent = SettingsManager::instance()
                             .value(RecentProjectsSetting).toStringList();
    recent.removeAll(absolutePath);
    recent.prepend(absolutePath);
    while (recent.size() > MaxRecentProjects)
        recent.removeLast();

    SettingsManager::instance().setValue(RecentProjectsSetting, recent);
    SettingsManager::instance().sync();
}

void GLXProjectsDialog::openSelectedProject()
{
    const auto* item = m_recentProjects->currentItem();
    if (!item)
        return;

    const QString path = item->data(Qt::UserRole).toString();
    if (!QFileInfo::exists(path))
    {
        QMessageBox::warning(this, tr("Project Not Found"),
                             tr("The project file could not be found:\n%1").arg(path));
        return;
    }

    emit openProjectRequested(path);
    accept();
}

void GLXProjectsDialog::browseForProject()
{
    const QString path = QFileDialog::getOpenFileName(
        this, tr("Open Project"), QString(), tr("GeoLayer-X Projects (*.earth);;All Files (*)"));
    if (path.isEmpty())
        return;

    emit openProjectRequested(path);
    accept();
}

void GLXProjectsDialog::updateRecentProjects()
{
    m_recentProjects->clear();
    const QStringList recent = SettingsManager::instance()
                                   .value(RecentProjectsSetting).toStringList();

    for (const QString& path : recent)
    {
        auto* item = new QListWidgetItem(QFileInfo(path).fileName(), m_recentProjects);
        item->setData(Qt::UserRole, path);
        item->setToolTip(path);
        if (!QFileInfo::exists(path))
            item->setText(tr("%1 (not found)").arg(item->text()));
    }

    if (m_recentProjects->count() == 0)
        m_tabs->setCurrentIndex(1);
}
