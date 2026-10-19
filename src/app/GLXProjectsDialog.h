#ifndef GLXPROJECTSDIALOG_H
#define GLXPROJECTSDIALOG_H

#include <QDialog>
#include <QString>

class QListWidget;
class QTabWidget;

namespace Ui {
class GLXProjectsDialog;
}

class GLXProjectsDialog : public QDialog
{
    Q_OBJECT

public:
    enum class TemplateType
    {
        Blank,
        Basemap
    };
    Q_ENUM(TemplateType)

    explicit GLXProjectsDialog(QWidget *parent = nullptr);
    ~GLXProjectsDialog();

    static void addRecentProject(const QString& projectPath);

signals:
    void openProjectRequested(const QString& projectPath);
    void newProjectRequested(GLXProjectsDialog::TemplateType type);

private:
    void openSelectedProject();
    void browseForProject();
    void updateRecentProjects();

    Ui::GLXProjectsDialog *ui;
    QListWidget *m_recentProjects = nullptr;
    QTabWidget *m_tabs = nullptr;
};

#endif // GLXPROJECTSDIALOG_H
