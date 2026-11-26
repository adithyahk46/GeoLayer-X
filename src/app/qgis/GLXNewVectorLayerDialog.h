#ifndef GLXNEWVECTORLAYERDIALOG_H
#define GLXNEWVECTORLAYERDIALOG_H

#include <QDialog>
#include <QList>
#include <QPair>
#include <QPushButton>
#include <QString>
#include <string>
#include <vector>
#include <utility>

#include <ogr_core.h>

#include "ui_GLXNewVectorLayerDialog.h"

class GLXNewVectorLayerDialog : public QDialog, private Ui::GLXNewVectorLayerDialogBase
{
    Q_OBJECT

public:
    explicit GLXNewVectorLayerDialog(QWidget *parent = nullptr);

    void attributes(QList<QPair<QString, QString>> &attributes) const;
    QString selectedFileFormat() const;
    QString selectedFileEncoding() const;
    QString filename() const;
    void setFilename(const QString &filename);
    QString crs() const;
    void setCrs(const QString &crs);
    QString selectedType() const;
    bool CreateEmptyShapefile(const std::string& filepath,
                                const std::string& encoding,
                                const std::string& geometryType,
                                const std::string& crs,
                                const std::vector<std::pair<std::string, OGRFieldType>>& fields);

public slots:
    void accept() override;

private slots:
    void mAddAttributeButton_clicked();
    void mRemoveAttributeButton_clicked();
    void mFileFormatComboBox_currentIndexChanged(int index);
    void mTypeBox_currentIndexChanged(int index);
    void checkOk();
    void showHelp();
    void nameChanged(const QString &name);
    void selectionChanged();
    void moveFieldsUp();
    void moveFieldsDown();
    void browseForFilename();

private:
    QPushButton *mOkButton = nullptr;
    void updateExtension();
};

#endif // GLXNEWVECTORLAYERDIALOG_H
