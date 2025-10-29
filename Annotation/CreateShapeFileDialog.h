#ifndef CREATESHAPEFILEDIALOG_H
#define CREATESHAPEFILEDIALOG_H

#include <QDialog>
#include <QFileDialog>
#include <QMessageBox>
#include <QStandardItemModel>
#include <QTextCodec>
#include <QDebug>

namespace Ui {
class CreateShapeFileDialog;
}

class CreateShapeFileDialog : public QDialog
{
    Q_OBJECT

public:
    explicit CreateShapeFileDialog(QWidget *parent = nullptr);
    ~CreateShapeFileDialog();

signals:
    void createShapeFile(const QString &outputPath,
                          const QString &encoding,
                          const QString &geometryType,
                          const QString &srs,
                          const QList<QVariantMap> &fields);


private:
    Ui::CreateShapeFileDialog *ui;
    void onBrowseOutputPath();
    void onAddField();
    void onOkClicked();
    void onRemoveField();
private:
    QStandardItemModel *fieldModel;

};

#endif // CREATESHAPEFILEDIALOG_H
