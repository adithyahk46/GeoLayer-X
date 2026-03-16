#ifndef COLORPICKERCHECKBOX_H
#define COLORPICKERCHECKBOX_H


#include <QCheckBox>
#include <QColor>

class ColorPickerCheckBox : public QCheckBox
{
    Q_OBJECT

public:
    explicit ColorPickerCheckBox(const QColor& color,
                                 QWidget* parent = nullptr);

    QColor color() const;

signals:
    void colorChanged(const QColor& color);

private slots:
    void onClicked();

private:

    QColor m_color;

    void updateAppearance();
};

#endif // COLORPICKERCHECKBOX_H
