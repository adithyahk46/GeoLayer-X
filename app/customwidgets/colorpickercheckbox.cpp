#include "colorpickercheckbox.h"


ColorPickerCheckBox::ColorPickerCheckBox(const QColor& color, QWidget* parent)
    : QCheckBox(parent),
      m_color(color)
{
    setCheckable(false);
    setTristate(false);
    setFocusPolicy(Qt::NoFocus);
    setCursor(Qt::PointingHandCursor);

    updateAppearance();

    connect(this, &QCheckBox::clicked,
            this, &ColorPickerCheckBox::onClicked);
}

QColor ColorPickerCheckBox::color() const
{
    return m_color;
}

void ColorPickerCheckBox::onClicked()
{
    QColor newColor = QColorDialog::getColor(
        m_color,
        this,
        "Select Color",
        QColorDialog::ShowAlphaChannel
    );

    if (!newColor.isValid())
        return;

    m_color = newColor;
    updateAppearance();
    emit colorChanged(m_color);
}

void ColorPickerCheckBox::updateAppearance()
{
    setText(m_color.name().toUpper());

    setStyleSheet(QString(R"(
        QCheckBox::indicator {
            width: 18px;
            height: 18px;
            background-color: %1;
        }
    )").arg(m_color.name()));
}
