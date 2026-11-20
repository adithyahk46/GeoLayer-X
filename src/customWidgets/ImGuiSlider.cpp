#include "imguislider.h"
#include <QPainter>
#include <QMouseEvent>
#include <QWheelEvent>

ImGuiSlider::ImGuiSlider(QWidget *parent)
    : QSlider(Qt::Horizontal, parent),
    m_clickToJump(false) {
    setMinimumHeight(24);
    setCursor(Qt::SplitHCursor);
}

void ImGuiSlider::setValueScale(int scale) {
    if (scale <= 0 || m_valueScale == scale)
        return;
    m_valueScale = scale;
    update();
}

void ImGuiSlider::setValuePrecision(int precision) {
    const int bounded = qBound(0, precision, 6);
    if (m_valuePrecision == bounded)
        return;
    m_valuePrecision = bounded;
    update();
}

void ImGuiSlider::mousePressEvent(QMouseEvent *event) {
    if (event->button() == Qt::LeftButton && m_clickToJump) {
        double ratio = static_cast<double>(event->pos().x()) / width();
        int newVal = minimum() + qRound(ratio * (maximum() - minimum()));
        setValue(newVal);
    }
    QSlider::mousePressEvent(event);
}

void ImGuiSlider::wheelEvent(QWheelEvent *event) {
    event->ignore();
}

void ImGuiSlider::paintEvent(QPaintEvent *) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, false);

    QRect rect = this->rect();

    // 1. Pull colors dynamically from the application's active QPalette / Theme
    QColor bgColor = palette().color(QPalette::Base);
    QColor activeBgColor = palette().color(QPalette::AlternateBase);
    QColor highlightColor = palette().color(QPalette::Highlight);
    QColor textColor = palette().color(QPalette::WindowText);

    // Fallback if AlternateBase isn't defined by the current theme
    if (!palette().isBrushSet(QPalette::Active, QPalette::AlternateBase)) {
        activeBgColor = bgColor.lighter(130);
    }

    // 2. Draw Background Track
    painter.fillRect(rect, bgColor);

    // Calculate fill proportion based on current slider value
    double range = maximum() - minimum();
    double ratio = (range > 0) ? static_cast<double>(value() - minimum()) / range : 0.0;
    int fillWidth = static_cast<int>(rect.width() * ratio);

    // Draw filled portion representing progress
    QRect fillRect(rect.left(), rect.top(), fillWidth, rect.height());
    painter.fillRect(fillRect, activeBgColor);

    // 3. Draw the moving handle block (using theme's highlight color)
    int handleWidth = 14;
    int handleHeight = rect.height() - 6;
    int handleX = static_cast<int>((rect.width() - handleWidth) * ratio);
    int handleY = (rect.height() - handleHeight) / 2;

    QRect handleRect(handleX, handleY, handleWidth, handleHeight);
    painter.fillRect(handleRect, highlightColor);

    // 4. Format Value Text (Respecting widget/app font)
    double displayedValue = static_cast<double>(value()) / m_valueScale;
    QString text = QString::number(displayedValue, 'f', m_valuePrecision);

    QFont font = this->font();
    painter.setFont(font);

    QFontMetrics fm(font);
    int textWidth = fm.horizontalAdvance(text);

    int textX = (rect.width() - textWidth) / 2;
    int textY = rect.top() + (rect.height() + fm.ascent() - fm.descent()) / 2;

    // 5. Draw Text on Top (Using theme's text color)
    painter.setPen(textColor);
    painter.drawText(textX, textY, text);
}