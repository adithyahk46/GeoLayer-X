#ifndef IMGUISLIDER_H
#define IMGUISLIDER_H
#pragma once

#include <QSlider>

#include "customWidgets_global.h"

class CUSTOMWIDGETS_EXPORT ImGuiSlider : public QSlider {
    Q_OBJECT
    Q_PROPERTY(bool clickToJump READ isClickToJumpEnabled WRITE setClickToJumpEnabled)
    Q_PROPERTY(int valueScale READ valueScale WRITE setValueScale)
    Q_PROPERTY(int valuePrecision READ valuePrecision WRITE setValuePrecision)

public:
    explicit ImGuiSlider(QWidget *parent = nullptr);

    bool isClickToJumpEnabled() const { return m_clickToJump; }
    void setClickToJumpEnabled(bool enabled) { m_clickToJump = enabled; }
    int valueScale() const { return m_valueScale; }
    void setValueScale(int scale);
    int valuePrecision() const { return m_valuePrecision; }
    void setValuePrecision(int precision);

protected:
    void paintEvent(QPaintEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;

private:
    bool m_clickToJump;
    int m_valueScale = 1000;
    int m_valuePrecision = 3;
};
#endif // IMGUISLIDER_H
