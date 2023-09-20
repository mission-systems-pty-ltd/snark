#pragma once

#include <QMainWindow>
#include <QMap>
#include <QPair>
#include <QSlider>
#include <QVBoxLayout>
#include <QDebug>
#include <QLabel>
#include <boost/ptr_container/ptr_vector.hpp>
#include "slider.h"

#include <cmath>
namespace snark { namespace sliders {

class FloatSlider : public QSlider {
    Q_OBJECT
public:
    FloatSlider(Qt::Orientation orientation, QWidget* parent = nullptr, int precision = 2)
        : QSlider(orientation, parent), m_precision(precision) {}

    float value() const { return (float)QSlider::value() / std::pow(10.0, m_precision); }

    void setValue(float value) { QSlider::setValue(std::round(value * std::pow(10.0, m_precision))); }

    void setMinimum(float value) { QSlider::setMinimum(std::round(value * std::pow(10.0, m_precision))); }

    void setMaximum(float value) { QSlider::setMaximum(std::round(value * std::pow(10.0, m_precision))); }

    // overload minimum to run min furst and then divide by 10^precision
    int minimum() const { return QSlider::minimum() / std::pow(10.0, m_precision); }

    // overload maximum to run max furst and then divide by 10^precision
    int maximum() const { return QSlider::maximum() / std::pow(10.0, m_precision); }

    float convertValue(int value) const { return (float)value / std::pow(10.0, m_precision); }

    void setName(const std::string& name){ this->name_ = name; }
    std::string name(){ return name_; }

private:
    int m_precision;
    std::string name_;
}; 

}} // namespace snark { namespace sliders {