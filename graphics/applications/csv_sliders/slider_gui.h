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
namespace snark { namespace graphics { namespace sliders {

class FloatSlider : public QSlider {
    Q_OBJECT
public:
    FloatSlider(Qt::Orientation orientation, QWidget* parent = nullptr, unsigned int precision = 2)
        : QSlider(orientation, parent)
        , m_precision(precision)
        , _precision_factor( std::pow( 10.0, m_precision ) ) {}

    float value() const { return (float)QSlider::value() / _precision_factor; }

    void setValue(float value) { QSlider::setValue(std::round(value * _precision_factor)); }

    void setMinimum(float value) { QSlider::setMinimum(std::round(value * _precision_factor)); }

    void setMaximum(float value) { QSlider::setMaximum(std::round(value * _precision_factor)); }

    // overload minimum to run min furst and then divide by 10^precision
    int minimum() const { return QSlider::minimum() / _precision_factor; }

    // overload maximum to run max furst and then divide by 10^precision
    int maximum() const { return QSlider::maximum() / _precision_factor; }

    float convertValue(int value) const { return (float)value / _precision_factor; }

    void setName(const std::string& name){ this->name_ = name; }
    std::string name() const { return name_; }

private:
    int m_precision;
    double _precision_factor;
    std::string name_;
}; 

}}} // namespace snark { namespace graphics { namespace sliders {
