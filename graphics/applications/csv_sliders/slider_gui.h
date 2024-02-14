#pragma once

#include <cmath>
#include <boost/ptr_container/ptr_vector.hpp>
#include <QDebug>
#include <QLabel>
#include <QMap>
#include <QPair>
#include <QSlider>
#include <QValidator>
#include <QVBoxLayout>
#include "slider.h"

namespace snark { namespace graphics { namespace sliders {

class FloatSlider : public QSlider {
    Q_OBJECT
public:
    FloatSlider(Qt::Orientation orientation, QWidget* parent = nullptr, unsigned int precision = 2)
        : QSlider(orientation, parent)
        , m_precision(precision)
        , _precision_factor( std::pow( 10.0, m_precision ) ) {}

    float value() const { return (float)QSlider::value() / _precision_factor; }

    void setValue(float value) { 
        QSlider::setValue(std::round(value * _precision_factor)); 
        value_updated_ = true;
    }

    void unsetUpdated() { value_updated_ = false; }

    bool valueUpdated() const { return value_updated_; }

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
    bool value_updated_{false};
    double _precision_factor;
    std::string name_;
}; 

class FloatValidator : public QValidator
{
public:
    FloatValidator(float min, float max, int decimals, QObject *parent = nullptr)
        : QValidator(parent), _min(min), _max(max), _decimals(decimals) {}

    QValidator::State validate(QString &s, int &pos) const override {
        Q_UNUSED(pos)
        if (s.isEmpty() || s == "-" || s == "+") {
            return QValidator::Intermediate;
        }
        bool ok;
        float value = s.toFloat(&ok);
        if (ok && value >= _min && value <= _max) {
            return QValidator::Acceptable;
        } else {
            return QValidator::Invalid;
        }
    }

private:
    float _min, _max;
    int _decimals;
};

}}} // namespace snark { namespace graphics { namespace sliders {
