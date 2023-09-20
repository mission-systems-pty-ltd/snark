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

class FloatSlider : public QSlider {
    Q_OBJECT
public:
    FloatSlider(Qt::Orientation orientation, QWidget* parent = nullptr, int precision = 2)
        : QSlider(orientation, parent), m_precision(precision) {}

    float value() const {
        return (float)QSlider::value() / std::pow(10.0, 2);
    }

    void setValue(float value) {
        QSlider::setValue(std::round(value * std::pow(10.0, m_precision)));
    }

    void setMinimum(float value) {
        QSlider::setMinimum(std::round(value * std::pow(10.0, m_precision)));
    }

    void setMaximum(float value) {
        QSlider::setMaximum(std::round(value * std::pow(10.0, m_precision)));
    }

    // overload minimum to run min furst and then divide by 10^precision
    int minimum() const {
        return QSlider::minimum() / std::pow(10.0, m_precision);
    }

    // overload maximum to run max furst and then divide by 10^precision
    int maximum() const {
        return QSlider::maximum() / std::pow(10.0, m_precision);
    }

    float convertValue(int value) const {
        return (float)value / std::pow(10.0, m_precision);
    }

private:
    int m_precision;
};

class SliderWindow : public QMainWindow {
    Q_OBJECT

public:
    // explicit SliderWindow(const QMap<QString, QPair<float, float>> &sliderMap, QWidget *parent = nullptr);
    explicit SliderWindow(const boost::ptr_vector< snark::sliders::slider_base >& sliders, QWidget *parent = nullptr);
    void setBinary(bool binary){ this->binary = binary; }
    void setFlush(bool flush){ this->flush = flush; }
private:
    void createSliders(const QMap<QString, QPair<float, float>> &sliderMap);
    void onSliderValueChanged();

    QVBoxLayout *sliderLayout;
    QVector<FloatSlider *> sliders;
    bool binary{false};
    bool flush{false};
};

