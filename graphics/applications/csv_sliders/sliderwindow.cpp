#include "sliderwindow.h"
#include <QLabel>
#include <QDebug>
#include <QHBoxLayout>
#include <iostream>
#include <sstream>

// SliderWindow::SliderWindow(const QMap<QString, QPair<float, float>> &sliderMap, QWidget *parent)
SliderWindow::SliderWindow(const boost::ptr_vector< snark::sliders::slider_base >& sliders, QWidget *parent)
        : QMainWindow(parent) {
    QWidget *centralWidget = new QWidget(this);
    setCentralWidget(centralWidget);

    sliderLayout = new QVBoxLayout(centralWidget);
    // createSliders(sliderMap);
}

void SliderWindow::createSliders(const QMap<QString, QPair<float, float>> &sliderMap) {
    for (const QString &sliderName : sliderMap.keys()) {
        QLabel *label = new QLabel(sliderName, this);
        sliderLayout->addWidget(label);

        FloatSlider *slider = new FloatSlider(Qt::Horizontal, this);
        slider->setMinimum(sliderMap[sliderName].first);
        slider->setMaximum(sliderMap[sliderName].second);
        
        QHBoxLayout *sliderValueLayout = new QHBoxLayout();
        
        QLabel *minValueLabel = new QLabel(QString::number(slider->minimum()), this);
        sliderValueLayout->addWidget(minValueLabel);

        QLabel *currentValueLabel = new QLabel(QString::number(slider->value()), this);
        sliderValueLayout->addWidget(currentValueLabel);
        
        QLabel *maxValueLabel = new QLabel(QString::number(slider->maximum()), this);
        sliderValueLayout->addWidget(maxValueLabel);

        sliderLayout->addWidget(slider);
        sliderLayout->addLayout(sliderValueLayout);

        sliders.push_back(slider);

        connect(slider, &FloatSlider::sliderReleased, this, &SliderWindow::onSliderValueChanged);
        connect(slider, &QSlider::valueChanged, [currentValueLabel, slider](int value) {
            currentValueLabel->setText(QString::number(slider->convertValue(value)));
        });
    }
}

void SliderWindow::onSliderValueChanged() {
    std::vector<float> values;
    for (const FloatSlider *slider : sliders) { values.push_back(slider->value()); }

    if(binary){
        std::cout.write(reinterpret_cast<const char*>(values.data()), values.size() * sizeof(float));
        std::cout.flush();
    }else{
        for (int i = 0; i < values.size(); i++) { std::cout << values[i]; if(i != values.size() - 1){ std::cout << ","; } }
        std::cout << std::endl;        
    }
}
