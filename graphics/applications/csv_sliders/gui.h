#pragma once

#include <cmath>
#include <boost/ptr_container/ptr_vector.hpp>
#include <QDebug>
#include <QDoubleSpinBox>
#include <QFontDatabase>
#include <QLabel>
#include <QLineEdit>
#include <QMap>
#include <QPair>
#include <QShortcut>
#include <QSlider>
#include <QThread>
#include <QValidator>
#include <QVBoxLayout>
#include "slider.h"

namespace snark { namespace graphics { namespace sliders {

class FloatSlider : public QSlider
{
    Q_OBJECT

    public:
        FloatSlider( Qt::Orientation orientation, QWidget* parent = nullptr, float default_value = 0, unsigned int precision = 2 )
            : QSlider(orientation, parent)
            , _default( default_value )
            , _precision( precision )
            , _precision_factor( std::pow( 10.0, _precision ) )
        {
        }

        float value() const { return float( QSlider::value() ) / _precision_factor; }

        void setValue( float value )
        { 
            QSlider::setValue( std::round( value * _precision_factor ) ); 
            value_updated_ = true;
        }

        void reset() { setValue( _default ); }

        void unsetUpdated() { value_updated_ = false; }

        bool valueUpdated() const { return value_updated_; }

        void setMinimum( float value ) { QSlider::setMinimum(std::round(value * _precision_factor)); }

        void setMaximum( float value ) { QSlider::setMaximum(std::round(value * _precision_factor)); }

        // overload minimum to run min furst and then divide by 10^precision
        int minimum() const { return QSlider::minimum() / _precision_factor; }

        // overload maximum to run max furst and then divide by 10^precision
        int maximum() const { return QSlider::maximum() / _precision_factor; }

        float convertValue( int value ) const { return ( value ) / _precision_factor; }

        void setName( const std::string& name ) { this->name_ = name; }

        std::string name() const { return name_; }

    private:
        float _default;
        unsigned int _precision;
        bool value_updated_{false};
        double _precision_factor;
        std::string name_;
}; 

class FloatValidator : public QValidator
{
    public:
        FloatValidator( float min, float max, int decimals, QObject *parent = nullptr ): QValidator(parent), _min(min), _max(max), _decimals(decimals) {}

        QValidator::State validate( QString &s, int &pos ) const override
        {
            Q_UNUSED( pos )
            if( s.isEmpty() || s == "-" || s == "+" ) { return QValidator::Intermediate; }
            bool ok{false};
            float value = s.toFloat( &ok );
            return ok && value >= _min && value <= _max ? QValidator::Acceptable : QValidator::Invalid;
        }

    private:
        float _min, _max;
        int _decimals;
};

template< typename T >
inline void draw_slider( QVBoxLayout& main_layout, const snark::graphics::sliders::config< T >& slider_config, std::vector< snark::graphics::sliders::FloatSlider*>& sliders, int max_name_length = 0 )
{
    QFont font;
    font.setFamily("Courier New");  // You can also try "Monospace", etc.
    font.setPointSize(12);
    QHBoxLayout* slider_layout = new QHBoxLayout();
    QString name = QString::fromStdString(slider_config.name);
    name = name.leftJustified(max_name_length, ' ');
    QLabel* name_label = new QLabel(name);
    name_label->setFont(font);
    slider_layout->addWidget(name_label);
    QHBoxLayout* nested_layout = new QHBoxLayout();
    QString range = QString("%1:%2").arg(slider_config.min).arg(slider_config.max);
    QLabel* range_label = new QLabel(range);
    range_label->setFont(font);
    range_label->setAlignment(Qt::AlignVCenter | Qt::AlignRight);
    nested_layout->addWidget(range_label, 1);
    snark::graphics::sliders::FloatSlider* slider = new snark::graphics::sliders::FloatSlider( Qt::Horizontal, nullptr, slider_config.default_value );
    slider->setName( slider_config.name );
    slider->setMinimum( slider_config.min );
    slider->setMaximum( slider_config.max );
    slider->setValue( slider_config.default_value );
    sliders.push_back( slider ); // Store the pointer
    nested_layout->addWidget( slider, 2 );
    slider_layout->addLayout( nested_layout );
    QDoubleSpinBox* value_spin_box = new QDoubleSpinBox();
    value_spin_box->setFont( font );
    value_spin_box->setMinimum( slider_config.min );
    value_spin_box->setMaximum( slider_config.max );
    value_spin_box->setFixedWidth(100);
    value_spin_box->setSingleStep( slider_config.step );  // Set the step size as you like
    value_spin_box->setValue( slider_config.default_value );
    QObject::connect( slider, &snark::graphics::sliders::FloatSlider::valueChanged, [ value_spin_box, slider ]()
    {
        value_spin_box->setValue(slider->value());
    } );
    QObject::connect( value_spin_box, QOverload< double >::of( &QDoubleSpinBox::valueChanged ), [ value_spin_box, slider ]( double value )
    {
        slider->setValue( static_cast< float >( value ) );
    } );
    slider_layout->addWidget( value_spin_box );
    main_layout.addLayout( slider_layout );
}

// todo
//   - move implementation to cpp
class main_window : public QWidget
{
    Q_OBJECT

    public:
        typedef std::vector< snark::graphics::sliders::FloatSlider* > sliders_t;

        sliders_t sliders;

        main_window()
            : _escape( QKeySequence( Qt::Key_Escape ), this, SLOT( _exit_hard() ) )
            , _ctrl_p( QKeySequence( tr( "Ctrl+P" ) ), this, SLOT( _print_current_values() ) )
            , _ctrl_r( QKeySequence( tr( "Ctrl+R" ) ), this, SLOT( _reset() ) )
            , _ctrl_w( QKeySequence( tr( "Ctrl+W" ) ), this, SLOT( _exit_hard() ) )
        {
        }

        static std::string usage()
        {
            return R"(hot keys
    <esc>    : exit
    <ctrl+p> : print current slider values as path-value to stderr
    <ctrl+r> : reset all sliders to default values)";
        }

    private slots:
        void _exit_hard() { exit( 0 ); } // quick and dirty for now, otherwise other threads hang

        void _print_current_values() { std::string d; for( auto& slider: sliders ) { std::cerr << d << slider->name() << "=" << slider->value(); d = ";"; } std::cerr << std::endl; }

        void _reset() { for( auto& slider: sliders ) { slider->reset(); } }
        
    private:
        QShortcut _escape;
        QShortcut _ctrl_p;
        QShortcut _ctrl_r;
        QShortcut _ctrl_w;
};

} } } // namespace snark { namespace graphics { namespace sliders {
