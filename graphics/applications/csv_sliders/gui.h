#pragma once

#include <cmath>
#include <boost/ptr_container/ptr_vector.hpp>
#include <QDebug>
#include <QLabel>
#include <QMap>
#include <QPair>
#include <QShortcut>
#include <QSlider>
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

// todo
//   - move implementation to csv
//   - ctrl+v: print current settings to stderr
//   - ctrl+r: reset to defaults
class main_window : public QWidget
{
    Q_OBJECT

    public:
        typedef std::vector< snark::graphics::sliders::FloatSlider* > sliders_t;
        sliders_t sliders;

        main_window()
            : _esc( new QShortcut( QKeySequence( Qt::Key_Escape ), this, SLOT( _exit_hard() ) ) )
            , _ctrl_p( new QShortcut( QKeySequence( tr( "Ctrl+P" ) ), this, SLOT( _print_current_values() ) ) )
            , _ctrl_r( new QShortcut( QKeySequence( tr( "Ctrl+R" ) ), this, SLOT( _reset() ) ) )
        {
        }

    private slots:
        void _exit_hard() { exit( 0 ); } // quick and dirty for now, otherwise other threads hang

        void _print_current_values() { std::cerr << "ctrl+p" << std::endl; } // todo

        void _reset() { for( auto& slider: sliders ) { slider->reset(); } }
        
    private:
        QShortcut* _esc{nullptr};
        QShortcut* _ctrl_p{nullptr};
        QShortcut* _ctrl_r{nullptr};
};

} } } // namespace snark { namespace graphics { namespace sliders {
