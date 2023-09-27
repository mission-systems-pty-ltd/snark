#pragma once
#include <iostream>
#include <QApplication>
#include <boost/ptr_container/ptr_vector.hpp>
#include <comma/application/command_line_options.h>
#include <comma/base/types.h>
#include <comma/csv/stream.h>
#include <comma/csv/traits.h>
#include <comma/name_value/parser.h>
#include <comma/visiting/traits.h>

namespace snark { namespace graphics { namespace sliders {

enum class slider_type { none_, float_, double_, uint16_, int32_, uint32_, int64_, uint64_, string_ };

template < typename T >
struct config
{
    std::string name;
    T max{1};
    T min{0};
    T default_value{0};
    T step{0.1};
    std::string format; 
    std::string style;
    bool vertical{false}; // todo: plug in
};

struct input
{
    comma::uint32 block{0};
};

class slider_base
{
public:
    slider_base() = default;
    slider_base( unsigned int offset, unsigned int size ) 
        : _offset(offset), _size( size ) 
    {}
    virtual ~slider_base() = default;
    virtual void write( char* buffer ) const = 0;
    virtual std::string as_string() const = 0;
    void set_name( const std::string& name ) { this->_name = name; }
    const std::string& name() const { return _name; }
    // virtual snark::graphics::sliders::slider_type type() =0;
    virtual const snark::graphics::sliders::slider_type type() const =0;// {return slider_type::none_; }
protected:
    std::string _name;
    unsigned int _offset{0};
    unsigned int _size{0};
    snark::graphics::sliders::slider_type _type{slider_type::none_};
};

// std::shared_ptr< std::vector<snark::graphics::sliders::slider_base> >
using sliders_ptr = std::vector<std::shared_ptr< snark::graphics::sliders::slider_base> >;
using slider_ptr = std::shared_ptr< snark::graphics::sliders::slider_base>;
template < typename T > struct slider_traits { static const char* ptr( const T& value ) { return reinterpret_cast< const char* >( &value ); } 
static const slider_type type() { return slider_type::none_; }  };
template <> struct slider_traits< std::string > { static const char* ptr( const std::string& value ) { return &value[0]; } 
static const slider_type type() { return slider_type::string_; }  };
template <> struct slider_traits< comma::uint16 > { static const char* ptr( const comma::uint16& value ) { return reinterpret_cast< const char* >( &value ); } 
static const slider_type type() { return slider_type::uint16_; }  };
template <> struct slider_traits< comma::int32 > { static const char* ptr( const comma::int32& value ) { return reinterpret_cast< const char* >( &value ); } 
static const slider_type type() { return slider_type::int32_; }  };
template <> struct slider_traits< comma::uint32 > { static const char* ptr( const comma::uint32& value ) { return reinterpret_cast< const char* >( &value ); } 
static const slider_type type() { return slider_type::uint32_; }  };
template <> struct slider_traits< comma::int64 > { static const char* ptr( const comma::int64& value ) { return reinterpret_cast< const char* >( &value ); } 
static const slider_type type() { return slider_type::int64_; }  };
template <> struct slider_traits< comma::uint64 > { static const char* ptr( const comma::uint64& value ) { return reinterpret_cast< const char* >( &value ); } 
static const slider_type type() { return slider_type::uint64_; }  };
template <> struct slider_traits< float > { static const char* ptr( const float& value ) { return reinterpret_cast< const char* >( &value ); } 
static const slider_type type() { return slider_type::float_; }  };
template <> struct slider_traits< double > { static const char* ptr( const double& value ) { return reinterpret_cast< const char* >( &value ); } 
static const slider_type type() { return slider_type::double_; }  };

// TODO: 
// template < T > slider_traits< T > struct type(  )
// template < typename T > struct slider_traits { static const char* ptr( const T& value ) { return reinterpret_cast< const char* >( &value ); } };
// template <> slider_traits< T >::type(  )

template < typename T >
class slider: public slider_base
{
public:
    slider( unsigned int offset, unsigned int size = 0 ): slider_base( offset, size == 0 ? sizeof( T ) : size ) {}
    std::string as_string() const { return std::to_string( _value ); }
    void write( char* buffer ) const { std::memcpy( buffer + _offset, slider_traits< T >::ptr( _value ), this->_size ); }
    slider<T>& set( const T& value ) { _value = value; return *this; }
    slider<T>& set_max( const T& value ) { _max = value; return *this; }
    slider<T>& set_min( const T& value ) { _min = value; return *this; }
    T min() const { return _min; }
    T max() const { return _max; }
    const snark::graphics::sliders::slider_type type( ) const { return slider_traits< T >::type(); }
private:
    T _value{0};
    T _max{0};
    T _min{0};
};

}}} // namespace snark { namespace graphics { namespace sliders {

