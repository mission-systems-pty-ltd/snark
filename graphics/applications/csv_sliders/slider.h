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


namespace snark { namespace sliders {

template < typename T >
struct config
{
    std::string name;
    T max{1};
    T min{0};
    T default_value{0};
    std::string format;
};

struct input
{
    comma::uint32 block{0};
};

class slider_base
{
public:
    slider_base() = default;
    slider_base( unsigned int offset, unsigned int size ){};
    virtual ~slider_base() = default;
    virtual void write( char* buffer ) const = 0;
    virtual std::string as_string() const = 0;
protected:
    unsigned int _offset{0};
    unsigned int _size{0};
};

template < typename T > struct slider_traits { static const char* ptr( const T& value ) { return reinterpret_cast< const char* >( &value ); } };
template <> struct slider_traits< std::string > { static const char* ptr( const std::string& value ) { return &value[0]; } };
template <> struct slider_traits< comma::uint16 > { static const char* ptr( const comma::uint16& value ) { return reinterpret_cast< const char* >( &value ); } };
template <> struct slider_traits< comma::int32 > { static const char* ptr( const comma::int32& value ) { return reinterpret_cast< const char* >( &value ); } };
template <> struct slider_traits< comma::uint32 > { static const char* ptr( const comma::uint32& value ) { return reinterpret_cast< const char* >( &value ); } };
template <> struct slider_traits< comma::int64 > { static const char* ptr( const comma::int64& value ) { return reinterpret_cast< const char* >( &value ); } };
template <> struct slider_traits< comma::uint64 > { static const char* ptr( const comma::uint64& value ) { return reinterpret_cast< const char* >( &value ); } };
template <> struct slider_traits< float > { static const char* ptr( const float& value ) { return reinterpret_cast< const char* >( &value ); } };
template <> struct slider_traits< double > { static const char* ptr( const double& value ) { return reinterpret_cast< const char* >( &value ); } };

template < typename T >
class slider: public slider_base
{
public:
    slider( unsigned int offset, unsigned int size = 0 ): slider_base( offset, size == 0 ? sizeof( T ) : size ) {}
    std::string as_string() const { return std::to_string( _value ); }
    void write( char* buffer ) const { std::memcpy( buffer + _offset, slider_traits< T >::ptr( _value ), this->_size ); }
private:
    T _value{0};
};

} } // namespace snark { namespace sliders {
