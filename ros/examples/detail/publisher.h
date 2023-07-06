// Copyright (c) 2023 Vsevolod Vlaskine

/// @author Vsevolod Vlaskine

#pragma once

#include <iostream>
#include <map>
#include <string>
#include <comma/base/exception.h>
#include <comma/csv/options.h>
#include <comma/csv/stream.h>
#include <comma/string/string.h>

namespace snark { namespace ros { namespace csv {

namespace detail {

struct publisher
{
    virtual ~publisher() {}
    virtual std::string input_fields() const = 0;
    virtual void init( std::istream& is, const comma::csv::options& csv, whatever_ros_stuff ) = 0;
    virtual bool publish() = 0;
};

template < typename T >
class publisher_of: public publisher
{
    public:
        std::string input_fields( bool full_xpath ) const { return comma::join( comma::csv::names< T >( full_xpath ), ',' ); }
        void init( std::istream& is, const comma::csv::options& csv, whatever_ros_stuff )
        {
            _istream.reset( new comma::csv::input_stream< T >( is, csv ) );
            // todo: init whatever_ros_stuff
        }
        bool publish()
        {
            auto p = _istream->read();
            if( !p ) { return false; }
            // todo: publish record on ros
            return true;
        }
    private:
        std::unique_ptr< comma::csv::input_stream< T > > _istream;
};

class publishers
{
    public:
        publisher* operator[]( const std::string& message ) // todo: move to cpp file
        {
            auto it = _map.find( message );
            COMMA_THROW_IF( it == _map.end(), "publisher for message type '" << message << "' not found" );
            return it->second;
        }

        void insert( const std::pair< std::string, publisher* >& p ) // todo: move to cpp file
        {
            COMMA_ASSERT( _map.insert( p ).second, "cannot insert publisher for message type '" << message << "': it already exists" );
        }

        const std::map< std::string, publisher* >& map() const { return _map; }

        static publishers& instance() { static publishers; return publishers; }

        static const publishers& const_instance() { return instance(); }

    private:
        std::map< std::string, publisher* > _map;
};

} // namespace detail {

class publisher
{
    public:
        publisher( const std::string& message, const std::string& topic, const comma::csv::options& csv ): _publisher( detail::publishers[message] ) ) { _publisher->init( csv, whatever_ros_stuff ); }
        std::string input_fields() const { return _publisher->input_fields(); }
        bool publish_once() { return _publisher->publish(); }

    private:
        detail::publisher* _publisher;
};

} } } // namespace snark { namespace ros { namespace csv {
