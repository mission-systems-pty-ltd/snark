// Copyright (c) 2023 Vsevolod Vlaskine

/// @author Vsevolod Vlaskine

#pragma once

#include <iostream>
#include <string>
#include <comma/csv/options.h>
#include <comma/csv/stream.h>
#include <comma/string/string.h>

namespace snark { namespace ros {

namespace detail {

struct publisher
{
    virtual ~publisher() {}
    virtual std::string input_fields() const = 0;
    virtual bool publish() = 0;
};

template < typename T >
class publisher_of: public publisher
{
    public:
        publisher( std::istream& is, const comma::csv::options& csv ): _is( is, csv ) {}
        std::string input_fields( bool full_xpath ) const { return comma::join( comma::csv::names< T >( full_xpath ), ',' ); }
        void topic( const std::string& topic, ros_stuff ); // todo
        bool publish()
        {
            auto p = _is.read();
            if( !p ) { return false; }
            // todo: publish record on ros (may require passing extra )
        }
    private:
        comma::csv::input_stream< T > _is;
};

publisher* get_publisher( const std::string& message, const std::string& topic ) // todo: pass whatever is needed for ros
{
    //something like
    auto p = publishers[message];
    p->topic( topic ); // or alike
}

} // namespace detail {

class publisher
{
    public:
        publisher( const std::string& message, const std::string& topic, const comma::csv::options& csv ): _publisher( detail::get_publisher( message, topic, ros_stuff ) ) {}
        std::string input_fields() const { return _publisher->input_fields(); }
        bool publish_once() { return _publisher->publish(); }

    private:
        detail::publisher* _publisher;
};

} } // namespace snark { namespace ros {
