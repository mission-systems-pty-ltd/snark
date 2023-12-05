/*
 * File:   serial_io.cpp
 * Author: Terraneo Federico
 * Distributed under the Boost Software License, Version 1.0.
 * Created on September 12, 2009, 3:47 PM
 *
 * v1.05: Fixed a bug regarding reading after a timeout (again).
 *
 * v1.04: Fixed bug with timeout set to zero
 *
 * v1.03: Fix for Mac OS X, now fully working on Mac.
 *
 * v1.02: Code cleanup, speed improvements, bug fixes.
 *
 * v1.01: Fixed a bug that caused errors while reading after a timeout.
 *
 * v1.00: First release.
 */

#include "serial_io.h"
#include <string>
#include <algorithm>
#include <iostream>
#include <boost/bind/bind.hpp>
#include <comma/base/exception.h>

using namespace std;
using namespace boost;

namespace snark { namespace ocean {

serial_io::serial_io() : io_(), port_(io_), timer_(io_),
        timeout_(posix_time::seconds(0)) {}

serial_io::serial_io(const std::string& devname, unsigned int baud_rate,
        asio::serial_port_base::parity opt_parity,
        asio::serial_port_base::character_size opt_csize,
        asio::serial_port_base::flow_control opt_flow,
        asio::serial_port_base::stop_bits opt_stop)
        : io_(), port_(io_), timer_(io_), timeout_(posix_time::seconds(0))
{
    open(devname, baud_rate, opt_parity, opt_csize, opt_flow, opt_stop);
}

void serial_io::open(const std::string& devname, unsigned int baud_rate,
        asio::serial_port_base::parity opt_parity,
        asio::serial_port_base::character_size opt_csize,
        asio::serial_port_base::flow_control opt_flow,
        asio::serial_port_base::stop_bits opt_stop)
{
    if(is_open()) close();
    port_.open(devname);
    port_.set_option(asio::serial_port_base::baud_rate(baud_rate));
    port_.set_option(opt_parity);
    port_.set_option(opt_csize);
    port_.set_option(opt_flow);
    port_.set_option(opt_stop);

}
serial_io::~serial_io() {}

bool serial_io::is_open() const
{
    return port_.is_open();
}

void serial_io::close()
{
    if(is_open()==false) return;
    port_.close();
}

void serial_io::set_timeout(const posix_time::time_duration& t)
{
    timeout_ = t;
}

void serial_io::write(const char *data, size_t size)
{
    asio::write(port_, asio::buffer(data,size));
}

void serial_io::write(const std::vector<char>& data)
{
    asio::write(port_, asio::buffer(&data[0], data.size()));
}

void serial_io::write(const std::string& s)
{
    asio::write(port_, asio::buffer(s.c_str(), s.size()));
}

void serial_io::read(char *data,  size_t size)
{
    if(read_data_.size()>0)//If there is some data from a previous read
    {
        istream is(&read_data_);
        size_t toRead=min(read_data_.size(), size);//How many bytes to read?
        is.read(data, toRead);
        data+=toRead;
        size-=toRead;
        if(size==0) return;//If read data was enough,  just return
    }

    setup_parameters_ = read_setup_parameters(data, size);
    perform_read_setup( setup_parameters_ );

    //For this code to work,  there should always be a timeout_, so the
    //request for no timeout_ is translated into a very long timeout_
    if(timeout_!=posix_time::seconds(0)) timer_.expires_from_now(timeout_);
    else timer_.expires_from_now(posix_time::hours(100000));

    timer_.async_wait(boost::bind(&serial_io::timeout_expired, this,
                asio::placeholders::error));

    result_ = result_in_progress;
    bytes_transferred_=0;
    for(;;)
    {
        io_.run_one();
        switch( result_ )
        {
            case  result_success:
                timer_.cancel();
                return;
            case result_timeout_expired:
                port_.cancel();
                throw(timeout_exception("timeout_ expired"));
            case  result_error:
                timer_.cancel();
                port_.cancel();
                throw(boost::system::system_error(boost::system::error_code(),
                        "Error while reading"));
            case  result_in_progress:
                continue;
            default:
                continue;
            //if  result_in_progress remain in the loop
        }
    }
}

std::vector<char> serial_io::read(size_t size)
{
    vector<char> result(size,'\0');//Allocate a vector with the desired size
    read(&result[0],size);//Fill it with values
    return result;
}

std::string serial_io::read_string(size_t size)
{
    string result(size,'\0');//Allocate a string with the desired size
    read(&result[0],size);//Fill it with values
    return result;
}

void serial_io::read_string_until_async(const string& delim)
{
    // Note: if read_data_ contains some previously read data, the call to
    // async_read_until (which is done in perform_read_setup) correctly handles
    // it. If the data is enough it will also immediately call readCompleted()
    setup_parameters_ = read_setup_parameters(delim);
    perform_read_setup( setup_parameters_ );

    //For this code to work, there should always be a timeout_, so the
    //request for no timeout_ is translated into a very long timeout_
    if(timeout_!=posix_time::seconds(0)) timer_.expires_from_now(timeout_);
    else timer_.expires_from_now(posix_time::hours(100000));

    timer_.async_wait(boost::bind(&serial_io::timeout_expired, this,
                asio::placeholders::error));

    result_ =  result_in_progress;
}

bool serial_io::check_async( string& line )
{
    io_.poll();
    switch( result_ )
    {
        case  result_success:
            {
                timer_.cancel();
                istream is(&read_data_);
                std::getline( is, line );
                return true;
            }
        case  result_timeout_expired:
            port_.cancel();
            COMMA_THROW( comma::exception, "serial timeout_ exception on read encounterred" );
            //throw(timeout_exception("timeout_ expired"));
        case  result_error:
            timer_.cancel();
            port_.cancel();
            throw(boost::system::system_error(boost::system::error_code(),
                    "Error while reading"));
        //if  result_in_progress
        default:
        {
            return false; // data is not ready
        }
    }

}


std::string serial_io::read_string_until(const std::string& delim)
{
    // Note: if read_data_ contains some previously read data, the call to
    // async_read_until (which is done in perform_read_setup) correctly handles
    // it. If the data is enough it will also immediately call readCompleted()
    setup_parameters_ = read_setup_parameters(delim);
    perform_read_setup( setup_parameters_);

    //For this code to work, there should always be a timeout_, so the
    //request for no timeout_ is translated into a very long timeout_
    if(timeout_!=posix_time::seconds(0)) timer_.expires_from_now(timeout_);
    else timer_.expires_from_now(posix_time::hours(100000));

    timer_.async_wait(boost::bind(&serial_io::timeout_expired, this,
                 asio::placeholders::error));

    result_ = result_in_progress;
    bytes_transferred_=0;
    for(;;)
    {
        io_.run_one();
        switch( result_ )
        {
            case  result_success:
                {
                    timer_.cancel();
                    bytes_transferred_-= delim.size();//Don't count delim
                    istream is(&read_data_);
                    string result(bytes_transferred_,'\0');//Alloc string
                    is.read(&result[0],bytes_transferred_);//Fill values
                    is.ignore(delim.size());//Remove delimiter from stream
                    return result;
                }
            case  result_timeout_expired:
                port_.cancel();
                throw(timeout_exception("timeout_ expired"));
            case  result_error:
                timer_.cancel();
                port_.cancel();
                throw(boost::system::system_error(boost::system::error_code(),
                        "Error while reading"));
            case  result_in_progress:
                continue;
            default:
                continue;
            //if  result_in_progress remain in the loop
        }
    }
}


void serial_io::perform_read_setup(const  read_setup_parameters& param)
{
    if(param.fixed_size)
    {
        asio::async_read(port_,asio::buffer(param.data,param.size),boost::bind(
                &serial_io::read_completed,this,asio::placeholders::error,
                asio::placeholders::bytes_transferred));
    } else {
        asio::async_read_until(port_,read_data_,param.delim,boost::bind(
                &serial_io::read_completed,this,asio::placeholders::error,
                asio::placeholders::bytes_transferred));
    }
}

void serial_io::timeout_expired( const boost::system::error_code& error)
{
     if(!error && result_ == result_in_progress) result_= result_timeout_expired;
 }

void serial_io::read_completed(const boost::system::error_code& error,
        const size_t bytes_transferred)
{
    if( error ==  boost::asio::error::would_block ) { return; }

    if(!error)
    {
        result_ = result_success;
        this->bytes_transferred_ = bytes_transferred;
        return;
    }

    //In case a asynchronous operation is cancelled due to a timeout_,
    //each OS seems to have its way to react.
    #ifdef _WIN32
    if(error.value()==995) return; //Windows spits out error 995
    #elif defined(__APPLE__)
    if(error.value()==45)
    {
        //Bug on OS X, it might be necessary to repeat the setup
        //http://osdir.com/ml/lib.boost.asio.user/2008-08/msg00004.html
        perform_read_setup(setup_parameters_);
        return;
    }
    #else //Linux
    if(error.value()==125) return; //Linux outputs error 125
    #endif

    result_ = result_error;
}
} } // namespace snark { namespace ocean {
