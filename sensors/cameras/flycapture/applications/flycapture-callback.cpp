// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2011 The University of Sydney
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. Neither the name of the University of Sydney nor the
//    names of its contributors may be used to endorse or promote products
//    derived from this software without specific prior written permission.
//
// NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
// GRANTED BY THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
// HOLDERS AND CONTRIBUTORS \"AS IS\" AND ANY EXPRESS OR IMPLIED
// WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
// OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
// IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include <boost/bind/bind.hpp>
#include <boost/thread.hpp>
#include <tbb/concurrent_queue.h>
#include <tbb/pipeline.h>
#include <tbb/task_scheduler_init.h>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/base/exception.h>
#include <comma/csv/stream.h>
#include <comma/name_value/map.h>
#include "../../../../imaging/cv_mat/filters.h"
#include "../../../../imaging/cv_mat/serialization.h"
#include "../../../../tbb/queue.h"
#include "../flycapture.h"
#include <boost/program_options.hpp>

static comma::signal_flag is_shutdown;
static bool verbose;
static unsigned int emptyFrameCounter = 0;
static unsigned int discard_more_than;
typedef std::pair< boost::posix_time::ptime, cv::Mat > Pair;
static snark::tbb::queue< Pair > queue;
static boost::scoped_ptr< snark::camera::flycapture::callback > callback;
static bool running = true;

static void spin_()
{
    static unsigned int i = 0;
    static char spinner[] = { '-', '/', '|', '\\' };
    if( i % 3 == 0 ) { std::cerr << '\r' << spinner[ i / 3 ] << "   "; }
    if( ++i >= 12 ) { i = 0; }
}

static void on_frame_( const Pair& p ) // quick and dirty
{
    std::cerr << "on_frame_" << std::endl;
    if( p.second.size().width == 0 )
    {
        emptyFrameCounter++;
        if( emptyFrameCounter > 20 )
        {
            COMMA_THROW( comma::exception, "got lots of empty frames, check that the packet size in the camera matches the mtu on your machine" );
        }
        if( verbose )
        {
            std::cerr << "flycapture-cat: got empty frame" << std::endl;
        }
        return;
    }
    emptyFrameCounter = 0;
    Pair q;
    if( is_shutdown || !running ) { return queue.push( q ); } // to force read exit
    q.first = p.first;
    p.second.copyTo( q.second ); // quick and dirty: full copy; todo: implement circular queue in flycapture::callback?
    queue.push( q );
    
    if( verbose ) { spin_(); }
    if( discard_more_than > 0 )
    {
        int size = queue.size();
        if( size > 1 )
        {
            int size_to_discard = size - discard_more_than;
            Pair p;
            for( int i = 0; i < size_to_discard; ++i ) { queue.pop( p ); } // clear() is not thread-safe
            if( verbose && size_to_discard > 0 ) { std::cerr << "flycapture-cat: discarded " << size_to_discard << " frames" << std::endl; }
        }
    }    
}

static Pair read_( tbb::flow_control& flow )
{
    if( queue.empty() || is_shutdown || !callback->good() || !std::cout.good() || !running )
    {
        flow.stop();
        return Pair();
    }
    Pair p;
    queue.pop( p );
    return p;
}

static void write_( snark::cv_mat::serialization& serialization, Pair p )
{
    if( p.second.size().width == 0 )
    {
        running = false;
    }
    static std::vector< char > buffer;
    buffer.resize( serialization.size( p ) );
    serialization.write( std::cout, p );
}

int main( int argc, char** argv )
{
    try
    {
        unsigned int id;
        std::string fields;
        std::string setattributes;
        unsigned int discard;
        boost::program_options::options_description description( "options" );
        description.add_options()
            ( "help,h", "display help message" )
            ( "set", boost::program_options::value< std::string >( &setattributes ), "set camera attributes as comma-separated name-value pairs and exit" )
            ( "serial", boost::program_options::value< unsigned int >( &id )->default_value( 0 ), "camera serial; default: first available camera" )
            ( "discard,d", "discard frames, if cannot keep up; same as --buffer=1" )
            ( "buffer", boost::program_options::value< unsigned int >( &discard )->default_value( 0 ), "maximum buffer size before discarding frames" )
            ( "fields,f", boost::program_options::value< std::string >( &fields )->default_value( "t,rows,cols,type" ), "header fields, possible values: t,rows,cols,type,size" )
            ( "list-attributes", "output current camera attributes" )
            ( "list-cameras", "list all cameras and exit" )
            ( "verbose,v", "be more verbose" )
            ( "header", "output header only" )
            ( "no-header", "output image data only" );

        boost::program_options::variables_map vm;
        boost::program_options::store( boost::program_options::parse_command_line( argc, argv, description), vm );
        boost::program_options::parsed_options parsed = boost::program_options::command_line_parser(argc, argv).options( description ).allow_unregistered().run();
        boost::program_options::notify( vm );

        if( vm.count( "header" ) + vm.count( "no-header" ) > 1 )
        { COMMA_THROW( comma::exception, "--header, and --no-header are mutually exclusive" ); }
        
        if ( vm.count( "help" ) )
        {
            std::cerr << "acquire images from a point grey flycapture camera, same as flycapture-cat but using a callback " << std::endl;
            std::cerr << "instead of a thread to acquire the images" << std::endl;
            std::cerr << "output to stdout as serialized cv::Mat" << std::endl;
            std::cerr << "usage: flycapture-capture [<options>] [<filters>]" << std::endl;
            std::cerr << "known bug: freezes on slow consumers even with --discard, use flycapture-cat instead" << std::endl;
            std::cerr << description << std::endl;
            std::cerr << snark::cv_mat::impl::filters<>::usage() << std::endl;
            return 1;
        }
        
        verbose = vm.count( "verbose" );
        if( vm.count( "list-cameras" ) )
        {
            const std::vector< unsigned int >& list = snark::camera::flycapture::list_camera_serials();
            std::cerr << "got " << list.size() << " cameras." << std::endl;
            for( std::size_t i = 0; i < list.size(); ++i ) // todo: serialize properly with name-value
            {
                std::cout << snark::camera::flycapture::describe_camera(list[i]) << std::endl;
            }
            return 0;
        }
        if ( vm.count( "discard" ) ) { discard = 1; }
        discard_more_than = discard;
        snark::camera::flycapture::attributes_type attributes;

        if( vm.count( "set" ) )
        {
            comma::name_value::map m( setattributes, ';', '=' );
            attributes.insert( m.get().begin(), m.get().end() );
        }

        if( verbose ) { std::cerr << "flycapture-cat: connecting..." << std::endl; }
        snark::camera::flycapture camera( id, attributes );
        if( verbose ) { std::cerr << "flycapture-cat: connected to camera " << camera.id() << std::endl; }
        if( verbose ) { std::cerr << "flycapture-cat: total bytes per frame: " << camera.total_bytes_per_frame() << std::endl; }
        if( !attributes.empty() ) { return 0; }
        if( vm.count( "list-attributes" ) )
        {
            attributes = camera.attributes(); // quick and dirty
            for( snark::camera::flycapture::attributes_type::const_iterator it = attributes.begin(); it != attributes.end(); ++it )
            {
                if( it != attributes.begin() ) { std::cout << std::endl; }
                std::cout << it->first;
                if( it->second != "" ) { std::cout << '=' << it->second; }
            }
            return 0;
        }
        std::vector< std::string > v = comma::split( fields, "," );
        comma::csv::format format;
        for( unsigned int i = 0; i < v.size(); ++i )
        {
            if( v[i] == "t" ) { format += "t"; }
            else { format += "ui"; }
        }
        std::vector< std::string > filterStrings = boost::program_options::collect_unrecognized( parsed.options, boost::program_options::include_positional );
        std::string filters;
        if( filterStrings.size() == 1 )
        {
            filters = filterStrings[0];
        }
        if( filterStrings.size() > 1 )
        {
            COMMA_THROW( comma::exception, "please provide filters as name-value string" );
        }
        boost::scoped_ptr< snark::cv_mat::serialization > serialization;
        if( vm.count( "no-header" ) )
        {
            serialization.reset( new snark::cv_mat::serialization( "", format ) );
        }
        else
        {
            serialization.reset( new snark::cv_mat::serialization( fields, format, vm.count( "header" ) ) );
        }       
        callback.reset( new snark::camera::flycapture::callback( camera, on_frame_ ) );
        tbb::task_scheduler_init init;
        tbb::filter_t< void, Pair > read( tbb::filter::serial_in_order, boost::bind( read_, boost::placeholders::_1 ) );
        tbb::filter_t< Pair, void > write( tbb::filter::serial_in_order, boost::bind( write_, boost::ref( *serialization), boost::placeholders::_1 ) );
        tbb::filter_t< void, Pair > imageFilters = read;

        if( !filters.empty() )
        {
            std::vector< snark::cv_mat::filter > cvMatFilters = snark::cv_mat::impl::filters<>::make( filters );
            for( std::size_t i = 0; i < cvMatFilters.size(); ++i )
            {
                tbb::filter::mode mode = tbb::filter::serial_in_order;
                if( cvMatFilters[i].parallel )
                {
                    mode = tbb::filter::parallel;
                }
                tbb::filter_t< Pair, Pair > filter( mode, boost::bind( cvMatFilters[i].filter_function, boost::placeholders::_1 ) );
                imageFilters = imageFilters & filter;
            }
        }

        if( verbose ) { std::cerr << "flycapture-cat: starting loop" << std::endl; }
        while( !is_shutdown && running )
        {
            tbb::parallel_pipeline( init.default_num_threads(), imageFilters & write );
            queue.wait();
        }
        if( verbose ) { std::cerr << "flycapture-cat: exited loop" << std::endl; }
        
        if( is_shutdown && verbose ) { std::cerr << "flycapture-cat: caught signal" << std::endl; }
        return 0;
    }
    catch( std::exception& ex )
    {
        std::cerr << "flycapture-cat: " << ex.what() << std::endl;
    }
    catch( ... )
    {
        std::cerr << "flycapture-cat: unknown exception" << std::endl;
    }
    return 1;
}

