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

#include <thread>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/shared_ptr.hpp>
#include <comma/application/command_line_options.h>
#include <comma/application/signal_flag.h>
#include <comma/base/exception.h>
#include <comma/csv/stream.h>
#include <comma/io/stream.h>
#include <comma/string/string.h>
#include <comma/visiting/traits.h>
#include "../message.h"
#include "../sensors.h"
#include "../traits.h"
#include "../streams.h"
#include "../detail/scip2.h"
#include "../detail/ust.h"

const char* name() { return "hokuyo-to-csv: "; }
using namespace snark::hokuyo;
namespace hok = snark::hokuyo;
bool debug_verbose=false;
bool verbose=false;
static void usage(bool verbose)
{
    std::cerr << std::endl;
    std::cerr << "It puts the laser scanner into scanning mode and broad cast laser data." << std::endl;
    std::cerr << "By default it scans using 1081 steps/rays/data points as fast as possible, you can limit it to 271 steps with --start-step." << std::endl;
    std::cerr << std::endl;
    std::cerr << "usage" << std::endl;
    std::cerr << "    hokuyo-to-csv --tcp <host:port> [ --fields t,x,y,z,range,bearing,elevation,intensity ]" << std::endl;
    std::cerr << "  or" << std::endl;
    std::cerr << "    hokuyo-to-csv --serial <device> [--baud-rate=<rate>] [ --fields t,x,y,z,block,range,bearing,elevation ]" << std::endl;
    std::cerr << std::endl;
    std::cerr << "options" << std::endl;
    std::cerr << "    --binary,-b:          output binary equivalent of csv" << std::endl;
    std::cerr << "    --fields=<fields>:    output only given fields" << std::endl;
    std::cerr << "    --dont-omit-on-error: default: omit. Do not omit output on error, set x,y,z to 0,0,0" << std::endl;
    std::cerr << "    --help,-h:            show this message, optionaly --verbose to see more help" << std::endl;
    std::cerr << "    --num-of-scans:       How many scans is requested for ME requests, default is 100 - 0 for continuous ( data verification problem with 0 )." << std::endl;
    std::cerr << "    --output-fields:      output fields to stdout and exit; requires --serial or --tcp" << std::endl;
    std::cerr << "    --output-format,--format: output binary format for given fields to stdout and exit; requires --serial or --tcp" << std::endl;
    std::cerr << "    --permissive:         do not throw error if receiving response for previous request to device" << std::endl;
    std::cerr << "    --reboot-on-error:    if failed to put scanner into scanning mode, reboot the scanner." << std::endl;
    std::cerr << "    --scan-break:         How many usec of sleep time between ME request and reponses received before issuing another ME request, default is 20us." << std::endl;
    std::cerr << "    --start-step=<0-890>: Scan starting at a start step and go to (step+270) wich covers 67.75\" which is 270\"/4." << std::endl;
    std::cerr << "                          Does not perform a full 270\" scan." << std::endl;
    std::cerr << "    --verbose,-v: show more information" << std::endl;
    std::cerr << std::endl;
    std::cerr << "TCP device option:" << std::endl;
    std::cerr << "    --tcp,--laser=:       the TCP connection to the laser <host:port>, mutually exclusive with --serial" << std::endl;
    std::cerr << "                          not removing --laser because of backward compatibility only, use --tcp" << std::endl;
    std::cerr << std::endl;
    std::cerr << "serial device option:" << std::endl;
    std::cerr << "    --baud-rate=[<bps>]:  connect using this baud rate, default 0 which means auto (tries all different settings)" << std::endl;
    std::cerr << "                          supported baud rates for URG-04LX: 19200, 57600, 115200, 500000, (750000)" << std::endl;
    std::cerr << "    --serial,--port=<device_name>:" << std::endl;
    std::cerr << "                          device filename for serial port to connect to (e.g. COM1 or /dev/ttyS0 or /dev/usb/ttyUSB0), mutually exclusive with --tcp " << std::endl;
    std::cerr << "    --set-baud-rate=[<bps>]:" << std::endl;
    std::cerr << "                          default: 500000, change the device's baud rate to <bps>; pass 0 to disable changing baud-rate" << std::endl;
    std::cerr << std::endl;
//     if( !verbose ) { exit(0); }
    
    std::cerr << "Output fields" << std::endl;
    std::cerr << "  TCP" << std::endl;
    std::cerr << "    fields: " << comma::join( comma::csv::names< snark::hokuyo::data_point >(), ','  ) << std::endl;
    std::cerr << "        t:                timestamp" << std::endl;
    std::cerr << "        x,y,z:            cartesian coordinates in sensor frame, where <0,0,0> is no data" << std::endl;
    std::cerr << "        range,bearing,elevation or r,b,e: polar coordinates in sensor frame" << std::endl;
    std::cerr << "        intensity or i:   intensity of the data point" << std::endl;
    std::cerr << "    format: " << comma::csv::format::value< snark::hokuyo::data_point >() << std::endl;
    std::cerr << std::endl;
    std::cerr << "  serial" << std::endl;
    std::cerr << "    fields: " << comma::join( comma::csv::names< snark::hokuyo::scip2_device::output_t >(), ','  ) << std::endl;
    std::cerr << "        t:                timestamp" << std::endl;
    std::cerr << "        block:            block of response data from device" << std::endl;
    std::cerr << "        x,y,z:            cartesian coordinates in sensor frame, where <0,0,0> is no data" << std::endl;
    std::cerr << "        range,bearing,elevation or r,b,e: polar coordinates in sensor frame" << std::endl;
    std::cerr << "    format: " << comma::csv::format::value< snark::hokuyo::scip2_device::output_t >() << std::endl;
    std::cerr << std::endl;
    std::cerr << "examples:" << std::endl;
    std::cerr << "  serial" << std::endl;
    std::cerr << "    sudo mknod /dev/usb/ttyUSB0 c 188 0" << std::endl;
    std::cerr << "    sudo hokuyo-to-csv --serial=\"/dev/usb/ttyUSB0\" --num-of-scans=5 --omit-error | view-points --fields=t,x,y,z,block" << std::endl;
    exit( 0 );
}


comma::signal_flag signaled;
boost::shared_ptr<stream_base> ios; //this must have a higher scope than turn_laser_on
bool reboot_on_error=false;
int start_step=-1;
comma::uint32 scan_break=20;
comma::uint32 num_of_scans=100;
comma::csv::options csv;
int end_step=-1; //should be zero for ust, as currently not supported
int frames=-1;
bool omit_on_error=true;


struct sample_interface
{
    typedef int data_t;
    typedef int output_t;
    //turn_laser_on/off or use common method
    //start receiving data
    void request_scan(stream_base& ios, int start_step, int end_step, int num_of_scans);
    //return false if num_of_scans in response is zero
    bool receive_response(stream_base& ios);
    //use after response is received
    data_t get_data(int scan);
    //is data valid
    bool is_valid(data_t& data) const;
    output_t convert(data_t& data);
    int get_steps() const;
};

//send one request scan and process multiple response
//T like test_interface
template<typename T>
bool one_scan(T& device, comma::csv::output_stream< typename T::output_t >& output_stream)
{
    device.request_scan(*ios, start_step, end_step, num_of_scans);
    while( !signaled && std::cin.good() )
    {
        bool more=device.receive_response(*ios);
        int steps=device.get_steps();
        for( int i=0; i<steps; ++i )
        {
            typename T::data_t data;
            data=device.get_data(i);
            if(!device.is_valid(data) && omit_on_error) { continue; }
            output_stream.write(device.convert(data));
        }
        output_stream.flush();
        // This means we are done
        if(frames!=-1 && --frames==0 )
            return false;
        if( num_of_scans != 0 && !more ) { return true; }
    }
    return false;

}

template<typename T>
void process(T& device)
{
    // Let put the laser into scanning mode
    turn_laser_on laser_on(*ios, (laser_device&)device, reboot_on_error);
    comma::csv::output_stream< typename T::output_t > output_stream( std::cout, csv );  
    while( one_scan( device, output_stream ) ) { 
        std::this_thread::sleep_for( std::chrono::microseconds(scan_break) );
    }
}

void output_samples()
{
    comma::csv::output_stream< snark::hokuyo::data_point > output( std::cout, csv );
    data_point pt;
    pt.x = 1; pt.y = 2; pt.z = 3;
    pt.intensity = 100;
    while( !signaled && std::cout.good() )
    {
        pt.t = boost::posix_time::microsec_clock::universal_time();
        output.write( pt );
        std::this_thread::sleep_for( std::chrono::milliseconds(100) );
    }
}

// it is higher than 1080 because 0 is a step
static const int UST_MAX_STEPS = 1081;
static const int UST_SMALL_STEPS = 271;


int main( int ac, char** av )
{
    comma::command_line_options options( ac, av );
    verbose = options.exists( "--verbose,-v" );
    if( options.exists( "--help,-h" ) ) { usage(verbose); }

    try
    {
        options.assert_mutually_exclusive("--tcp,--serial");
        options.assert_mutually_exclusive("--laser,--serial");
        scan_break = options.value< comma::uint32 > ( "--scan-break", 20 ); // time in us
        num_of_scans = options.value< comma::uint32 > ( "--num-of-scans", 100 ); // time in us
        reboot_on_error = options.exists( "--reboot-on-error" );
        start_step=options.value<int>("--start-step", -1);
        end_step=options.value<int>("--end-step", -1);
        debug_verbose=options.exists("--debug");
        frames=options.value<int>("--frames",-1);
        omit_on_error = ! options.exists("--dont-omit-on-error");
        bool permissive = options.exists( "--permissive" );
        std::vector< std::string > unnamed = options.unnamed( "--output-fields,--output-format,--format,--verbose,-v,--flush,--reboot-on-error,--debug,--output-samples,--dont-omit-on-errork,--permissive",
                                                              "--frames,--scan-break,--num-of-scans,--start-step,--end-step,--serial,--port,--tcp,--laser,--fields,--binary,-b,--baud-rate,--set-baud-rate");
        if(!unnamed.empty()) { std::cerr<<"invalid option(s):"<< comma::join(unnamed, ',') <<std::endl; return 1; }
        
        scip2_device scip2;
        ust_device<UST_SMALL_STEPS> ust_small( permissive );
        ust_device<UST_MAX_STEPS> ust_max( permissive );
        const bool serial = options.exists( "--serial");
        laser_device& device = serial ? scip2 : (start_step ?  (laser_device&)ust_small : (laser_device&)ust_max);
        
        // Sets up output data
        csv.fields = options.value< std::string >( "--fields", "" );
        std::vector< std::string > v = comma::split( csv.fields, ',' );
        for( std::size_t i = 0; i < v.size(); ++i ) // convenience shortcuts
        {
            if( v[i] == "i" ) { v[i] = "intensity"; }
            else if( v[i] == "r" ) { v[i] = "range"; }
            else if( v[i] == "b" ) { v[i] = "bearing"; }
            else if( v[i] == "e" ) { v[i] = "elevation"; }
        }
        csv.fields = comma::join( v, ',' );
        csv.full_xpath = false;
        // see sick-ldmrs-to-csv
        if( options.exists( "--output-format,--format" ) ) 
        {
            if( !options.exists("--tcp") && !options.exists("--serial") ) { std::cerr << name() << "please specify --serial or --tcp" << std::endl; return 1; }
            if(serial) { std::cout << comma::csv::format::value< scip2_device::output_t >( csv.fields, false ) << std::endl; }
            else { std::cout << comma::csv::format::value< snark::hokuyo::data_point >( csv.fields, false ) << std::endl; }
            return 0; 
            
        }
        if(options.exists("--output-fields"))
        {
            if( !options.exists("--tcp") && !options.exists("--serial") ) { std::cerr << name() << "please specify --serial or --tcp" << std::endl; return 1; }
            if(!csv.fields.empty()) {std::cout<<csv.fields<<std::endl;}
            else if(serial) { std::cout<<comma::join(comma::csv::names<scip2_device::output_t>(), ',')<<std::endl;}
            else {std::cout<<comma::join(comma::csv::names<snark::hokuyo::data_point>(), ',')<<std::endl;}
            return 0;
        }
        if( options.exists( "--binary,-b" ) ) 
        {
            if(serial) { csv.format( comma::csv::format::value< scip2_device::output_t >( csv.fields, false ) ); }
            else { csv.format( comma::csv::format::value< snark::hokuyo::data_point >( csv.fields, false ) ); }
        }
        
        if( options.exists( "--output-samples" ) ) { output_samples(); return 0; }
        
        device.init(options);
        /// Connect to the laser
        ios = device.connect();
        device.setup(*ios);
        if(serial)
        {
            if(debug_verbose) { ((serial_stream&)*ios).dump_setting(); }
//             debug_msg("change serial settings");
//             const char* ss_cmd="SS500000\n";
            process(scip2);
        }
        else
        {
            if(start_step) { process(ust_small); } else { process(ust_max); }
        }
    }
    catch( std::exception& ex ) { std::cerr << name() << ex.what() << std::endl; return 1; }
    catch( ... ) { std::cerr << name() << "unknown exception" << std::endl; return 1; }
    return 0;
}
