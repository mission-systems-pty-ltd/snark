// Copyright (c) 2011 The University of Sydney

#include <iostream>
#include <iterator>
#include <math.h>
#include <vector>
#include <boost/optional.hpp>
#include <fftw3.h>
#include <comma/visiting/traits.h>
#include <comma/application/command_line_options.h>
#include <comma/application/verbose.h>
#include <comma/csv/options.h>
#include <comma/csv/stream.h>
#include <comma/string/string.h>
#include "detail/shuffle-tied.h"

static std::size_t input_samples{0}, input_size{0};
static bool filter_input=false;
static bool real_input=true;
static bool logarithmic_output=false;
static bool dB_output=false;
static bool complex_polar=false;
static bool magnitude=false;
static bool phase=false;
static bool real=false;
static bool imaginary=false;
static bool split=false;
static std::size_t bin_size=0;
static boost::optional<double> bin_overlap;
static bool tied=true;

static void usage( bool verbose )
{
    std::cerr << R"(
perform fft on input data

usage:  cat input.csv | fft-math <options> > converted.csv

input:  an array of double or interleaved complex double, length
        of sequence is specified with --size
output: array of pair (real, imaginary) of double; use --output-size
        to get size of array of doubles with the specified options

options
    --bin-overlap=[<overlap>]:  if specified, each bin will contain 
                                this portion of the last bin's data,
                                range: 0 (no overlap) to 1
    --bin-size=[<size>]:        cut data into several bins of this size
                                and perform fft on each bin, when not
                                specified calculates fft on the whole data
    --complex-input,--complex:  when not specified, expect real-valued
                                input, ala rfft
    --decibels,--dB,--db:       scale output to 20*log10 for magnitude
                                (phase is not affected)
    --filter:                   when specified, filters input using a cut
                                window to get limited output
    --logarithmic,--log:        scale output to logarithm of 10 for
                                magnitude (phase is not affected)
    --output-fields:            print output fields to stdout and exit
                                depends on input fields and size
    --output-format:            print binary format of output to stdout
                                and exit depends on input fields and size
    --output-size:              print size of output data array and exit
                                e.g. if output format is t,ui,256d, then
                                output size is 256
    --samples,--size=<samples>: number of samples; for complex input
                                each sample is represented by comma-separated
                                real and imaginary parts; for real input
                                each sample is just the real part
    --verbose,-v:               more verbose output

output options
    --imaginary: output imaginary part only output is binary array of doubles
                 with half the size of input
    --magnitude: output magnitude only output is binary array of double with
                 half the size of input
    --phase:     output phase only output is binary array of double with half
                 the size of input
    --polar:     output in complex polar form; (magnitude, phase)
    --real:      output real part only output is binary array of double with half
                 the size of input
    --shuffle,--shuffle-fields,--shuffled-fields=[<csv_fields>]: comma separated
                 list of input fields to be written to stdout; if not specified
                 prepend all input fields to the output
    --split:     output array of real/magnitude followed by array of imaginary/phase
                 part; when not specified real/magnitude and imaginary/phase parts
                 are interleaved
    --untied,--discard-input: only write output to stdout, if unspecified,
                 write input to stdout and append output (see examples)
)" << std::endl;
    std::cerr << "csv options" << std::endl;
    std::cerr << comma::csv::options::usage( verbose ) << std::endl;
    std::cerr << R"(examples
    echo 0,1,2,3,4,5,6,7 | math-fft --samples=8
    echo 0,1,2,3,4,5,6,7 | math-fft --samples=8 --untied
    echo 0,1,2,3,4,5,6,7 | math-fft --samples=8 --untied --split
    echo 0,1,2,3,4,5,6,7 | math-fft --samples=4 --complex
    echo 0,1,2,3,4,5,6,7 | math-fft --samples=4 --complex --untied
    echo 0,1,2,3,4,5,6,7 \
        | csv-to-bin 8f \
        | math-fft --samples=4 --binary 8f --complex --untied \
        | csv-to-bin 8d
    cat data.bin | math-fft --binary=\"t,16000f\" --fields=t,data --samples=16000
)" << std::endl;
    exit( 0 );
}

// math-fft --fields ,,data --format t,ui,16000f
// math-fft --fields ,,,,,data --format t,ui,s[$(( 16000 * 4  ))],t,ui,16000f

struct input_t
{
    std::vector<double> data;
    
    input_t() : data(input_size) {}
};

struct output_t
{
    std::vector<double> data;
    
    output_t()
    {
        std::size_t len=bin_size;
        if(magnitude || phase || real || imaginary) { len/=2; }
        data.resize(len);
    }
    
    void reset() { std::memset(&data[0],0,data.size()*sizeof(data[0])); }
};

namespace comma { namespace visiting {

template <> struct traits< input_t >
{
    template< typename K, typename V > static void visit( const K& k, input_t& p, V& v ) { v.apply( "data", p.data ); }
    template< typename K, typename V > static void visit( const K& k, const input_t& p, V& v ) { v.apply( "data", p.data ); }
};

template <> struct traits< output_t >
{
    template< typename K, typename V > static void visit( const K& k, const output_t& p, V& v ) { v.apply( "output", p.data ); }
};

} } // namespace comma { namespace visiting {

template<typename T>
T* allocate_fftw_array(std::size_t samples)
{
    return reinterpret_cast<T*>(fftw_malloc(sizeof(T)*samples));
}

// // a quick and dirty helper class
// struct fft
// {
//     std::vector<double> h;
//     double* input;
//     fftw_complex* output;
//     fftw_plan plan;

//     fft(std::size_t size) : h( size ), input(allocate_fftw_array<double>(size)) ,
//         output(allocate_fftw_array<fftw_complex>(size / 2 + 1 )) ,
//         plan( fftw_plan_dft_r2c_1d( size, input, output, 0 ) )
//     {
//         for( std::size_t i = 0; i < size; ++i ) { h[i] = 0.54 - 0.46 * std::cos( M_PI * 2 * i / size ); }
//     }
    
//     ~fft()
//     {
//         fftw_destroy_plan(plan);
//         fftw_free( input ); // seems that fftw_destroy_plan() releases it
//         fftw_free( output ); // seems that fftw_destroy_plan() releases it
//     }
    
//     void execute() { fftw_execute( plan); }
    
//     std::size_t output_size() const { return h.size() / 2; }
// };

class fft
{
private:
    std::vector<double> h;
    fftw_complex* c_input;
    double* input;
    fftw_complex* output;
    fftw_plan plan;

public:
    fft(std::size_t samples) : 
        h( samples ), 
        c_input(allocate_fftw_array<fftw_complex>(samples)) ,
        input(allocate_fftw_array<double>(samples)) ,
        output( allocate_fftw_array<fftw_complex>( real_input ? (samples / 2 + 1) : samples ) ) ,
        plan( real_input ? fftw_plan_dft_r2c_1d( samples, input, output, 0 ) : fftw_plan_dft_1d( samples, c_input, output, FFTW_FORWARD, FFTW_MEASURE ) )
    {
        for( std::size_t i = 0; i < samples; ++i ) { h[i] = 0.54 - 0.46 * std::cos( M_PI * 2 * i / samples ); }

    }
    
    ~fft()
    {
        fftw_destroy_plan(plan);
        fftw_free( input ); // seems that fftw_destroy_plan() releases it
        fftw_free( output ); // seems that fftw_destroy_plan() releases it
    }
    
    void execute() { fftw_execute( plan); }
    
    std::size_t output_size() const { return real_input ? h.size()/2 : h.size(); }

    static void calculate(const double* data, std::size_t size, std::vector<double>& output)
    {
        std::size_t samples = real_input ? size : (size/2);
        fft fft( samples );
        if(filter_input)
        {
            // for(std::size_t i=0;i<size;i++) { fft.input[i][0] = fft.h[i] * data[i]; fft.input[i][1] = fft.h[i] * data[i]; }
            if (real_input) 
            {
                for(std::size_t i=0;i<samples;i++) { fft.input[i] = fft.h[i] * data[i]; }
            }
            else 
            {
                for(std::size_t i=0;i<samples;i++) { fft.c_input[i][0] = fft.h[i] * data[i]; fft.c_input[i][1] = fft.h[i] * data[i]; }
            }
        }
        else 
        { 
            if (real_input) { ::memcpy(fft.input, data, samples * sizeof(double) ); }
            else { memcpy(fft.c_input, data, samples * sizeof(fftw_complex) ); }
            // memcpy(fft.input, data, size * sizeof(double) ); 
        }        
        fft.execute();
        if(magnitude)
        {
            if(fft.output_size()>output.size()) { COMMA_THROW(comma::exception, "size mismatch, output "<<output.size()<<" fft output "<<fft.output_size()); }
            for(std::size_t j=0;j<fft.output_size();j++)
            {
                double a = std::abs( std::complex<double>( fft.output[j][0], fft.output[j][1] ) );
                if(logarithmic_output) { a = std::log10(a); }
                if(dB_output) { a = 20*std::log10(a); }
                output[j]=a;
            }
        }
        else if(phase)
        {
            if(fft.output_size()>output.size()) { COMMA_THROW(comma::exception, "size mismatch, output "<<output.size()<<" fft output "<<fft.output_size()); }
            for(std::size_t j=0;j<fft.output_size();j++)
            {
                double a= std::arg( std::complex<double>( fft.output[j][0], fft.output[j][1] ) );
                output[j]=a;
            }
        }
        else if(real)
        {
            if(fft.output_size()>output.size()) { COMMA_THROW(comma::exception, "size mismatch, output "<<output.size()<<" fft output "<<fft.output_size()); }
            for(std::size_t j=0;j<fft.output_size();j++)
            {
                double a= fft.output[j][0];
                output[j]=a;
            }
        }
        else if(imaginary)
        {
            if(fft.output_size()>output.size()) { COMMA_THROW(comma::exception, "size mismatch, output "<<output.size()<<" fft output "<<fft.output_size()); }
            for(std::size_t j=0;j<fft.output_size();j++)
            {
                double a= fft.output[j][1];
                output[j]=a;
            }
        }
        else if(complex_polar)
        {
            std::size_t k=0;
            std::size_t step=2;
            std::size_t off=1;
            if(split)
            {
                step=1;
                off=output.size()/2;
            }
            if(2*fft.output_size()>output.size() ) { COMMA_THROW(comma::exception, "size mismatch, output "<<output.size()<<" fft output "<<fft.output_size()); }
            for(std::size_t j=0; j<fft.output_size(); j++, k+=step)
            {
                double a = std::abs( std::complex<double>( fft.output[j][0], fft.output[j][1] ) );
                double b = std::arg( std::complex<double>( fft.output[j][0], fft.output[j][1] ) );
                if(logarithmic_output) { a = std::log10(a); }
                if(dB_output) { a = 20*std::log10(a); }

                output[k]=a;
                output[k+off]=b;
            }
        }
        else
        {
            std::size_t k=0;
            std::size_t step=2;
            std::size_t off=1;
            if(split)
            {
                step=1;
                off=output.size()/2;
            }
            if(2*fft.output_size()>output.size() ) { COMMA_THROW(comma::exception, "size mismatch, output "<<output.size()<<" fft output "<<fft.output_size()); }
            for(std::size_t j=0; j<fft.output_size(); j++, k+=step)
            {
                output[k]=fft.output[j][0];
                output[k+off]=fft.output[j][1];
            }
        }
    }
};

struct app
{
    void process(const comma::csv::options& csv, const boost::optional<std::string>& shuffle_fields)
    {
        if(bin_overlap && int(bin_size*(1-*bin_overlap)) <= 0) { COMMA_THROW( comma::exception, "bin size and overlap don't work" ); }
        input_t sample;
        comma::csv::input_stream< input_t > is( std::cin, csv, sample );
        comma::csv::output_stream< output_t > os( std::cout, csv.binary(), true );
        ::shuffle_tied< input_t, output_t > shuffle_tied( is
                                                        , os
                                                        , csv
                                                        , shuffle_fields
                                                        , "data=" + boost::lexical_cast< std::string >( sample.data.size() ) );
        output_t output;
        while( std::cin.good() )
        {
            const input_t* input=is.read();
            if(!input) { break; }
            for( std::size_t bin_offset = 0; bin_offset< input_size; bin_offset += bin_overlap ? bin_size * ( 1 - *bin_overlap ) : bin_size )
            {
                output.reset();
                fft::calculate(&input->data[bin_offset], std::min(bin_size,input_size-bin_offset), output.data);
                if( tied ) { shuffle_tied.append( output ); } else { os.write(output); }
            }
        }
    }
    
    std::size_t get_output_size() const { return output_t().data.size(); }
    
    void output_format() const { std::cout<<output_t().data.size()<<"d"<<std::endl; }
    
    void output_fields() { std::cout<<comma::join(comma::csv::names< output_t >(true),',')<<std::endl; }
};

template< typename T > static std::ostream& operator<< (std::ostream& os, const std::vector<T>& v) { std::copy(v.begin(), v.end(), std::ostream_iterator<T>(os, " ")); return os; }

template< typename T > static void range_check(T value, T min, T max, const char* label) { if(value<min || value>max) { COMMA_THROW( comma::exception, label << " out of range " << min << " to " << max ); } }

int main( int argc, char** argv )
{
    comma::command_line_options options( argc, argv, usage );
    try
    {
        comma::csv::options csv(options,"data");
        csv.full_xpath = false;
        filter_input = options.exists( "--filter" );
        real_input = !options.exists( "--complex-input,--complex" );
        logarithmic_output = options.exists( "--logarithmic,--log" );
        dB_output = options.exists( "--decibels,--dB,--db" );
        input_samples = options.value< std::size_t >( "--size,--samples" );
        input_size = real_input ? input_samples : ( input_samples * 2 );
        complex_polar = options.exists( "--polar" );
        magnitude = options.exists( "--magnitude" );
        phase = options.exists( "--phase" );
        real = options.exists( "--real" );
        imaginary = options.exists( "--imaginary" );
        split = options.exists( "--split" );
        bin_size = options.value<std::size_t>( "--bin-size", input_size );
        range_check< std::size_t >( bin_size, 0, input_size, "bin_size" );
        bin_overlap=options.optional<double>("--bin-overlap");
        if(bin_overlap) { range_check<double>(*bin_overlap,0,1,"bin_overlap"); }
        boost::optional< std::string > shuffle_fields = options.optional< std::string >( "--shuffle,--shuffle-fields,--shuffled-fields" );
        tied = !options.exists( "--untied,--discard-input" );
        options.assert_mutually_exclusive( "--shuffle,--shuffle-fields,--shuffled-fields", "--untied,--discard-input" );
        options.assert_mutually_exclusive( "--polar", "--magnitude,--real,--phase,--imaginary" );
        options.assert_mutually_exclusive( "--magnitude,--real", "--phase,--imaginary" );
        options.assert_mutually_exclusive( "--magnitude,--phase", "--real,--imaginary" );
        options.assert_mutually_exclusive( "--logarithmic,--log,--dB,--decibels,--db", "--real,--imaginary,--phase" );
        options.assert_mutually_exclusive( "--logarithmic,--log", "--dB,--decibels,--db" );
        std::vector< std::string > unnamed = options.unnamed( "--verbose,-v,--output-size,--output-format,--output-fields,--filter,--complex-input,--complex,--logarithmic,--log,--dB,--decibels,--db,--polar,--magnitude,--phase,--real,--imaginary,--split,--untied,--discard-input" , "-.*" );
        COMMA_ASSERT_BRIEF( unnamed.empty(), "invalid option(s): " << comma::join( unnamed, ' ' ) );
        app app;
        if(options.exists("--output-size")) { std::cout<< app.get_output_size() << std::endl; return 0; }
        if(options.exists("--output-format")) { app.output_format(); return 0; }
        if(options.exists("--output-fields")) { app.output_fields(); return 0; }
        app.process( csv, shuffle_fields );
        return 0;
    }
    catch( std::exception& ex ) { comma::say() << ": " << ex.what() << std::endl; }
    catch( ... ) { comma::say() << ": " << "unknown exception" << std::endl; }
    return 1;
}
