// Copyright (c) 2011 The University of Sydney

#pragma once

#include <iostream>
#include <string>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <opencv2/core/core.hpp>
#include <comma/application/command_line_options.h>
#include <comma/base/types.h>
#include <comma/csv/binary.h>
#include <comma/csv/options.h>

namespace snark { namespace cv_mat {

/// e.g. type_from_string( "3uw" ) would return 18 (CV_3UC)
unsigned type_from_string( const std::string& s );

std::string type_as_string( int type );

/// e.g. format_from_type( 18 ) would return "3uw"
std::string format_from_type( unsigned int type );

/// all types in csv name,format,type
std::string all_image_types( void );

class serialization
{
    public:
        /// header for cv::Mat serialization
        struct header
        {
            boost::posix_time::ptime timestamp;
            comma::uint32 rows{0};
            comma::uint32 cols{0};
            comma::uint32 type{0}; /// cv::Mat type (see http://opencv.willowgarage.com/documentation/cpp/basic_structures.html)
            comma::uint32 size{0}; /// data size, convenience member equal to rows*cols
            
            header();
            header( const cv::Mat& m );
            header( const std::pair< boost::posix_time::ptime, cv::Mat >& p ); /// constructor
            header( const boost::posix_time::ptime& t, const cv::Mat & p ); /// constructor
            static const char* default_fields() { return "t,rows,cols,type"; }
            static const char* default_format() { return "t,3ui"; }
            static const std::size_t fields_num = 4;  // ignore size field
            
            // TBD This can be wrapped in a smart share pointer as it it often copied e.g. cv::Mat is also a smart pointer
            typedef std::vector< char > buffer_t;
        };
        
        /// options, a helper class
        struct options
        {
            std::string fields;
            comma::csv::format format;
            
            comma::uint32 rows{0};
            comma::uint32 cols{0};
            std::string type;
            bool no_header{false};
            bool header_only{false};
            bool timestamp{false};
            
            header get_header() const; /// make header (to be used as default)
            static std::string usage();
            static std::string type_usage();
            static std::pair< options, options > make( const comma::command_line_options& o, bool ignore_csv_options = false );
        };
        
        /// default constructor
        serialization();

        /// constructor
        serialization( const std::string& fields, const comma::csv::format& format, bool headerOnly = false, const header& default_header = header(), bool set_timestamp = false );
        
        /// constructor
        serialization( const options& options );

        /// serialize cv::Mat, return image size in buffer
        std::size_t put( const std::pair< boost::posix_time::ptime, cv::Mat >& m, char* buf ) const;

        /// deserialize cv::Mat, return image size in buffer
        /// @note get timestamp from header()
        std::size_t get( std::pair< boost::posix_time::ptime, cv::Mat >& m, const char* buf ) const;

        /// deserialize cv::Mat, convenience class
        std::pair< boost::posix_time::ptime, cv::Mat > get( const char* buf ) const;
        
        /// deserialize header only
        header get_header( const char* buf ) const;
        
        /// return true, if constructed with no-header option
        bool no_header() const;

        /// return true, if constructed with no-header option
        bool header_only() const { return _header_only; }

        /// return usage
        static const std::string& usage();

        /// return necessary buffer size
        std::size_t size( const cv::Mat& m ) const;
        std::size_t size( const std::pair< boost::posix_time::ptime, cv::Mat >& m ) const;
        std::size_t size( const std::pair< header::buffer_t, cv::Mat >& m ) const;

        /// read from stream, if eof, return empty cv::Mat
        /// @todo: make nothrow
        template < typename H >
        std::pair< H, cv::Mat > read( std::istream& is ) noexcept;
                
        /// return last header buffer after read()
        const std::vector< char >& header_buffer() const;

        /// write to stream
        void write( std::ostream& os, const std::pair< boost::posix_time::ptime, cv::Mat >& m, bool flush = true );

        /// c-style write to stdout, to be used if issues seen with write() - see cpp file for details
        void write_to_stdout( const std::pair< boost::posix_time::ptime, cv::Mat >& m, bool flush = true );
        
        /// write to stream
        void write( std::ostream& os, const std::pair< header::buffer_t, cv::Mat >& m, bool flush = true );
        
        /// c-style write to stdout, to be used if issues seen with write() - see cpp file for details
        void write_to_stdout( const std::pair< header::buffer_t, cv::Mat >& m, bool flush = true );
        
        /// Returns the C-style pointer to the header's binary serializer, NULL if no-header specified in serialisation
        const comma::csv::binary< header >* header_binary() const;

        /// when serialization::read() from a tbb pipeline, at the moment of writing it was not 
        /// possible to establish whether the pipeline exited gracefully or with an error
        /// thus, use this method instead: it returns true, "" on success and
        /// false, <error message> otherwise
        const std::string& last_error() const;

        /// make serialization from input/output command line options
        static std::pair< serialization, serialization > make( const comma::command_line_options& o, bool ignore_csv_options = false );

    private:
        boost::optional< comma::csv::binary< header > > m_binary;
        boost::optional< comma::csv::binary< header > > _no_header_binary;
        /// Same header binary as m_binary, however it ignores the timestamp field 't'
        boost::optional< comma::csv::binary< header > > m_binary_no_timestamp; // ignores timestamp 't' field
        std::vector< char > m_buffer;
        bool _header_only;
        header m_header; /// default header
        bool _set_timestamp{false};
        std::string _last_error;
};

} }  // namespace snark{ namespace cv_mat {
