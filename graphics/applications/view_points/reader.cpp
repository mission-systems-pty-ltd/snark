// Copyright (c) 2011 The University of Sydney

/// @author Vsevolod Vlaskine, Cedric Wohlleber

#include "reader.h"
#include <Eigen/Geometry>
#include <comma/string/split.h>
#include "../../../math/rotation_matrix.h"

namespace snark { namespace graphics { namespace view {

#if Qt3D_VERSION>=2
const color_t stock::red(255,0,0);
const color_t stock::green(0,255,0);
const color_t stock::blue(0,0,255);
#endif

Reader::Reader( const reader_parameters& params
              , colored* c
              , const std::string& label
              , const Eigen::Vector3d& offset )
    : reader_parameters( params )
    , m_num_points( 0 )
    , m_colored( c )
    , m_shutdown( false )
    , _is_stdin( options.filename == "-" )
    , m_show( true )
    , m_istream( options.filename, options.binary() ? comma::io::mode::binary : comma::io::mode::ascii, comma::io::mode::non_blocking )
    , m_pass_through( params.pass_through ? &std::cout : 0 )
    , updated_( false )
    , id_( 0 )
    , m_label( label )
    , m_offset( offset )
{}

void Reader::shutdown()
{
    m_shutdown = true;
    if( m_thread ) { m_thread->join(); }
    m_istream.close();
}

bool Reader::isShutdown() const { return m_shutdown; }

void Reader::show( bool s ) { m_show = s; }

bool Reader::show() const { return m_show; }

void Reader::read()
{
    for( ; !m_shutdown && read_once(); ++m_num_points );
    std::cerr << "view-points: end of \"" << title << "\" (" << options.filename << "); read " << m_num_points << " record(s)" << std::endl;
    m_shutdown = true;
    if( m_pass_through ) { std::cout.flush(); close( STDOUT_FILENO ); } // this probably shouldn't be here, but shutdown() is never called, so it can't be there.
}

bool Reader::update_point( const Eigen::Vector3d& offset )
{
    if( !m_point ) { return false; } // is it safe to do it without locking the mutex?
    boost::mutex::scoped_lock lock( m_mutex );
    if( !updated_ ) { return false; }
    m_translation = *m_point;
    m_offset = offset;
    if( m_orientation )
    {
        const Eigen::Quaterniond& q = snark::rotation_matrix( *m_orientation ).quaternion();
        m_quaternion = QQuaternion( q.w(), q.x(), q.y(), q.z() );
    }
    updated_ = false;
    return true;
}

} } } // namespace snark { namespace graphics { namespace view {
