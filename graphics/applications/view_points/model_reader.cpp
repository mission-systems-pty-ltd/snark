// Copyright (c) 2011 The University of Sydney

/// @author Cedric Wohlleber

#include "model_reader.h"
#if Qt3D_VERSION==1
#include "qt3d_v1/viewer.h"
#endif

namespace snark { namespace graphics { namespace view {

/// constructor
/// @param params csv options for the position input
/// @param mo model options
/// @param c color used for the label
/// @param label text displayed as label
model_reader::model_reader( const reader_parameters& params
                        , const model_options& mo
                        , snark::graphics::view::colored* c
                        , const std::string& label )
    : Reader( reader_parameters( params ), c, label, Eigen::Vector3d( 0, 1, 1 ) ) // TODO make offset configurable ?
    , m_file( mo.filename )
    , m_flip( mo.flip )
    , scale_( mo.scale )
    , colored_( c )
{
}

void model_reader::start()
{
#if Qt3D_VERSION==1
    if( m_file.substr( m_file.size() - 3, 3 ) == "ply" )
    {
        boost::optional< QColor4ub > color;
        if( dynamic_cast< const fixed* >( colored_ ) ) { color = colored_->color( Eigen::Vector3d( 0, 0, 0 ), 0, 0, QColor4ub() ); } // quick and dirty
        m_plyLoader = PlyLoader( m_file, color, scale_ );
    }
    if( !m_plyLoader )
    {
        if( !comma::math::equal( scale_, 1.0 ) ) { std::cerr << "view-points: warning: scale supported only for ply models; others: todo" << std::endl; }
        model.load( m_file );
    }
#elif Qt3D_VERSION>=2
    model.load( m_file );
#endif
    m_thread.reset( new boost::thread( boost::bind( &Reader::read, boost::ref( *this ) ) ) );
}

std::size_t model_reader::update( const Eigen::Vector3d& offset ) { return update_point( offset ) ? 1 : 0; }

bool model_reader::empty() const { return !m_point; }

const Eigen::Vector3d& model_reader::some_point() const
{
    boost::mutex::scoped_lock lock( m_mutex );
    return *m_point;
}

#if Qt3D_VERSION==1
void model_reader::render( Viewer& viewer, QGLPainter* painter )
{
    painter->modelViewMatrix().push();
    Eigen::Vector3d d = m_translation - m_offset;
    painter->modelViewMatrix().translate( QVector3D( d.x(), d.y(), d.z() ) );
    painter->modelViewMatrix().rotate( m_quaternion );
    if( m_flip ) { painter->modelViewMatrix().rotate( 180, 1, 0, 0 ); }
    if( m_plyLoader ) { m_plyLoader->draw( painter ); }
    else { model.render(painter); }
    painter->modelViewMatrix().pop();
    if( !m_label.empty() ) { viewer.draw_label( painter, m_translation - m_offset, m_color, m_label ); }
}

#elif Qt3D_VERSION>=2
void model_reader::add_shaders( snark::graphics::qopengl::viewer_base* viewer_base ) { model.add_shaders( viewer_base ); }

void model_reader::update_view()
{
    model.update_view();
    model.mesh_shader->update_transform( m_translation - m_offset, m_orientation ? *m_orientation : Eigen::Vector3d( 0, 0, 0 ) );
    model.mesh_shader->visible = m_show;
}
#endif

bool model_reader::read_once()
{
    if( !m_stream ) // quick and dirty: handle named pipes
    {
        if( !m_istream() ) { return true; }
        PointWithId default_point;
        default_point.point = Eigen::Vector3d( 0, 0, 0 );
        default_point.orientation = Eigen::Vector3d( 0, 0, 0 );
        m_stream.reset( new comma::csv::input_stream< PointWithId >( *m_istream(), options, default_point ) );
        if( m_pass_through ) { m_passed.reset( new comma::csv::passed< PointWithId >( *m_stream, *m_pass_through )); }
        else { m_passed.reset(); }
    }
    const PointWithId* p = m_stream->read();
    if( p == NULL ) { m_shutdown = true; return false; }
    if( m_passed ) { m_passed->write(); }
    boost::mutex::scoped_lock lock( m_mutex );
    m_point = p->point;
    m_orientation = p->orientation;
    m_color = m_colored->color( p->point, p->id, p->scalar, p->color );
    updated_ = true;
    return true;
}

} } } // namespace snark { namespace graphics { namespace view {
