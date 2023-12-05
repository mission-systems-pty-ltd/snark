// Copyright (c) 2011 The University of Sydney

/// @author Cedric Wohlleber

#include "texture_reader.h"

namespace snark { namespace graphics { namespace view {

texture_reader::image_::image_( const texture_reader::image_options& o ) : image( &o.filename[0] )
{
    texture.setImage( image );
    material.setTexture( &texture );
    comma::uint32 width = o.pixel_size ? *o.pixel_size * image.width() : o.width;
    comma::uint32 height = o.pixel_size ? *o.pixel_size * image.height() : o.height;
    if( width == 0 ) { COMMA_THROW( comma::exception, "got zero width for image " << o.filename ); }
    if( height == 0 ) { COMMA_THROW( comma::exception, "got zero height for image " << o.filename ); }
    QVector3D a( 0, 0, 0 );
    QVector3D b( width, 0, 0 );
    QVector3D c( width, height, 0 );
    QVector3D d( 0, height, 0 );
    QVector2D ta( 0, 0 );
    QVector2D tb( 1, 0 );
    QVector2D tc( 1, 1 );
    QVector2D td( 0, 1 );
    geometry.appendVertex( a, b, c, d );
    geometry.appendTexCoord(ta, tb, tc, td);
    builder.addQuads( geometry );
    node = builder.finalizedSceneNode();
    node->setMaterial( &material );
}

texture_reader::texture_reader( const reader_parameters& params, const std::vector< image_options >& io )
    : Reader( reader_parameters( params ), NULL, "", Eigen::Vector3d( 0, 1, 1 ) )
{
    for( unsigned int i = 0; i < io.size(); ++i ) { images_.push_back( new image_( io[i] ) ); }
}

void texture_reader::start() { m_thread.reset( new boost::thread( boost::bind( &Reader::read, boost::ref( *this ) ) ) ); }

std::size_t texture_reader::update( const Eigen::Vector3d& offset ) { return update_point( offset ) ? 1 : 0; }

bool texture_reader::empty() const { return !m_point; }

const Eigen::Vector3d& texture_reader::some_point() const
{
    boost::mutex::scoped_lock lock( m_mutex );
    return *m_point;
}

void texture_reader::render( Viewer& viewer, QGLPainter* painter )
{
    if( !m_point ) { return; }
    comma::uint32 id = id_;
    if( id >= images_.size() ) { return; }
    painter->setStandardEffect( QGL::FlatReplaceTexture2D );
    painter->modelViewMatrix().push();
    Eigen::Vector3d d = m_translation - m_offset;
    painter->modelViewMatrix().translate( QVector3D( d.x(), d.y(), d.z() ) );
    painter->modelViewMatrix().rotate( m_quaternion );
    images_[id].node->draw( painter );
    painter->modelViewMatrix().pop();
}

// todo: something like this:
// void texture_reader::render( Viewer& viewer, QGLPainter* painter )
// {
//     boost::mutex::scoped_lock lock( m_mutex );
//     if( !points_.empty() ) { return; }
//     for( const auto& point: points_ )
//     {
//         if( p.id() >= images_.size() ) { continue; }
//         painter->setStandardEffect( QGL::FlatReplaceTexture2D );
//         painter->modelViewMatrix().push();
//         Eigen::Vector3d d = p.translation() - m_offset;
//         painter->modelViewMatrix().translate( QVector3D( d.x(), d.y(), d.z() ) );
//         painter->modelViewMatrix().rotate( m_quaternion );
//         images_[p.id()].node->draw( painter );
//         painter->modelViewMatrix().pop();
//     }
// }

bool texture_reader::read_once()
{
    if( !m_stream ) // quick and dirty: handle named pipes
    {
        if( !m_istream() ) { return true; }
        m_stream.reset( new comma::csv::input_stream< PointWithId >( *m_istream(), options ) );
        if( m_pass_through ) { m_passed.reset( new comma::csv::passed< PointWithId >( *m_stream, *m_pass_through, flush )); }
        else { m_passed.reset(); }
    }
    const PointWithId* p = m_stream->read();
    if( p == NULL ) { m_shutdown = true; return false; }
    if( m_passed ) { m_passed->write(); }
    boost::mutex::scoped_lock lock( m_mutex );
    m_point = p->point;
    m_orientation = p->orientation;
    id_ = p->id;
    updated_ = true;
    return true;
}

} } } // namespace snark { namespace graphics { namespace view {
