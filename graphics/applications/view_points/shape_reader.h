// Copyright (c) 2011 The University of Sydney

/// @author Vsevolod Vlaskine, Cedric Wohlleber

#pragma once

#ifdef WIN32
#include <winsock2.h>
//#include <windows.h>
#endif

#include "shape_with_id.h"
#include "reader.h"
#if Qt3D_VERSION==1
#include "qt3d_v1/viewer.h"
#elif Qt3D_VERSION>=2
#include "../../qt5.5/qopengl/labels.h"
#endif

namespace snark { namespace graphics { namespace view {
    
template< typename S, typename How = how_t::points >
class ShapeReader : public shape_reader_base
{
    public:
        ShapeReader( const reader_parameters& params, colored* c, const std::string& label, const S& sample = ShapeWithId< S >().shape );

        void start();
        std::size_t update( const Eigen::Vector3d& offset );
        const Eigen::Vector3d& some_point() const;
        bool read_once();
        #if Qt3D_VERSION==1
        void render( Viewer& viewer, QGLPainter *painter = NULL );
        #endif
        bool empty() const;

#if Qt3D_VERSION>=2
public:
    virtual void add_shaders(snark::graphics::qopengl::viewer_base* viewer_base);
    virtual void update_view();
protected:
    void update_shape();
    void update_labels();
private:
    std::shared_ptr< snark::graphics::qopengl::shape > shape;
    std::shared_ptr< snark::graphics::qopengl::label_shader > label_shader;
#endif

    private:
        typedef std::deque< ShapeWithId< S > > deque_t_;
        deque_t_ m_deque;
        mutable boost::mutex m_mutex;
        boost::scoped_ptr< comma::csv::input_stream< ShapeWithId< S > > > m_stream;
        boost::scoped_ptr< comma::csv::passed< ShapeWithId< S > > > m_passed;

        ShapeWithId< S > sample_;
};
#if Qt3D_VERSION>=2
template< typename S, typename How >
inline void ShapeReader< S, How >::add_shaders( snark::graphics::qopengl::viewer_base* viewer_base )
{
    shape.reset( shape_traits< S, How >::make_shape( gl_parameters( point_size, fill ) ) );
    viewer_base->add_shape(shape);
    label_shader = std::shared_ptr< snark::graphics::qopengl::label_shader >( new snark::graphics::qopengl::label_shader() );
    viewer_base->add_label_shader(label_shader);
}
template< typename S, typename How >
inline void ShapeReader< S, How >::update_view()
{
    update_shape();
    update_labels();
}
template< typename S, typename How >
inline void ShapeReader< S, How >::update_shape()
{
    if( !shape ) { return; }
    shape->visible = m_show;
    shape->update( buffer_.values().data(), buffer_.size() );
}

template< typename S, typename How >
inline void ShapeReader< S, How >::update_labels()
{
    label_shader->clear();
    label_shader->visible=m_show;
    label_shader->labels.reserve(labels_.size());
    for( unsigned int i = 0; i < labels_.size(); i++ )
    {
        if(!labels_.values()[i].text.empty())
        {
            label_shader->labels.push_back(std::shared_ptr<snark::graphics::qopengl::label>( new snark::graphics::qopengl::text_label( labels_.values()[i].position - m_offset, labels_.values()[i].text, labels_.values()[i].color, this->font_size ) ) );
        }
    }
    if( !m_label.empty() )
    {
        label_shader->labels.push_back(std::shared_ptr<snark::graphics::qopengl::label>( new snark::graphics::qopengl::text_label( m_translation - m_offset, m_label, m_color, this->font_size ) ) );
    }
    label_shader->update();
}
#endif

template< typename S, typename How >
inline ShapeReader< S, How >::ShapeReader( const reader_parameters& params, colored* c, const std::string& label, const S& sample  )
    : shape_reader_base( params, c, label, shape_traits< S, How >::size )
    , sample_( sample )
{
}

template< typename S, typename How >
inline void ShapeReader< S, How >::start() { m_thread.reset( new boost::thread( boost::bind( &Reader::read, boost::ref( *this ) ) ) ); }

template< typename S, typename How >
inline std::size_t ShapeReader< S, How >::update( const Eigen::Vector3d& offset )
{
    boost::mutex::scoped_lock lock( m_mutex );
    for( typename deque_t_::iterator it = m_deque.begin(); it != m_deque.end(); ++it )
    {
        shape_traits< S, How >::update( *this, *it, offset );
        labels_.add( label_t( shape_traits< S, How >::center( it->shape ), it->color, it->label ), it->block );
    }
    if( m_shutdown )
    {
        buffer_.toggle();
        labels_.toggle();
    }
    std::size_t s = m_deque.size();
    m_deque.clear();
    updated_ = true;
    update_point( offset );
    return s;
}

template< typename S, typename How >
inline bool ShapeReader< S, How >::empty() const
{
    boost::mutex::scoped_lock lock( m_mutex );
    return m_deque.empty();
}

template< typename S, typename How >
inline const Eigen::Vector3d& ShapeReader< S, How >::some_point() const
{
    boost::mutex::scoped_lock lock( m_mutex );
    return shape_traits< S, How >::some_point( m_deque.front().shape );
}

#if Qt3D_VERSION==1
template< typename S, typename How >
inline void ShapeReader< S, How >::render( Viewer& viewer, QGLPainter* painter )
{
    painter->setStandardEffect(QGL::FlatPerVertexColor);
    painter->clearAttributes();

    QGLAttributeValue position_attribute( sizeof((( vertex_t* )0 )->position ) / sizeof( float )
                                        , GL_FLOAT
                                        , sizeof( vertex_t )
                                        , (char *)buffer_.values().data() + offsetof( vertex_t, position )
                                        , buffer_.size() );

    QGLAttributeValue color_attribute( sizeof((( vertex_t* )0 )->color ) / sizeof( char )
                                     , GL_UNSIGNED_BYTE
                                     , sizeof( vertex_t )
                                     , (char *)buffer_.values().data() + offsetof( vertex_t, color )
                                     , buffer_.size());

    painter->setVertexAttribute( QGL::Position, position_attribute );
    painter->setVertexAttribute( QGL::Color, color_attribute );

    shape_traits< S, How >::draw( painter, buffer_.size(), fill );
    for( unsigned int i = 0; i < labels_.size(); i++ ) { viewer.draw_label( painter, labels_.values()[i].position - m_offset, labels_.values()[i].color, labels_.values()[i].text ); }
    if( !m_label.empty() ) { viewer.draw_label( painter, m_translation - m_offset, m_color, m_label ); }
}
#endif

template< typename S, typename How >
inline bool ShapeReader< S, How >::read_once()
{
    try
    {
        if( !m_stream ) // quick and dirty: handle named pipes
        {
            if( !m_istream() )
            {
#ifndef WIN32
                // HACK poll on blocking pipe
                ::usleep( 1000 );
#endif
                return true;
            }
            m_stream.reset( new comma::csv::input_stream< ShapeWithId< S > >( *m_istream(), options, sample_ ) );
            if( m_pass_through ) { m_passed.reset( new comma::csv::passed< ShapeWithId< S > >( *m_stream, *m_pass_through )); }
            else { m_passed.reset(); }
        }
        const ShapeWithId< S >* p = m_stream->read();
        if( p == NULL )
        {
            m_shutdown = true;
            return false;
        }
        if( m_passed ) { m_passed->write(); }
        ShapeWithId< S > v = *p;
        const Eigen::Vector3d& center = shape_traits< S, How >::center( v.shape );
        v.color = m_colored->color( center, p->id, p->scalar, p->color );
        boost::mutex::scoped_lock lock( m_mutex );
        m_deque.push_back( v );
        m_point = shape_traits< S, How >::some_point( v.shape );
        m_color = v.color;
        return true;
    }
    catch( std::exception& ex ) { std::cerr << "view-points: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "view-points: unknown exception" << std::endl; }
    return false;
}

} } } // namespace snark { namespace graphics { namespace view {
