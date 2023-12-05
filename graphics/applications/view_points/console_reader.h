// Copyright (c) 2022 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#pragma once

#include "shape_reader.h"
#if Qt3D_VERSION==1
#include "qt3d_v1/viewer.h"
#elif Qt3D_VERSION>=2
#include "../../qt5.5/qopengl/labels.h"
#endif

namespace snark { namespace graphics { namespace view {
    
template< typename S, typename How = how_t::points >
class console_reader : public shape_reader< S, How >
{
    public:
        console_reader( const reader_parameters& params, colored* c, const std::string& label, const S& sample = ShapeWithId< S >().shape );
        bool read_once();
        #if Qt3D_VERSION==1
        void render( Viewer& viewer, QGLPainter *painter = NULL );
        #endif

    private:
        boost::scoped_ptr< comma::csv::passed< ShapeWithId< S > > > m_passed;
};

template< typename S, typename How >
inline bool console_reader< S, How >::read_once()
{
    // todo
    try
    {
//         if( !m_stream ) // quick and dirty: handle named pipes
//         {
//             if( !m_istream() )
//             {
// #ifndef WIN32
//                 // HACK poll on blocking pipe
//                 ::usleep( 1000 );
// #endif
//                 return true;
//             }
//             m_stream.reset( new comma::csv::input_stream< ShapeWithId< S > >( *m_istream(), options, sample_ ) );
//             if( m_pass_through ) { m_passed.reset( new comma::csv::passed< ShapeWithId< S > >( *m_stream, *m_pass_through, flush )); }
//             else { m_passed.reset(); }
//         }
//         const ShapeWithId< S >* p = m_stream->read();
//         if( p == NULL )
//         {
//             m_shutdown = true;
//             return false;
//         }
//         if( m_passed ) { m_passed->write(); }
//         ShapeWithId< S > v = *p;
//         const Eigen::Vector3d& center = shape_traits< S, How >::center( v.shape );
//         v.color = m_colored->color( center, p->id, p->scalar, p->color );
//         boost::mutex::scoped_lock lock( m_mutex );
//         m_deque.push_back( v );
//         m_point = shape_traits< S, How >::some_point( v.shape );
//         m_color = v.color;
        return true;
    }
    catch( std::exception& ex ) { std::cerr << "view-points: " << ex.what() << std::endl; }
    catch( ... ) { std::cerr << "view-points: unknown exception" << std::endl; }
    return false;
}

} } } // namespace snark { namespace graphics { namespace view {
