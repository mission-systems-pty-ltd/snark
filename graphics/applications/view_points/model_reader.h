// Copyright (c) 2011 The University of Sydney

/// @author Cedric Wohlleber

#pragma once

#include "reader.h"

#if Qt3D_VERSION==1
#include "qt3d_v1/model.h"
#include "qt3d_v1/ply_loader.h"

#elif Qt3D_VERSION>=2
#include "qopengl/model.h"
#endif

namespace snark { namespace graphics { namespace view {

struct model_options
{
    std::string filename;
    bool flip;
    double scale;
    model_options() : flip( false ), scale( 1.0 ) {}
};

/// display 3d models ( obj or 3ds ), set its position from an input csv stream
class model_reader : public Reader
{
    public:
        model_reader( const reader_parameters& params
                   , const model_options& options
                   , colored* c
                   , const std::string& label );

        void start();
        std::size_t update( const Eigen::Vector3d& offset );
        const Eigen::Vector3d& somePoint() const;
        bool read_once();
        bool empty() const;
        
#if Qt3D_VERSION==1
        void render( Viewer& viewer, QGLPainter *painter );
        
#elif Qt3D_VERSION>=2
    virtual void add_shaders(snark::graphics::qopengl::viewer_base* viewer_base);
    virtual void update_view();
#endif

    protected:
        boost::scoped_ptr< comma::csv::input_stream< PointWithId > > m_stream;
        boost::scoped_ptr< comma::csv::passed< PointWithId > > m_passed;
        
#if Qt3D_VERSION==1
        qt3d_v1::model model;
        boost::optional< PlyLoader > m_plyLoader;
        
#elif Qt3D_VERSION>=2
        qopengl::model model;
#endif
        const std::string m_file;
        bool m_flip;
        double scale_;
        const colored* colored_;
};

} } } // namespace snark { namespace graphics { namespace view {
    
namespace comma { namespace visiting {

template <> struct traits< snark::graphics::view::model_options >
{
    template < typename Key, class Visitor > static void visit( Key, snark::graphics::view::model_options& p, Visitor& v )
    {
        v.apply( "filename", p.filename );
        v.apply( "flip", p.flip );
        v.apply( "scale", p.scale );
    }

    template < typename Key, class Visitor > static void visit( Key, const snark::graphics::view::model_options& p, Visitor& v )
    {
        v.apply( "filename", p.filename );
        v.apply( "flip", p.flip );
        v.apply( "scale", p.scale );
    }
};

} } // namespace comma { namespace visiting {
   
