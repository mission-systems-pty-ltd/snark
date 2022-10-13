// Copyright (c) 2011 The University of Sydney

/// @author Cedric Wohlleber

#pragma once

#include <boost/ptr_container/ptr_vector.hpp>
#include "../reader.h"
#include <Qt3D/qglbuilder.h>

namespace snark { namespace graphics { namespace view {

/// display an image as a texture, set its position from an input csv stream
class texture_reader : public Reader
{
    public:
        struct image_options
        {
            std::string filename;
            double width;
            double height;
            boost::optional< double > pixel_size;
            image_options() : width( 1 ), height( 1 ) {}
            image_options( const std::string& filename ) : filename( filename ), width( 1 ), height( 1 ) {}
            image_options( const std::string& filename, double pixel_size ) : filename( filename ), width( 0 ), height( 0 ), pixel_size( pixel_size ) {}
            image_options( const std::string& filename, double width, double height ) : filename( filename ), width( width ), height( height ) {}
        };

        texture_reader( const reader_parameters& params, const std::vector< image_options >& io );

        void start();
        std::size_t update( const Eigen::Vector3d& offset );
        const Eigen::Vector3d& some_point() const;
        bool read_once();
        void render( Viewer& viewer, QGLPainter *painter );
        bool empty() const;

    protected:
        boost::scoped_ptr< comma::csv::input_stream< PointWithId > > m_stream;
        boost::scoped_ptr< comma::csv::passed< PointWithId > > m_passed;
        struct image_
        {
            QImage image;
            QGeometryData geometry;
            QGLBuilder builder;
            QGLSceneNode* node;
            QGLTexture2D texture;
            QGLMaterial material;

            image_( const image_options& o );
        };
        boost::ptr_vector< image_ > images_;
};

} } } // namespace snark { namespace graphics { namespace view {

#endif /*SNARK_GRAPHICS_APPLICATIONS_VIEWPOINTS_TEXTURE_READER_H_*/
