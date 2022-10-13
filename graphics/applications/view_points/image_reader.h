// Copyright (c) 2017 The University of Sydney

#pragma once

#include "reader.h"

#if Qt3D_VERSION>=2
#include "../../qt5.5/qopengl/textures.h"
#endif

namespace snark { namespace graphics { namespace view {

struct image_options
{
    std::string filename;
    Eigen::Vector2d size;
    image_options();
    image_options(const std::string& filename);
    image_options(const std::string& filename, double pixel_size );
    image_options(const std::string& filename, double width, double height);
};

#if Qt3D_VERSION>=2
//TODO move this to qt3d_v2?
struct image
{
    image(const image_options& options);
    void update_view(const Eigen::Vector3d& position,const Eigen::Vector3d& orientation);
    snark::math::closed_interval<float,3> extents() const;
    
    Eigen::Vector2d size;
    std::shared_ptr<snark::graphics::qopengl::textures::image> image_texture;
};
typedef image image_t;
#endif
    
struct image_reader : public Reader
{
    image_reader(const reader_parameters& params, const std::vector<image_options>& io);

    void start();
    std::size_t update( const Eigen::Vector3d& offset );
    const Eigen::Vector3d& some_point() const;
    bool read_once();
    bool empty() const;
    
#if Qt3D_VERSION==1
    void render( Viewer& viewer, QGLPainter *painter );
    
#elif Qt3D_VERSION>=2
    virtual void add_shaders(snark::graphics::qopengl::viewer_base* viewer_base);
    virtual void update_view();
#endif
        
    
protected:
    boost::scoped_ptr< comma::csv::input_stream< PointWithId > > stream;
    boost::scoped_ptr< comma::csv::passed< PointWithId > > passed;
#if Qt3D_VERSION>=2
    std::shared_ptr<snark::graphics::qopengl::texture_shader> texture_shader;
    std::vector<std::unique_ptr<image_t>> images;
#endif
};
    
} } } // namespace snark { namespace graphics { namespace view {
    
