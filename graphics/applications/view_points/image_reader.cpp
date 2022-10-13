// Copyright (c) 2017 The University of Sydney

#include "image_reader.h"

namespace snark { namespace graphics { namespace view {

image_options::image_options() : size(1,1) {}
image_options::image_options(const std::string& filename) : filename(filename), size(1,1) { }
image_options::image_options(const std::string& filename, double pixel_size) : filename(filename), size(0,pixel_size) { }
image_options::image_options(const std::string& filename, double width, double height) : filename(filename), size(width,height) { }

#if Qt3D_VERSION>=2
image::image(const image_options& options)
{
    QImage qimage(options.filename.c_str());
    size=options.size;
    if(options.size.x()==0) { size=Eigen::Vector2d(qimage.size().width(),qimage.size().height())*options.size.y(); }
    image_texture.reset(new snark::graphics::qopengl::textures::image(qimage));
}

void image::update_view(const Eigen::Vector3d& position,const Eigen::Vector3d& orientation) { image_texture->update_quad(position,orientation,size); }

snark::math::closed_interval<float,3> image::extents() const
{
    auto quad=image_texture->get_quad();
    snark::math::closed_interval<float,3> ext=snark::math::closed_interval<float,3>(quad[0].position);
    for( std::size_t i=1; i<quad.size(); i++ ) { ext = ext.hull( quad[i].position ); }
    return ext;
}
#endif
    
image_reader::image_reader( const reader_parameters& params, const std::vector< image_options >& io )
    : Reader( reader_parameters( params ), NULL, "", Eigen::Vector3d( 0, 1, 1 ) )
{
#if Qt3D_VERSION>=2
    for( unsigned int i = 0; i < io.size(); ++i ) { images.push_back(std::unique_ptr<image_t>(new image_t( io[i] ) )); }
#endif
}

void image_reader::start() { m_thread.reset( new boost::thread( boost::bind( &Reader::read, boost::ref( *this ) ) ) ); }

std::size_t image_reader::update( const Eigen::Vector3d& offset ) { return updatePoint( offset ) ? 1 : 0; }

bool image_reader::empty() const { return !m_point; }

const Eigen::Vector3d& image_reader::some_point() const
{
    boost::mutex::scoped_lock lock( m_mutex );
    return *m_point;
}

bool image_reader::read_once()
{
    if( !stream ) // quick and dirty: handle named pipes
    {
        if( !m_istream() ) { return true; }
        stream.reset( new comma::csv::input_stream< PointWithId >( *m_istream(), options ) );
        if( m_pass_through ) { passed.reset( new comma::csv::passed< PointWithId >( *stream, *m_pass_through )); }
        else { passed.reset(); }
    }
    const PointWithId* p = stream->read();
    if( p == NULL ) { m_shutdown = true; return false; }
    if( passed ) { passed->write(); }
    boost::mutex::scoped_lock lock( m_mutex );
    m_point = p->point;
    m_orientation = p->orientation;
    id_ = p->id;
    updated_ = true;
    return true;
}

#if Qt3D_VERSION==1
void image_reader::render( Viewer& viewer, QGLPainter* painter )
{
    if( !m_point ) { return; }
//    comma::uint32 id = id_;
//     if( id >= images_.size() ) { return; }
    painter->setStandardEffect( QGL::FlatReplaceTexture2D );
    painter->modelViewMatrix().push();
    Eigen::Vector3d d = m_translation - m_offset;
    painter->modelViewMatrix().translate( QVector3D( d.x(), d.y(), d.z() ) );
    painter->modelViewMatrix().rotate( m_quaternion );
//     images_[id].node->draw( painter );
    painter->modelViewMatrix().pop();
}

#elif Qt3D_VERSION>=2
void image_reader::add_shaders(snark::graphics::qopengl::viewer_base* viewer_base)
{
//     std::cerr<<"image_reader::add_shaders"<<std::endl;
    texture_shader.reset(new snark::graphics::qopengl::texture_shader());
    viewer_base->add_texture_shader(texture_shader);
    for( auto& i : images ) { texture_shader->textures.push_back(i->image_texture); }
}
void image_reader::update_view()
{
    for( unsigned i = 0; i < images.size(); i++ ) { images[i]->image_texture->visible=(i==id_); }
    if(m_point && id_<images.size())
    {
        images[id_]->update_view( *m_point - m_offset, m_orientation ? *m_orientation : Eigen::Vector3d(0,0,0));
        m_extents=images[id_]->extents();
    }
    texture_shader->visible = m_show;
    texture_shader->update();
}
    
#endif

} } } // namespace snark { namespace graphics { namespace view {
