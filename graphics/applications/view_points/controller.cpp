// Copyright (c) 2016 The University of Sydney
// Copyright (c) 2020 Vsevolod Vlaskine

#include <signal.h>
#include <iomanip>
#include <iostream>
#ifndef Q_MOC_RUN
#include <boost/filesystem/operations.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/thread/thread_time.hpp>
#include <comma/csv/stream.h>
#include <comma/name_value/ptree.h>
#include <comma/visiting/apply.h>
#endif
#include "controller.h"
#include "reader.h"

#if Qt3D_VERSION>=2

namespace snark { namespace graphics { namespace view {

void controller::add(std::unique_ptr<snark::graphics::view::Reader>&& reader)
{
#if Qt3D_VERSION>=2
    reader->add_shaders( viewer.get() ); //TODO rename add_shaders to init
#endif
    readers.push_back( std::move( reader ) );
}

void controller::init()
{
    if( m_cameraReader ) { m_cameraReader->start(); }
    for( auto& i : readers ) { i->start(); }
}

controller::controller( const color_t& background_color
                      , const qt3d::camera_options& camera_options
                      , bool exit_on_end_of_input
                      , const boost::optional< comma::csv::options >& camera_csv
                      , const boost::optional< Eigen::Vector3d >& cameraposition
                      , const boost::optional< Eigen::Vector3d >& cameraorientation
                      , const std::string& camera_config_file_name
                      , const QVector3D& scene_center
                      , double scene_radius
                      , bool output_camera_config
                      , bool output_camera_position
                      , const snark::graphics::view::click_mode& click_mode
                      , const std::string& grab_options )
    : m_lookAt( false )
    , m_cameraposition( cameraposition )
    , m_cameraorientation( cameraorientation )
    , m_exit_on_end_of_input( exit_on_end_of_input )
    , _camera_config_from_file( !camera_config_file_name.empty() )
{
#if Qt3D_VERSION==1
//     viewer=new viewer_t(background_color, camera_options, scene_center, scene_radius,parent);
    COMMA_THROW( comma::exception," not implemented ");
#elif Qt3D_VERSION>=2
    viewer.reset( new viewer_t( this,background_color, camera_options, scene_center, scene_radius, click_mode, grab_options ) );
    viewer->output_camera_config = output_camera_config; // super-quick and dirty
    viewer->output_camera_position = output_camera_position; // super-quick and dirty
#endif
    if( !camera_config_file_name.empty() ) { viewer->load_camera_config( camera_config_file_name ); }
    if( camera_csv ) { m_cameraReader.reset( new CameraReader( *camera_csv ) ); }
    m_cameraFixed = m_cameraposition || m_cameraReader || !camera_config_file_name.empty();
}

controller::~controller() { shutdown( false ); }

void controller::inhibit_stdout() { viewer->stdout_allowed = false; }

void controller::shutdown( bool kill )
{
    m_shutdown = true;
    if( kill )
    {
        #ifdef WIN32
        ::raise( SIGTERM ); // according to windows docs, SIGINT does not work on windows
        #else // #ifdef WIN32
        ::raise( SIGINT ); // quick and dirty... or maybe not so dirty: to interrupt blocking read() in reader threads, if closed from qt window
        #endif // #ifdef WIN32
    }
    if( m_cameraReader ) { m_cameraReader->shutdown(); }
    for( auto& i: readers ) { i->shutdown(); }
}

void controller::_update_view()
{
    //std::cerr << "--> controller::_update_view(): a" << std::endl;
    if( !_extents ) { return; }
    //std::cerr << "--> controller::_update_view(): b" << std::endl;
    QVector3D min( _extents->min().x(), _extents->min().y(), _extents->min().z() );
    QVector3D max( _extents->max().x(), _extents->max().y(), _extents->max().z() );
    viewer->update_view( min, max );
}

void controller::read()
{
//     if( viewer == NULL ) { return; }
    for( unsigned int i = 0; !viewer->m_offset && i < readers.size(); ++i )
    {
        if( readers[i]->empty() ) { continue; }
        Eigen::Vector3d p = readers[i]->some_point();
        viewer->m_offset = std::fabs( p.x() ) > 1000 || std::fabs( p.y() ) > 1000 || std::fabs( p.z() ) > 1000 ? p : Eigen::Vector3d( 0, 0, 0 );
        std::cerr << "view-points: reader no." << i << std::setprecision( 16 ) << " scene offset: " << viewer->m_offset->x() << ',' << viewer->m_offset->y() << ',' << viewer->m_offset->z() << "; scene radius: " << viewer->scene_radius << std::endl;
    }
    if( !viewer->m_offset ) { return; }
    bool need_update = false;
    for( unsigned int i = 0; i < readers.size(); ++i )
    {
        if( readers[i]->update( *viewer->m_offset ) > 0 ) { need_update = true; };
    }
    m_shutdown = true;
    bool ready_to_look = true;
    for( unsigned int i = 0; m_shutdown && i < readers.size(); ++i )
    {
        m_shutdown = m_shutdown && readers[i]->isShutdown();
        ready_to_look = ready_to_look && ( readers[i]->isShutdown() || ( readers.size() > 1 && readers[i]->is_stdin() ) );
    }
    bool extents_ready = readers[0]->m_extents && readers[0]->m_num_points > 0 && ( m_shutdown || ready_to_look || readers[0]->m_num_points >= readers[0]->size / 10 );
    if( extents_ready )
    {
        if( !_extents || *_extents != *( readers[0]->m_extents ) ) // quick and dirty; // todo? call _update_view() on camera moves?
        {
            _extents = readers[0]->m_extents;
            if( !_camera_config_from_file ) { _update_view(); } // quick and dirty
        }
    }
    if( !m_cameraReader && m_cameraposition )
    {
        viewer->set_camera_position( *m_cameraposition, *m_cameraorientation );
        _update_view();
        m_cameraposition.reset();
        m_cameraorientation.reset();
        need_update = true;
    }
    else if( m_cameraReader )
    {
        Eigen::Vector3d position = m_cameraReader->position();
        Eigen::Vector3d orientation = m_cameraReader->orientation();
        if( !m_cameraposition || !m_cameraposition->isApprox( position ) || !m_cameraorientation->isApprox( orientation ) ) // todo? move to viewer::set_camera_position()?
        {
            m_cameraposition = position;
            m_cameraorientation = orientation;
            viewer->set_camera_position( position, orientation );
            _update_view();
            need_update = true;
        }
    }
    else if( extents_ready )
    {
        if( !m_cameraFixed && !m_lookAt )
        {
            m_lookAt = true;
            viewer->look_at_center();
        }
    }
    if( need_update ) { update_view(); }
    if( m_shutdown && m_exit_on_end_of_input ) { shutdown(); }
}

void controller::update_view()
{
#if Qt3D_VERSION>=2
    //update shapes
    viewer->begin_update();
    for( auto& i : readers ) { i->update_view(); }
    viewer->end_update();
#endif
    viewer->update();
}

void controller::load_camera_config( const std::string& file_name ) { viewer->load_camera_config(file_name); }

void controller::write_camera_config( std::ostream& os ) { viewer->write_camera_config(os); }

} } } // namespace snark { namespace graphics { namespace view {

#endif
