// Copyright (c) 2016 The University of Sydney
// Copyright (c) 2020 Vsevolod Vlaskine

#pragma once

#if Qt3D_VERSION==1
#include "qt3d_v1/viewer.h"

#elif Qt3D_VERSION>=2
#include "qopengl/viewer.h"

#endif

#include <QMainWindow>
#include <vector>
#include <memory>
#include "../../../math/interval.h"
#include "reader.h"
#ifndef Q_MOC_RUN
#include <boost/property_tree/ptree.hpp>
#include "boost/optional.hpp"
#include "boost/scoped_ptr.hpp"
#endif
#include "click_mode.h"
#include "camera_reader.h"
#include "types.h"

namespace snark { namespace graphics { namespace view {

template < typename T > struct controller_traits;

#if Qt3D_VERSION==1

typedef Viewer controller; //mock controller for now, TODO refactor Viewer and controller

template <> struct controller_traits< controller >
{
    static viewer_t* get_widget( std::shared_ptr< controller >& t ) { return t.get(); }
};

#elif Qt3D_VERSION>=2
/**
 * manages readers and camera
 * contains a viewer which performs the rendering and is Qt3d version dependant
 */
class controller : public controller_base
{
public:
    std::shared_ptr< viewer_t > viewer;
    std::vector< std::unique_ptr< snark::graphics::view::Reader > > readers;

    /// @todo split into several constructors; make camera configuration a separate class
    controller( const color_t& background_color
              , const viewer_t::camera::options& camera_options
              , bool exit_on_end_of_input
              , const boost::optional< comma::csv::options >& camera_csv
              , const boost::optional< Eigen::Vector3d >& camera_position
              , const boost::optional< Eigen::Vector3d >& camera_orientation
              , const std::string& camera_config_filename
              , const QVector3D& scene_center
              , double scene_radius
              , bool output_camera_config
              , bool output_camera_position
              , const snark::graphics::view::click_mode& click_mode
              , const std::string& grab_options
              , const std::string& on_exit_options );
    ~controller();
    void inhibit_stdout();
    void shutdown( bool kill = true );
    void tick() { read(); }

    void read();
    void update_view();
    void load_camera_config( const std::string& filename );
    void write_camera_config( std::ostream& os );

private:
    bool m_shutdown;
    boost::scoped_ptr< CameraReader > _camera_reader;
    boost::optional< Eigen::Vector3d > m_cameraposition;
    boost::optional< Eigen::Vector3d > m_cameraorientation;
    bool _initial_camera_position_from_scene_extents{true};
    bool _initial_camera_position_set{false};
    //add camera_position_output
    bool m_exit_on_end_of_input;
    std::string _camera_config_filename;  // quick and dirty
    bool _camera_config_from_file{false}; // quick and dirty
    bool _camera_config_ready_to_load{false}; // todo: quick and dirty; vodoo; redesign camera management
    boost::optional< snark::math::closed_interval< float, 3 > > _extents;

    void _update_view();

public:
    void add( std::unique_ptr< snark::graphics::view::Reader >&& reader );
    void init();
};

template <> struct controller_traits< controller >
{
    static viewer_t* get_widget( std::shared_ptr< controller >& t ) { return t->viewer.get(); }
};
#endif

} } } // namespace snark { namespace graphics { namespace view {
