// Copyright (c) 2017 The University of Sydney
// Copyright (c) 2020 Kent Hu
// Copyright (c) 2020 Vsevolod Vlaskine

#pragma once

#include <memory>
#include <string>
#include <QKeyEvent>
#include <QMainWindow>
#include <boost/optional.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <comma/base/types.h>
#include <comma/io/stream.h>
#include "../../../qt5.5/qopengl/widget.h"
#include "../click_mode.h"
#include "../types.h"

namespace snark { namespace graphics { namespace qt3d { class camera_options; } } }

namespace snark { namespace graphics { namespace view { namespace qopengl {

typedef snark::graphics::qopengl::color_t color_t;

/**
 * render and camera functions
 * qt3d v2 specific rednering, most functions are implemented in widget
 * this class implements interface used by controller
 */
class viewer : public snark::graphics::qopengl::widget
{
    Q_OBJECT

public:
    QVector3D scene_center;
    boost::optional< Eigen::Vector3d > m_offset;
    controller_base* handler;
    bool scene_radius_fixed;
    bool scene_center_fixed;
    bool stdout_allowed;
    bool output_camera_config;
    bool output_camera_position;
    snark::graphics::view::click_mode click_mode;

    viewer( controller_base* handler
          , const color_t& background_color
          , const qt3d::camera_options& camera_options
          , const QVector3D& scene_center
          , double scene_radius
          , const snark::graphics::view::click_mode& click_mode
          , QMainWindow* parent = nullptr );

    void reset_handler(controller_base* h = nullptr);
    void look_at_center();
    void load_camera_config( const std::string& file_name );
    void set_camera_position( const Eigen::Vector3d& position, const Eigen::Vector3d& orientation );
    void update_view( const QVector3D& min, const QVector3D& max );
    void write_camera_config( std::ostream& os, bool on_change = false );

public slots:
    void toggle_block_mode( bool );
    void toggle_label_mode( bool );

protected:
    void init() Q_DECL_OVERRIDE;
    void paintGL() Q_DECL_OVERRIDE;
    void keyPressEvent( QKeyEvent *event ) Q_DECL_OVERRIDE;
    void double_right_click( const boost::optional< QVector3D >& point ) override;

private slots:
    void on_timeout();

private:
    boost::optional< snark::graphics::qopengl::camera_transform > previous_camera_;
    void write_camera_position_( std::ostream& os, bool on_change = false );
    class _image_grab_t // todo: move to a more generic location?
    {
        public:
            _image_grab_t( unsigned int fps = 30, const std::string& name = "" ): _fps( fps ), _period( boost::posix_time::microseconds( 1000000 / fps ) ), _name( name ) {}
            ~_image_grab_t() { _close(); }
            operator bool() const { return bool( _ostream ); }
            void toggle();
            void grab( QOpenGLWidget* w );
            void write( const char* buf, unsigned int size );
        private:
            unsigned int _fps;
            boost::posix_time::time_duration _period;
            boost::posix_time::ptime _last;
            std::string _name;
            std::string _current_name;
            std::unique_ptr< comma::io::ostream > _ostream;
            void _reopen();
            void _close();
    };
    _image_grab_t _image_grab;
};

} } } } // namespace snark { namespace graphics { namespace view { namespace qopengl {
