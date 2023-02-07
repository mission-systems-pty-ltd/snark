// Copyright (c) 2011 The University of Sydney

/// @author Vsevolod Vlaskine, Cedric Wohlleber

#pragma once

#ifndef Q_MOC_RUN
#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>
#include "../../../math/interval.h"
#include <comma/csv/options.h>
#include <comma/csv/stream.h>
#include <comma/io/stream.h>
#endif
#include <Eigen/Core>
#include "colored.h"
#if Qt3D_VERSION==1
#include <Qt3D/qglview.h>
#elif Qt3D_VERSION>=2
#include <QQuaternion>
#include "../../qt5.5/qopengl/viewer_base.h"
#endif
#include "types.h"

namespace snark { namespace graphics { namespace view {

#if Qt3D_VERSION==1
class Viewer;
#endif

struct reader_parameters
{
    comma::csv::options options;
    std::string title;
    std::string groups;
    std::size_t size;
    unsigned int point_size;
    bool pass_through;
    bool fill; // quick and dirty
    std::string labels; //currently used for axis labels e.g. "x:y:z"
    double length;  //currently used for axis length
    bool has_color; //currently used for shape=axis, when false use three different color for xyz axises
    unsigned int font_size; // quick and dirty, used for text labels

    reader_parameters( const comma::csv::options& options
                     , const std::string& title
                     , const std::string& groups
                     , std::size_t size
                     , unsigned int point_size
                     , bool pass_through
                     , bool fill 
                     , const std::string& labels
                     , double length
                     , bool has_color
                     , unsigned int font_size )
        : options( options )
        , title( title )
        , groups( groups )
        , size( size )
        , point_size( point_size )
        , pass_through( pass_through )
        , fill( fill )
        , labels( labels )
        , length( length )
        , has_color( has_color )
        , font_size( font_size )
    {}
};

class Reader : public reader_parameters
{
    public:
        Reader( const reader_parameters& params
              , colored* c
              , const std::string& label
              , const Eigen::Vector3d& offset = Eigen::Vector3d( 0, 0, 0 ) );
        virtual ~Reader() {}
        virtual void start() = 0;
        virtual std::size_t update( const Eigen::Vector3d& offset ) = 0;
        virtual const Eigen::Vector3d& some_point() const = 0;
        virtual bool read_once() = 0;
        virtual bool empty() const = 0;
        
#if Qt3D_VERSION==1
        friend class Viewer;
        virtual void render( Viewer& viewer, QGLPainter *painter ) = 0;
        
#elif Qt3D_VERSION>=2
    friend class controller;
    virtual void add_shaders(snark::graphics::qopengl::viewer_base* viewer_base)=0;
    virtual void update_view()=0;
#endif

        void show( bool s );
        bool show() const;
        bool isShutdown() const;
        bool is_stdin() const { return _is_stdin; }
        void shutdown();
        void read();
        const view::colored& get_colored() const { return *m_colored; } // quick and dirty
             
    protected:
        bool update_point( const Eigen::Vector3d& offset );

        boost::optional< snark::math::closed_interval< float, 3 > > m_extents;
        unsigned int m_num_points;
        boost::scoped_ptr< view::colored > m_colored;
        bool m_shutdown;
        bool _is_stdin;
        bool m_show;
        comma::io::istream m_istream;
        std::ostream* m_pass_through;
        boost::scoped_ptr< boost::thread > m_thread;
        mutable boost::mutex m_mutex;
        boost::optional< Eigen::Vector3d > m_point;
        bool updated_; // quick and dirty, terrible
        boost::optional< Eigen::Vector3d > m_orientation;
        comma::uint32 id_; // todo: quick and dirty; replace m_point, m_orientation, etc with PointWithId point_;
        color_t m_color;
        std::string m_label;
        Eigen::Vector3d m_translation;
        Eigen::Vector3d m_offset;
        QQuaternion m_quaternion;
};

} } } // namespace snark { namespace graphics { namespace view {
