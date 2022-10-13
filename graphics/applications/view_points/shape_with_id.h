// Copyright (c) 2011 The University of Sydney

/// @author Vsevolod Vlaskine, Cedric Wohlleber

#pragma once

#include <array>
#include <type_traits>
#include <boost/array.hpp>
#include <boost/optional.hpp>
#include <comma/base/types.h>
#include <comma/visiting/traits.h>
#include "../../../math/rotation_matrix.h"
#include "../../../graphics/block_buffer.h"
#if Qt3D_VERSION==1
#include <Qt3D/qglnamespace.h>
#include <Qt3D/qglpainter.h>
#else
#include "../../../graphics/qt5.5/qopengl/shapes.h"
#endif
#include <memory>
#include "types.h"
#include "../../../math/roll_pitch_yaw.h"
#include "../../../visiting/traits.h"
#include "traits.h"
#include "shape_reader_base.h"

namespace snark { namespace graphics { namespace view {

#if Qt3D_VERSION>=2
    
typedef snark::graphics::qopengl::shape shape_t;

struct gl_parameters
{
    gl_parameters( unsigned weight, bool fill ) : weight( weight ), fill( fill ) {}
    unsigned weight;
    bool fill;
};

#endif

namespace detail {

template < typename S > struct shape_traits { static S zero() { return S(); } };
template <> struct shape_traits< Eigen::Vector3d > { static Eigen::Vector3d zero() { return Eigen::Vector3d::Zero(); } };
template <> struct shape_traits< std::pair< Eigen::Vector3d, Eigen::Vector3d > > { static std::pair< Eigen::Vector3d, Eigen::Vector3d > zero() { return std::make_pair( Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero() ); } };

} // namespace detail {

template < class S >
struct ShapeWithId // quick and dirty
{
    typedef S Shape;
    ShapeWithId() : shape( detail::shape_traits< S >::zero() ), id( 0 ), block( 0 ) {}
    ShapeWithId( const S& shape ) : shape( shape ), id( 0 ), block( 0 ), scalar( 0 ), fill( false ) {}
    S shape;
    comma::uint32 id;
    comma::uint32 block;
    color_t color;
    std::string label;
    double scalar;
    bool fill; // todo: just a placeholder for now, plug in if possible or tear down
};

struct how_t { struct points; struct lines; struct loop; }; // quick and dirty; for points only

template < class S, typename How = how_t::points >
struct shape_traits {}; // quick and dirty

template <>
struct shape_traits< snark::math::closed_interval< double, 3 > >
{
    static const unsigned int size = 24; // quick and dirty, really wasteful on memory; todo! specialised shape class for boxes
        
#if Qt3D_VERSION>=2
    static shape_t* make_shape( const gl_parameters& gl ) { return new snark::graphics::qopengl::shapes::lines( gl.weight ); }
#endif

    static void update( shape_reader_base& reader, const ShapeWithId< snark::math::closed_interval< double, 3 > >& s, const Eigen::Vector3d& offset )
    {
        Eigen::Vector3f min = ( s.shape.min() - offset ).cast< float >();
        Eigen::Vector3f max = ( s.shape.max() - offset ).cast< float >();

        #if Qt3D_VERSION==1
        reader.add_vertex( vertex_t( QVector3D( min.x(), min.y(), min.z() ), s.color ), s.block );
        reader.add_vertex( vertex_t( QVector3D( min.x(), min.y(), max.z() ), s.color ), s.block );
        reader.add_vertex( vertex_t( QVector3D( min.x(), max.y(), max.z() ), s.color ), s.block );
        reader.add_vertex( vertex_t( QVector3D( min.x(), max.y(), min.z() ), s.color ), s.block );
        reader.add_vertex( vertex_t( QVector3D( max.x(), min.y(), min.z() ), s.color ), s.block );
        reader.add_vertex( vertex_t( QVector3D( max.x(), min.y(), max.z() ), s.color ), s.block );
        reader.add_vertex( vertex_t( QVector3D( max.x(), max.y(), max.z() ), s.color ), s.block );
        reader.add_vertex( vertex_t( QVector3D( max.x(), max.y(), min.z() ), s.color ), s.block );
        #else
        reader.add_vertex( vertex_t( Eigen::Vector3f( min.x(), min.y(), min.z() ), s.color ), s.block );
        reader.add_vertex( vertex_t( Eigen::Vector3f( max.x(), min.y(), min.z() ), s.color ), s.block );
        reader.add_vertex( vertex_t( Eigen::Vector3f( max.x(), min.y(), min.z() ), s.color ), s.block );
        reader.add_vertex( vertex_t( Eigen::Vector3f( max.x(), max.y(), min.z() ), s.color ), s.block );
        reader.add_vertex( vertex_t( Eigen::Vector3f( max.x(), max.y(), min.z() ), s.color ), s.block );
        reader.add_vertex( vertex_t( Eigen::Vector3f( min.x(), max.y(), min.z() ), s.color ), s.block );
        reader.add_vertex( vertex_t( Eigen::Vector3f( min.x(), max.y(), min.z() ), s.color ), s.block );
        reader.add_vertex( vertex_t( Eigen::Vector3f( min.x(), min.y(), min.z() ), s.color ), s.block );
        
        reader.add_vertex( vertex_t( Eigen::Vector3f( min.x(), min.y(), max.z() ), s.color ), s.block );
        reader.add_vertex( vertex_t( Eigen::Vector3f( max.x(), min.y(), max.z() ), s.color ), s.block );
        reader.add_vertex( vertex_t( Eigen::Vector3f( max.x(), min.y(), max.z() ), s.color ), s.block );
        reader.add_vertex( vertex_t( Eigen::Vector3f( max.x(), max.y(), max.z() ), s.color ), s.block );
        reader.add_vertex( vertex_t( Eigen::Vector3f( max.x(), max.y(), max.z() ), s.color ), s.block );
        reader.add_vertex( vertex_t( Eigen::Vector3f( min.x(), max.y(), max.z() ), s.color ), s.block );
        reader.add_vertex( vertex_t( Eigen::Vector3f( min.x(), max.y(), max.z() ), s.color ), s.block );
        reader.add_vertex( vertex_t( Eigen::Vector3f( min.x(), min.y(), max.z() ), s.color ), s.block );
        
        reader.add_vertex( vertex_t( Eigen::Vector3f( min.x(), min.y(), min.z() ), s.color ), s.block );
        reader.add_vertex( vertex_t( Eigen::Vector3f( min.x(), min.y(), max.z() ), s.color ), s.block );
        reader.add_vertex( vertex_t( Eigen::Vector3f( min.x(), max.y(), min.z() ), s.color ), s.block );
        reader.add_vertex( vertex_t( Eigen::Vector3f( min.x(), max.y(), max.z() ), s.color ), s.block );
        reader.add_vertex( vertex_t( Eigen::Vector3f( max.x(), min.y(), min.z() ), s.color ), s.block );
        reader.add_vertex( vertex_t( Eigen::Vector3f( max.x(), min.y(), max.z() ), s.color ), s.block );
        reader.add_vertex( vertex_t( Eigen::Vector3f( max.x(), max.y(), min.z() ), s.color ), s.block );
        reader.add_vertex( vertex_t( Eigen::Vector3f( max.x(), max.y(), max.z() ), s.color ), s.block );
        #endif

        reader.extent_hull (snark::math::closed_interval< float, 3 >( min, max ) );
    }

    #if Qt3D_VERSION==1
    static void draw( QGLPainter* painter, unsigned int size, bool fill )
    {
        const boost::array< unsigned short, 8  > baseIndices = { { 0, 4, 1, 5, 2, 6, 3, 7 } };
        for( unsigned int i = 0; i < size; i += 8 )
        {
            painter->draw( QGL::LineLoop, 4, i );
            painter->draw( QGL::LineLoop, 4, i + 4 );
            boost::array< unsigned short, 8  > lineIndices = baseIndices;
            for( unsigned int k = 0; k < lineIndices.size(); ++k ) { lineIndices[k] += i; }
            painter->draw( QGL::Lines, &lineIndices[0], 8 );
        }
    }
    #endif

    static const Eigen::Vector3d& some_point( const snark::math::closed_interval< double, 3 >& extents ) { return extents.min(); }

    static Eigen::Vector3d center( const snark::math::closed_interval< double, 3 >& extents ) { return ( extents.min() + extents.max() ) / 2; }
};

template <>
struct shape_traits< std::pair< Eigen::Vector3d, Eigen::Vector3d > >
{
    static const unsigned int size = 2;
    
#if Qt3D_VERSION>=2
    static shape_t* make_shape( const gl_parameters& gl ) { return new snark::graphics::qopengl::shapes::lines( gl.weight ); }
#endif

    static void update( shape_reader_base& reader, const ShapeWithId< std::pair< Eigen::Vector3d, Eigen::Vector3d > >& s, const Eigen::Vector3d& offset )
    {
        Eigen::Vector3f first = ( s.shape.first - offset ).cast< float >();
        Eigen::Vector3f second = ( s.shape.second - offset ).cast< float >();
        //if(    comma::math::equal( first.x(), second.x() )
        //    && comma::math::equal( first.y(), second.y() )
        //    && comma::math::equal( first.z(), second.z() ) ) { return; } // todo: draw a point instead?
        reader.add_vertex( vertex_t( first, s.color ), s.block );
        reader.add_vertex( vertex_t( second, s.color ), s.block );

        reader.extent_hull(first);
        reader.extent_hull(second);
    }

    #if Qt3D_VERSION==1
    static void draw( QGLPainter* painter, unsigned int size, bool fill ) { painter->draw( QGL::Lines, size ); }
    #endif

    static const Eigen::Vector3d& some_point( const std::pair< Eigen::Vector3d, Eigen::Vector3d >& line ) { return line.first; }

    static Eigen::Vector3d center( const std::pair< Eigen::Vector3d, Eigen::Vector3d >& line ) { return ( line.first + line.second ) / 2; }
};

template < std::size_t Size >
struct loop { boost::array< Eigen::Vector3d, Size > corners; };

template < std::size_t Size >
struct shape_traits< loop< Size > >
{
    static_assert( Size > 2, "expected size greater than 2" );
    static const unsigned int size = Size;
    
#if Qt3D_VERSION>=2
    static shape_t* make_shape( const gl_parameters& gl )
    {
        if( Size == 3 || gl.fill ) { return new snark::graphics::qopengl::shapes::triangles( gl.fill ); } else { return new snark::graphics::qopengl::shapes::line_loop( gl.weight, Size ); }
    }
#endif

    static void update(shape_reader_base& reader, const ShapeWithId< loop< Size > >& s,const Eigen::Vector3d& offset)
    {
        for( unsigned int i = 0; i < Size; ++i )
        {
            Eigen::Vector3f v = ( s.shape.corners[i] - offset ).template cast< float >();
            reader.add_vertex( vertex_t( v, s.color ), s.block );
            reader.extent_hull(v);
        }
    }

    #if Qt3D_VERSION==1
    static void draw( QGLPainter* painter, unsigned int size, bool fill )
    {
        for( unsigned int i = 0; i < size; i += Size ) { painter->draw( fill && Size == 3 ? QGL::Triangles : QGL::LineLoop, Size, i ); }
    }
    #endif
    
    static const Eigen::Vector3d& some_point( const loop< Size >& c ) { return c.corners[0]; }
    
    static Eigen::Vector3d center( const loop< Size >& c ) // quick and dirty
    { 
        Eigen::Vector3d m = Eigen::Vector3d::Zero();
        for( unsigned int i = 0; i < Size; ++i ) { m += c.corners[i]; }
        return m / Size;
    }
};

struct orientation // quick and dirty
{
    double roll;
    double pitch;
    double yaw;
    
    orientation(): roll( 0 ), pitch( 0 ), yaw( 0 ) {}
    orientation( double roll, double pitch, double yaw ): roll( roll ), pitch( pitch ), yaw( yaw ) {}
    operator Eigen::Vector3d() const { return Eigen::Vector3d( roll, pitch, yaw ); }
};

template < std::size_t Size >
struct Ellipse // todo: quick and dirty, make size runtime variable
{
    Eigen::Vector3d center;
    snark::graphics::view::orientation orientation;
    double major;
    double minor;
    
    Ellipse() : center( 0, 0, 0 ), orientation( 0, 0, 0 ) {}
};

template < std::size_t Size >
struct shape_traits< Ellipse< Size > >
{
    static const unsigned int size = Size;
    
#if Qt3D_VERSION>=2
    static shape_t* make_shape( const gl_parameters& gl ) { return new snark::graphics::qopengl::shapes::line_loop( gl.weight, Size ); }
#endif

    static void update( shape_reader_base& reader, const ShapeWithId< Ellipse < Size > >& s,const Eigen::Vector3d& offset )
    {
        Eigen::Vector3d c = s.shape.center - offset;
        const Eigen::Matrix3d& r = rotation_matrix::rotation( s.shape.orientation );
        static const double step = 3.14159265358979323846l * 2 / Size;
        double angle = 0;
        for( std::size_t i = 0; i < Size; ++i, angle += step ) // todo? use native opengl rotation and normals instead
        {
            Eigen::Vector3d v = r * Eigen::Vector3d( std::cos( angle ) * s.shape.major, std::sin( angle ) * s.shape.minor, 0 );
            Eigen::Vector3d p( v.x(), v.y(), v.z() );
            Eigen::Vector3f point = ( p + c ).cast< float >();
            reader.add_vertex( vertex_t( point, s.color ), s.block );
            reader.extent_hull(point);
        }
    }

    #if Qt3D_VERSION==1
    static void draw( QGLPainter* painter, unsigned int size, bool fill )
    {
        for( unsigned int i = 0; i < size; i += Size ) { painter->draw( QGL::LineLoop, Size, i ); }
    }
    #endif

    static const Eigen::Vector3d& some_point( const Ellipse< Size >& ellipse ) { return ellipse.center; }

    static Eigen::Vector3d center( const Ellipse< Size >& ellipse ) { return ellipse.center; }
};

template < std::size_t Size >
struct arc // todo: quick and dirty; generalize for ellipse; and maybe get rid of ellipse class; todo: quick and dirty, make size runtime variable
{
    Eigen::Vector3d begin;
    boost::optional< Eigen::Vector3d > middle;
    Eigen::Vector3d end;
    Eigen::Vector3d centre;
    arc() : centre( 0, 0, 0 ) {}
};

template < std::size_t Size >
struct shape_traits< arc< Size > >
{
    static const unsigned int size = Size;

    static_assert( Size % 2 == 0, "expected even size" ); // quick and dirty: for simplicity support only only even sizes
    
#if Qt3D_VERSION>=2
    static shape_t* make_shape( const gl_parameters& gl ) { return new snark::graphics::qopengl::shapes::line_strip( gl.weight, Size ); }
#endif

    static void update(shape_reader_base& reader, const ShapeWithId<arc< Size >>& s,const Eigen::Vector3d& offset)
    {
        const arc< Size >& a=s.shape;
        if( ( a.begin - a.end ).squaredNorm() < ( 0.001 * 0.001 ) ) // real quick and dirty: if begin and end coincide
        {
            Eigen::Vector3d v = a.begin;
            Eigen::Vector3d step = ( a.end - a.begin ) / Size;
            for( std::size_t i = 0; i < Size; ++i, v += step ) // just to make the gl buffer sizes right
            {
                Eigen::Vector3f point = ( v - offset ).cast< float >();
                reader.add_vertex( vertex_t( point, s.color ), s.block );
                reader.extent_hull(point);
            }
            return;
        }
        // get centre
        Eigen::Vector3d centre;
        Eigen::Vector3d normal;
        if( a.middle )
        {
            Eigen::Vector3d begin_middle = a.begin - *a.middle;
            Eigen::Vector3d middle_end = *a.middle - a.end;
            Eigen::Vector3d end_begin = a.end - a.begin;
            normal = begin_middle.cross( middle_end );
            double k = normal.squaredNorm() * 2;
            double alpha = -middle_end.squaredNorm() * begin_middle.dot( end_begin ) / k;
            double beta = -end_begin.squaredNorm() * middle_end.dot( begin_middle ) / k;
            double gamma = -begin_middle.squaredNorm() * end_begin.dot( middle_end ) / k;
            centre = a.begin * alpha + *a.middle * beta + a.end * gamma;
        }
        else
        {
            centre = a.centre;
            normal = ( a.begin - a.centre ).cross( a.end - a.centre );
        }
        normal.normalize();
        // get rotation from begin to end with Size steps
        Eigen::Vector3d b = a.begin - centre;
        Eigen::Vector3d e = a.end - centre;
        Eigen::AngleAxis< double > aa( Eigen::Quaternion< double >::FromTwoVectors( b, e ) );
        Eigen::AngleAxis< double > nn( aa.angle() / ( Size - 1 ), normal );
        const Eigen::Matrix3d& r = nn.toRotationMatrix();
        Eigen::Vector3d c = centre - offset;
        Eigen::Vector3d v = b;
        for( std::size_t i = 0; i < Size; ++i, v = r * v ) // todo: use native opengl rotation and normals instead
        {
            Eigen::Vector3f point = ( v + c ).cast< float >();
            reader.add_vertex( vertex_t( point, s.color ), s.block );
            reader.extent_hull(point);
        }
    }

    #if Qt3D_VERSION==1
    static void draw( QGLPainter* painter, unsigned int size, bool fill )
    {
        for( unsigned int i = 0; i < size; i += Size )
        {
            painter->draw( QGL::Lines, Size, i );
            painter->draw( QGL::Lines, Size - 1, i + 1 );
        }
    }
    #endif

    static const Eigen::Vector3d& some_point( const arc< Size >& a ) { return a.begin; }

    static Eigen::Vector3d center( const arc< Size >& a ) { return a.middle ? *a.middle : a.centre; } // quick and dirty
    //static Eigen::Vector3d center( const arc< Size >& a ) { return a.middle; } // quick and dirty
};

#if Qt3D_VERSION==1
template < typename How > struct draw_traits_;

template <> struct draw_traits_< how_t::points >
{
    static const QGL::DrawingMode drawing_mode = QGL::Points;

    static void draw( QGLPainter* painter, unsigned int size, bool fill )
    {
        painter->draw( QGL::Points, size );
    }
};

template <> struct draw_traits_< how_t::loop >
{
    static const QGL::DrawingMode drawing_mode = QGL::DrawingMode();

    static void draw( QGLPainter* painter, unsigned int size, bool fill )
    {
        painter->draw( QGL::LineLoop, size );
    }
};

template <> struct draw_traits_< how_t::lines >
{
    static const QGL::DrawingMode drawing_mode = QGL::DrawingMode();

    static void draw( QGLPainter* painter, unsigned int size, bool fill )
    {
        painter->draw( QGL::Lines, size );
        if( size > 1 ) { painter->draw( QGL::Lines, size - 1, 1 ); }
    }
};
#elif Qt3D_VERSION>=2

template<typename T> struct how_traits {};

template<> struct how_traits< how_t::points >
{
    static shape_t* make_shape( const gl_parameters& gl ) { return new snark::graphics::qopengl::shapes::point( gl.weight ); }
};

template<> struct how_traits< how_t::loop >
{
    static shape_t* make_shape( const gl_parameters& gl ) { return new snark::graphics::qopengl::shapes::line_loop( gl.weight ); }
};

template <> struct how_traits< how_t::lines >
{
    static shape_t* make_shape( const gl_parameters& gl ) { return new snark::graphics::qopengl::shapes::line_strip( gl.weight ); }
};

#endif

template< typename How >
struct shape_traits< Eigen::Vector3d, How >
{
    #if Qt3D_VERSION==1
    static const QGL::DrawingMode drawingMode = draw_traits_< How >::drawing_mode;
    #endif
    static const unsigned int size = 1;

    
#if Qt3D_VERSION>=2
    static shape_t* make_shape( const gl_parameters& gl ) { return how_traits< How >::make_shape( gl ); }
#endif

    static void update( shape_reader_base& reader, const ShapeWithId< Eigen::Vector3d >& s,const Eigen::Vector3d& offset )
    {
        Eigen::Vector3d point = s.shape - offset;
        reader.add_vertex( vertex_t( point, s.color), s.block );
        reader.extent_hull(point.cast<float>());
    }

    #if Qt3D_VERSION==1
    static void draw( QGLPainter* painter, unsigned int size, bool fill ) { draw_traits_< How >::draw( painter, size, fill ); }
    #endif

    static const Eigen::Vector3d& some_point( const Eigen::Vector3d& point ) { return point; }

    static const Eigen::Vector3d& center( const Eigen::Vector3d& point ) { return point; }
};

struct axis
{
    Eigen::Vector3d position;
    snark::roll_pitch_yaw orientation;
    axis() : position( 0, 0, 0 ), orientation( 0, 0, 0 ) {}
};


template<>
struct shape_traits< axis >
{
    static const unsigned int size = 6;
#if Qt3D_VERSION>=2
    static shape_t* make_shape( const gl_parameters& gl ) { return new snark::graphics::qopengl::shapes::lines( gl.weight ); }
#endif

    static void update(shape_reader_base& reader, const ShapeWithId<axis>& s,const Eigen::Vector3d& offset)
    {
        std::vector<std::string> v=comma::split(reader.labels,':');
        Eigen::Vector3d pos=s.shape.position - offset;
        Eigen::Matrix3d ori=rotation_matrix::rotation( s.shape.orientation );
        Eigen::Vector3d una(reader.length,0,0);
        Eigen::Vector3d p=pos + ori*una;
        color_t color=s.color;
        if(!reader.has_color) { color=COLOR_RED; }
        reader.add_vertex( vertex_t(pos,color),s.block);
        reader.add_vertex( vertex_t(p,color),s.block);
        if(v.size()>0) { reader.add_label(label_t(p,color,v[0]),s.block); }

        Eigen::Matrix3d y_r=rotation_matrix::rotation(Eigen::Vector3d(0,0,M_PI/2));
        p=pos+ori*y_r*una;
        if(!reader.has_color) { color=COLOR_GREEN; }
        reader.add_vertex( vertex_t(pos,color),s.block);
        reader.add_vertex( vertex_t(p,color),s.block);
        if(v.size()>1) { reader.add_label(label_t(p,color,v[1]),s.block); }

        Eigen::Matrix3d z_r=rotation_matrix::rotation(Eigen::Vector3d(0,-M_PI/2,0));
        p=pos+ori*z_r*una;
        if(!reader.has_color) { color=COLOR_BLUE; }
        reader.add_vertex( vertex_t(pos,color),s.block);
        reader.add_vertex( vertex_t(p,color),s.block);
        if(v.size()>2) { reader.add_label(label_t(p,color,v[2]),s.block); }

        reader.extent_hull(pos.cast<float>());
        reader.extent_hull( p.cast<float>() );
    }

    #if Qt3D_VERSION==1
    static void draw( QGLPainter* painter, unsigned int size, bool fill ) { painter->draw( QGL::Lines, size ); }
    #endif

    static const Eigen::Vector3d& some_point( const axis& axis ) { return axis.position; }

    static Eigen::Vector3d center( const axis& axis ) { return axis.position; }
};


} } } // namespace snark { namespace graphics { namespace view {

namespace comma { namespace visiting {

template < typename S > struct traits< snark::graphics::view::ShapeWithId< S > >
{
    template < typename Key, class Visitor >
    static void visit( Key, snark::graphics::view::ShapeWithId< S >& p, Visitor& v )
    {
        v.apply( "shape", p.shape );
        v.apply( "id", p.id );
        v.apply( "block", p.block );
        v.apply( "colour", p.color );
        v.apply( "label", p.label );
        v.apply( "scalar", p.scalar );
        v.apply( "fill", p.fill );
    }

    template < typename Key, class Visitor >
    static void visit( Key, const snark::graphics::view::ShapeWithId< S >& p, Visitor& v )
    {
        v.apply( "shape", p.shape );
        v.apply( "id", p.id );
        v.apply( "block", p.block );
        v.apply( "colour", p.color );
        v.apply( "label", p.label );
        v.apply( "scalar", p.scalar );
        v.apply( "fill", p.fill );
    }
};

template <> struct traits< snark::graphics::view::orientation >
{
    template < typename Key, class Visitor >
    static void visit( Key, snark::graphics::view::orientation& p, Visitor& v )
    {
        v.apply( "roll", p.roll );
        v.apply( "pitch", p.pitch );
        v.apply( "yaw", p.yaw );
    }
    
    template < typename Key, class Visitor >
    static void visit( Key, const snark::graphics::view::orientation& p, Visitor& v )
    {
        v.apply( "roll", p.roll );
        v.apply( "pitch", p.pitch );
        v.apply( "yaw", p.yaw );
    }
};

template < std::size_t Size > struct traits< snark::graphics::view::Ellipse< Size > >
{
    template < typename Key, class Visitor >
    static void visit( Key, snark::graphics::view::Ellipse< Size >& p, Visitor& v )
    {
        v.apply( "center", p.center );
        v.apply( "orientation", p.orientation );
        v.apply( "major", p.major );
        v.apply( "minor", p.minor );
    }

    template < typename Key, class Visitor >
    static void visit( Key, const snark::graphics::view::Ellipse< Size >& p, Visitor& v )
    {
        v.apply( "center", p.center );
        v.apply( "orientation", p.orientation );
        v.apply( "major", p.major );
        v.apply( "minor", p.minor );
    }
};

template < std::size_t Size > struct traits< snark::graphics::view::arc< Size > >
{
    template < typename Key, class Visitor >
    static void visit( Key, snark::graphics::view::arc< Size >& p, Visitor& v )
    {
        v.apply( "begin", p.begin );
        if( p.middle ) { v.apply( "middle", *p.middle ); } // quick and dirty
        v.apply( "end", p.end );
        v.apply( "centre", p.centre );
    }

    template < typename Key, class Visitor >
    static void visit( Key, const snark::graphics::view::arc< Size >& p, Visitor& v )
    {
        v.apply( "begin", p.begin );
        if( p.middle ) { v.apply( "middle", *p.middle ); } // quick and dirty
        v.apply( "end", p.end );
        v.apply( "centre", p.centre );
    }
};

template < typename T > struct traits< snark::math::closed_interval< T, 3 > >
{
    template < typename Key, class Visitor >
    static void visit( Key, snark::math::closed_interval< T, 3 >& p, Visitor& v )
    {
        Eigen::Matrix< T, 3, 1 > min;
        Eigen::Matrix< T, 3, 1 > max;
        v.apply( "min", min );
        v.apply( "max", max );
        p = snark::math::closed_interval< T, 3 >( min, max );
    }

    template < typename Key, class Visitor >
    static void visit( Key, const snark::math::closed_interval< T, 3 >& p, Visitor& v )
    {
        v.apply( "min", p.min() );
        v.apply( "max", p.max() );
    }
};

template < std::size_t Size > struct traits< snark::graphics::view::loop< Size > >
{
    template < typename Key, class Visitor >
    static void visit( Key, snark::graphics::view::loop< Size >& p, Visitor& v )
    {
        v.apply( "corners", p.corners );
    }

    template < typename Key, class Visitor >
    static void visit( Key, const snark::graphics::view::loop< Size >& p, Visitor& v )
    {
        v.apply( "corners", p.corners );
    }
};

template<>
struct traits< snark::graphics::view::axis >
{
    template < typename Key, class Visitor >
    static void visit( Key, snark::graphics::view::axis& p, Visitor& v )
    {
        v.apply( "position", p.position );
        v.apply( "orientation", p.orientation );
    }

    template < typename Key, class Visitor >
    static void visit( Key, const snark::graphics::view::axis& p, Visitor& v )
    {
        v.apply( "position", p.position );
        v.apply( "orientation", p.orientation );
    }
};

} } // namespace comma { namespace visiting {
