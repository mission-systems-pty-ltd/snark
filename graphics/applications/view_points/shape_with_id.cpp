// Copyright (c) 2011 The University of Sydney

/// @author Vsevolod Vlaskine, Cedric Wohlleber

#include "shape_with_id.h"

namespace snark { namespace graphics { namespace view {

void shape_traits< snark::math::closed_interval< double, 3 > >::update( shape_reader_base& reader, const ShapeWithId< snark::math::closed_interval< double, 3 > >& s, const Eigen::Vector3d& offset )
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

void shape_traits< std::pair< Eigen::Vector3d, Eigen::Vector3d > >::update( shape_reader_base& reader, const ShapeWithId< std::pair< Eigen::Vector3d, Eigen::Vector3d > >& s, const Eigen::Vector3d& offset )
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

void shape_traits< axis >::update( shape_reader_base& reader, const ShapeWithId< axis >& s, const Eigen::Vector3d& offset )
{
    color_t color = s.color;
    const Eigen::Vector3d& pos = s.shape.position - offset;
    const Eigen::Matrix3d& ori = rotation_matrix::rotation( s.shape.orientation );
    Eigen::Vector3d una{ reader.length, 0, 0 };
    Eigen::Vector3d p = pos + ori * una;
    if( !reader.has_color ) { color = COLOR_RED; }
    reader.add_vertex( vertex_t( pos, color ), s.block );
    reader.add_vertex( vertex_t( p, color ), s.block );
    // todo! there is inconsistency in how labels are treated for axis shape and other shapes:
    //       for other shapes, label offset is subtracted in shape reader, thus here as a
    //       workaround, axis labels have no offset appied while the axis vertices do
    //       have offset applied, hence label_t( p + offset, ... ); this is inconsistent
    //       and error-prone; make it more consistent: apply offset to everything at one place
    reader.add_label( label_t( p + offset, color, reader.axis_labels()[0] ), s.block );
    Eigen::Matrix3d y_r = rotation_matrix::rotation( Eigen::Vector3d( 0, 0, M_PI/2 ) );
    p = pos + ori * y_r * una;
    if( !reader.has_color ) { color = COLOR_GREEN; }
    reader.add_vertex( vertex_t( pos, color ), s.block );
    reader.add_vertex( vertex_t( p, color ), s.block );
    reader.add_label( label_t( p + offset, color, reader.axis_labels()[1] ), s.block );
    Eigen::Matrix3d z_r = rotation_matrix::rotation( Eigen::Vector3d( 0, -M_PI / 2, 0 ) );
    p = pos + ori * z_r * una;
    if( !reader.has_color ) { color = COLOR_BLUE; }
    reader.add_vertex( vertex_t( pos, color ), s.block );
    reader.add_vertex( vertex_t( p, color ), s.block );
    reader.add_label( label_t( p + offset, color, reader.axis_labels()[2] ), s.block );
    reader.extent_hull( pos.cast< float >() );
    reader.extent_hull( p.cast< float >() );
}

} } } // namespace snark { namespace graphics { namespace view {
