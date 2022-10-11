// Copyright (c) 2018 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#pragma once

#include <comma/application/command_line_options.h>
#include "../../../math/roll_pitch_yaw.h"
#include "../../../math/traits.h"
#include "../../../visiting/eigen.h"
#include "../../../visiting/traits.h"

namespace snark { namespace points_calc {

struct pose
{
    Eigen::Vector3d coordinates;
    snark::roll_pitch_yaw orientation;
    pose( const Eigen::Vector3d& coordinates = Eigen::Vector3d::Zero(), const snark::roll_pitch_yaw& orientation = snark::roll_pitch_yaw() ): coordinates( coordinates ), orientation( orientation ) {}
};

} } // namespace snark { namespace points_calc {

namespace snark { namespace points_calc { namespace frame {

namespace integrate {

struct traits
{
    typedef snark::points_calc::pose input;
    typedef snark::points_calc::pose output;
    static std::string input_fields();
    static std::string input_format();
    static std::string output_fields();
    static std::string output_format();
    static std::string usage();
    static int run( const comma::command_line_options& options );
};

} // namespace integrate {

namespace multiply {

struct traits
{
    struct input: public snark::points_calc::pose
    { 
        double factor;
        input( const pose& p = pose(), double factor = 1. ): pose( p ), factor( factor ) {}
    };
    typedef snark::points_calc::pose output;
    static std::string input_fields();
    static std::string input_format();
    static std::string output_fields();
    static std::string output_format();
    static std::string usage();
    static int run( const comma::command_line_options& options );
};

} // namespace multiply {

} } } // namespace snark { namespace points_calc { namespace frame {

namespace comma { namespace visiting {

template <> struct traits< snark::points_calc::pose >
{
    template< typename K, typename V > static void visit( const K&, const snark::points_calc::pose& t, V& v )
    {
        v.apply( "coordinates", t.coordinates );
        v.apply( "orientation", t.orientation );
    }

    template< typename K, typename V > static void visit( const K&, snark::points_calc::pose& t, V& v )
    {
        v.apply( "coordinates", t.coordinates );
        v.apply( "orientation", t.orientation );
    }
};

template <> struct traits< snark::points_calc::frame::multiply::traits::input >
{
    template< typename K, typename V > static void visit( const K& k, const snark::points_calc::frame::multiply::traits::input& t, V& v )
    {
        comma::visiting::traits< snark::points_calc::pose >::visit( k, t, v );
        v.apply( "factor", t.factor );
    }

    template< typename K, typename V > static void visit( const K& k, snark::points_calc::frame::multiply::traits::input& t, V& v )
    {
        comma::visiting::traits< snark::points_calc::pose >::visit( k, t, v );
        v.apply( "factor", t.factor );
    }
};

} } // namespace comma { namespace visiting {
