// Copyright (c) 2011 The University of Sydney

/// @author Vsevolod Vlaskine

#pragma once

#include <string>
#include <unordered_map>
#include <boost/optional.hpp>
#include "types.h"
#ifndef Q_MOC_RUN
#include "../../../render/colour_map.h"
#include "point_with_id.h"
#endif
#include <comma/base/types.h>

namespace snark { namespace graphics { namespace view {

struct colored
{
    virtual color_t color( const Eigen::Vector3d& point
                         , comma::uint32 id
                         , double scalar
                         , const color_t& c ) const = 0;
    virtual ~colored() {}
    virtual boost::optional< std::pair< double, double > > extents() const { return boost::none; }
};

class fixed : public colored
{
    public:
        fixed( const std::string& name );
        virtual color_t color( const Eigen::Vector3d& point
                             , comma::uint32 id
                             , double scalar
                             , const color_t& c ) const;

    private:
        color_t color_;
};

struct by_height : public colored // todo: refactor and merge with byscalar
{
    by_height( double from
             , double to
             , const color_t& from_color = color_t( 255, 0, 0 )
             , const color_t& to_color = color_t( 0, 0, 255 )
             , bool cyclic = false
             , bool linear = true
             , bool sharp = false );

    double from, to, sum, diff, middle;
    color_t from_color, to_color, average_color;
    bool cyclic, linear, sharp;
    boost::optional< std::pair< double, double > > extents() const { return std::make_pair( from, to ); }

    virtual color_t color( const Eigen::Vector3d& point
                         , comma::uint32 id
                         , double scalar
                         , const color_t& c ) const;
};

class by_scalar : public colored
{
    public:
        by_scalar( double from
                 , double to
                 , const color_t& from_color
                 , const color_t& to_color
                 , bool alpha_by_scalar
                 , bool cyclic = false );

        by_scalar( double from
                 , double to
                 , const snark::render::colour_map::values& map
                 , bool alpha_by_scalar
                 , bool cyclic = false );

        virtual color_t color( const Eigen::Vector3d& point
                             , comma::uint32 id
                             , double scalar
                             , const color_t& c ) const;

        boost::optional< std::pair< double, double > > extents() const { return std::make_pair( from, to ); }

    protected:
        double from, to, diff;
        boost::optional< snark::render::colour_map::values > map;
        color_t from_color;
        color_t to_color;
        bool _alpha_by_scalar;
        bool _cyclic;
};

class by_id : public colored
{
    public:
        by_id( const color_t& backgroundcolor );

        by_id( const color_t& backgroundcolor
             , double from
             , double to );

        virtual color_t color( const Eigen::Vector3d& point
                             , comma::uint32 id
                             , double scalar
                             , const color_t& c ) const;

    private:
        const color_t background_;
        bool has_scalar_;
        double from_;
        double diff_;
};

class by_id_color_map : public colored
{
    public:
        by_id_color_map( const std::unordered_map< comma::uint32, color_t >& colors, color_t not_found, bool cyclic );

        virtual color_t color( const Eigen::Vector3d& point
                             , comma::uint32 id
                             , double scalar
                             , const color_t& c ) const;

    private:
        std::unordered_map< comma::uint32, color_t > colors_;
        color_t not_found_;
        bool cyclic_;
};

struct by_rgb : public colored
{
    virtual color_t color( const Eigen::Vector3d& point
                         , comma::uint32 id
                         , double scalar
                         , const color_t& c ) const;
};

colored* color_from_string( const std::string& s, const std::string& fields, const color_t& backgroundcolor );

} } } // namespace snark { namespace graphics { namespace view {
