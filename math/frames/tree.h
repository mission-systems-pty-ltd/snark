// Copyright (c) 2023 Vsevolod Vlaskine
// All rights reserved.

#pragma once

#include <iostream>
#include <string>
#include <comma/name_value/ptree.h>
#include "../position.h"

namespace snark { namespace frames {

class tree // todo? template on payload?
{
    public:
        template < typename Offset >
        struct properties
        {
            std::string file;
            std::string path;
            Offset offset;
        };

        template < typename Offset >
        static tree make( std::istream& is, const comma::xpath& path=comma::xpath() ) { tree t; return make< Offset >( is, path, t );}

        template < typename Offset >
        static tree make( const std::string& filename, const comma::xpath& path=comma::xpath() ) { tree t; return make< Offset >( filename, path, t ); }

        const boost::property_tree::ptree& operator()() const { return _tree; }
        
    private:
        boost::property_tree::ptree _tree;
        tree() = default;
        template < typename Offset >
        static tree& make( std::istream& is, const comma::xpath& path, tree& t );
        template < typename Offset >
        static tree& make( const std::string& filename, const comma::xpath& path, tree& t );
};

} } // namespace snark { namespace frames {
