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
        static tree make( std::istream& is, const comma::xpath& path=comma::xpath(), bool offset_as_csv = false ) { tree t; return make( is, path, offset_as_csv, t );}

        static tree make( const std::string& filename, const comma::xpath& path=comma::xpath(), bool offset_as_csv = false  ) { tree t; return make( filename, path, offset_as_csv, t ); }

        const boost::property_tree::ptree& operator()() const { return _tree; }
        
    private:
        boost::property_tree::ptree _tree;
        tree() = default;
        static tree& make( std::istream& is, const comma::xpath& path, bool offset_as_csv, tree& t );
        static tree& make( const std::string& filename, const comma::xpath& path, bool offset_as_csv, tree& t );
};

} } // namespace snark { namespace frames {
