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
        struct offset_format { enum values { nested = 0, flat = 1, csv = 2 }; };

        static tree make( std::istream& is, const comma::xpath& path=comma::xpath(), offset_format::values format = offset_format::nested ) { tree t; return make( is, path, format, t );}

        static tree make( const std::string& filename, const comma::xpath& path=comma::xpath(), offset_format::values format = offset_format::nested  ) { tree t; return make( filename, path, format, t ); }

        const boost::property_tree::ptree& operator()() const { return _tree; }

        std::vector< position > operator()( const comma::xpath& path, const std::string& name = "frame" ) const;
        
    private:
        boost::property_tree::ptree _tree;
        tree() = default;
        static tree& make( std::istream& is, const comma::xpath& path, offset_format::values format, tree& t );
        static tree& make( const std::string& filename, const comma::xpath& path, offset_format::values format, tree& t );
};

} } // namespace snark { namespace frames {
