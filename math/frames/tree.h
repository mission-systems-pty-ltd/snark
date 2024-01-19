// Copyright (c) 2023 Vsevolod Vlaskine
// All rights reserved.

#pragma once

#include <iostream>
#include <string>
#include <comma/name_value/ptree.h>
#include "../pose.h"

namespace snark { namespace frames {

// todo
// - make
//   - pass field name
//   - traverse tree
//   - handle file
//   - handle path
//   - populate offset field
//       - from nested
//       - from flat
//       - from csv
//   - output offsets
//       - from nested
//       - from flat
//       - from csv
// - methods
//   - frame relative to another frame
//   ? list frames (whatever it means)
class tree
{
    public:
        struct frame_format
        { 
            enum values { nested = 0, flat = 1, csv = 2 };
            static values from_string( const std::string& s );
        };

        tree() = default;
        static tree make( std::istream& is, const comma::xpath& path=comma::xpath(), frame_format::values format = frame_format::nested ) { tree t; return _make( is, path, format, t );}
        static tree make( std::istream& is, frame_format::values format ) { tree t; return _make( is, comma::xpath(), format, t ); }
        static tree make( const std::string& filename, const comma::xpath& path=comma::xpath(), frame_format::values format = frame_format::nested  ) { tree t; return _make( filename, path, format, t ); }
        static tree make( const std::string& filename, frame_format::values format ) { tree t; return _make( filename, comma::xpath(), format, t ); }
        std::vector< pose > operator()( const comma::xpath& path, const std::string& name = "frame" ) const;
        const boost::property_tree::ptree& operator()() const { return _tree; }
        bool empty() const { return _tree.empty(); }
        
    private:
        frame_format::values _format{frame_format::nested};
        boost::property_tree::ptree _tree;
        static tree& _make( std::istream& is, const comma::xpath& path, frame_format::values format, tree& t );
        static tree& _make( const std::string& filename, const comma::xpath& path, frame_format::values format, tree& t );
};

} } // namespace snark { namespace frames {
