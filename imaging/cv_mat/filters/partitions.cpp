// Copyright (c) 2019 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include <algorithm>
#include <array>
#include <deque>
#include <map>
#include <memory>
#include <sstream>
#include <unordered_map>
#include <unordered_set>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/functional/hash.hpp>
#include <boost/lexical_cast.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <tbb/parallel_for.h>
#include <comma/base/exception.h>
#include <comma/base/types.h>
#include <comma/string/split.h>
#include "partitions.h"

namespace snark { namespace cv_mat { namespace filters { namespace partitions {

template < typename S, typename T = S >
struct pair_hash {
    std::size_t operator()(const std::pair< T, T > &pair) const {
        std::size_t seed = 0;
        boost::hash_combine(seed, pair.first);
        boost::hash_combine(seed, pair.second);
        return seed;
    }
};

template < typename H >
partition< H >::partition(boost::optional< cv::Scalar > do_not_visit_value, comma::int32 none, bool merge, bool keep_id,
                          comma::int32 start_from, unsigned int min_partition_size, unsigned int degrees)
        : do_not_visit_value_(do_not_visit_value), none_(none), merge_(merge), keep_id_(keep_id),
          start_from_(start_from), id_(start_from), min_partition_size_(min_partition_size), degrees_(degrees) {
}

template < typename T >
static void assign_(std::array< unsigned char, 4 > &a, T v) { std::memcpy(&a[0], &v, sizeof(v)); }

template < typename H >
std::pair< H, cv::Mat > partition< H >::operator()(std::pair< H, cv::Mat > m) {
    if (true) {
        if (m.second.channels() > 1) {
            COMMA_THROW(comma::exception, "partition: currently support only single-channel images; got "
                    << m.second.channels() << " channels");
        }
        if (merge_ && m.second.type() != CV_32SC1) {
            COMMA_THROW(comma::exception, "partition: asked to merge, expected image of type i (CV_32SC1), got: "
                    << m.second.type());
        }
        cv::Mat partitions(m.second.rows, m.second.cols, CV_32SC1, cv::Scalar(none_));
        comma::uint32 id = keep_id_ ? id_ : start_from_;
        comma::uint32 start_from = id;
        std::array< unsigned char, 4 > do_not_visit;
        if (do_not_visit_value_) {
            switch (m.second.type()) // pain! quick and dirty
            {
                case CV_8SC1:
                case CV_8UC1:
                    assign_< unsigned char >(do_not_visit, (*do_not_visit_value_)[0]);
                    break;
                case CV_16SC1:
                case CV_16UC1:
                    assign_< comma::uint16 >(do_not_visit, (*do_not_visit_value_)[0]);
                    break;
                case CV_32SC1:
                    assign_< comma::int32 >(do_not_visit, (*do_not_visit_value_)[0]);
                    break;
                case CV_32FC1:
                    static_assert( sizeof( float ) == 4, "expected float of size 4" );
                    assign_< float >(do_not_visit, (*do_not_visit_value_)[0]);
                    break;
                default:
                    break; // never here
            }
        }
        auto neighbour_id = [&](int i, int j, int io, int jo) -> int {
            int ni = i + io;
            if (ni < 0 || ni >= m.second.rows) { return none_; }
            int nj = j + jo;
            if (nj < 0 || nj >= m.second.cols) { return none_; }
            int nid = partitions.template at< comma::int32 >(ni, nj);
            if (nid == none_) { return none_; }
            return std::memcmp(m.second.ptr(i, j), m.second.ptr(ni, nj), m.second.elemSize()) == 0 ? nid : none_;
        };
        auto set_pixel = [&](int i, int j) {
            if (do_not_visit_value_ &&
                std::memcmp(m.second.ptr(i, j), &do_not_visit[0], m.second.elemSize()) == 0) { return; }
            int nid = none_;
            if (degrees_ == 8) { nid = neighbour_id(i, j, -1, -1); }
            if (nid == none_) { nid = neighbour_id(i, j, -1, 0); }
            if (nid == none_ && degrees_ == 8) { nid = neighbour_id(i, j, -1, 1); }
            if (nid == none_) { nid = neighbour_id(i, j, 0, -1); }
            if (nid == none_) { partitions.template at< comma::int32 >(i, j) = id++; }
            else { partitions.template at< comma::int32 >(i, j) = nid; }
        };
        for (int i = 0; i < m.second.rows; ++i) { for (int j = 0; j < m.second.cols; ++j) { set_pixel(i, j); }}
        std::deque< std::unordered_set< int > > equivalencies; // todo! quick and dirty, watch performance
        std::vector< int > indices(id - start_from, -1);
        auto update_equivalencies = [&](int pid, int i, int j, int io, int jo) {
            int nid = neighbour_id(i, j, io, jo);
            if (nid == none_ || nid == pid) { return; }
            int nix = nid - start_from;
            int pix = pid - start_from;
            if (indices[nix] == -1) {
                equivalencies[indices[pix]].insert(nix);
                indices[nix] = indices[pix];
            } else {
                if (indices[pix] != indices[nix]) {
                    equivalencies[indices[nix]].insert(equivalencies[indices[pix]].begin(),
                                                       equivalencies[indices[pix]].end());
                    int to_clear = indices[pix];
                    for (int k: equivalencies[indices[pix]]) { indices[k] = indices[nix]; }
                    equivalencies[to_clear].clear();
                }
            }
        };
        auto make_equivalencies = [&](int i, int j) // todo: wasteful? fill the map on the first pass?
        {
            int pid = partitions.template at< comma::int32 >(i, j);
            if (pid == none_) { return; }
            int pix = pid - start_from;
            if (indices[pix] == -1) {
                indices[pix] = equivalencies.size();
                equivalencies.push_back(std::unordered_set< int >());
                equivalencies.back().insert(pix);
            }
            update_equivalencies(pid, i, j, -1, 0);
            update_equivalencies(pid, i, j, 0, -1);
            update_equivalencies(pid, i, j, 0, 1);
            update_equivalencies(pid, i, j, 1, 0);
            if (degrees_ == 8) {
                update_equivalencies(pid, i, j, -1, -1);
                update_equivalencies(pid, i, j, -1, 1);
                update_equivalencies(pid, i, j, 1, -1);
                update_equivalencies(pid, i, j, 1, 1);
            }
        };
        for (int i = 0; i < m.second.rows; ++i) { for (int j = 0; j < m.second.cols; ++j) { make_equivalencies(i, j); }}
        std::vector< int > ids(equivalencies.size(), -1);
        id = start_from;
        for (unsigned int i = 0; i < ids.size(); ++i) { if (!equivalencies[i].empty()) { ids[i] = id++; }}
        std::vector< unsigned int > sizes(id - start_from, 0);
        for (int i = 0; i < m.second.rows; ++i) {
            for (int j = 0; j < m.second.cols; ++j) {
                auto &p = partitions.template at< comma::int32 >(i, j);
                if (p != none_) {
                    p = ids[indices[p - start_from]];
                    ++sizes[p - start_from];
                }
            }
        }
        if (min_partition_size_ > 0) // uber quick and dirty
        {
            for (int i = 0; i < m.second.rows; ++i) {
                for (int j = 0; j < m.second.cols; ++j) {
                    auto &p = partitions.template at< comma::int32 >(i, j);
                    if (p != none_ && sizes[p - start_from] < min_partition_size_) { p = none_; }
                }
            }
        }
        if (keep_id_) { id_ = id; }
        std::pair< H, cv::Mat > n;
        n.first = m.first;
        if (merge_) {
            std::vector< cv::Mat > channels(2);
            channels[0] = m.second;
            channels[1] = partitions;
            cv::merge(channels, n.second);
        } else {
            n.second = partitions;
        }
        return n;
    }


    if (m.second.channels() > 1) {
        COMMA_THROW(comma::exception, "partition: currently support only single-channel images; got "
                << m.second.channels() << " channels");
    }
    if (merge_ && m.second.type() != CV_32SC1) {
        COMMA_THROW(comma::exception, "partition: asked to merge, expected image of type i (CV_32SC1), got: "
                << m.second.type());
    }
    cv::Mat partitions(m.second.rows, m.second.cols, CV_32SC1, cv::Scalar(none_));
    comma::uint32 id = keep_id_ ? id_ : start_from_;
    std::unordered_set< std::pair< int, int >, pair_hash< int > > neighbours;
    auto insert_neighbour = [&](int i, int j, int io, int jo, int none) {
        int ni = i + io;
        if (ni < 0 || ni >= m.second.rows) { return; }
        int nj = j + jo;
        if (nj < 0 || nj >= m.second.cols) { return; }
        if (partitions.template at< comma::int32 >(ni, nj) != none) { return; }
        if (do_not_visit_value_ &&
            std::memcmp(m.second.ptr(ni, nj), &(*do_not_visit_value_), m.second.elemSize()) == 0) { return; }
        if (std::memcmp(m.second.ptr(ni, nj), m.second.ptr(i, j), m.second.elemSize()) != 0) { return; }
        neighbours.insert(std::make_pair(ni, nj));
    };
    auto visit_partition_at = [&](int i, int j, int current_id, int none) -> unsigned int {
        if (partitions.template at< comma::int32 >(i, j) != none) { return 0; }
        if (do_not_visit_value_ &&
            std::memcmp(m.second.ptr(i, j), &(*do_not_visit_value_), m.second.elemSize()) == 0) { return 0; }
        unsigned int size = 0;
        neighbours.insert(std::make_pair(i, j));
        while (!neighbours.empty()) {
            int ci = neighbours.begin()->first;
            int cj = neighbours.begin()->second;
            partitions.template at< comma::int32 >(ci, cj) = current_id;
            ++size;
            neighbours.erase(neighbours.begin());
            insert_neighbour(ci, cj, -1, 0, none);
            insert_neighbour(ci, cj, 0, -1, none);
            insert_neighbour(ci, cj, 0, 1, none);
            insert_neighbour(ci, cj, 1, 0, none);
            if (degrees_ == 8) {
                insert_neighbour(ci, cj, -1, -1, none);
                insert_neighbour(ci, cj, -1, 1, none);
                insert_neighbour(ci, cj, 1, -1, none);
                insert_neighbour(ci, cj, 1, 1, none);
            }
        }
        return size;
    };
    std::vector< std::pair< int, int > > discarded; // todo? use deque?
    for (int i = 0; i < m.second.rows; ++i) {
        for (int j = 0; j < m.second.cols; ++j) {
            unsigned int size = visit_partition_at(i, j, id, none_);
            if (size == 0) { continue; }
            if (size < min_partition_size_) {
                discarded.push_back(std::make_pair(i, j));
            } // todo: quick and dirty, watch performance
            else { ++id; }
        }
    }
    for (const auto &p: discarded) {
        visit_partition_at(p.first, p.second, none_, partitions.template at< comma::int32 >(p.first, p.second));
    }
    std::pair< H, cv::Mat > n;
    n.first = m.first;
    if (merge_) {
        std::vector< cv::Mat > channels(2);
        channels[0] = m.second;
        channels[1] = partitions;
        cv::merge(channels, n.second);
    } else {
        n.second = partitions;
    }
    if (keep_id_) { id_ = id; }
    return n;
}

template < typename H >
std::pair< typename partition< H >::functor_t, bool > partition< H >::make(const std::string &options) {
    boost::optional< cv::Scalar > do_not_visit;
    comma::int32 none = -1;
    bool merge = false;
    bool keep_id = false;
    comma::int32 start_from = 0;
    unsigned int min_partition_size = 0;
    unsigned int degrees = 8;
    if (!options.empty()) {
        std::vector< std::string > s = comma::split(options, ',');
        for (auto& i : s) // todo: quick and dirty, use visiting
        {
            if (i == "merge") { merge = true; }
            else if (i == "keep-id") { keep_id = true; }
            else if (i.substr(0, 5) == "none:") { none = boost::lexical_cast< comma::int32 >(i.substr(5)); }
            else if (i.substr(0, 8) == "degrees:") { degrees = boost::lexical_cast< comma::uint32 >(i.substr(8)); }
            else if (i.substr(0, 13) == "do-not-visit:") { do_not_visit = boost::lexical_cast< comma::int32 >(i.substr(13)); }
            else if (i.substr(0, 11) == "start-from:") { start_from = boost::lexical_cast< comma::int32 >(i.substr(11)); }
            else if (i.substr(0, 9) == "min-size:") { min_partition_size = boost::lexical_cast< comma::int32 >(i.substr(9)); }
            else {COMMA_THROW(comma::exception, "partition: expected an option, got: '" << i << "'"); }
        }
    }
    if (degrees != 4 && degrees != 8) { COMMA_THROW(comma::exception, "partition: degrees:4 or degrees:8, got: degrees:" << degrees); }
    if (start_from <= none) { COMMA_THROW(comma::exception, "partition: expected start-from > none, got: start-from: " << start_from << " none: " << none); }
    return std::make_pair(partition< H >(do_not_visit, none, merge, keep_id, start_from, min_partition_size, degrees), !keep_id);
}

template < typename H >
typename std::string partition< H >::usage(unsigned int indent) {
    std::string offset(indent, ' ');
    std::ostringstream oss;
    oss << offset << "partition=<options>\n";
    oss << offset << "    <options>\n";
    oss << offset << "        degrees:<value>: 4: pixels connected top/down, left/right\n";
    oss << offset << "                         8: pixels connected through diagonals, too (default)\n";
    oss << offset << "        do-not-visit:<value>: value that does not represent any class\n";
    oss << offset << "                              e.g. 0: do not partition black pixels\n";
    oss << offset << "        keep-id: keep incrementing id from image to image; otherwise, for each image, \n";
    oss << offset << "                 start id with the value of start-with option\n";
    oss << offset << "        merge: if present and image is of type i (32-bit int), output two-channel image:\n";
    oss << offset << "               first channel: original image, second: partition ids\n";
    oss << offset << "        min-size:<pixels>: min partition size to keep; set output pixels of smaller partitions to none id\n";
    oss << offset << "        none:<id>: id that does not represent any class in output; default: -1\n";
    oss << offset << "        start-with:<id>: start id numbers from <id>; default: 0\n";
    return oss.str();
}

template
class partition< boost::posix_time::ptime >;

template
class partition< std::vector< char > >;

struct vertex {
    bool operator<(const vertex &other) const { return *id < *other.id; };

    comma::int32 *id;
    unsigned int degree;
};

struct compare_by_degree {
    bool operator()(const vertex &a, const vertex &b) const { return a.degree > b.degree; }
};

template < typename N >
class graph {
    public:
        graph(cv::Mat m, unsigned int channel, comma::int32 background, unsigned int max_colours)
                : ids_{{-1, -1}}, min_id_(background + 1), max_colours_(max_colours) {

            auto insert_edge = [this](N val, set_t &first_neighbours, comma::int32 *first_id_ptr) {
                comma::int32 *second_id_ptr = ids_.find(val) == ids_.end() ? &(ids_[val] = val) : &ids_[val];
                set_t *second_neighbours = &adjacency_[second_id_ptr];
                first_neighbours.insert(second_id_ptr);
                second_neighbours->insert(first_id_ptr);
            };

            for (int i = 0; i < m.rows; ++i) {
                N *ptr = m.ptr< N >(i);
                N *ptr_below = i < m.rows - 1 ? m.ptr< N >(i + 1): nullptr;
                for (int j = channel; j < m.cols * m.channels(); j += m.channels()) {
                    N val = ptr[j];
                    if (val == background) { continue; }
                    comma::int32 *first_id_ptr = ids_.find(val) == ids_.end() ? &(ids_[val] = val) : &ids_[val];
                    set_t *first_neighbours = &adjacency_[first_id_ptr];

                    if( ptr_below ) {
                        N val_below = ptr_below[j];
                        if (val != val_below && val_below != background) { insert_edge(val_below, *first_neighbours, first_id_ptr); }
                    }
                    if (j < (m.cols - 1) * m.channels()) {
                        N val_right = ptr[j + m.channels()];
                        if (val != val_right && val_right != background) { insert_edge(val_right, *first_neighbours, first_id_ptr); }
                    }
                }
            }
        }

        void reduce() {
            std::multimap< vertex, set_t, compare_by_degree > adjacency{};
            // move graph from map to multimap and order by degree
            for (auto it = adjacency_.begin(); it != adjacency_.end();) {
                auto neighbours = std::move(it->second);
                auto ptr = std::move(it->first);
                it = adjacency_.erase(it);
                // if vertex has no neighbours, can be assigned min id automatically
                *ptr = min_id_;
                if (neighbours.empty()) { continue; }
                // set all other vertices that do have neighbours to min id + 6
                *ptr += 6;
                // insert to multimap
                adjacency.emplace(vertex{std::move(ptr), static_cast<unsigned int>(neighbours.size())}, std::move(neighbours));
            }
            std::vector< unsigned char > colours_taken(max_colours_);
            for (const auto& pair : adjacency) {
                std::fill(std::begin(colours_taken), std::end(colours_taken), false);
                for (comma::int32 *o_vertex : pair.second) {
                    auto colour = *o_vertex - min_id_;  // offset by min id in partitions
                    if (colour >= 0 && colour < static_cast<int>(colours_taken.size())) { colours_taken[colour] = true; }
                }
                unsigned int i = 0;
                for (; i < max_colours_ && colours_taken[i]; ++i);
                if (i == colours_taken.size()) { COMMA_THROW(comma::exception, "partitions-reduce: adjacent vertices have used all " << max_colours_ << " available colours"); }
                *pair.first.id = i + min_id_;
            }
        }

        const std::unordered_map< comma::int32, comma::int32 > &ids() const { return ids_; }

    private:
        typedef std::unordered_set< comma::int32 * > set_t;
        std::unordered_map< comma::int32 *, set_t > adjacency_;
        std::unordered_map< comma::int32, comma::int32 > ids_;  // old -> new
        comma::int32 min_id_;
        unsigned int max_colours_;
};

template < typename H >
template < typename T, int I >
cv::Mat reduce< H >::process_(cv::Mat m, int type) {
    graph< T > g(m, channel_, background_, colours_);
    g.reduce();
    const auto& lookup_table = g.ids();
//    std::cerr << "--> lookup_table.size(): " << lookup_table.size() << std::endl;
//    for (auto p: lookup_table ) { std::cerr << "--> first: " << p.first << " second: " << p.second << std::endl; }
    auto channel = channel_;
    cv::Mat reduced_channel(m.rows, m.cols, I);
    tbb::parallel_for(size_t(0), size_t(m.rows), [&m, &reduced_channel, &lookup_table, channel](size_t i) {
        auto ptr_a = m.template ptr< T >(i);
        T* ptr_b = reduced_channel.ptr< T >(i);
        for (auto j = 0; j < m.cols; ++j)
        {
            auto it = lookup_table.find( ptr_a[j * m.channels() + channel] );
            if (it == lookup_table.end()) { COMMA_THROW( comma::exception, "pixel " << ptr_a[j * m.channels() + channel] << " at " << i << ", " << ( j * m.channels() + channel ) << " not found in the lookup table" ); }
            ptr_b[j] = it->second;
        }
    });
    if (!merge_) { return reduced_channel; }
    cv::Mat out(m.rows, m.cols, type);
    std::vector< int > from_to(out.channels() * 2);
    for (auto i = 0; i < out.channels(); ++i) { from_to[i * 2] = from_to[i * 2 + 1] = i; }
    cv::mixChannels(std::vector< cv::Mat >{m, reduced_channel}, std::vector< cv::Mat >{out}, from_to);
    return out;
}

template < typename H >
std::pair< H, cv::Mat > reduce< H >::operator()(std::pair< H, cv::Mat > m) {
    if (m.second.channels() > 3) {
        COMMA_THROW(comma::exception, "partitions-reduce: not more than 3 channels, got " << m.second.channels());
    }
    std::pair< H, cv::Mat > out{std::move(m.first), {}};
    switch (m.second.depth()) {
        case CV_8U:
            out.second = process_< unsigned char, CV_8UC1 >(m.second, CV_8UC(m.second.channels() + 1));
            break;
        case CV_8S:
            out.second = process_< char, CV_8SC1 >(m.second, CV_8SC(m.second.channels() + 1));
            break;
        case CV_16U:
            out.second = process_< comma::uint16, CV_16UC1 >(m.second, CV_16UC(m.second.channels() + 1));
            break;
        case CV_16S:
            out.second = process_< comma::int16, CV_16SC1 >(m.second, CV_16SC(m.second.channels() + 1));
            break;
        case CV_32S:
            out.second = process_< comma::int32, CV_32SC1 >(m.second, CV_32SC(m.second.channels() + 1));
            break;
        default: COMMA_THROW(comma::exception, "partitions-reduce: expected image depth, got value: " << m.second.depth() << "; not supported (yet?)");
    }
    return out;
}

template < typename H >
std::pair< typename reduce< H >::functor_t, bool > reduce< H >::make(const std::string &options) {
    unsigned int colours = 6;
    unsigned int channel = 0;
    comma::int32 background = -1;
    bool merge = false;
    if (!options.empty()) {
        std::vector< std::string > s = comma::split(options, ',');
        for (const auto& i : s) // todo: quick and dirty, use visiting
        {
            if (i == "merge") { merge = true; }
            else if (i.substr(0, 7) == "colors:" ) { colours = boost::lexical_cast< unsigned int >(i.substr(7)); }
            else if (i.substr(0, 8) == "colours:" ) { colours = boost::lexical_cast< unsigned int >(i.substr(8)); }
            else if (i.substr(0, 8) == "channel:") { channel = boost::lexical_cast< unsigned int >(i.substr(8)); }
            else if (i.substr(0, 11) == "background:") { background = boost::lexical_cast< comma::int32 >(i.substr(11)); }
            else {COMMA_THROW(comma::exception, "partition-reduce: expected an option, got: '" << i << "'"); }
        }
    }
    return std::make_pair(reduce< H >(channel, background, merge, colours), true);
}

template < typename H >
typename std::string reduce< H >::usage(unsigned int indent) {
    std::string offset(indent, ' ');
    std::ostringstream oss;
    oss << offset << "partitions-reduce=[<properties>]; reduce number of unique partition\n";
    oss << offset << "    <properties>\n";
    oss << offset << "        channel:<channel>; partition channel number in image; default: 0\n";
    oss << offset << "        background:<background>; pixel value that is not assigned any partition; default: -1\n";
    oss << offset << "        merge: if present merge reduced partitions channel to original image\n";
    oss << offset << "        colours:<max-number-of-colours>; default: 6\n";
    return oss.str();
}

template
class reduce< boost::posix_time::ptime >;

template
class reduce< std::vector< char>>;

}}}}  // namespace snark { namespace cv_mat { namespace impl { namespace partitions {
