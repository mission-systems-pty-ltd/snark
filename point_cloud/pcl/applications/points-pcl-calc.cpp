// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2017 The University of Sydney
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
// 3. Neither the name of the University of Sydney nor the
//    names of its contributors may be used to endorse or promote products
//    derived from this software without specific prior written permission.
//
// NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE
// GRANTED BY THIS LICENSE.  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT
// HOLDERS AND CONTRIBUTORS \"AS IS\" AND ANY EXPRESS OR IMPLIED
// WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
// OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
// IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/// @author Navid Pirmarzdashti

// #define PCL_NO_PRECOMPILE

#include <comma/application/command_line_options.h>
#include <comma/application/verbose.h>
#include <comma/csv/options.h>
#include <comma/csv/ascii.h>
#include <comma/csv/stream.h>
#include <comma/csv/traits.h>
#include <comma/name_value/parser.h>
#include <iostream>
#include <vector>
#include <fstream>
#include <Eigen/Core>
#include "../../../visiting/eigen.h"
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/registration/icp.h>
//#include <pcl/filters/statistical_outlier_removal.h>

static void usage(bool detail)
{
    std::cerr<<"    points pcl calc operations" << std::endl;
    std::cerr << std::endl;
    std::cerr<< "usage: " << comma::verbose.app_name() << " <operation> [ <options> ]" << std::endl;
    std::cerr << std::endl;
    std::cerr<< "operations:"<< std::endl;
    std::cerr<< "    region-growing-segmentation"<< std::endl;
    std::cerr<< "    normal-estimator"<< std::endl;
//    std::cerr<< "    statistical-outlier"<< std::endl;
    std::cerr<< "    iterative-closest-point"<< std::endl;
    std::cerr << std::endl;
    std::cerr << "options:" << std::endl;
    std::cerr << "    --help,-h: show help" << std::endl;
    std::cerr << "    --verbose,-v: show detailed messages" << std::endl;
    std::cerr << "    --input-fields: print input fields and exit"<<std::endl;
    std::cerr << "    --input-format: print input format and exit"<<std::endl;
    std::cerr << "    --output-fields: print output fields and exit"<<std::endl;
    std::cerr << "    --output-format: print output format and exit"<<std::endl;
    std::cerr << std::endl;
    std::cerr<< "region-growing-segmentation"<< std::endl;
    std::cerr<< "     fields: block,point/x,point/y,point/z,normal/x,normal/y,normal/z,normal/curvature"<< std::endl;
    std::cerr<< "     output: <input>,id"<< std::endl;
    std::cerr<< "     options:"<< std::endl;
    std::cerr << "        --min-cluster-size=<n>: the minimum number of points that a cluster needs to contain in order to be considered valid; default: 50"<< std::endl;
    std::cerr << "        --max-cluster-size=<n>: the maximum number of points that a cluster needs to contain in order to be considered valid; default 1000000"<< std::endl;
    std::cerr << "        --number-of-neighbours=<n>: number of nearest neighbours used for KNN (k-nearest neighbours search); default: 30"<< std::endl;
    std::cerr << "        --smoothness-threshold=<f>: smoothness threshold used for testing the points; default: "<<3.0 / 180.0 * M_PI<< std::endl;
    std::cerr << "        --curvature-threshold=<f>: residual threshold used for testing the points; default: 1.0"<< std::endl;
    std::cerr << "        --discard: discard points with invalid cluster, when not specified all points will be written to output, invalid points with id=0"<< std::endl;
    std::cerr << std::endl;
    std::cerr<< "normal-estimator"<< std::endl;
    std::cerr<< "     fields: block,point/x,point/y,point/z"<< std::endl;
    std::cerr<< "     output: <input>,x,y,z,curvature"<< std::endl;
    std::cerr<< "     options:"<< std::endl;
    std::cerr << "       --k,--k-neighbours=<n>: number of k nearest neighbors to use for the feature estimation; default: 50"<< std::endl;
    std::cerr << std::endl;
//    std::cerr<< "statistical-outlier"<< std::endl;
//    std::cerr<< "     fields: block,point/x,point/y,point/z"<< std::endl;
//    std::cerr<< "     output: block,point/x,point/y,point/z"<< std::endl;
//    std::cerr<< "     options:"<< std::endl;
//    std::cerr << "       -k,--k-mean=<n>: number of k nearest neighbors analyze; default: 50"<< std::endl;
//    std::cerr << "       --s,--stddev=<d>: standard deviation; default 1"<< std::endl;
//    std::cerr << std::endl;
     std::cerr<< "iterative-closest-point"<< std::endl;
    std::cerr << std::endl;
    std::cerr<< "     WARNING: work in progress, output format subject to change. talk to authors if you need to use icp operation"<< std::endl;
    std::cerr << std::endl;
    std::cerr<< "     second point cloud file is passed as unnamed option"<< std::endl;
    std::cerr<< "     fields: block,point/x,point/y,point/z"<< std::endl;
    std::cerr<< "     output: <input>,matrix[16]: 4x4 affine transformation matrix"<< std::endl;
    std::cerr<< "     options:"<< std::endl;
    std::cerr << "       --iterations=<n>: maximum number of iterations; the algorithm will stop after this many iterations even if it has not converged; default 100"<< std::endl;
    std::cerr << "       --transformation-epsilon=<d>: distance threshold; if distance between two correspondent points is larger than this, the points will not be used for aligning"<< std::endl;
    std::cerr << "       --max-correspondence-distance=<d>: transformations epsilon for convergence ; if difference between two consecutive transformations are less than this, the algorithm will stop"<< std::endl;
    std::cerr << "       --euclidean-fitness-epsilon=<d>: error epsilon for convergence; if average Euclidean distance of points is less than this, the algorithm will stop"<< std::endl;
    std::cerr << std::endl;
    if(detail)
    {
        std::cerr << "csv options:" << std::endl;
        std::cerr<< comma::csv::options::usage() << std::endl;
    }
    else
    {
        std::cerr << "use -v or --verbose to see more detail" << std::endl;
    }
    std::cerr << std::endl;
    std::cerr << "examples" << std::endl;
    std::cerr<< "     cat points.csv | "<<comma::verbose.app_name()<<" normal-estimator | "<<comma::verbose.app_name()<<" region-growing-segmentation " << std::endl;
    std::cerr << std::endl;
    std::cerr<< "     cat points1.csv | "<<comma::verbose.app_name()<<" iterative-closest-point points2.csv" << std::endl;
    std::cerr << std::endl;
}

struct input_point
{
    std::uint32_t block;
    pcl::PointXYZ point;
    input_point() : block(0) { }
};

struct input_point_normal
{
    std::uint32_t block;
    pcl::PointXYZ point;
    pcl::Normal normal;
    input_point_normal() : block(0) { }
};

struct segmentation_output
{
    std::uint32_t id;
    segmentation_output(std::uint32_t id=0) : id(id) { }
};

struct icp_output
{
    pcl::Registration<pcl::PointXYZ,pcl::PointXYZ>::Matrix4 transformation;
};

namespace comma { namespace visiting {
    
template <> struct traits< pcl::PointXYZ >
{
    template< typename K, typename V > static void visit( const K& k, pcl::PointXYZ& p, V& v )
    {
        v.apply( "x", p.x );
        v.apply( "y", p.y );
        v.apply( "z", p.z );
    }
    template< typename K, typename V > static void visit( const K& k, const pcl::PointXYZ& p, V& v )
    {
        v.apply( "x", p.x );
        v.apply( "y", p.y );
        v.apply( "z", p.z );
    }
};

template <> struct traits< pcl::Normal >
{
    template< typename K, typename V > static void visit( const K& k, pcl::Normal& p, V& v )
    {
        v.apply( "x", p.normal_x );
        v.apply( "y", p.normal_y );
        v.apply( "z", p.normal_z );
        v.apply( "curvature", p.curvature );
    }
    template< typename K, typename V > static void visit( const K& k, const pcl::Normal& p, V& v )
    {
        v.apply( "x", p.normal_x );
        v.apply( "y", p.normal_y );
        v.apply( "z", p.normal_z );
        v.apply( "curvature", p.curvature );
    }
};

template <> struct traits< input_point >
{
    template< typename K, typename V > static void visit( const K& k, input_point& p, V& v )
    {
        v.apply( "block", p.block );
        v.apply( "point", p.point );
    }
    template< typename K, typename V > static void visit( const K& k, const input_point& p, V& v )
    {
        v.apply( "block", p.block );
        v.apply( "point", p.point );
    }
};

template <> struct traits< input_point_normal >
{
    template< typename K, typename V > static void visit( const K& k, input_point_normal& p, V& v )
    {
        v.apply( "block", p.block );
        v.apply( "point", p.point );
        v.apply( "normal", p.normal );
    }
    template< typename K, typename V > static void visit( const K& k, const input_point_normal& p, V& v )
    {
        v.apply( "block", p.block );
        v.apply( "point", p.point );
        v.apply( "normal", p.normal );
    }
};

template <> struct traits< segmentation_output >
{
    template< typename K, typename V > static void visit( const K& k, segmentation_output& p, V& v )
    {
        v.apply( "id", p.id);
    }
    template< typename K, typename V > static void visit( const K& k, const segmentation_output& p, V& v )
    {
        v.apply( "id", p.id);
    }
};

template <> struct traits< icp_output >
{
    template< typename K, typename V > static void visit( const K& k, icp_output& p, V& v )
    {
        v.apply( "transformation", p.transformation );
    }
    template< typename K, typename V > static void visit( const K& k, const icp_output& p, V& v )
    {
        v.apply( "transformation", p.transformation );
    }
};

} } // namespace comma { namespace visiting {

class app_i
{
public:
    virtual ~app_i() { }
    virtual void print_input_fields()=0;
    virtual void print_input_format()=0;
    virtual void print_output_fields()=0;
    virtual void print_output_format()=0;
    virtual void run()=0;
};

//template< typename T, typename S >
//class filter_app_t : public app_i
//{
//public:
//    typedef T input_t;
//    typedef S output_t;
//
//protected:
//    comma::csv::options csv;
//    std::undordered_map< PointXYZ, std::string > buffer;
//    std::size_t reserve;
//    
//    virtual void clear()=0;
//    virtual void push_back(const input_t& p)=0;
//    virtual void process()=0;
//    //process cloud
//    virtual void write_output(comma::csv::output_stream<output_t>& os)=0;
//
//public:
//    app_t(const comma::command_line_options& options) : csv(options)
//    {
//        csv.full_xpath=true;
//        reserve=options.value<std::size_t>("--reserve",10000);
//    }
//    virtual void run()
//    {
//        input_t default_input;
//        comma::csv::input_stream<input_t> is( std::cin, csv, default_input);
//        std::uint32_t block=0;
//        buffer.reserve(reserve);
//        while( is.ready() || std::cin.good() )
//        {
//            const input_t* p=is.read();
//            if ( (!p || block != p->block ) && buffer.size() )
//            {
//                process();
//                write_output(os);
//                //reset
//                buffer.clear();
//                clear();
//            }
//            if(!p){break;}
//            block=p->block;
//            buffer.emplace( p.point, is.last() );
//            push_back(*p);
//        }
//    }
//    void print_input_fields()
//    {
//        std::cout<<comma::join( comma::csv::names<input_t>(true), ',' ) << std::endl;
//    }
//    void print_input_format()
//    {
//        std::cout<<comma::csv::format::value<input_t>() << std::endl;
//    }
//    void print_output_fields()
//    {
//        std::cout<<comma::join( comma::csv::names<output_t>(true), ',' ) << std::endl;
//    }
//    void print_output_format()
//    {
//        std::cout<<comma::csv::format::value<output_t>() << std::endl;
//    }
//};

template<typename T,typename S>
class app_t : public app_i
{
public:
    typedef T input_t;
    typedef S output_t;
protected:
    comma::csv::options csv;
    input_t default_input;
    std::vector<std::string> buffer;
    std::size_t reserve;
    
    virtual void clear()=0;
    virtual void push_back(const input_t& p)=0;
    virtual void process()=0;
    //process cloud
    virtual void write_output(comma::csv::output_stream<output_t>& os)=0;
public:
    app_t(const comma::command_line_options& options) : csv(options)
    {
        csv.full_xpath=true;
        reserve=options.value<std::size_t>("--reserve",10000);
    }
    virtual void run()
    {
        comma::csv::input_stream<input_t> is( std::cin, csv, default_input);
        comma::csv::output_stream<output_t> os( std::cout, csv.binary(), true, csv.flush );
        std::uint32_t block=0;
        buffer.reserve(reserve);
        while( is.ready() || std::cin.good() )
        {
            const input_t* p=is.read();
            if ( (!p || block != p->block ) && buffer.size() )
            {
                process();
                write_output(os);
                //reset
                buffer.clear();
                clear();
            }
            if(!p){break;}
            block=p->block;
            buffer.push_back(is.last());
            push_back(*p);
        }
    }
    void print_input_fields()
    {
        std::cout<<comma::join( comma::csv::names<input_t>(true), ',' ) << std::endl;
    }
    void print_input_format()
    {
        std::cout<<comma::csv::format::value<input_t>() << std::endl;
    }
    void print_output_fields()
    {
        std::cout<<comma::join( comma::csv::names<output_t>(true), ',' ) << std::endl;
    }
    void print_output_format()
    {
        std::cout<<comma::csv::format::value<output_t>() << std::endl;
    }
};

struct normal_estimator : public app_t<input_point,pcl::Normal>
{
    typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    typename pcl::PointCloud<pcl::Normal> normals;
    int k;
    normal_estimator(const comma::command_line_options& options) : app_t(options),
        cloud(new pcl::PointCloud<pcl::PointXYZ>())
    {
        k=options.value<int>("--k,--k-neighbours",50);
    }
    void clear()
    {
        cloud->clear();
        normals.clear();
    }
    void push_back(const input_t& p)
    {
        cloud->push_back(p.point);
    }
    void process()
    {
        pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
        normal_estimator.setSearchMethod(tree);
        normal_estimator.setInputCloud(cloud);
        normal_estimator.setKSearch(k);
        normal_estimator.compute(normals);
    }
    void write_output(comma::csv::output_stream<output_t>& os)
    {
        for(std::size_t i=0;i<buffer.size()&&i<normals.size();i++)
        {
            os.append(buffer[i],normals[i]);
        }
    }
};

//struct statistical_outlier : public filter_app_t<input_point,input_point>
//{
//    typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
//    typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered;
//    int kmean;
//    double stddev;
//
//    normal_estimator(const comma::command_line_options& options) : app_t(options),
//        cloud(new pcl::PointCloud<pcl::PointXYZ>())
//    {
//        kmean=options.value<int>("-k,--k-mean",50);
//        stddev = options.value< double >( "-s,--stddev", 1.0 );
//    }
//    void clear()
//    {
//        cloud->clear();
//        cloud_filtered.clear();
//    }
//    void push_back(const input_t& p)
//    {
//        cloud->push_back(p.point);
//    }
//    void process()
//    {
//        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
//        sor.setInputCloud (cloud);
//        sor.setMeanK (kmean);
//        sor.setStddevMulThresh (1.0);
//        sor.filter (*cloud_filtered);
//    }
//    void write_output(comma::csv::output_stream<output_t>& os)
//    {
//        for( auto const& pi : cloud_filtered.points )
//        {
//            auto const oi = buffer.find( pi );
//            if( buffer.end() != oi )
//            {
//                std::cout.write( oi->second[ 0 ], oi->second.size() );
//                if( !csv.binary() ) { std::cout << "\n"; }
//                if( csv.flush() ) { std::cout.flush(); }
//            }
//        }
//    }
//};
//

struct iterative_closest_point : public app_t<input_point,icp_output>
{
    typedef typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr;
    std::pair<cloud_ptr,cloud_ptr> clouds;
    icp_output output;
    int iterations;
    boost::optional<double> transformation_epsilon;
    boost::optional<double> max_correspondence_distance;
    boost::optional<double> euclidean_fitness_epsilon;
    comma::csv::options second_csv;
    iterative_closest_point(const comma::command_line_options& options, const comma::csv::options& second) : app_t(options),
        clouds(cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>()),cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>())), second_csv(second)
    {
        iterations=options.value<int>("--iterations",100);
        transformation_epsilon=options.optional<double>("--transformation-epsilon");
        max_correspondence_distance=options.optional<double>("--max-correspondence-distance");
        euclidean_fitness_epsilon=options.optional<double>("--euclidean-fitness-epsilon");
        second_csv.full_xpath=true;
    }
    void run()
    {
        comma::csv::input_stream<input_t> is(std::cin,csv,default_input);
        std::ifstream ifs(second_csv.filename);
        if(!ifs.is_open()) { COMMA_THROW( comma::exception, "failed to open file: "<<second_csv.filename); }
        comma::csv::input_stream<input_t> is2(ifs,second_csv);
        comma::csv::output_stream<output_t> os(std::cout,csv.binary(),true,csv.flush);
        std::uint32_t block=0;
        buffer.reserve(reserve);
        bool read1=is.ready() || std::cin.good();
        bool read2=is2.ready() || ifs.good();
        const input_t* p1=NULL;
        const input_t* p2=NULL;
        while(is.ready() || std::cin.good() || is2.ready() || ifs.good())
        {
            if(read1)
            {
                p1=is.read();
                if(!p1 || block != p1->block) { read1=false; }
            }
            if(read2)
            {
                p2=is2.read();
                if(!p2 || block != p2->block) { read2=false; }
            }
            if(!read1 && !read2 && buffer.size())
            {
                process();
                write_output(os);
                //reset
                buffer.clear();
                clear();
                read1=is.ready() || std::cin.good();
                read2=is2.ready() || ifs.good();
            }
            if(!p1&&!p2){break;}
            if(p1)
            {
                block=p1->block;
                buffer.push_back(is.last());
            }
            push_back(p1,p2);
        }
    }
    void clear()
    {
        clouds.first->clear();
        clouds.second->clear();
        output=icp_output();
        cloud_registered->clear();
    }
    void push_back(const input_t& p) { COMMA_THROW(comma::exception,"invalid method called!"); }
    void push_back(const input_t* p1,const input_t* p2)
    {
        if(p1) { clouds.first->push_back(p1->point); }
        if(p2) { clouds.second->push_back(p2->point); }
    }
    cloud_ptr cloud_registered;
    void process()
    {
        pcl::IterativeClosestPoint<pcl::PointXYZ,pcl::PointXYZ> icp;
        cloud_registered.reset(new pcl::PointCloud<pcl::PointXYZ>());
        icp.setInputSource(clouds.first);
        icp.setInputTarget(clouds.second);
        icp.setMaximumIterations(iterations);
        if(max_correspondence_distance)
        {
            comma::verbose<<"max_correspondence_distance "<<*max_correspondence_distance<<std::endl;
            icp.setMaxCorrespondenceDistance(*max_correspondence_distance);
        }
        if(transformation_epsilon) { icp.setTransformationEpsilon(*transformation_epsilon); }
        if(euclidean_fitness_epsilon) { icp.setEuclideanFitnessEpsilon(*euclidean_fitness_epsilon); }
        icp.align(*cloud_registered);
        output.transformation=icp.getFinalTransformation();
    }
    void write_output(comma::csv::output_stream<output_t>& os)
    {
        for(std::size_t i=0;i<buffer.size();i++)
        {
            os.append(buffer[i],output);
        }
    }
};

struct region_growing_segmentation : public app_t<input_point_normal,segmentation_output>
{
    typename pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    typename pcl::PointCloud<pcl::Normal>::Ptr normals;
    std::vector<output_t> outputs;

    int min_cluster_size;
    int max_cluster_size;
    unsigned number_of_neighbours;
    float smoothness_threshold;
    float curvature_threshold;
    bool discard;
    region_growing_segmentation(const comma::command_line_options& options) : app_t(options), 
        cloud(new pcl::PointCloud<pcl::PointXYZ>()), normals(new pcl::PointCloud <pcl::Normal>)
    {
        min_cluster_size=options.value<int>("--min-cluster-size",50);
        max_cluster_size=options.value<int>("--max-cluster-size",1000000);
        number_of_neighbours=options.value<unsigned>("--number-of-neighbours",30);
        smoothness_threshold=options.value<float>("--smoothness-threshold",3.0 / 180.0 * M_PI);
        curvature_threshold=options.value<float>("--curvature-threshold",1.0);
        discard=options.exists("--discard");
        cloud->points.reserve(reserve);
    }
    void clear()
    {
        cloud->clear();
        normals->clear();
        outputs.clear();
    }
    void push_back(const input_t& p)
    {
        cloud->push_back(p.point);
        normals->push_back(p.normal);
    }
    void process()
    {
        std::vector<pcl::PointIndices> clusters;
        pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
//         comma::verbose<<"min,max,k "<<min_cluster_size<<" "<<max_cluster_size<<" "<<number_of_neighbours<<std::endl;
        reg.setMinClusterSize(min_cluster_size);
        reg.setMaxClusterSize(max_cluster_size);
        reg.setSearchMethod(tree);
        reg.setNumberOfNeighbours(number_of_neighbours);
        reg.setInputCloud(cloud);
        //reg.setIndices (indices);
        reg.setInputNormals(normals);
        reg.setSmoothnessThreshold(smoothness_threshold);
        reg.setCurvatureThreshold(curvature_threshold);
        reg.extract(clusters);
        outputs.resize(buffer.size(),segmentation_output(0));
        comma::verbose<<"clusters.size() "<<clusters.size()<<" clusters[0].indices.size() "<<clusters[0].indices.size()<<std::endl;
        for(std::uint32_t i=0;i<clusters.size();i++)
        {
            for(auto j : clusters[i].indices)
            {
                outputs[j].id=i+1;
            }
        }
    }
    void write_output(comma::csv::output_stream<output_t>& os)
    {
        for(std::size_t i=0;i<buffer.size()&&i<outputs.size();i++)
        {
            if( !discard || outputs[i].id)
            {
                os.append(buffer[i],outputs[i]);
            }
        }
    }
};

// todo:
// change template parameter to subclass and use T::input_t, T::output_t instead of T and S
// add bash completion
// mkdir points-pcl-calc
// mv app* to points-pcl-calc/app.h or common.h
// normal estimator 
//      mv to points-pcl-calc/feature.h
//      input fields: accept x,y,z translate to point/x ... (see view-points for example)
// region growing segmentation 
//      mv to points-pcl-calc/segmentation.h
//      input fields: accept x,y,z translate to point/x ... (see view-points for example)
// do we need to support point double precision for above? (do we need both float and double, or just double)
// mv icp to points-pcl-calc/registration.h

int main(int argc,char** argv)
{
    comma::command_line_options options(argc,argv,usage);
    try
    {
        const std::vector< std::string >& unnamed=options.unnamed( "--verbose,-v,--input-fields,--input-format,--output-fields,--output-format,--discard","-.*" );
        if(unnamed.size()<1) { COMMA_THROW( comma::exception, "expected an operation, got none"); }
        const std::string& operation=unnamed[0];
        std::unique_ptr<app_i> app;
        if(operation=="iterative-closest-point")
        {
            if(unnamed.size()!=2) { COMMA_THROW( comma::exception, "expected one input stream, got "<<unnamed.size()-1<<" "<< comma::join(unnamed,',') ); }
            comma::name_value::parser parser("filename",';','=',false);
            app.reset(new iterative_closest_point(options,parser.get<comma::csv::options>(unnamed[1])));
        }
        else
        {
            if(unnamed.size()!=1) { COMMA_THROW( comma::exception, "expected an operation, got "<<unnamed.size()<<" "<< comma::join(unnamed,',') ); }
            if(operation=="region-growing-segmentation")
            {
                app.reset(new region_growing_segmentation(options));
            }
            else if(operation=="normal-estimator")
            {
                app.reset(new normal_estimator(options));
            }
//            else if(operation=="statistical-filter")
//            {
//                app.reset(new statistical_outlier(options));
//            }
            else
            {
                std::cerr<<comma::verbose.app_name()<<": unrecognized operation "<<operation<<std::endl;
                return 1;
            }
        }
        if(options.exists("--input-fields")) { app->print_input_fields(); return 0; }
        if(options.exists("--input-format")) { app->print_input_format(); return 0; }
        if(options.exists("--output-fields")) { app->print_output_fields(); return 0; }
        if(options.exists("--output-format")) { app->print_output_format(); return 0; }
        app->run();
        return 0;
    }
    catch( std::exception& ex )
    {
        std::cerr << comma::verbose.app_name() << ": exception: " << ex.what() << std::endl;
    }
    catch( ... )
    {
        std::cerr << comma::verbose.app_name() << ": " << "unknown exception" << std::endl;
    }
    return 1;
}
