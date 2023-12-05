// This file is part of snark, a generic and flexible library for robotics research
// Copyright (c) 2011 The University of Sydney
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


#include <gtest/gtest.h>
#include <boost/bind/bind.hpp>
#include <Eigen/Core>
#include "../covariance.h"
#include "../gaussian_process.h"

static const int nTestPoints = 10;
static const double tolerance = 1e-10;
//const double addDataMeanTolerance = 1e-2; //less accurate with due to recentering of the data, unless all data has mean at first item

static void test_gaussian_process( const Eigen::MatrixXd& inputDomains
                                 , const Eigen::MatrixXd& outputDomains
                                 , const Eigen::VectorXd& inputTargets
                                 , const double* matlab_means
                                 , const double* matlab_variances )
{
    snark::squared_exponential_covariance covariance( 1.0, 3.0, 0.1 );
    snark::gaussian_process gp( inputDomains
                                     , inputTargets
                                     , boost::bind( &snark::squared_exponential_covariance::covariance, boost::ref( covariance ), boost::placeholders::_1, boost::placeholders::_2 )
                                     , covariance.self_covariance() );
    Eigen::VectorXd outputMeans;
    Eigen::VectorXd outputVariances;
    gp.evaluate( outputDomains, outputMeans, outputVariances );
    for( int i = 0; i < nTestPoints; ++i )
    {
        EXPECT_NEAR( outputMeans(i), matlab_means[i], tolerance );
        EXPECT_NEAR( outputVariances(i), matlab_variances[i], tolerance );
    }
}

// quick and dirty, just copied from qlib
TEST( gaussian_process, one )
{
    const double MATLAB_MEANS1D[nTestPoints] =
    {
        0.415211767559116,
        0.997994703193976,
        0.580270600105819,
        -0.336831594034236,
        -0.956688275562129,
        -0.684437434476884,
        0.206412764625444,
        0.936608767483993,
        0.736060198350861,
        0.195873279476876
    };

    const double MATLAB_VARIANCES1D[nTestPoints] =
    {
        0.218260368284028,
        0.200391089040661,
        0.198149959766281,
        0.197780686599830,
        0.197724408155331,
        0.197780686599831,
        0.198149959766281,
        0.200391089040661,
        0.218260368284029,
        0.633960736027611
    };
    Eigen::MatrixXd inputDomains( nTestPoints, 1 );
    Eigen::MatrixXd outputDomains( nTestPoints, 1 );
    Eigen::VectorXd inputTargets( nTestPoints );
    for( int i=0; i < nTestPoints; ++i )
    {
        inputDomains(i) = (double)i;
        //inputVariances(i) = 0.01;
        inputTargets(i) = std::sin( (double)i );
        outputDomains(i) = (double)i + 0.5; //all outputs offset by .5 from inputs
    }
    test_gaussian_process( inputDomains
                         , outputDomains
                         , inputTargets
                         , MATLAB_MEANS1D
                         , MATLAB_VARIANCES1D );
}

TEST( gaussian_process, two )
{
    const double MATLAB_MEANS2D[nTestPoints] =
    {   0.836042657193777,
        0.570177318613353,
        -0.252108756855903,
        -0.836892315469272,
        -0.679657682729650,
        0.436707916309799,
        1.017506079736731,
        0.618705437836371,
        -0.373572217057213,
        -0.733065684706373 };

    const double MATLAB_VARIANCES2D[nTestPoints] =
    {
        0.279611483151919,
        0.262054820063053,
        0.262054820063054,
        0.279611483151919,
        0.656321135104577,
        0.681048543755416,
        0.667653218007387,
        0.667653218007388,
        0.681048543755416,
        1.054185492497780
    };
    Eigen::MatrixXd inputDomains( nTestPoints, 2 );
    Eigen::MatrixXd outputDomains( nTestPoints, 2 );
    Eigen::VectorXd inputTargets( nTestPoints );
    for( int r = 0; r < 5; ++r )
    {
        //This described a grid of points, 5x2
        // The output domain is a shifted 5x2 grid, shifted by .5 in each dimension
        inputDomains(r,0) = (double)r;
        inputDomains(r+5,0) = (double)r;
        inputDomains(r,1) = 0.0;
        inputDomains(r+5,1) = 1.0;
    }
    outputDomains = inputDomains;
    outputDomains.array() += 0.5;
    for( int i = 0; i<nTestPoints; ++i ) { inputTargets(i) = std::cos( (double)i ); }
    test_gaussian_process( inputDomains
                         , outputDomains
                         , inputTargets
                         , MATLAB_MEANS2D
                         , MATLAB_VARIANCES2D );
}

TEST( gaussian_process, some )
{
    static const double MATLAB_MEANS1Db[nTestPoints] =
    {   0.417851544484513,
        0.996451029400908,
        0.588900979135186,
        -0.344551373153785,
        -0.967435174697384,
        -0.695723526870022,
        0.205711951673409,
        0.934053962879207,
        0.738446405517442,
        0.175476309530114 };

    static const double MATLAB_VARIANCES1Db[nTestPoints] =
    {
        0.217812693169121,
        0.150085277711070,
        0.149524436348359,
        0.149430177110409,
        0.149417317368447,
        0.149431272225461,
        0.198117472035869,
        0.200347998037075,
        0.218203700211636,
        0.633645425760025
    };

    //Final test with some eval points identically equal to data points
    Eigen::MatrixXd inputDomainsAA( 15, 1 );
    Eigen::VectorXd inputTargetsAA( 15 );
    Eigen::MatrixXd outputDomains( nTestPoints, 1 );
    for( int i=0; i < 10; ++i )
    {
        inputDomainsAA(i) = (double)i;
        inputTargetsAA(i) = std::sin( (double)i );
        outputDomains(i) = (double)i + 0.5; //all outputs offset by .5 from inputs
    }

    inputDomainsAA(10) = (double)1.5;
    inputTargetsAA(10) = std::sin( (double)1.5 );
    inputDomainsAA(11) = (double)2.5;
    inputTargetsAA(11) = std::sin( (double)2.5 );
    inputDomainsAA(12) = (double)3.5;
    inputTargetsAA(12) = std::sin( (double)3.5 );
    inputDomainsAA(13) = (double)4.5;
    inputTargetsAA(13) = std::sin( (double)4.5 );
    inputDomainsAA(14) = (double)5.5;
    inputTargetsAA(14) = std::sin( (double)5.5 );
    
    test_gaussian_process( inputDomainsAA
                         , outputDomains
                         , inputTargetsAA
                         , MATLAB_MEANS1Db
                         , MATLAB_VARIANCES1Db );
}

int gaussian_processTest( int ac, char** av )
{
    ::testing::InitGoogleTest( &ac, av );
    return RUN_ALL_TESTS();
}
