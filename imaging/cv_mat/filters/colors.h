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

#pragma once

#define OPENCV_HAS_BALANCE_WHITE !(defined(CV_VERSION_EPOCH) && CV_VERSION_EPOCH == 2)

#include <opencv2/core/core.hpp>
#if OPENCV_HAS_BALANCE_WHITE && SNARK_OPENCV_CONTRIB
#include <opencv2/xphoto/white_balance.hpp>
#else // OPENCV_HAS_BALANCE_WHITE && SNARK_OPENCV_CONTRIB
#include <comma/base/exception.h>
#endif // OPENCV_HAS_BALANCE_WHITE && SNARK_OPENCV_CONTRIB

namespace snark { namespace cv_mat { namespace filters { namespace colors {

#if OPENCV_HAS_BALANCE_WHITE && SNARK_OPENCV_CONTRIB
    template < typename H >
    class balance_white
    {
        public:
            balance_white();
            std::pair< H, cv::Mat > operator()( std::pair< H, cv::Mat > m );
            static std::string usage( unsigned int indent );
        private:
            cv::Ptr< cv::xphoto::SimpleWB > wb_;
    };
#else // OPENCV_HAS_BALANCE_WHITE && SNARK_OPENCV_CONTRIB
    template < typename H >
    struct balance_white
    {
        #if !OPENCV_HAS_BALANCE_WHITE
        #define STR_HELPER(x) #x
        #define STR(x) STR_HELPER(x)
        #define BALANCE_WHITE_ERROR_MSG "balance-white not implemented for opencv version " STR(CV_MAJOR_VERSION) "." STR(CV_MINOR_VERSION) " that you have"
        #else // OPENCV_HAS_BALANCE_WHITE
        #define BALANCE_WHITE_ERROR_MSG "balance-white disabled; enable by recompiling with snark_build_imaging_opencv_contrib=ON"
        #endif // OPENCV_HAS_BALANCE_WHITE
        balance_white() { COMMA_THROW( comma::exception, BALANCE_WHITE_ERROR_MSG ); }
        std::pair< H, cv::Mat > operator()( std::pair< H, cv::Mat > m ) { COMMA_THROW( comma::exception, BALANCE_WHITE_ERROR_MSG ); }
        static std::string usage( unsigned int indent );
    };
#endif // OPENCV_HAS_BALANCE_WHITE && SNARK_OPENCV_CONTRIB

template < typename H >
class map
{
    public:
        map( int type ): _type( type ) {}
        std::pair< H, cv::Mat > operator()( std::pair< H, cv::Mat > m );
        static std::string usage( unsigned int indent );
    private:
        int _type{0}; 
};

} } } }  // namespace snark { namespace cv_mat { namespace filters { namespace colors {
