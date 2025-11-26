// Copyright (c) 2025 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#pragma once

#include <string>
#include <vector>
#include <boost/optional.hpp>
#include <opencv2/core/core.hpp>
#include <comma/application/command_line_options.h>
#include "../../../imaging/cv_mat/serialization.h"

namespace snark { namespace cv_calc { namespace grep {

std::string options();

int run( const comma::command_line_options& options, const snark::cv_mat::serialization& input_options, const snark::cv_mat::serialization& output_options );

class non_zero
{
    public:
        non_zero() = default;
        non_zero( const std::string& s );
        operator bool() const;
        void size( unsigned int image_size );
        bool keep( unsigned int count ) const;
        bool keep( const cv::Mat& m ) const;
        unsigned int count( const cv::Mat& m ) const;
        const uchar* ptr;

    private:
        std::pair< boost::optional< double >, boost::optional< double > > ratio_;
        std::pair< boost::optional< unsigned int >, boost::optional< unsigned int > > size_;
        bool empty_;
        bool keep_counting_( unsigned int count ) const;
};

} } } // namespace snark { namespace cv_calc { namespace grep {