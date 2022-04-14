// Copyright (c) 2019 Vsevolod Vlaskine

/// @author kent hu

#pragma once

#include <boost/function.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <memory>
#include <string>

namespace snark { namespace cv_mat { namespace filters {

template <typename H>
class contraharmonic {
    typedef boost::function<std::pair<H, cv::Mat>(std::pair<H, cv::Mat>)> functor_t;

   public:
    contraharmonic(const std::string& kernel, double power, int side) : kernel_(kernel), power_(power), side_(side){};
    std::pair<H, cv::Mat> operator()(std::pair<H, cv::Mat> m);
    static std::pair<functor_t, bool> make(const std::string& options);
    static std::string usage(unsigned int indent = 0);

   private:
    const std::string kernel_;
    const double power_;
    const int side_;

    template <typename T>
    cv::Mat do_parallel(cv::Mat in);  // helper function
};

} } } /// namespace snark { namespace cv_mat { namespace filters {
