// Copyright (c) 2019 Vsevolod Vlaskine

/// @author kent hu

#include "contraharmonic.h"
#include "../utils.h"

#include <comma/base/exception.h>
#include <comma/base/types.h>
#include <comma/math/compare.h>
#include <comma/string/split.h>
#include <tbb/parallel_for.h>
#include <boost/bind.hpp>
#include <boost/date_time/posix_time/ptime.hpp>
#include <boost/lexical_cast.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace kernels {
namespace operations {
template <typename T>
class operation {
    public:
        operation(unsigned int channels) noexcept : channels_(channels){};
        virtual ~operation() = default;
        virtual void operator()(const T* row_ptr, unsigned int x, unsigned int y) = 0;  // apply filter to row pointer at y and pixel x, y
        virtual std::vector<double> get() const = 0;            // return result, up to derived classes on how to return
        virtual void clear() = 0;  // clear the results, up to the derived classes on how to filter

    protected:
        unsigned int channels_;
};

template <typename T>
class contraharmonic final : public operation<T> {
   public:
    contraharmonic(unsigned int channels, double power) noexcept : operation<T>(channels), num_(channels, 0), den_(channels, 0), power_(power){};

    void operator()(const T* row_ptr, unsigned int x, unsigned int y) override {
        for (unsigned int channel = 0; channel != this->channels_; ++channel) {
            num_[channel] += std::pow(static_cast<double>(row_ptr[this->channels_ * x + channel]), power_ + 1);
            den_[channel] += std::pow(static_cast<double>(row_ptr[this->channels_ * x + channel]), power_);
        }
    };

    std::vector<double> get() const override {
        std::vector<double> result(this->channels_, 0);
        std::transform(begin(num_), end(num_), begin(den_), begin(result), [](double a, double b) { return comma::math::equal( b, 0 ) ? 0 : a / b; });
        return result;
    };

    void clear() override {
        std::fill(begin(num_), end(num_), 0);
        std::fill(begin(den_), end(den_), 0);
    }

   private:
    std::vector<double> num_;
    std::vector<double> den_;
    const double power_;
};
}  // namespace operations

bool static inline check_bounds(int value, int min, int max) { return value >= min && value < max; }

template <typename T>
class kernel {
   public:
    kernel(cv::Mat im) : im_(im){};
    virtual ~kernel() = default;
    virtual void stride(operations::operation<T>& filter) const = 0;                   // pixel traversal method over image, i.e. square, rectangle, circle, triangle
    void set_x( unsigned int x) { x_ = x; };  // change current pixel x
    void set_y( unsigned int y) { y_ = y; };  // change current pixel y

   protected:
    const cv::Mat im_;
    unsigned int x_;
    unsigned int y_;
};
template class kernel< unsigned char >;
template class kernel< char >;
template class kernel< comma::uint16 >;
template class kernel< comma::int16 >;
template class kernel< comma::int32 >;
template class kernel< float >;
template class kernel< double >;

template <typename T>
class square final : public kernel<T> {
    public:
        square(cv::Mat im, unsigned int side) : kernel<T>(im), side_(side){};
        void stride(operations::operation<T>& filter) const override {
            for (int ky = -(side_ / 2); ky < std::ceil(static_cast<double>(side_) / 2); ++ky) {
                int y = this->y_ + ky;
                if (!check_bounds(y, 0, this->im_.rows)) { continue; }
                const T* row_ptr = this->im_.template ptr<T>(y);
                for (int kx = -(side_ / 2); kx < std::ceil(static_cast<double>(side_) / 2); ++kx) {
                    int x = this->x_ + kx;
                    if (check_bounds(x, 0, this->im_.cols)) { filter(row_ptr, x, y); }
                }
            }
        };

    private:
        unsigned int side_;
};
}  // namespace kernels

/*
class circle {

}
*/

// template <typename T>
// class kernel {
//     class const_iterator {
//        public:
//         const_iterator(const T* src_iter, const T* dst_iter) : src_iter_(src_iter), dst_iter_(dst_iter){};
//         ~const_iterator() = default;

//         friend class kernel;

//         // TODO: check correct types
//         using iterator_category = std::bidirectional_iterator_tag;
//         using value_type = T;
//         using reference = T&;
//         using pointer = T*;
//         using difference_type = int;

//         reference operator*();  // TODO
//         // pre
//         virtual const_iterator& operator++() noexcept;  // TODO
//         // post
//         const_iterator operator++(difference_type) {
//             auto tmp(*this);
//             ++(*this);
//             return tmp;
//         }
//         // pre
//         virtual const_iterator& operator--() noexcept;  // TODO
//         // post
//         const_iterator operator--(difference_type) {
//             auto tmp(*this);
//             --(*this);
//             return tmp;
//         }

//         friend bool operator==(const const_iterator& lhs, const const_iterator& rhs) noexcept {
//             return lhs.src_iter_ == rhs.src_iter_ and lhs.dst_iter_ == rhs.dst_iter_;
//         };
//         friend bool operator!=(const const_iterator& lhs, const const_iterator& rhs) { return !(lhs == rhs); }

//        protected:
//         T* src_iter_;
//         T* dst_iter_;
//         // TODO: is a sentinel required? as well as a reverse sentinel?
//     };
//     using const_reverse_iterator = std::reverse_iterator<const_iterator>;

//    public:
//     virtual ~kernel() = default;

//     // void stride(cv::Mat, cv::Mat, int, int, double) = 0;
//     void set_x(int x) { x_ = x; }
//     void set_y(int y) { y_ = y; }
//     void set_im(cv::Mat im) { im_ = im; }

//     virtual const_iterator begin() const = 0;
//     virtual const_iterator end() const = 0;

//     virtual const_iterator cbegin() const = 0;
//     virtual const_iterator cend() const = 0;

//     virtual const_reverse_iterator rbegin() const = 0;
//     virtual const_reverse_iterator rend() const = 0;

//     virtual const_reverse_iterator crbegin() const = 0;
//     virtual const_reverse_iterator crend() const = 0;

//    protected:
//     cv::Mat im_;
//     int x_;
//     int y_;

//     bool check_bounds(int value, int min, int max) { return value < min || value >= max; }
// };

// template <typename T>
// class square final : public kernel<T> {
//     class const_iterator : public kernel<T>::const_iterator {};
//     using const_reverse_iterator = std::reverse_iterator<const_iterator>;

//    public:
//     square(int side) : side_{side} {};

//     const_iterator begin() const { return {}; }
//     const_iterator end() const { return {}; }

//     const_iterator cbegin() const { return {}; }
//     const_iterator cend() const { return {}; }

//     const_reverse_iterator rbegin() const { return {}; }
//     const_reverse_iterator rend() const { return {}; }

//     const_reverse_iterator crbegin() const { return {}; }
//     const_reverse_iterator crend() const { return {}; }

//    private:
//     int side_;
// };

// class square final : public kernel {
//    public:
//     square(int side) : side_{side} {};
//     square(const square &other) = default;
//     square(square &&other) = default;
//     square &operator=(const square &other) = default;
//     square &operator=(square &&other) = default;
//     ~square() = default;

//     template <typename T>
//     void stride(cv::Mat in, cv::Mat out, int px, int py, double power) {
//         std::vector<double> num(in.channels(), 0);
//         std::vector<double> den(in.channels(), 0);
//         for (int ky = -(side_ / 2); ky < std::ceil(static_cast<double>(side_) / 2); ++ky) {
//             int y = ky + py;
//             if (check_bounds(y, 0, in.rows)) {
//                 continue;
//             }
//             const T *row_ptr = in.template ptr<T>(y);
//             for (int kx = -(side_ / 2); kx < std::ceil(static_cast<double>(side_) / 2); ++kx) {
//                 int x = kx + px;
//                 if (check_bounds(x, 0, in.cols)) {
//                     continue;
//                 }
//                 for (int channel = 0; channel < in.channels(); ++channel) {
//                     num[channel] += std::pow(row_ptr[in.channels() * x + channel], power + 1);
//                     den[channel] += std::pow(row_ptr[in.channels() * x + channel], power);
//                 }
//             }
//         }
//         T *row_ptr = out.template ptr<T>(py);
//         for (int channel = 0; channel < in.channels(); ++channel) {
//             row_ptr[out.channels() * px + channel] = num[channel] / den[channel];
//             if (std::isnan(row_ptr[out.channels() * px + channel])) {
//                 row_ptr[out.channels() * px + channel] = 0;
//             }
//         }
//     };

//    private:
//     int side_;
// };

// }  // namespace kernels

namespace snark { namespace cv_mat { namespace filters {

template <typename T>
struct handle {
    handle(cv::Mat out, std::unique_ptr<kernels::kernel<T>> kernel, std::unique_ptr<kernels::operations::operation<T>> operation)
        : out_(out), kernel_(std::move(kernel)), filter_(std::move(operation)){};

    void operator()(int start, int end) {
        for (int py = start; py != end; ++py) {
            T* row_ptr = out_.template ptr<T>(py);
            kernel_->set_y(py);
            for (int px = 0; px != out_.cols; ++px) {
                kernel_->set_x(px);
                kernel_->stride(*filter_);
                const auto& result = filter_->get();
                for (int channel = 0; channel != out_.channels(); ++channel) { row_ptr[out_.channels() * px + channel] = result[channel]; }
                filter_->clear();
            }
        }
    };

    cv::Mat out_;
    std::unique_ptr<kernels::kernel<T>> kernel_;
    std::unique_ptr<kernels::operations::operation<T>> filter_;
};

template <typename H>
template <typename T>
cv::Mat contraharmonic<H>::do_parallel(cv::Mat in) {
    int sizes[]{in.rows, in.cols};
    cv::Mat out(in.dims, sizes, in.type());

    tbb::parallel_for(size_t(0), size_t(in.rows), [=](size_t i) {
        std::unique_ptr<kernels::kernel<T>> kernel;
        std::unique_ptr<kernels::operations::operation<T>> operation(new kernels::operations::contraharmonic<T>(in.channels(), power_));
        if (kernel_ == "square") { kernel.reset(new kernels::square<T>(in, side_)); }
        handle<T>(out, std::move(kernel), std::move(operation))(i, i + 1);
    });
    return out;
}

template <typename H>
std::pair<H, cv::Mat> contraharmonic<H>::operator()(std::pair<H, cv::Mat> m) {
    if (!m.second.isContinuous()) { COMMA_THROW(comma::exception, "matrix not continuous; non-continous image data not supported"); }
    cv::Mat out;
    switch (m.second.depth()) {
        case CV_8U: out = do_parallel<unsigned char>(m.second); break;
        case CV_8S: out = do_parallel<char>(m.second); break;
        case CV_16U: out = do_parallel<comma::uint16>(m.second); break;
        case CV_16S: out = do_parallel<comma::int16>(m.second); break;
        case CV_32S: out = do_parallel<comma::int32>(m.second); break;
        case CV_32F: out = do_parallel<float>(m.second); break;
        case CV_64F: out = do_parallel<double>(m.second); break;
        default: COMMA_THROW(comma::exception, "expected image data, got unsupported value: " << m.second.type());
    }
    return std::make_pair(m.first, out);
}

template <typename H>
std::pair<typename contraharmonic<H>::functor_t, bool> contraharmonic<H>::make(const std::string& options) {
    const auto& tokens = comma::split(options, ',');
    if (tokens.size() < 2) { COMMA_THROW(comma::exception, "contraharmonic: expected options, got: '" << options << "'"); }
    double power = 0;
    try { power = boost::lexical_cast<double>(tokens[0]); }
    catch (std::exception& ex) { COMMA_THROW(comma::exception, "contraharmonic: expected <power>, got: '" << options << "'; " << ex.what()); }
    if (tokens[1] == "square") {
        if (tokens.size() < 3) { COMMA_THROW(comma::exception, "contraharmonic: expected <power>,square,<side>, got: '" << options << "'"); }
        int side = boost::lexical_cast<double>(tokens[2]);
        return std::make_pair(contraharmonic<H>(tokens[1], power, side), true);
        // return std::make_pair(contraharmonic<H, kernels::square >( power, side ), true);
        // return std::make_pair(contraharmonic<H >( power, kernels::square(), side ), true);
    }
    COMMA_THROW(comma::exception, "contraharmonic: expected kernel shape, got: '" << tokens[1] << "'; only 'square' supported at the moment");
}

template <typename H>
typename std::string contraharmonic<H>::usage(unsigned int indent) {
    std::string offset(indent, ' ');
    std::ostringstream oss;
    oss << offset << "contraharmonic=<power>,<kernel_shape>,<kernel_geometry>, e.g: contraharmonic=3,square,5\n";
    oss << offset
        << "               <power>: order of the filter; positive values remove pepper noise; negative values "
           "remove "
           "salt noise\n";
    oss << offset
        << "               <kernel_shape>: shape of kernel filter; default: square; todo: support other shapes, "
           "e.g. "
           "rectangle\n";
    oss << offset << "               <kernel_geometry>:\n";
    oss << offset << "                   square,<side>, e.g: 'square,5'\n";
    return oss.str();
}

template class contraharmonic<boost::posix_time::ptime>;
template class contraharmonic<std::vector<char>>;

}  // namespace impl
}  // namespace cv_mat
}  // namespace snark
