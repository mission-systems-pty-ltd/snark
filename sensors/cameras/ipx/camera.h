// This file is provided in addition to snark and is not an integral
// part of snark library.
// Copyright (c) 2019 Vsevolod Vlaskine
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
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

#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <boost/bind/bind.hpp>
#include <boost/date_time/posix_time/ptime.hpp>
#include <opencv2/core/core.hpp>
#include <IpxCameraApi.h>

namespace snark { namespace ipx {

/// convenience wrapper due to ipx workflow idiosynchrasies
/// behaves identical to std::unique_ptr, but calls Release on destruction
template < typename T >
class unique_ptr
{
    public:
        unique_ptr() {}
        
        unique_ptr( T* t ): ptr_( t, []( T* t ) { t->Release(); } ) {}
        
        void reset( T* t ) { ptr_.reset( t ); }
        
        T& operator*() { return *ptr_; }
        
        const T& operator*() const { return *ptr_; }
        
        T* operator->() { return ptr_.get(); }
        
        const T* operator->() const { return ptr_.get(); }
        
        operator bool() { return bool( ptr_ ); }        
        
    private:
        std::unique_ptr< T, std::function< void( T* ) > > ptr_;
};
    
// template < typename T >
// class unique_ptr
// {
//     public:
//         unique_ptr() {}
//         
//         unique_ptr( T* t ): ptr_( t ) {}
//         
//         ~unique_ptr() { if( ptr_ ) { ptr_->Release(); } }
//         
//         void reset( T* t = nullptr ) { if( ptr_ ) { ptr_->Release(); } ptr_.reset( t ); }
//         
//         T& operator*() { return *ptr_; }
//         
//         const T& operator*() const { return *ptr_; }
//         
//         T* operator->() { return ptr_.get(); }
//         
//         const T* operator->() const { return ptr_.get(); }
//         
//         operator bool() { return bool( ptr_ ); }        
//         
//     private:
//         std::unique_ptr< T > ptr_;
// };

class camera;

class system
{
    public:
        system();
        
        ~system();
        
        std::string interfaces_description(); // due to ipx api, cannot be const
        
        std::string devices_description(); // due to ipx api, cannot be const
        
        IpxCam::DeviceInfo* device_info( const std::string& id = "" );
        
    private:
        IpxCam::System* system_;
        IpxCam::InterfaceList* interface_list_;
};

class camera
{
    public:
        camera( IpxCam::DeviceInfo* device_info );
        
        camera( IpxCam::Device* device );
        
        ~camera();
        
        std::string get_parameter( const std::string& path ); // todo! traverse path
        
        void set_parameter( const std::string& path, const std::string& value ); // todo! traverse path
        
        std::string list_parameters(); // todo? use Device::SaveConfiguration() instead?
        
        void connect();
        
        void start_acquisition() {} // todo
        
        IpxCam::Device& device() { return *device_; }
        
        const IpxCam::Device& device() const { return *device_; }
    
    private:
        IpxCam::Device * device_;
        std::string id_;
};

class stream
{
    public:
        typedef std::pair< boost::posix_time::ptime, cv::Mat > pair_t;
        
        stream( camera& c ): camera_( c ) {} // todo
        
        pair_t read() { return pair_t(); } // todo
    
    private:
        camera& camera_;
};

} } // namespace snark { namespace ipx {
