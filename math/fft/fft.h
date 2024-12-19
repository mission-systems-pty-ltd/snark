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

#include <array>
#include <cmath>

namespace snark { 

/// Cooley-Tukey in-place fft based on Danielson-Lanczos algorithm
///
/// see: Cooley-Tukey algorithm from Numerical Recipes in C++
///
/// see also: Vlodymyr Myrnyy, Simple and Efficient Fast Fourier Transform, Dr. Dobbs' Journal
/// http://www.drdobbs.com/cpp/a-simple-and-efficient-fft-implementatio/199500857
template < typename T >
void fft( T* data, std::size_t size );

/// Cooley-Tukey in-place fft based on Danielson-Lanczos algorithm
template < typename T, std::size_t N >
void fft( std::array< T, N >& data ) { fft( &data[0], N ); }

/// Cooley-Tukey fft based on Danielson-Lanczos algorithm (convenience function)
template < typename T, std::size_t N >
std::array< T, N > fft( const std::array< T, N >& data );



template < typename T, std::size_t N >
inline std::array< T, N > fft( const std::array< T, N >& data )
{
    std::array< T, N > a = data;
    fft( &a[0], N );
    return a;
}

template < typename T >
inline void fft( T* data, std::size_t size )
{
    unsigned long n, mmax, m, j, istep, i;
    double wtemp, wr, wpr, wpi, wi, theta;
    double tempr, tempi;
 
    // reverse-binary reindexing
    n = size << 1;
    j = 1;
    for( i = 1; i < n; i += 2 )
    {
        if( j > i )
        {
            std::swap( data[ j - 1 ], data[ i - 1 ] );
            std::swap( data[j], data[i]);
        }
        m = size;
        while( m >= 2 && j > m )
        {
            j -= m;
            m >>= 1;
        }
        j += m;
    };
 
    // Danielson-Lanczos method
    mmax = 2;
    while( n > mmax )
    {
        istep = mmax << 1;
        theta = -( 2 * M_PI / mmax );
        wtemp = std::sin( 0.5 * theta );
        wpr = -2.0 * wtemp * wtemp;
        wpi = std::sin( theta );
        wr = 1.0;
        wi = 0.0;
        for( m = 1; m < mmax; m += 2 )
        {
            for( i = m; i <= n; i += istep )
            {
                j = i + mmax;
                tempr = wr * data[j-1] - wi * data[j];
                tempi = wr * data[j] + wi * data[j-1];
                data[j-1] = data[i-1] - tempr;
                data[j] = data[i] - tempi;
                data[i-1] += tempr;
                data[i] += tempi;
            }
            wtemp = wr;
            wr += wr * wpr - wi * wpi;
            wi += wi * wpr + wtemp * wpi;
        }
        mmax = istep;
    }
}

}  // namespace snark{ 
