// Copyright (c) 2024 Vsevolod Vlaskine

/// @author vsevolod vlaskine

#include <complex>
#include "polynomial.h"

namespace snark {

template struct polynomial< float, 1, 0 >;
template struct polynomial< float, 1, 1 >;
template struct polynomial< float, 1, 2 >;
template struct polynomial< float, 1, 3 >;
template struct polynomial< float, 1, 4 >;
template struct polynomial< float, 1, 5 >;
template struct polynomial< float, 1, 6 >;
template struct polynomial< double, 1, 0 >;
template struct polynomial< double, 1, 1 >;
template struct polynomial< double, 1, 2 >;
template struct polynomial< double, 1, 3 >;
template struct polynomial< double, 1, 4 >;
template struct polynomial< double, 1, 5 >;
template struct polynomial< double, 1, 6 >;
template struct polynomial< std::complex< float >, 1, 0 >;
template struct polynomial< std::complex< float >, 1, 1 >;
template struct polynomial< std::complex< float >, 1, 2 >;
template struct polynomial< std::complex< float >, 1, 3 >;
template struct polynomial< std::complex< float >, 1, 4 >;
template struct polynomial< std::complex< float >, 1, 5 >;
template struct polynomial< std::complex< float >, 1, 6 >;
template struct polynomial< std::complex< double >, 1, 0 >;
template struct polynomial< std::complex< double >, 1, 1 >;
template struct polynomial< std::complex< double >, 1, 2 >;
template struct polynomial< std::complex< double >, 1, 3 >;
template struct polynomial< std::complex< double >, 1, 4 >;
template struct polynomial< std::complex< double >, 1, 5 >;
template struct polynomial< std::complex< double >, 1, 6 >;

template struct polynomial< float, 2, 0 >;
template struct polynomial< float, 2, 1 >;
template struct polynomial< float, 2, 2 >;
template struct polynomial< float, 2, 3 >;
template struct polynomial< float, 2, 4 >;
template struct polynomial< float, 2, 5 >;
template struct polynomial< float, 2, 6 >;
template struct polynomial< double, 2, 0 >;
template struct polynomial< double, 2, 1 >;
template struct polynomial< double, 2, 2 >;
template struct polynomial< double, 2, 3 >;
template struct polynomial< double, 2, 4 >;
template struct polynomial< double, 2, 5 >;
template struct polynomial< double, 2, 6 >;
template struct polynomial< std::complex< float >, 2, 0 >;
template struct polynomial< std::complex< float >, 2, 1 >;
template struct polynomial< std::complex< float >, 2, 2 >;
template struct polynomial< std::complex< float >, 2, 3 >;
template struct polynomial< std::complex< float >, 2, 4 >;
template struct polynomial< std::complex< float >, 2, 5 >;
template struct polynomial< std::complex< float >, 2, 6 >;
template struct polynomial< std::complex< double >, 2, 0 >;
template struct polynomial< std::complex< double >, 2, 1 >;
template struct polynomial< std::complex< double >, 2, 2 >;
template struct polynomial< std::complex< double >, 2, 3 >;
template struct polynomial< std::complex< double >, 2, 4 >;
template struct polynomial< std::complex< double >, 2, 5 >;
template struct polynomial< std::complex< double >, 2, 6 >;

} // namespace snark {
