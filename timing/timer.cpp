// Copyright (c) 2011 The University of Sydney

#include "timer.h"

namespace snark {

timer::timer( const boost::posix_time::time_duration& duration ): _duration( duration ) {}

bool timer::expired() const { return _end == boost::posix_time::not_a_date_time ? false : ( boost::get_system_time() >= _end ); }

void timer::reset() { _end = boost::get_system_time() + _duration; }

} // namespace snark { 
