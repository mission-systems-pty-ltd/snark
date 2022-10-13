// Copyright (c) 2011 The University of Sydney

#pragma once

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread_time.hpp>

namespace snark {

class timer // todo: add chrono support
{
public:
    timer( const boost::posix_time::time_duration& duration );
    void reset();
    bool expired() const;
private:
    boost::posix_time::time_duration _duration;
    boost::posix_time::ptime _end;
};

} // namespace snrk {
