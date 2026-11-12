//  (C) Copyright Jim Douglas 2005. 
//  Use, modification and distribution are subject to the 
//  Boost Software License, Version 1.0. (See accompanying file 
//  LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)

//  See http://www.boost.org for most recent version.

//  QNX specific config options:

#define CROSPI_BOOST_PLATFORM "QNX"

#define CROSPI_BOOST_HAS_UNISTD_H
#include <crospi_boost/config/detail/posix_features.hpp>

// QNX claims XOpen version 5 compatibility, but doesn't have an nl_types.h
// or log1p and expm1:
#undef  CROSPI_BOOST_HAS_NL_TYPES_H
#undef  CROSPI_BOOST_HAS_LOG1P
#undef  CROSPI_BOOST_HAS_EXPM1

#define CROSPI_BOOST_HAS_PTHREADS
#define CROSPI_BOOST_HAS_PTHREAD_MUTEXATTR_SETTYPE

#define CROSPI_BOOST_HAS_GETTIMEOFDAY
#define CROSPI_BOOST_HAS_CLOCK_GETTIME
#define CROSPI_BOOST_HAS_NANOSLEEP





