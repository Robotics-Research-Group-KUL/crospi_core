//  (C) Copyright Jessica Hamilton 2014.
//  Use, modification and distribution are subject to the
//  Boost Software License, Version 1.0. (See accompanying file
//  LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)

//  See http://www.boost.org for most recent version.

//  Haiku specific config options:

#define CROSPI_BOOST_PLATFORM "Haiku"

#define CROSPI_BOOST_HAS_UNISTD_H
#define CROSPI_BOOST_HAS_STDINT_H

#ifndef CROSPI_BOOST_DISABLE_THREADS
#  define CROSPI_BOOST_HAS_THREADS
#endif

#define CROSPI_BOOST_NO_CXX11_HDR_TYPE_TRAITS
#define CROSPI_BOOST_NO_CXX11_ATOMIC_SMART_PTR
#define CROSPI_BOOST_NO_CXX11_STATIC_ASSERT
#define CROSPI_BOOST_NO_CXX11_VARIADIC_MACROS

//
// thread API's not auto detected:
//
#define CROSPI_BOOST_HAS_SCHED_YIELD
#define CROSPI_BOOST_HAS_GETTIMEOFDAY

// boilerplate code:
#include <crospi_boost/config/detail/posix_features.hpp>
