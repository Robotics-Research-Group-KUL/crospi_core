#ifndef CROSPI_BOOST_CONFIG_PRAGMA_MESSAGE_HPP_INCLUDED
#define CROSPI_BOOST_CONFIG_PRAGMA_MESSAGE_HPP_INCLUDED

//  Copyright 2017 Peter Dimov.
//
//  Distributed under the Boost Software License, Version 1.0.
//
//  See accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt
//
//  CROSPI_BOOST_PRAGMA_MESSAGE("message")
//
//  Expands to the equivalent of #pragma message("message")
//
//  Note that this header is C compatible.

#include <crospi_boost/config/helper_macros.hpp>

#if defined(CROSPI_BOOST_DISABLE_PRAGMA_MESSAGE)
# define CROSPI_BOOST_PRAGMA_MESSAGE(x)
#elif defined(__INTEL_COMPILER)
# define CROSPI_BOOST_PRAGMA_MESSAGE(x) __pragma(message(__FILE__ "(" CROSPI_BOOST_STRINGIZE(__LINE__) "): note: " x))
#elif defined(__GNUC__)
# define CROSPI_BOOST_PRAGMA_MESSAGE(x) _Pragma(CROSPI_BOOST_STRINGIZE(message(x)))
#elif defined(_MSC_VER)
# define CROSPI_BOOST_PRAGMA_MESSAGE(x) __pragma(message(__FILE__ "(" CROSPI_BOOST_STRINGIZE(__LINE__) "): note: " x))
#else
# define CROSPI_BOOST_PRAGMA_MESSAGE(x)
#endif

#endif // CROSPI_BOOST_CONFIG_PRAGMA_MESSAGE_HPP_INCLUDED
