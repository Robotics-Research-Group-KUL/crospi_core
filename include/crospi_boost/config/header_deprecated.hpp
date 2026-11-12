#ifndef CROSPI_BOOST_CONFIG_HEADER_DEPRECATED_HPP_INCLUDED
#define CROSPI_BOOST_CONFIG_HEADER_DEPRECATED_HPP_INCLUDED

//  Copyright 2017 Peter Dimov.
//
//  Distributed under the Boost Software License, Version 1.0.
//
//  See accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt
//
//  CROSPI_BOOST_HEADER_DEPRECATED("<alternative>")
//
//  Expands to the equivalent of
//    CROSPI_BOOST_PRAGMA_MESSAGE("This header is deprecated. Use <alternative> instead.")
//
//  Note that this header is C compatible.

#include <crospi_boost/config/pragma_message.hpp>

#if defined(CROSPI_BOOST_ALLOW_DEPRECATED_HEADERS) || defined(CROSPI_BOOST_ALLOW_DEPRECATED)
# define CROSPI_BOOST_HEADER_DEPRECATED(a)
#else
# define CROSPI_BOOST_HEADER_DEPRECATED(a) CROSPI_BOOST_PRAGMA_MESSAGE("This header is deprecated. Use " a " instead.")
#endif

#endif // CROSPI_BOOST_CONFIG_HEADER_DEPRECATED_HPP_INCLUDED
