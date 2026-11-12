//  abi_sufffix header  -------------------------------------------------------//

// (c) Copyright John Maddock 2003
   
// Use, modification and distribution are subject to the Boost Software License,
// Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt).

// This header should be #included AFTER code that was preceded by a #include
// <crospi_boost/config/abi_prefix.hpp>.

#ifndef CROSPI_BOOST_CONFIG_ABI_PREFIX_HPP
# error Header crospi_boost/config/abi_suffix.hpp must only be used after crospi_boost/config/abi_prefix.hpp
#else
# undef CROSPI_BOOST_CONFIG_ABI_PREFIX_HPP
#endif

// the suffix header occurs after all of our code:
#ifdef CROSPI_BOOST_HAS_ABI_HEADERS
#  include CROSPI_BOOST_ABI_SUFFIX
#endif

#if defined( CROSPI_BOOST_BORLANDC )
#pragma nopushoptwarn
#endif
