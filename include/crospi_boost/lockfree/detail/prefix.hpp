//  Copyright (C) 2009, 2016 Tim Blechmann
//
//  Distributed under the Boost Software License, Version 1.0. (See
//  accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt)

#ifndef CROSPI_BOOST_LOCKFREE_PREFIX_HPP_INCLUDED
#define CROSPI_BOOST_LOCKFREE_PREFIX_HPP_INCLUDED

#include <crospi_boost/predef.h>

/* this file defines the following macros:
   CROSPI_BOOST_LOCKFREE_PTR_COMPRESSION: use tag/pointer compression to utilize parts
                                   of the virtual address space as tag (at least 16bit)
*/

namespace crospi_boost { namespace lockfree { namespace detail {

#ifdef __cpp_inline_variables
#    define inline_constexpr inline
#else
#    define inline_constexpr
#endif

#if CROSPI_BOOST_ARCH_SYS390
inline_constexpr constexpr size_t cacheline_bytes = 256;
#elif CROSPI_BOOST_ARCH_PPC
inline_constexpr constexpr size_t cacheline_bytes = 128;
#elif CROSPI_BOOST_ARCH_ARM && ( CROSPI_BOOST_OS_MACOS || CROSPI_BOOST_OS_IOS )
// technically this is for apple's the M chips, but the A chip are probably similar
inline_constexpr constexpr size_t cacheline_bytes = 128;
#else
inline_constexpr constexpr size_t cacheline_bytes = 64;
#endif

}}} // namespace crospi_boost::lockfree::detail

#if CROSPI_BOOST_ARCH_X86_64 || ( ( CROSPI_BOOST_ARCH_ARM >= CROSPI_BOOST_VERSION_NUMBER( 8, 0, 0 ) ) && !CROSPI_BOOST_PLAT_ANDROID )
#    define CROSPI_BOOST_LOCKFREE_PTR_COMPRESSION 1
#endif

#undef inline_constexpr

#endif /* CROSPI_BOOST_LOCKFREE_PREFIX_HPP_INCLUDED */
