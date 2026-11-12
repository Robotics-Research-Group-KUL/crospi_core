/*
Copyright Rene Rivera 2011-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_ARCHITECTURE_PYRAMID_H
#define CROSPI_BOOST_PREDEF_ARCHITECTURE_PYRAMID_H

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_ARCH_PYRAMID`

Pyramid 9810 architecture.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `pyr` | {predef_detection}
|===
*/ // end::reference[]

#define CROSPI_BOOST_ARCH_PYRAMID CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(pyr)
#   undef CROSPI_BOOST_ARCH_PYRAMID
#   define CROSPI_BOOST_ARCH_PYRAMID CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#endif

#if CROSPI_BOOST_ARCH_PYRAMID
#   define CROSPI_BOOST_ARCH_PYRAMID_AVAILABLE
#endif

#if CROSPI_BOOST_ARCH_PYRAMID
#   undef CROSPI_BOOST_ARCH_WORD_BITS_32
#   define CROSPI_BOOST_ARCH_WORD_BITS_32 CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#endif

#define CROSPI_BOOST_ARCH_PYRAMID_NAME "Pyramid 9810"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_ARCH_PYRAMID,CROSPI_BOOST_ARCH_PYRAMID_NAME)
