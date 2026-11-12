/*
Copyright Rene Rivera 2008-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_ARCHITECTURE_Z_H
#define CROSPI_BOOST_PREDEF_ARCHITECTURE_Z_H

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_ARCH_Z`

http://en.wikipedia.org/wiki/Z/Architecture[z/Architecture] architecture.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__SYSC_ZARCH__+` | {predef_detection}
|===
*/ // end::reference[]

#define CROSPI_BOOST_ARCH_Z CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__SYSC_ZARCH__)
#   undef CROSPI_BOOST_ARCH_Z
#   define CROSPI_BOOST_ARCH_Z CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#endif

#if CROSPI_BOOST_ARCH_Z
#   define CROSPI_BOOST_ARCH_Z_AVAILABLE
#endif

#if CROSPI_BOOST_ARCH_Z
#   undef CROSPI_BOOST_ARCH_WORD_BITS_64
#   define CROSPI_BOOST_ARCH_WORD_BITS_64 CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#endif

#define CROSPI_BOOST_ARCH_Z_NAME "z/Architecture"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_ARCH_Z,CROSPI_BOOST_ARCH_Z_NAME)
