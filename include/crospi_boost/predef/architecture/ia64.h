/*
Copyright Rene Rivera 2008-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_ARCHITECTURE_IA64_H
#define CROSPI_BOOST_PREDEF_ARCHITECTURE_IA64_H

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_ARCH_IA64`

http://en.wikipedia.org/wiki/Ia64[Intel Itanium 64] architecture.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__ia64__+` | {predef_detection}
| `+_IA64+` | {predef_detection}
| `+__IA64__+` | {predef_detection}
| `+__ia64+` | {predef_detection}
| `+_M_IA64+` | {predef_detection}
| `+__itanium__+` | {predef_detection}
|===
*/ // end::reference[]

#define CROSPI_BOOST_ARCH_IA64 CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__ia64__) || defined(_IA64) || \
    defined(__IA64__) || defined(__ia64) || \
    defined(_M_IA64) || defined(__itanium__)
#   undef CROSPI_BOOST_ARCH_IA64
#   define CROSPI_BOOST_ARCH_IA64 CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#endif

#if CROSPI_BOOST_ARCH_IA64
#   define CROSPI_BOOST_ARCH_IA64_AVAILABLE
#endif

#if CROSPI_BOOST_ARCH_IA64
#   undef CROSPI_BOOST_ARCH_WORD_BITS_64
#   define CROSPI_BOOST_ARCH_WORD_BITS_64 CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#endif

#define CROSPI_BOOST_ARCH_IA64_NAME "Intel Itanium 64"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_ARCH_IA64,CROSPI_BOOST_ARCH_IA64_NAME)
