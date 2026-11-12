/*
Copyright Rene Rivera 2008-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_ARCHITECTURE_SYS370_H
#define CROSPI_BOOST_PREDEF_ARCHITECTURE_SYS370_H

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_ARCH_SYS370`

http://en.wikipedia.org/wiki/System/370[System/370] architecture.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__370__+` | {predef_detection}
| `+__THW_370__+` | {predef_detection}
|===
*/ // end::reference[]

#define CROSPI_BOOST_ARCH_SYS370 CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__370__) || defined(__THW_370__)
#   undef CROSPI_BOOST_ARCH_SYS370
#   define CROSPI_BOOST_ARCH_SYS370 CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#endif

#if CROSPI_BOOST_ARCH_SYS370
#   define CROSPI_BOOST_ARCH_SYS370_AVAILABLE
#endif

#if CROSPI_BOOST_ARCH_SYS370
#   undef CROSPI_BOOST_ARCH_WORD_BITS_32
#   define CROSPI_BOOST_ARCH_WORD_BITS_32 CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#endif

#define CROSPI_BOOST_ARCH_SYS370_NAME "System/370"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_ARCH_SYS370,CROSPI_BOOST_ARCH_SYS370_NAME)
