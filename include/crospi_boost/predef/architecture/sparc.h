/*
Copyright Rene Rivera 2008-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_ARCHITECTURE_SPARC_H
#define CROSPI_BOOST_PREDEF_ARCHITECTURE_SPARC_H

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_ARCH_SPARC`

http://en.wikipedia.org/wiki/SPARC[SPARC] architecture.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__sparc__+` | {predef_detection}
| `+__sparc+` | {predef_detection}

| `+__sparcv9+` | 9.0.0
| `+__sparc_v9__+` | 9.0.0
| `+__sparcv8+` | 8.0.0
| `+__sparc_v8__+` | 8.0.0
|===
*/ // end::reference[]

#define CROSPI_BOOST_ARCH_SPARC CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__sparc__) || defined(__sparc)
#   undef CROSPI_BOOST_ARCH_SPARC
#   if !defined(CROSPI_BOOST_ARCH_SPARC) && (defined(__sparcv9) || defined(__sparc_v9__))
#       define CROSPI_BOOST_ARCH_SPARC CROSPI_BOOST_VERSION_NUMBER(9,0,0)
#   endif
#   if !defined(CROSPI_BOOST_ARCH_SPARC) && (defined(__sparcv8) || defined(__sparc_v8__))
#       define CROSPI_BOOST_ARCH_SPARC CROSPI_BOOST_VERSION_NUMBER(8,0,0)
#   endif
#   if !defined(CROSPI_BOOST_ARCH_SPARC)
#       define CROSPI_BOOST_ARCH_SPARC CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#   endif
#endif

#if CROSPI_BOOST_ARCH_SPARC
#   define CROSPI_BOOST_ARCH_SPARC_AVAILABLE
#endif

#if CROSPI_BOOST_ARCH_SPARC
#   if CROSPI_BOOST_ARCH_SPARC >= CROSPI_BOOST_VERSION_NUMBER(9,0,0)
#       undef CROSPI_BOOST_ARCH_WORD_BITS_64
#       define CROSPI_BOOST_ARCH_WORD_BITS_64 CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#   else
#       undef CROSPI_BOOST_ARCH_WORD_BITS_32
#       define CROSPI_BOOST_ARCH_WORD_BITS_32 CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#   endif
#endif

#define CROSPI_BOOST_ARCH_SPARC_NAME "SPARC"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_ARCH_SPARC,CROSPI_BOOST_ARCH_SPARC_NAME)
