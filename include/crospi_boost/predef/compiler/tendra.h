/*
Copyright Rene Rivera 2008-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_COMPILER_TENDRA_H
#define CROSPI_BOOST_PREDEF_COMPILER_TENDRA_H

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_COMP_TENDRA`

http://en.wikipedia.org/wiki/TenDRA_Compiler[TenDRA C/{CPP}] compiler.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__TenDRA__+` | {predef_detection}
|===
*/ // end::reference[]

#define CROSPI_BOOST_COMP_TENDRA CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__TenDRA__)
#   define CROSPI_BOOST_COMP_TENDRA_DETECTION CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#endif

#ifdef CROSPI_BOOST_COMP_TENDRA_DETECTION
#   if defined(CROSPI_BOOST_PREDEF_DETAIL_COMP_DETECTED)
#       define CROSPI_BOOST_COMP_TENDRA_EMULATED CROSPI_BOOST_COMP_TENDRA_DETECTION
#   else
#       undef CROSPI_BOOST_COMP_TENDRA
#       define CROSPI_BOOST_COMP_TENDRA CROSPI_BOOST_COMP_TENDRA_DETECTION
#   endif
#   define CROSPI_BOOST_COMP_TENDRA_AVAILABLE
#   include <crospi_boost/predef/detail/comp_detected.h>
#endif

#define CROSPI_BOOST_COMP_TENDRA_NAME "TenDRA C/C++"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_COMP_TENDRA,CROSPI_BOOST_COMP_TENDRA_NAME)

#ifdef CROSPI_BOOST_COMP_TENDRA_EMULATED
#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_COMP_TENDRA_EMULATED,CROSPI_BOOST_COMP_TENDRA_NAME)
#endif
