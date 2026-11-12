/*
Copyright Rene Rivera 2008-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_COMPILER_SGI_MIPSPRO_H
#define CROSPI_BOOST_PREDEF_COMPILER_SGI_MIPSPRO_H

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_COMP_SGI`

http://en.wikipedia.org/wiki/MIPSpro[SGI MIPSpro] compiler.
Version number available as major, minor, and patch.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__sgi+` | {predef_detection}
| `sgi` | {predef_detection}

| `+_SGI_COMPILER_VERSION+` | V.R.P
| `+_COMPILER_VERSION+` | V.R.P
|===
*/ // end::reference[]

#define CROSPI_BOOST_COMP_SGI CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__sgi) || defined(sgi)
#   if !defined(CROSPI_BOOST_COMP_SGI_DETECTION) && defined(_SGI_COMPILER_VERSION)
#       define CROSPI_BOOST_COMP_SGI_DETECTION CROSPI_BOOST_PREDEF_MAKE_10_VRP(_SGI_COMPILER_VERSION)
#   endif
#   if !defined(CROSPI_BOOST_COMP_SGI_DETECTION) && defined(_COMPILER_VERSION)
#       define CROSPI_BOOST_COMP_SGI_DETECTION CROSPI_BOOST_PREDEF_MAKE_10_VRP(_COMPILER_VERSION)
#   endif
#   if !defined(CROSPI_BOOST_COMP_SGI_DETECTION)
#       define CROSPI_BOOST_COMP_SGI_DETECTION CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#   endif
#endif

#ifdef CROSPI_BOOST_COMP_SGI_DETECTION
#   if defined(CROSPI_BOOST_PREDEF_DETAIL_COMP_DETECTED)
#       define CROSPI_BOOST_COMP_SGI_EMULATED CROSPI_BOOST_COMP_SGI_DETECTION
#   else
#       undef CROSPI_BOOST_COMP_SGI
#       define CROSPI_BOOST_COMP_SGI CROSPI_BOOST_COMP_SGI_DETECTION
#   endif
#   define CROSPI_BOOST_COMP_SGI_AVAILABLE
#   include <crospi_boost/predef/detail/comp_detected.h>
#endif

#define CROSPI_BOOST_COMP_SGI_NAME "SGI MIPSpro"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_COMP_SGI,CROSPI_BOOST_COMP_SGI_NAME)

#ifdef CROSPI_BOOST_COMP_SGI_EMULATED
#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_COMP_SGI_EMULATED,CROSPI_BOOST_COMP_SGI_NAME)
#endif
