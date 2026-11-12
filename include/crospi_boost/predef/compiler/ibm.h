/*
Copyright Rene Rivera 2008-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_COMPILER_IBM_H
#define CROSPI_BOOST_PREDEF_COMPILER_IBM_H

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_COMP_IBM`

http://en.wikipedia.org/wiki/VisualAge[IBM XL C/{CPP}] compiler.
Version number available as major, minor, and patch.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__IBMCPP__+` | {predef_detection}
| `+__xlC__+` | {predef_detection}
| `+__xlc__+` | {predef_detection}

| `+__COMPILER_VER__+` | V.R.P
| `+__xlC__+` | V.R.P
| `+__xlc__+` | V.R.P
| `+__IBMCPP__+` | V.R.P
|===
*/ // end::reference[]

#define CROSPI_BOOST_COMP_IBM CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__IBMCPP__) || defined(__xlC__) || defined(__xlc__)
#   if !defined(CROSPI_BOOST_COMP_IBM_DETECTION) && defined(__COMPILER_VER__)
#       define CROSPI_BOOST_COMP_IBM_DETECTION CROSPI_BOOST_PREDEF_MAKE_0X_VRRPPPP(__COMPILER_VER__)
#   endif
#   if !defined(CROSPI_BOOST_COMP_IBM_DETECTION) && defined(__xlC__)
#       define CROSPI_BOOST_COMP_IBM_DETECTION CROSPI_BOOST_PREDEF_MAKE_0X_VVRR(__xlC__)
#   endif
#   if !defined(CROSPI_BOOST_COMP_IBM_DETECTION) && defined(__xlc__)
#       define CROSPI_BOOST_COMP_IBM_DETECTION CROSPI_BOOST_PREDEF_MAKE_0X_VVRR(__xlc__)
#   endif
#   if !defined(CROSPI_BOOST_COMP_IBM_DETECTION)
#       define CROSPI_BOOST_COMP_IBM_DETECTION CROSPI_BOOST_PREDEF_MAKE_10_VRP(__IBMCPP__)
#   endif
#endif

#ifdef CROSPI_BOOST_COMP_IBM_DETECTION
#   if defined(CROSPI_BOOST_PREDEF_DETAIL_COMP_DETECTED)
#       define CROSPI_BOOST_COMP_IBM_EMULATED CROSPI_BOOST_COMP_IBM_DETECTION
#   else
#       undef CROSPI_BOOST_COMP_IBM
#       define CROSPI_BOOST_COMP_IBM CROSPI_BOOST_COMP_IBM_DETECTION
#   endif
#   define CROSPI_BOOST_COMP_IBM_AVAILABLE
#   include <crospi_boost/predef/detail/comp_detected.h>
#endif

#define CROSPI_BOOST_COMP_IBM_NAME "IBM XL C/C++"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_COMP_IBM,CROSPI_BOOST_COMP_IBM_NAME)

#ifdef CROSPI_BOOST_COMP_IBM_EMULATED
#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_COMP_IBM_EMULATED,CROSPI_BOOST_COMP_IBM_NAME)
#endif
