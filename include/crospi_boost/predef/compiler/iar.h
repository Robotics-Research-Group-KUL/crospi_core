/*
Copyright Rene Rivera 2008-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_COMPILER_IAR_H
#define CROSPI_BOOST_PREDEF_COMPILER_IAR_H

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_COMP_IAR`

IAR C/{CPP} compiler.
Version number available as major, minor, and patch.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__IAR_SYSTEMS_ICC__+` | {predef_detection}

| `+__VER__+` | V.R.P
|===
*/ // end::reference[]

#define CROSPI_BOOST_COMP_IAR CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__IAR_SYSTEMS_ICC__)
#   define CROSPI_BOOST_COMP_IAR_DETECTION CROSPI_BOOST_PREDEF_MAKE_10_VVRR(__VER__)
#endif

#ifdef CROSPI_BOOST_COMP_IAR_DETECTION
#   if defined(CROSPI_BOOST_PREDEF_DETAIL_COMP_DETECTED)
#       define CROSPI_BOOST_COMP_IAR_EMULATED CROSPI_BOOST_COMP_IAR_DETECTION
#   else
#       undef CROSPI_BOOST_COMP_IAR
#       define CROSPI_BOOST_COMP_IAR CROSPI_BOOST_COMP_IAR_DETECTION
#   endif
#   define CROSPI_BOOST_COMP_IAR_AVAILABLE
#   include <crospi_boost/predef/detail/comp_detected.h>
#endif

#define CROSPI_BOOST_COMP_IAR_NAME "IAR C/C++"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_COMP_IAR,CROSPI_BOOST_COMP_IAR_NAME)

#ifdef CROSPI_BOOST_COMP_IAR_EMULATED
#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_COMP_IAR_EMULATED,CROSPI_BOOST_COMP_IAR_NAME)
#endif
