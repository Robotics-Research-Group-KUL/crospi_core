/*
Copyright Rene Rivera 2008-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_COMPILER_DIGITALMARS_H
#define CROSPI_BOOST_PREDEF_COMPILER_DIGITALMARS_H

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_COMP_DMC`

http://en.wikipedia.org/wiki/Digital_Mars[Digital Mars] compiler.
Version number available as major, minor, and patch.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__DMC__+` | {predef_detection}

| `+__DMC__+` | V.R.P
|===
*/ // end::reference[]

#define CROSPI_BOOST_COMP_DMC CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__DMC__)
#   define CROSPI_BOOST_COMP_DMC_DETECTION CROSPI_BOOST_PREDEF_MAKE_0X_VRP(__DMC__)
#endif

#ifdef CROSPI_BOOST_COMP_DMC_DETECTION
#   if defined(CROSPI_BOOST_PREDEF_DETAIL_COMP_DETECTED)
#       define CROSPI_BOOST_COMP_DMC_EMULATED CROSPI_BOOST_COMP_DMC_DETECTION
#   else
#       undef CROSPI_BOOST_COMP_DMC
#       define CROSPI_BOOST_COMP_DMC CROSPI_BOOST_COMP_DMC_DETECTION
#   endif
#   define CROSPI_BOOST_COMP_DMC_AVAILABLE
#   include <crospi_boost/predef/detail/comp_detected.h>
#endif

#define CROSPI_BOOST_COMP_DMC_NAME "Digital Mars"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_COMP_DMC,CROSPI_BOOST_COMP_DMC_NAME)

#ifdef CROSPI_BOOST_COMP_DMC_EMULATED
#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_COMP_DMC_EMULATED,CROSPI_BOOST_COMP_DMC_NAME)
#endif
