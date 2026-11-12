/*
Copyright Rene Rivera 2008-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_COMPILER_MPW_H
#define CROSPI_BOOST_PREDEF_COMPILER_MPW_H

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_COMP_MPW`

http://en.wikipedia.org/wiki/Macintosh_Programmer%27s_Workshop[MPW {CPP}] compiler.
Version number available as major, and minor.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__MRC__+` | {predef_detection}
| `MPW_C` | {predef_detection}
| `MPW_CPLUS` | {predef_detection}

| `+__MRC__+` | V.R.0
|===
*/ // end::reference[]

#define CROSPI_BOOST_COMP_MPW CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__MRC__) || defined(MPW_C) || defined(MPW_CPLUS)
#   if !defined(CROSPI_BOOST_COMP_MPW_DETECTION) && defined(__MRC__)
#       define CROSPI_BOOST_COMP_MPW_DETECTION CROSPI_BOOST_PREDEF_MAKE_0X_VVRR(__MRC__)
#   endif
#   if !defined(CROSPI_BOOST_COMP_MPW_DETECTION)
#       define CROSPI_BOOST_COMP_MPW_DETECTION CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#   endif
#endif

#ifdef CROSPI_BOOST_COMP_MPW_DETECTION
#   if defined(CROSPI_BOOST_PREDEF_DETAIL_COMP_DETECTED)
#       define CROSPI_BOOST_COMP_MPW_EMULATED CROSPI_BOOST_COMP_MPW_DETECTION
#   else
#       undef CROSPI_BOOST_COMP_MPW
#       define CROSPI_BOOST_COMP_MPW CROSPI_BOOST_COMP_MPW_DETECTION
#   endif
#   define CROSPI_BOOST_COMP_MPW_AVAILABLE
#   include <crospi_boost/predef/detail/comp_detected.h>
#endif

#define CROSPI_BOOST_COMP_MPW_NAME "MPW C++"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_COMP_MPW,CROSPI_BOOST_COMP_MPW_NAME)

#ifdef CROSPI_BOOST_COMP_MPW_EMULATED
#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_COMP_MPW_EMULATED,CROSPI_BOOST_COMP_MPW_NAME)
#endif
