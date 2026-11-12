/*
Copyright Rene Rivera 2008-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_COMPILER_WATCOM_H
#define CROSPI_BOOST_PREDEF_COMPILER_WATCOM_H

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_COMP_WATCOM`

http://en.wikipedia.org/wiki/Watcom[Watcom {CPP}] compiler.
Version number available as major, and minor.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__WATCOMC__+` | {predef_detection}

| `+__WATCOMC__+` | V.R.P
|===
*/ // end::reference[]

#define CROSPI_BOOST_COMP_WATCOM CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__WATCOMC__)
#   define CROSPI_BOOST_COMP_WATCOM_DETECTION CROSPI_BOOST_PREDEF_MAKE_10_VVRR(__WATCOMC__)
#endif

#ifdef CROSPI_BOOST_COMP_WATCOM_DETECTION
#   if defined(CROSPI_BOOST_PREDEF_DETAIL_COMP_DETECTED)
#       define CROSPI_BOOST_COMP_WATCOM_EMULATED CROSPI_BOOST_COMP_WATCOM_DETECTION
#   else
#       undef CROSPI_BOOST_COMP_WATCOM
#       define CROSPI_BOOST_COMP_WATCOM CROSPI_BOOST_COMP_WATCOM_DETECTION
#   endif
#   define CROSPI_BOOST_COMP_WATCOM_AVAILABLE
#   include <crospi_boost/predef/detail/comp_detected.h>
#endif

#define CROSPI_BOOST_COMP_WATCOM_NAME "Watcom C++"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_COMP_WATCOM,CROSPI_BOOST_COMP_WATCOM_NAME)

#ifdef CROSPI_BOOST_COMP_WATCOM_EMULATED
#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_COMP_WATCOM_EMULATED,CROSPI_BOOST_COMP_WATCOM_NAME)
#endif
