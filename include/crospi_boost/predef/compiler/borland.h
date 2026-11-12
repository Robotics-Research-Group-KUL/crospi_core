/*
Copyright Rene Rivera 2008-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_COMPILER_BORLAND_H
#define CROSPI_BOOST_PREDEF_COMPILER_BORLAND_H

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_COMP_BORLAND`

http://en.wikipedia.org/wiki/C_plus_plus_builder[Borland {CPP}] compiler.
Version number available as major, minor, and patch.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__BORLANDC__+` | {predef_detection}
| `+__CODEGEARC__+` | {predef_detection}

| `+__BORLANDC__+` | V.R.P
| `+__CODEGEARC__+` | V.R.P
|===
*/ // end::reference[]

#define CROSPI_BOOST_COMP_BORLAND CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__BORLANDC__) || defined(__CODEGEARC__)
#   if !defined(CROSPI_BOOST_COMP_BORLAND_DETECTION) && (defined(__CODEGEARC__))
#       define CROSPI_BOOST_COMP_BORLAND_DETECTION CROSPI_BOOST_PREDEF_MAKE_0X_VVRP(__CODEGEARC__)
#   endif
#   if !defined(CROSPI_BOOST_COMP_BORLAND_DETECTION)
#       define CROSPI_BOOST_COMP_BORLAND_DETECTION CROSPI_BOOST_PREDEF_MAKE_0X_VVRP(__BORLANDC__)
#   endif
#endif

#ifdef CROSPI_BOOST_COMP_BORLAND_DETECTION
#   define CROSPI_BOOST_COMP_BORLAND_AVAILABLE
#   if defined(CROSPI_BOOST_PREDEF_DETAIL_COMP_DETECTED)
#       define CROSPI_BOOST_COMP_BORLAND_EMULATED CROSPI_BOOST_COMP_BORLAND_DETECTION
#   else
#       undef CROSPI_BOOST_COMP_BORLAND
#       define CROSPI_BOOST_COMP_BORLAND CROSPI_BOOST_COMP_BORLAND_DETECTION
#   endif
#   include <crospi_boost/predef/detail/comp_detected.h>
#endif

#define CROSPI_BOOST_COMP_BORLAND_NAME "Borland C++"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_COMP_BORLAND,CROSPI_BOOST_COMP_BORLAND_NAME)

#ifdef CROSPI_BOOST_COMP_BORLAND_EMULATED
#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_COMP_BORLAND_EMULATED,CROSPI_BOOST_COMP_BORLAND_NAME)
#endif
