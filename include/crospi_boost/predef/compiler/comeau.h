/*
Copyright Rene Rivera 2008-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_COMPILER_COMEAU_H
#define CROSPI_BOOST_PREDEF_COMPILER_COMEAU_H

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

#define CROSPI_BOOST_COMP_COMO CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

/* tag::reference[]
= `CROSPI_BOOST_COMP_COMO`

http://en.wikipedia.org/wiki/Comeau_C/C%2B%2B[Comeau {CPP}] compiler.
Version number available as major, minor, and patch.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__COMO__+` | {predef_detection}

| `+__COMO_VERSION__+` | V.R.P
|===
*/ // end::reference[]

#if defined(__COMO__)
#   if !defined(CROSPI_BOOST_COMP_COMO_DETECTION) && defined(__COMO_VERSION__)
#       define CROSPI_BOOST_COMP_COMO_DETECTION CROSPI_BOOST_PREDEF_MAKE_0X_VRP(__COMO_VERSION__)
#   endif
#   if !defined(CROSPI_BOOST_COMP_COMO_DETECTION)
#       define CROSPI_BOOST_COMP_COMO_DETECTION CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#   endif
#endif

#ifdef CROSPI_BOOST_COMP_COMO_DETECTION
#   if defined(CROSPI_BOOST_PREDEF_DETAIL_COMP_DETECTED)
#       define CROSPI_BOOST_COMP_COMO_EMULATED CROSPI_BOOST_COMP_COMO_DETECTION
#   else
#       undef CROSPI_BOOST_COMP_COMO
#       define CROSPI_BOOST_COMP_COMO CROSPI_BOOST_COMP_COMO_DETECTION
#   endif
#   define CROSPI_BOOST_COMP_COMO_AVAILABLE
#   include <crospi_boost/predef/detail/comp_detected.h>
#endif

#define CROSPI_BOOST_COMP_COMO_NAME "Comeau C++"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_COMP_COMO,CROSPI_BOOST_COMP_COMO_NAME)

#ifdef CROSPI_BOOST_COMP_COMO_EMULATED
#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_COMP_COMO_EMULATED,CROSPI_BOOST_COMP_COMO_NAME)
#endif
