/*
Copyright Rene Rivera 2008-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_COMPILER_DIAB_H
#define CROSPI_BOOST_PREDEF_COMPILER_DIAB_H

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_COMP_DIAB`

http://www.windriver.com/products/development_suite/wind_river_compiler/[Diab C/{CPP}] compiler.
Version number available as major, minor, and patch.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__DCC__+` | {predef_detection}

| `+__VERSION_NUMBER__+` | V.R.P
|===
*/ // end::reference[]

#define CROSPI_BOOST_COMP_DIAB CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__DCC__)
#   define CROSPI_BOOST_COMP_DIAB_DETECTION CROSPI_BOOST_PREDEF_MAKE_10_VRPP(__VERSION_NUMBER__)
#endif

#ifdef CROSPI_BOOST_COMP_DIAB_DETECTION
#   if defined(CROSPI_BOOST_PREDEF_DETAIL_COMP_DETECTED)
#       define CROSPI_BOOST_COMP_DIAB_EMULATED CROSPI_BOOST_COMP_DIAB_DETECTION
#   else
#       undef CROSPI_BOOST_COMP_DIAB
#       define CROSPI_BOOST_COMP_DIAB CROSPI_BOOST_COMP_DIAB_DETECTION
#   endif
#   define CROSPI_BOOST_COMP_DIAB_AVAILABLE
#   include <crospi_boost/predef/detail/comp_detected.h>
#endif

#define CROSPI_BOOST_COMP_DIAB_NAME "Diab C/C++"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_COMP_DIAB,CROSPI_BOOST_COMP_DIAB_NAME)

#ifdef CROSPI_BOOST_COMP_DIAB_EMULATED
#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_COMP_DIAB_EMULATED,CROSPI_BOOST_COMP_DIAB_NAME)
#endif
