/*
Copyright Rene Rivera 2008-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_COMPILER_DIGNUS_H
#define CROSPI_BOOST_PREDEF_COMPILER_DIGNUS_H

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_COMP_SYSC`

http://www.dignus.com/dcxx/[Dignus Systems/{CPP}] compiler.
Version number available as major, minor, and patch.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__SYSC__+` | {predef_detection}

| `+__SYSC_VER__+` | V.R.P
|===
*/ // end::reference[]

#define CROSPI_BOOST_COMP_SYSC CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__SYSC__)
#   define CROSPI_BOOST_COMP_SYSC_DETECTION CROSPI_BOOST_PREDEF_MAKE_10_VRRPP(__SYSC_VER__)
#endif

#ifdef CROSPI_BOOST_COMP_SYSC_DETECTION
#   if defined(CROSPI_BOOST_PREDEF_DETAIL_COMP_DETECTED)
#       define CROSPI_BOOST_COMP_SYSC_EMULATED CROSPI_BOOST_COMP_SYSC_DETECTION
#   else
#       undef CROSPI_BOOST_COMP_SYSC
#       define CROSPI_BOOST_COMP_SYSC CROSPI_BOOST_COMP_SYSC_DETECTION
#   endif
#   define CROSPI_BOOST_COMP_SYSC_AVAILABLE
#   include <crospi_boost/predef/detail/comp_detected.h>
#endif

#define CROSPI_BOOST_COMP_SYSC_NAME "Dignus Systems/C++"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_COMP_SYSC,CROSPI_BOOST_COMP_SYSC_NAME)

#ifdef CROSPI_BOOST_COMP_SYSC_EMULATED
#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_COMP_SYSC_EMULATED,CROSPI_BOOST_COMP_SYSC_NAME)
#endif
