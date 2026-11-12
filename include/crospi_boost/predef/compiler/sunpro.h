/*
Copyright Rene Rivera 2008-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_COMPILER_SUNPRO_H
#define CROSPI_BOOST_PREDEF_COMPILER_SUNPRO_H

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_COMP_SUNPRO`

http://en.wikipedia.org/wiki/Oracle_Solaris_Studio[Oracle Solaris Studio] compiler.
Version number available as major, minor, and patch.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__SUNPRO_CC+` | {predef_detection}
| `+__SUNPRO_C+` | {predef_detection}

| `+__SUNPRO_CC+` | V.R.P
| `+__SUNPRO_C+` | V.R.P
| `+__SUNPRO_CC+` | VV.RR.P
| `+__SUNPRO_C+` | VV.RR.P
|===
*/ // end::reference[]

#define CROSPI_BOOST_COMP_SUNPRO CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__SUNPRO_CC) || defined(__SUNPRO_C)
#   if !defined(CROSPI_BOOST_COMP_SUNPRO_DETECTION) && defined(__SUNPRO_CC)
#       if (__SUNPRO_CC < 0x5100)
#           define CROSPI_BOOST_COMP_SUNPRO_DETECTION CROSPI_BOOST_PREDEF_MAKE_0X_VRP(__SUNPRO_CC)
#       else
#           define CROSPI_BOOST_COMP_SUNPRO_DETECTION CROSPI_BOOST_PREDEF_MAKE_0X_VVRRP(__SUNPRO_CC)
#       endif
#   endif
#   if !defined(CROSPI_BOOST_COMP_SUNPRO_DETECTION) && defined(__SUNPRO_C)
#       if (__SUNPRO_C < 0x5100)
#           define CROSPI_BOOST_COMP_SUNPRO_DETECTION CROSPI_BOOST_PREDEF_MAKE_0X_VRP(__SUNPRO_C)
#       else
#           define CROSPI_BOOST_COMP_SUNPRO_DETECTION CROSPI_BOOST_PREDEF_MAKE_0X_VVRRP(__SUNPRO_C)
#       endif
#   endif
#   if !defined(CROSPI_BOOST_COMP_SUNPRO_DETECTION)
#       define CROSPI_BOOST_COMP_SUNPRO_DETECTION CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#   endif
#endif

#ifdef CROSPI_BOOST_COMP_SUNPRO_DETECTION
#   if defined(CROSPI_BOOST_PREDEF_DETAIL_COMP_DETECTED)
#       define CROSPI_BOOST_COMP_SUNPRO_EMULATED CROSPI_BOOST_COMP_SUNPRO_DETECTION
#   else
#       undef CROSPI_BOOST_COMP_SUNPRO
#       define CROSPI_BOOST_COMP_SUNPRO CROSPI_BOOST_COMP_SUNPRO_DETECTION
#   endif
#   define CROSPI_BOOST_COMP_SUNPRO_AVAILABLE
#   include <crospi_boost/predef/detail/comp_detected.h>
#endif

#define CROSPI_BOOST_COMP_SUNPRO_NAME "Oracle Solaris Studio"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_COMP_SUNPRO,CROSPI_BOOST_COMP_SUNPRO_NAME)

#ifdef CROSPI_BOOST_COMP_SUNPRO_EMULATED
#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_COMP_SUNPRO_EMULATED,CROSPI_BOOST_COMP_SUNPRO_NAME)
#endif
