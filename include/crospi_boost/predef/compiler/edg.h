/*
Copyright Rene Rivera 2008-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_COMPILER_EDG_H
#define CROSPI_BOOST_PREDEF_COMPILER_EDG_H

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_COMP_EDG`

http://en.wikipedia.org/wiki/Edison_Design_Group[EDG {CPP} Frontend] compiler.
Version number available as major, minor, and patch.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__EDG__+` | {predef_detection}

| `+__EDG_VERSION__+` | V.R.0
|===
*/ // end::reference[]

#define CROSPI_BOOST_COMP_EDG CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__EDG__)
#   define CROSPI_BOOST_COMP_EDG_DETECTION CROSPI_BOOST_PREDEF_MAKE_10_VRR(__EDG_VERSION__)
#endif

#ifdef CROSPI_BOOST_COMP_EDG_DETECTION
#   if defined(CROSPI_BOOST_PREDEF_DETAIL_COMP_DETECTED)
#       define CROSPI_BOOST_COMP_EDG_EMULATED CROSPI_BOOST_COMP_EDG_DETECTION
#   else
#       undef CROSPI_BOOST_COMP_EDG
#       define CROSPI_BOOST_COMP_EDG CROSPI_BOOST_COMP_EDG_DETECTION
#   endif
#   define CROSPI_BOOST_COMP_EDG_AVAILABLE
#   include <crospi_boost/predef/detail/comp_detected.h>
#endif

#define CROSPI_BOOST_COMP_EDG_NAME "EDG C++ Frontend"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_COMP_EDG,CROSPI_BOOST_COMP_EDG_NAME)

#ifdef CROSPI_BOOST_COMP_EDG_EMULATED
#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_COMP_EDG_EMULATED,CROSPI_BOOST_COMP_EDG_NAME)
#endif
