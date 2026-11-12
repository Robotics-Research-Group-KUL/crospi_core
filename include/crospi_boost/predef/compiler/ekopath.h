/*
Copyright Rene Rivera 2008-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_COMPILER_EKOPATH_H
#define CROSPI_BOOST_PREDEF_COMPILER_EKOPATH_H

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_COMP_PATH`

http://en.wikipedia.org/wiki/PathScale[EKOpath] compiler.
Version number available as major, minor, and patch.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__PATHCC__+` | {predef_detection}

| `+__PATHCC__+`, `+__PATHCC_MINOR__+`, `+__PATHCC_PATCHLEVEL__+` | V.R.P
|===
*/ // end::reference[]

#define CROSPI_BOOST_COMP_PATH CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__PATHCC__)
#   define CROSPI_BOOST_COMP_PATH_DETECTION \
        CROSPI_BOOST_VERSION_NUMBER(__PATHCC__,__PATHCC_MINOR__,__PATHCC_PATCHLEVEL__)
#endif

#ifdef CROSPI_BOOST_COMP_PATH_DETECTION
#   if defined(CROSPI_BOOST_PREDEF_DETAIL_COMP_DETECTED)
#       define CROSPI_BOOST_COMP_PATH_EMULATED CROSPI_BOOST_COMP_PATH_DETECTION
#   else
#       undef CROSPI_BOOST_COMP_PATH
#       define CROSPI_BOOST_COMP_PATH CROSPI_BOOST_COMP_PATH_DETECTION
#   endif
#   define CROSPI_BOOST_COMP_PATH_AVAILABLE
#   include <crospi_boost/predef/detail/comp_detected.h>
#endif

#define CROSPI_BOOST_COMP_PATH_NAME "EKOpath"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_COMP_PATH,CROSPI_BOOST_COMP_PATH_NAME)

#ifdef CROSPI_BOOST_COMP_PATH_EMULATED
#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_COMP_PATH_EMULATED,CROSPI_BOOST_COMP_PATH_NAME)
#endif
