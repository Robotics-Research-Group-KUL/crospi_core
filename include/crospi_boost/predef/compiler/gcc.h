/*
Copyright Rene Rivera 2008-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_COMPILER_GCC_H
#define CROSPI_BOOST_PREDEF_COMPILER_GCC_H

/* Other compilers that emulate this one need to be detected first. */

#include <crospi_boost/predef/compiler/clang.h>

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_COMP_GNUC`

http://en.wikipedia.org/wiki/GNU_Compiler_Collection[Gnu GCC C/{CPP}] compiler.
Version number available as major, minor, and patch (if available).

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__GNUC__+` | {predef_detection}

| `+__GNUC__+`, `+__GNUC_MINOR__+`, `+__GNUC_PATCHLEVEL__+` | V.R.P
| `+__GNUC__+`, `+__GNUC_MINOR__+` | V.R.0
|===
*/ // end::reference[]

#define CROSPI_BOOST_COMP_GNUC CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__GNUC__)
#   if !defined(CROSPI_BOOST_COMP_GNUC_DETECTION) && defined(__GNUC_PATCHLEVEL__)
#       define CROSPI_BOOST_COMP_GNUC_DETECTION \
            CROSPI_BOOST_VERSION_NUMBER(__GNUC__,__GNUC_MINOR__,__GNUC_PATCHLEVEL__)
#   endif
#   if !defined(CROSPI_BOOST_COMP_GNUC_DETECTION)
#       define CROSPI_BOOST_COMP_GNUC_DETECTION \
            CROSPI_BOOST_VERSION_NUMBER(__GNUC__,__GNUC_MINOR__,0)
#   endif
#endif

#ifdef CROSPI_BOOST_COMP_GNUC_DETECTION
#   if defined(CROSPI_BOOST_PREDEF_DETAIL_COMP_DETECTED)
#       define CROSPI_BOOST_COMP_GNUC_EMULATED CROSPI_BOOST_COMP_GNUC_DETECTION
#   else
#       undef CROSPI_BOOST_COMP_GNUC
#       define CROSPI_BOOST_COMP_GNUC CROSPI_BOOST_COMP_GNUC_DETECTION
#   endif
#   define CROSPI_BOOST_COMP_GNUC_AVAILABLE
#   include <crospi_boost/predef/detail/comp_detected.h>
#endif

#define CROSPI_BOOST_COMP_GNUC_NAME "Gnu GCC C/C++"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_COMP_GNUC,CROSPI_BOOST_COMP_GNUC_NAME)

#ifdef CROSPI_BOOST_COMP_GNUC_EMULATED
#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_COMP_GNUC_EMULATED,CROSPI_BOOST_COMP_GNUC_NAME)
#endif
