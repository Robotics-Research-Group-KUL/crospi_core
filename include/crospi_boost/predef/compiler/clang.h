/*
Copyright Rene Rivera 2008-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_COMPILER_CLANG_H
#define CROSPI_BOOST_PREDEF_COMPILER_CLANG_H

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_COMP_CLANG`

http://en.wikipedia.org/wiki/Clang[Clang] compiler.
Version number available as major, minor, and patch.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__clang__+` | {predef_detection}

| `+__clang_major__+`, `+__clang_minor__+`, `+__clang_patchlevel__+` | V.R.P
|===
*/ // end::reference[]

#define CROSPI_BOOST_COMP_CLANG CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__clang__)
#   define CROSPI_BOOST_COMP_CLANG_DETECTION CROSPI_BOOST_VERSION_NUMBER(__clang_major__,__clang_minor__,__clang_patchlevel__)
#endif

#ifdef CROSPI_BOOST_COMP_CLANG_DETECTION
#   if defined(CROSPI_BOOST_PREDEF_DETAIL_COMP_DETECTED)
#       define CROSPI_BOOST_COMP_CLANG_EMULATED CROSPI_BOOST_COMP_CLANG_DETECTION
#   else
#       undef CROSPI_BOOST_COMP_CLANG
#       define CROSPI_BOOST_COMP_CLANG CROSPI_BOOST_COMP_CLANG_DETECTION
#   endif
#   define CROSPI_BOOST_COMP_CLANG_AVAILABLE
#   include <crospi_boost/predef/detail/comp_detected.h>
#endif

#define CROSPI_BOOST_COMP_CLANG_NAME "Clang"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_COMP_CLANG,CROSPI_BOOST_COMP_CLANG_NAME)

#ifdef CROSPI_BOOST_COMP_CLANG_EMULATED
#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_COMP_CLANG_EMULATED,CROSPI_BOOST_COMP_CLANG_NAME)
#endif
