/*
Copyright Rene Rivera 2008-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_PLAT_MINGW64_H
#define CROSPI_BOOST_PREDEF_PLAT_MINGW64_H

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_PLAT_MINGW64`

https://mingw-w64.org/[MinGW-w64] platform.
Version number available as major, minor, and patch.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__MINGW64__+` | {predef_detection}

| `+__MINGW64_VERSION_MAJOR+`, `+__MINGW64_VERSION_MINOR+` | V.R.0
|===
*/ // end::reference[]

#define CROSPI_BOOST_PLAT_MINGW64 CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__MINGW64__)
#   include <_mingw.h>
#   if !defined(CROSPI_BOOST_PLAT_MINGW64_DETECTION) && (defined(__MINGW64_VERSION_MAJOR) && defined(__MINGW64_VERSION_MINOR))
#       define CROSPI_BOOST_PLAT_MINGW64_DETECTION \
            CROSPI_BOOST_VERSION_NUMBER(__MINGW64_VERSION_MAJOR,__MINGW64_VERSION_MINOR,0)
#   endif
#   if !defined(CROSPI_BOOST_PLAT_MINGW64_DETECTION)
#       define CROSPI_BOOST_PLAT_MINGW64_DETECTION CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#   endif
#endif

#ifdef CROSPI_BOOST_PLAT_MINGW64_DETECTION
#   define CROSPI_BOOST_PLAT_MINGW64_AVAILABLE
#   if defined(CROSPI_BOOST_PREDEF_DETAIL_PLAT_DETECTED)
#       define CROSPI_BOOST_PLAT_MINGW64_EMULATED CROSPI_BOOST_PLAT_MINGW64_DETECTION
#   else
#       undef CROSPI_BOOST_PLAT_MINGW64
#       define CROSPI_BOOST_PLAT_MINGW64 CROSPI_BOOST_PLAT_MINGW64_DETECTION
#   endif
#   include <crospi_boost/predef/detail/platform_detected.h>
#endif

#define CROSPI_BOOST_PLAT_MINGW64_NAME "MinGW-w64"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_PLAT_MINGW64,CROSPI_BOOST_PLAT_MINGW64_NAME)

#ifdef CROSPI_BOOST_PLAT_MINGW64_EMULATED
#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_PLAT_MINGW64_EMULATED,CROSPI_BOOST_PLAT_MINGW64_NAME)
#endif
