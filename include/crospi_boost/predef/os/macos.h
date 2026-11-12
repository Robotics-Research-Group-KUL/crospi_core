/*
Copyright Rene Rivera 2008-2015
Copyright Franz Detro 2014
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_OS_MACOS_H
#define CROSPI_BOOST_PREDEF_OS_MACOS_H

/* Special case: iOS will define the same predefs as MacOS, and additionally
 '__ENVIRONMENT_IPHONE_OS_VERSION_MIN_REQUIRED__'. We can guard against that,
 but only if we detect iOS first. Hence we will force include iOS detection
 * before doing any MacOS detection.
 */
#include <crospi_boost/predef/os/ios.h>

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_OS_MACOS`

http://en.wikipedia.org/wiki/Mac_OS[Mac OS] operating system.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `macintosh` | {predef_detection}
| `Macintosh` | {predef_detection}
| `+__APPLE__+` | {predef_detection}
| `+__MACH__+` | {predef_detection}

| `+__APPLE__+`, `+__MACH__+` | 10.0.0
| `_otherwise_` | 9.0.0
|===
*/ // end::reference[]

#define CROSPI_BOOST_OS_MACOS CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if !defined(CROSPI_BOOST_PREDEF_DETAIL_OS_DETECTED) && ( \
    defined(macintosh) || defined(Macintosh) || \
    (defined(__APPLE__) && defined(__MACH__)) \
    )
#   undef CROSPI_BOOST_OS_MACOS
#   if !defined(CROSPI_BOOST_OS_MACOS) && defined(__APPLE__) && defined(__MACH__)
#       define CROSPI_BOOST_OS_MACOS CROSPI_BOOST_VERSION_NUMBER(10,0,0)
#   endif
#   if !defined(CROSPI_BOOST_OS_MACOS)
#       define CROSPI_BOOST_OS_MACOS CROSPI_BOOST_VERSION_NUMBER(9,0,0)
#   endif
#endif

#if CROSPI_BOOST_OS_MACOS
#   define CROSPI_BOOST_OS_MACOS_AVAILABLE
#   include <crospi_boost/predef/detail/os_detected.h>
#endif

#define CROSPI_BOOST_OS_MACOS_NAME "Mac OS"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_OS_MACOS,CROSPI_BOOST_OS_MACOS_NAME)
