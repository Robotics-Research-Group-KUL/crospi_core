/*
Copyright Rene Rivera 2008-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_OS_WINDOWS_H
#define CROSPI_BOOST_PREDEF_OS_WINDOWS_H

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_OS_WINDOWS`

http://en.wikipedia.org/wiki/Category:Microsoft_Windows[Microsoft Windows] operating system.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+_WIN32+` | {predef_detection}
| `+_WIN64+` | {predef_detection}
| `+__WIN32__+` | {predef_detection}
| `+__TOS_WIN__+` | {predef_detection}
| `+__WINDOWS__+` | {predef_detection}
|===
*/ // end::reference[]

#define CROSPI_BOOST_OS_WINDOWS CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if !defined(CROSPI_BOOST_PREDEF_DETAIL_OS_DETECTED) && ( \
    defined(_WIN32) || defined(_WIN64) || \
    defined(__WIN32__) || defined(__TOS_WIN__) || \
    defined(__WINDOWS__) \
    )
#   undef CROSPI_BOOST_OS_WINDOWS
#   define CROSPI_BOOST_OS_WINDOWS CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#endif

#if CROSPI_BOOST_OS_WINDOWS
#   define CROSPI_BOOST_OS_WINDOWS_AVAILABLE
#   include <crospi_boost/predef/detail/os_detected.h>
#endif

#define CROSPI_BOOST_OS_WINDOWS_NAME "Microsoft Windows"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_OS_WINDOWS,CROSPI_BOOST_OS_WINDOWS_NAME)
