/*
Copyright Rene Rivera 2008-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_OS_HPUX_H
#define CROSPI_BOOST_PREDEF_OS_HPUX_H

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_OS_HPUX`

http://en.wikipedia.org/wiki/HP-UX[HP-UX] operating system.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `hpux` | {predef_detection}
| `+_hpux+` | {predef_detection}
| `+__hpux+` | {predef_detection}
|===
*/ // end::reference[]

#define CROSPI_BOOST_OS_HPUX CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if !defined(CROSPI_BOOST_PREDEF_DETAIL_OS_DETECTED) && ( \
    defined(hpux) || defined(_hpux) || defined(__hpux) \
    )
#   undef CROSPI_BOOST_OS_HPUX
#   define CROSPI_BOOST_OS_HPUX CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#endif

#if CROSPI_BOOST_OS_HPUX
#   define CROSPI_BOOST_OS_HPUX_AVAILABLE
#   include <crospi_boost/predef/detail/os_detected.h>
#endif

#define CROSPI_BOOST_OS_HPUX_NAME "HP-UX"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_OS_HPUX,CROSPI_BOOST_OS_HPUX_NAME)
