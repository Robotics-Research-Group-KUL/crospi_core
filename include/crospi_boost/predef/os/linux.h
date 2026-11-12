/*
Copyright Rene Rivera 2008-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_OS_LINUX_H
#define CROSPI_BOOST_PREDEF_OS_LINUX_H

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_OS_LINUX`

http://en.wikipedia.org/wiki/Linux[Linux] operating system.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `linux` | {predef_detection}
| `+__linux+` | {predef_detection}
| `+__linux__+` | {predef_detection}
| `+__gnu_linux__+` | {predef_detection}
|===
*/ // end::reference[]

#define CROSPI_BOOST_OS_LINUX CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if !defined(CROSPI_BOOST_PREDEF_DETAIL_OS_DETECTED) && ( \
    defined(linux) || defined(__linux) || \
    defined(__linux__) || defined(__gnu_linux__) \
    )
#   undef CROSPI_BOOST_OS_LINUX
#   define CROSPI_BOOST_OS_LINUX CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#endif

#if CROSPI_BOOST_OS_LINUX
#   define CROSPI_BOOST_OS_LINUX_AVAILABLE
#   include <crospi_boost/predef/detail/os_detected.h>
#endif

#define CROSPI_BOOST_OS_LINUX_NAME "Linux"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_OS_LINUX,CROSPI_BOOST_OS_LINUX_NAME)
