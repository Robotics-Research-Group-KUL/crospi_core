/*
Copyright Rene Rivera 2011-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_OS_VMS_H
#define CROSPI_BOOST_PREDEF_OS_VMS_H

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_OS_VMS`

http://en.wikipedia.org/wiki/OpenVMS[VMS] operating system.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `VMS` | {predef_detection}
| `+__VMS+` | {predef_detection}

| `+__VMS_VER+` | V.R.P
|===
*/ // end::reference[]

#define CROSPI_BOOST_OS_VMS CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if !defined(CROSPI_BOOST_PREDEF_DETAIL_OS_DETECTED) && ( \
    defined(VMS) || defined(__VMS) \
    )
#   undef CROSPI_BOOST_OS_VMS
#   if defined(__VMS_VER)
#       define CROSPI_BOOST_OS_VMS CROSPI_BOOST_PREDEF_MAKE_10_VVRR00PP00(__VMS_VER)
#   else
#       define CROSPI_BOOST_OS_VMS CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#   endif
#endif

#if CROSPI_BOOST_OS_VMS
#   define CROSPI_BOOST_OS_VMS_AVAILABLE
#   include <crospi_boost/predef/detail/os_detected.h>
#endif

#define CROSPI_BOOST_OS_VMS_NAME "VMS"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_OS_VMS,CROSPI_BOOST_OS_VMS_NAME)
