/*
Copyright Rene Rivera 2008-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_OS_CYGWIN_H
#define CROSPI_BOOST_PREDEF_OS_CYGWIN_H

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_OS_CYGWIN`

http://en.wikipedia.org/wiki/Cygwin[Cygwin] evironment.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__CYGWIN__+` | {predef_detection}

| `CYGWIN_VERSION_API_MAJOR`, `CYGWIN_VERSION_API_MINOR` | V.R.0
|===
*/ // end::reference[]

#define CROSPI_BOOST_OS_CYGWIN CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if !defined(CROSPI_BOOST_PREDEF_DETAIL_OS_DETECTED) && ( \
    defined(__CYGWIN__) \
    )
#   include <cygwin/version.h>
#   undef CROSPI_BOOST_OS_CYGWIN
#   define CROSPI_BOOST_OS_CYGWIN \
        CROSPI_BOOST_VERSION_NUMBER(CYGWIN_VERSION_API_MAJOR,\
                             CYGWIN_VERSION_API_MINOR, 0)
#endif

#if CROSPI_BOOST_OS_CYGWIN
#   define CROSPI_BOOST_OS_CYGWIN_AVAILABLE
#   include <crospi_boost/predef/detail/os_detected.h>
#endif

#define CROSPI_BOOST_OS_CYGWIN_NAME "Cygwin"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_OS_CYGWIN,CROSPI_BOOST_OS_CYGWIN_NAME)
