/*
Copyright Rene Rivera 2008-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_OS_SOLARIS_H
#define CROSPI_BOOST_PREDEF_OS_SOLARIS_H

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_OS_SOLARIS`

http://en.wikipedia.org/wiki/Solaris_Operating_Environment[Solaris] operating system.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `sun` | {predef_detection}
| `+__sun+` | {predef_detection}
|===
*/ // end::reference[]

#define CROSPI_BOOST_OS_SOLARIS CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if !defined(CROSPI_BOOST_PREDEF_DETAIL_OS_DETECTED) && ( \
    defined(sun) || defined(__sun) \
    )
#   undef CROSPI_BOOST_OS_SOLARIS
#   define CROSPI_BOOST_OS_SOLARIS CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#endif

#if CROSPI_BOOST_OS_SOLARIS
#   define CROSPI_BOOST_OS_SOLARIS_AVAILABLE
#   include <crospi_boost/predef/detail/os_detected.h>
#endif

#define CROSPI_BOOST_OS_SOLARIS_NAME "Solaris"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_OS_SOLARIS,CROSPI_BOOST_OS_SOLARIS_NAME)
