/*
Copyright Rene Rivera 2012-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_OS_BSD_BSDI_H
#define CROSPI_BOOST_PREDEF_OS_BSD_BSDI_H

#include <crospi_boost/predef/os/bsd.h>

/* tag::reference[]
= `CROSPI_BOOST_OS_BSD_BSDI`

http://en.wikipedia.org/wiki/BSD/OS[BSDi BSD/OS] operating system.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__bsdi__+` | {predef_detection}
|===
*/ // end::reference[]

#define CROSPI_BOOST_OS_BSD_BSDI CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if !defined(CROSPI_BOOST_PREDEF_DETAIL_OS_DETECTED) && ( \
    defined(__bsdi__) \
    )
#   ifndef CROSPI_BOOST_OS_BSD_AVAILABLE
#       undef CROSPI_BOOST_OS_BSD
#       define CROSPI_BOOST_OS_BSD CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#       define CROSPI_BOOST_OS_BSD_AVAILABLE
#   endif
#   undef CROSPI_BOOST_OS_BSD_BSDI
#   define CROSPI_BOOST_OS_BSD_BSDI CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#endif

#if CROSPI_BOOST_OS_BSD_BSDI
#   define CROSPI_BOOST_OS_BSD_BSDI_AVAILABLE
#   include <crospi_boost/predef/detail/os_detected.h>
#endif

#define CROSPI_BOOST_OS_BSD_BSDI_NAME "BSDi BSD/OS"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_OS_BSD_BSDI,CROSPI_BOOST_OS_BSD_BSDI_NAME)
