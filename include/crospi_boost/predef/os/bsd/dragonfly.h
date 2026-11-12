/*
Copyright Rene Rivera 2012-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_OS_BSD_DRAGONFLY_H
#define CROSPI_BOOST_PREDEF_OS_BSD_DRAGONFLY_H

#include <crospi_boost/predef/os/bsd.h>

/* tag::reference[]
= `CROSPI_BOOST_OS_BSD_DRAGONFLY`

http://en.wikipedia.org/wiki/DragonFly_BSD[DragonFly BSD] operating system.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__DragonFly__+` | {predef_detection}
|===
*/ // end::reference[]

#define CROSPI_BOOST_OS_BSD_DRAGONFLY CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if !defined(CROSPI_BOOST_PREDEF_DETAIL_OS_DETECTED) && ( \
    defined(__DragonFly__) \
    )
#   ifndef CROSPI_BOOST_OS_BSD_AVAILABLE
#       undef CROSPI_BOOST_OS_BSD
#       define CROSPI_BOOST_OS_BSD CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#       define CROSPI_BOOST_OS_BSD_AVAILABLE
#   endif
#   undef CROSPI_BOOST_OS_BSD_DRAGONFLY
#   if defined(__DragonFly__)
#       define CROSPI_BOOST_OS_DRAGONFLY_BSD CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#   endif
#endif

#if CROSPI_BOOST_OS_BSD_DRAGONFLY
#   define CROSPI_BOOST_OS_BSD_DRAGONFLY_AVAILABLE
#   include <crospi_boost/predef/detail/os_detected.h>
#endif

#define CROSPI_BOOST_OS_BSD_DRAGONFLY_NAME "DragonFly BSD"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_OS_BSD_DRAGONFLY,CROSPI_BOOST_OS_BSD_DRAGONFLY_NAME)
