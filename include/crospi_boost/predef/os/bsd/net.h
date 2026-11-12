/*
Copyright Rene Rivera 2012-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_OS_BSD_NET_H
#define CROSPI_BOOST_PREDEF_OS_BSD_NET_H

#include <crospi_boost/predef/os/bsd.h>

/* tag::reference[]
= `CROSPI_BOOST_OS_BSD_NET`

http://en.wikipedia.org/wiki/Netbsd[NetBSD] operating system.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__NETBSD__+` | {predef_detection}
| `+__NetBSD__+` | {predef_detection}

| `+__NETBSD_version+` | V.R.P
| `NetBSD0_8` | 0.8.0
| `NetBSD0_9` | 0.9.0
| `NetBSD1_0` | 1.0.0
| `+__NetBSD_Version+` | V.R.P
|===
*/ // end::reference[]

#define CROSPI_BOOST_OS_BSD_NET CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if !defined(CROSPI_BOOST_PREDEF_DETAIL_OS_DETECTED) && ( \
    defined(__NETBSD__) || defined(__NetBSD__) \
    )
#   ifndef CROSPI_BOOST_OS_BSD_AVAILABLE
#       undef CROSPI_BOOST_OS_BSD
#       define CROSPI_BOOST_OS_BSD CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#       define CROSPI_BOOST_OS_BSD_AVAILABLE
#   endif
#   undef CROSPI_BOOST_OS_BSD_NET
#   if defined(__NETBSD__)
#       if defined(__NETBSD_version)
#           if __NETBSD_version < 500000
#               define CROSPI_BOOST_OS_BSD_NET \
                    CROSPI_BOOST_PREDEF_MAKE_10_VRP000(__NETBSD_version)
#           else
#               define CROSPI_BOOST_OS_BSD_NET \
                    CROSPI_BOOST_PREDEF_MAKE_10_VRR000(__NETBSD_version)
#           endif
#       else
#           define CROSPI_BOOST_OS_BSD_NET CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#       endif
#   elif defined(__NetBSD__)
#       if !defined(CROSPI_BOOST_OS_BSD_NET) && defined(NetBSD0_8)
#           define CROSPI_BOOST_OS_BSD_NET CROSPI_BOOST_VERSION_NUMBER(0,8,0)
#       endif
#       if !defined(CROSPI_BOOST_OS_BSD_NET) && defined(NetBSD0_9)
#           define CROSPI_BOOST_OS_BSD_NET CROSPI_BOOST_VERSION_NUMBER(0,9,0)
#       endif
#       if !defined(CROSPI_BOOST_OS_BSD_NET) && defined(NetBSD1_0)
#           define CROSPI_BOOST_OS_BSD_NET CROSPI_BOOST_VERSION_NUMBER(1,0,0)
#       endif
#       if !defined(CROSPI_BOOST_OS_BSD_NET) && defined(__NetBSD_Version)
#           define CROSPI_BOOST_OS_BSD_NET \
                CROSPI_BOOST_PREDEF_MAKE_10_VVRR00PP00(__NetBSD_Version)
#       endif
#       if !defined(CROSPI_BOOST_OS_BSD_NET)
#           define CROSPI_BOOST_OS_BSD_NET CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#       endif
#   endif
#endif

#if CROSPI_BOOST_OS_BSD_NET
#   define CROSPI_BOOST_OS_BSD_NET_AVAILABLE
#   include <crospi_boost/predef/detail/os_detected.h>
#endif

#define CROSPI_BOOST_OS_BSD_NET_NAME "NetBSD"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_OS_BSD_NET,CROSPI_BOOST_OS_BSD_NET_NAME)
