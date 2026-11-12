/*
Copyright Rene Rivera 2008-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_OS_QNXNTO_H
#define CROSPI_BOOST_PREDEF_OS_QNXNTO_H

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_OS_QNX`

http://en.wikipedia.org/wiki/QNX[QNX] operating system.
Version number available as major, and minor if possible. And
version 4 is specifically detected.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__QNX__+` | {predef_detection}
| `+__QNXNTO__+` | {predef_detection}

| `+_NTO_VERSION+` | V.R.0
| `+__QNX__+` | 4.0.0
|===
*/ // end::reference[]

#define CROSPI_BOOST_OS_QNX CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if !defined(CROSPI_BOOST_PREDEF_DETAIL_OS_DETECTED) && ( \
    defined(__QNX__) || defined(__QNXNTO__) \
    )
#   undef CROSPI_BOOST_OS_QNX
#   if !defined(CROSPI_BOOST_OS_QNX) && defined(_NTO_VERSION)
#       define CROSPI_BOOST_OS_QNX CROSPI_BOOST_PREDEF_MAKE_10_VVRR(_NTO_VERSION)
#   endif
#   if !defined(CROSPI_BOOST_OS_QNX) && defined(__QNX__)
#       define CROSPI_BOOST_OS_QNX CROSPI_BOOST_VERSION_NUMBER(4,0,0)
#   endif
#   if !defined(CROSPI_BOOST_OS_QNX)
#       define CROSPI_BOOST_OS_QNX CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#   endif
#endif

#if CROSPI_BOOST_OS_QNX
#   define CROSPI_BOOST_OS_QNX_AVAILABLE
#   include <crospi_boost/predef/detail/os_detected.h>
#endif

#define CROSPI_BOOST_OS_QNX_NAME "QNX"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_OS_QNX,CROSPI_BOOST_OS_QNX_NAME)
