/*
Copyright Rene Rivera 2008-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_ARCHITECTURE_SYS390_H
#define CROSPI_BOOST_PREDEF_ARCHITECTURE_SYS390_H

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_ARCH_SYS390`

http://en.wikipedia.org/wiki/System/390[System/390] architecture.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__s390__+` | {predef_detection}
| `+__s390x__+` | {predef_detection}
|===
*/ // end::reference[]

#define CROSPI_BOOST_ARCH_SYS390 CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__s390__) || defined(__s390x__)
#   undef CROSPI_BOOST_ARCH_SYS390
#   define CROSPI_BOOST_ARCH_SYS390 CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#endif

#if CROSPI_BOOST_ARCH_SYS390
#   define CROSPI_BOOST_ARCH_SYS390_AVAILABLE
#endif

#if CROSPI_BOOST_ARCH_SYS390
#   undef CROSPI_BOOST_ARCH_WORD_BITS_32
#   define CROSPI_BOOST_ARCH_WORD_BITS_32 CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#endif

#define CROSPI_BOOST_ARCH_SYS390_NAME "System/390"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_ARCH_SYS390,CROSPI_BOOST_ARCH_SYS390_NAME)
