/*
Copyright Rene Rivera 2013-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_ARCHITECTURE_BLACKFIN_H
#define CROSPI_BOOST_PREDEF_ARCHITECTURE_BLACKFIN_H

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_ARCH_BLACKFIN`

Blackfin Processors from Analog Devices.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__bfin__+` | {predef_detection}
| `+__BFIN__+` | {predef_detection}
| `bfin` | {predef_detection}
| `BFIN` | {predef_detection}
|===
*/ // end::reference[]

#define CROSPI_BOOST_ARCH_BLACKFIN CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__bfin__) || defined(__BFIN__) || \
    defined(bfin) || defined(BFIN)
#   undef CROSPI_BOOST_ARCH_BLACKFIN
#   define CROSPI_BOOST_ARCH_BLACKFIN CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#endif

#if CROSPI_BOOST_ARCH_BLACKFIN
#   define CROSPI_BOOST_ARCH_BLACKFIN_AVAILABLE
#endif

#if CROSPI_BOOST_ARCH_BLACKFIN
#   undef CROSPI_BOOST_ARCH_WORD_BITS_16
#   define CROSPI_BOOST_ARCH_WORD_BITS_16 CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#endif

#define CROSPI_BOOST_ARCH_BLACKFIN_NAME "Blackfin"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_ARCH_BLACKFIN,CROSPI_BOOST_ARCH_BLACKFIN_NAME)
