/*
Copyright Konstantin Ivlev 2021
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_ARCHITECTURE_E2K_H
#define CROSPI_BOOST_PREDEF_ARCHITECTURE_E2K_H

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_ARCH_E2K`

https://en.wikipedia.org/wiki/Elbrus_2000[E2K] architecture.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__e2k__+` | {predef_detection}

| `+__e2k__+` | V.0.0
|===
*/ // end::reference[]

#define CROSPI_BOOST_ARCH_E2K CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__e2k__)
#   undef CROSPI_BOOST_ARCH_E2K
#   if !defined(CROSPI_BOOST_ARCH_E2K) && defined(__iset__)
#       define CROSPI_BOOST_ARCH_E2K CROSPI_BOOST_VERSION_NUMBER(__iset__,0,0)
#   endif
#   if !defined(CROSPI_BOOST_ARCH_E2K)
#       define CROSPI_BOOST_ARCH_E2K CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#   endif
#endif

#if CROSPI_BOOST_ARCH_E2K
#   define CROSPI_BOOST_ARCH_E2K_AVAILABLE
#endif

#if CROSPI_BOOST_ARCH_E2K
#   define CROSPI_BOOST_ARCH_WORD_BITS_64 CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#endif

#define CROSPI_BOOST_ARCH_E2K_NAME "E2K"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_ARCH_E2K,CROSPI_BOOST_ARCH_E2K_NAME)
