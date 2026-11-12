/*
Copyright Rene Rivera 2008-2021
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_ARCHITECTURE_X86_64_H
#define CROSPI_BOOST_PREDEF_ARCHITECTURE_X86_64_H

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_ARCH_X86_64`

https://en.wikipedia.org/wiki/X86-64[X86-64] architecture.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__x86_64+` | {predef_detection}
| `+__x86_64__+` | {predef_detection}
| `+__amd64__+` | {predef_detection}
| `+__amd64+` | {predef_detection}
| `+_M_X64+` | {predef_detection}
|===
*/ // end::reference[]

#define CROSPI_BOOST_ARCH_X86_64 CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__x86_64) || defined(__x86_64__) || \
    defined(__amd64__) || defined(__amd64) || \
    defined(_M_X64)
#   undef CROSPI_BOOST_ARCH_X86_64
#   define CROSPI_BOOST_ARCH_X86_64 CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#endif

#if CROSPI_BOOST_ARCH_X86_64
#   define CROSPI_BOOST_ARCH_X86_64_AVAILABLE
#endif

#if CROSPI_BOOST_ARCH_X86_64
#   undef CROSPI_BOOST_ARCH_WORD_BITS_64
#   define CROSPI_BOOST_ARCH_WORD_BITS_64 CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#endif

#define CROSPI_BOOST_ARCH_X86_64_NAME "Intel x86-64"

#include <crospi_boost/predef/architecture/x86.h>

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_ARCH_X86_64,CROSPI_BOOST_ARCH_X86_64_NAME)
