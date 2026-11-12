/*
Copyright Rene Rivera 2008-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_ARCHITECTURE_SUPERH_H
#define CROSPI_BOOST_PREDEF_ARCHITECTURE_SUPERH_H

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_ARCH_SH`

http://en.wikipedia.org/wiki/SuperH[SuperH] architecture:
If available versions [1-5] are specifically detected.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__sh__+` | {predef_detection}

| `+__SH5__+` | 5.0.0
| `+__SH4__+` | 4.0.0
| `+__sh3__+` | 3.0.0
| `+__SH3__+` | 3.0.0
| `+__sh2__+` | 2.0.0
| `+__sh1__+` | 1.0.0
|===
*/ // end::reference[]

#define CROSPI_BOOST_ARCH_SH CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__sh__)
#   undef CROSPI_BOOST_ARCH_SH
#   if !defined(CROSPI_BOOST_ARCH_SH) && (defined(__SH5__))
#       define CROSPI_BOOST_ARCH_SH CROSPI_BOOST_VERSION_NUMBER(5,0,0)
#   endif
#   if !defined(CROSPI_BOOST_ARCH_SH) && (defined(__SH4__))
#       define CROSPI_BOOST_ARCH_SH CROSPI_BOOST_VERSION_NUMBER(4,0,0)
#   endif
#   if !defined(CROSPI_BOOST_ARCH_SH) && (defined(__sh3__) || defined(__SH3__))
#       define CROSPI_BOOST_ARCH_SH CROSPI_BOOST_VERSION_NUMBER(3,0,0)
#   endif
#   if !defined(CROSPI_BOOST_ARCH_SH) && (defined(__sh2__))
#       define CROSPI_BOOST_ARCH_SH CROSPI_BOOST_VERSION_NUMBER(2,0,0)
#   endif
#   if !defined(CROSPI_BOOST_ARCH_SH) && (defined(__sh1__))
#       define CROSPI_BOOST_ARCH_SH CROSPI_BOOST_VERSION_NUMBER(1,0,0)
#   endif
#   if !defined(CROSPI_BOOST_ARCH_SH)
#       define CROSPI_BOOST_ARCH_SH CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#   endif
#endif

#if CROSPI_BOOST_ARCH_SH
#   define CROSPI_BOOST_ARCH_SH_AVAILABLE
#endif

#if CROSPI_BOOST_ARCH_SH
#   if CROSPI_BOOST_ARCH_SH >= CROSPI_BOOST_VERSION_NUMBER(5,0,0)
#       undef CROSPI_BOOST_ARCH_WORD_BITS_64
#       define CROSPI_BOOST_ARCH_WORD_BITS_64 CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#   elif CROSPI_BOOST_ARCH_SH >= CROSPI_BOOST_VERSION_NUMBER(3,0,0)
#       undef CROSPI_BOOST_ARCH_WORD_BITS_32
#       define CROSPI_BOOST_ARCH_WORD_BITS_32 CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#   else
#       undef CROSPI_BOOST_ARCH_WORD_BITS_16
#       define CROSPI_BOOST_ARCH_WORD_BITS_16 CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#   endif
#endif

#define CROSPI_BOOST_ARCH_SH_NAME "SuperH"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_ARCH_SH,CROSPI_BOOST_ARCH_SH_NAME)
