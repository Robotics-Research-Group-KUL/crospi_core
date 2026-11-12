/*
Copyright Rene Rivera 2008-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_ARCHITECTURE_ALPHA_H
#define CROSPI_BOOST_PREDEF_ARCHITECTURE_ALPHA_H

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_ARCH_ALPHA`

http://en.wikipedia.org/wiki/DEC_Alpha[DEC Alpha] architecture.

[options="header"]
|===
| {predef_symbol} | {predef_version}
| `+__alpha__+` | {predef_detection}
| `+__alpha+` | {predef_detection}
| `+_M_ALPHA+` | {predef_detection}

| `+__alpha_ev4__+` | 4.0.0
| `+__alpha_ev5__+` | 5.0.0
| `+__alpha_ev6__+` | 6.0.0
|===
*/ // end::reference[]

#define CROSPI_BOOST_ARCH_ALPHA CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__alpha__) || defined(__alpha) || \
    defined(_M_ALPHA)
#   undef CROSPI_BOOST_ARCH_ALPHA
#   if !defined(CROSPI_BOOST_ARCH_ALPHA) && defined(__alpha_ev4__)
#       define CROSPI_BOOST_ARCH_ALPHA CROSPI_BOOST_VERSION_NUMBER(4,0,0)
#   endif
#   if !defined(CROSPI_BOOST_ARCH_ALPHA) && defined(__alpha_ev5__)
#       define CROSPI_BOOST_ARCH_ALPHA CROSPI_BOOST_VERSION_NUMBER(5,0,0)
#   endif
#   if !defined(CROSPI_BOOST_ARCH_ALPHA) && defined(__alpha_ev6__)
#       define CROSPI_BOOST_ARCH_ALPHA CROSPI_BOOST_VERSION_NUMBER(6,0,0)
#   endif
#   if !defined(CROSPI_BOOST_ARCH_ALPHA)
#       define CROSPI_BOOST_ARCH_ALPHA CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#   endif
#endif

#if CROSPI_BOOST_ARCH_ALPHA
#   define CROSPI_BOOST_ARCH_ALPHA_AVAILABLE
#endif

#if CROSPI_BOOST_ARCH_ALPHA
#   undef CROSPI_BOOST_ARCH_WORD_BITS_64
#   define CROSPI_BOOST_ARCH_WORD_BITS_64 CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#endif

#define CROSPI_BOOST_ARCH_ALPHA_NAME "DEC Alpha"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_ARCH_ALPHA,CROSPI_BOOST_ARCH_ALPHA_NAME)
