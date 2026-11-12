/*
Copyright Rene Rivera 2008-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_ARCHITECTURE_MIPS_H
#define CROSPI_BOOST_PREDEF_ARCHITECTURE_MIPS_H

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_ARCH_MIPS`

http://en.wikipedia.org/wiki/MIPS_architecture[MIPS] architecture.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__mips__+` | {predef_detection}
| `+__mips+` | {predef_detection}
| `+__MIPS__+` | {predef_detection}

| `+__mips+` | V.0.0
| `+_MIPS_ISA_MIPS1+` | 1.0.0
| `+_R3000+` | 1.0.0
| `+_MIPS_ISA_MIPS2+` | 2.0.0
| `+__MIPS_ISA2__+` | 2.0.0
| `+_R4000+` | 2.0.0
| `+_MIPS_ISA_MIPS3+` | 3.0.0
| `+__MIPS_ISA3__+` | 3.0.0
| `+_MIPS_ISA_MIPS4+` | 4.0.0
| `+__MIPS_ISA4__+` | 4.0.0
|===
*/ // end::reference[]

#define CROSPI_BOOST_ARCH_MIPS CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__mips__) || defined(__mips) || \
    defined(__MIPS__)
#   undef CROSPI_BOOST_ARCH_MIPS
#   if !defined(CROSPI_BOOST_ARCH_MIPS) && (defined(__mips))
#       define CROSPI_BOOST_ARCH_MIPS CROSPI_BOOST_VERSION_NUMBER(__mips,0,0)
#   endif
#   if !defined(CROSPI_BOOST_ARCH_MIPS) && (defined(_MIPS_ISA_MIPS1) || defined(_R3000))
#       define CROSPI_BOOST_ARCH_MIPS CROSPI_BOOST_VERSION_NUMBER(1,0,0)
#   endif
#   if !defined(CROSPI_BOOST_ARCH_MIPS) && (defined(_MIPS_ISA_MIPS2) || defined(__MIPS_ISA2__) || defined(_R4000))
#       define CROSPI_BOOST_ARCH_MIPS CROSPI_BOOST_VERSION_NUMBER(2,0,0)
#   endif
#   if !defined(CROSPI_BOOST_ARCH_MIPS) && (defined(_MIPS_ISA_MIPS3) || defined(__MIPS_ISA3__))
#       define CROSPI_BOOST_ARCH_MIPS CROSPI_BOOST_VERSION_NUMBER(3,0,0)
#   endif
#   if !defined(CROSPI_BOOST_ARCH_MIPS) && (defined(_MIPS_ISA_MIPS4) || defined(__MIPS_ISA4__))
#       define CROSPI_BOOST_ARCH_MIPS CROSPI_BOOST_VERSION_NUMBER(4,0,0)
#   endif
#   if !defined(CROSPI_BOOST_ARCH_MIPS)
#       define CROSPI_BOOST_ARCH_MIPS CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#   endif
#endif

#if CROSPI_BOOST_ARCH_MIPS
#   define CROSPI_BOOST_ARCH_MIPS_AVAILABLE
#endif

#if CROSPI_BOOST_ARCH_MIPS
#   if CROSPI_BOOST_ARCH_MIPS >= CROSPI_BOOST_VERSION_NUMBER(3,0,0)
#       undef CROSPI_BOOST_ARCH_WORD_BITS_64
#       define CROSPI_BOOST_ARCH_WORD_BITS_64 CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#   else
#       undef CROSPI_BOOST_ARCH_WORD_BITS_32
#       define CROSPI_BOOST_ARCH_WORD_BITS_32 CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#   endif
#endif

#define CROSPI_BOOST_ARCH_MIPS_NAME "MIPS"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_ARCH_MIPS,CROSPI_BOOST_ARCH_MIPS_NAME)
