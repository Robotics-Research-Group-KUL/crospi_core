/*
Copyright Rene Rivera 2008-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_COMPILER_COMPAQ_H
#define CROSPI_BOOST_PREDEF_COMPILER_COMPAQ_H

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_COMP_DEC`

http://www.openvms.compaq.com/openvms/brochures/deccplus/[Compaq C/{CPP}] compiler.
Version number available as major, minor, and patch.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__DECCXX+` | {predef_detection}
| `+__DECC+` | {predef_detection}

| `+__DECCXX_VER+` | V.R.P
| `+__DECC_VER+` | V.R.P
|===
*/ // end::reference[]

#define CROSPI_BOOST_COMP_DEC CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__DECC) || defined(__DECCXX)
#   if !defined(CROSPI_BOOST_COMP_DEC_DETECTION) && defined(__DECCXX_VER)
#       define CROSPI_BOOST_COMP_DEC_DETECTION CROSPI_BOOST_PREDEF_MAKE_10_VVRR0PP00(__DECCXX_VER)
#   endif
#   if !defined(CROSPI_BOOST_COMP_DEC_DETECTION) && defined(__DECC_VER)
#       define CROSPI_BOOST_COMP_DEC_DETECTION CROSPI_BOOST_PREDEF_MAKE_10_VVRR0PP00(__DECC_VER)
#   endif
#   if !defined(CROSPI_BOOST_COMP_DEC_DETECTION)
#       define CROSPI_BOOST_COMP_DEC_DETECTION CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#   endif
#endif

#ifdef CROSPI_BOOST_COMP_DEC_DETECTION
#   if defined(CROSPI_BOOST_PREDEF_DETAIL_COMP_DETECTED)
#       define CROSPI_BOOST_COMP_DEC_EMULATED CROSPI_BOOST_COMP_DEC_DETECTION
#   else
#       undef CROSPI_BOOST_COMP_DEC
#       define CROSPI_BOOST_COMP_DEC CROSPI_BOOST_COMP_DEC_DETECTION
#   endif
#   define CROSPI_BOOST_COMP_DEC_AVAILABLE
#   include <crospi_boost/predef/detail/comp_detected.h>
#endif

#define CROSPI_BOOST_COMP_DEC_NAME "Compaq C/C++"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_COMP_DEC,CROSPI_BOOST_COMP_DEC_NAME)

#ifdef CROSPI_BOOST_COMP_DEC_EMULATED
#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_COMP_DEC_EMULATED,CROSPI_BOOST_COMP_DEC_NAME)
#endif
