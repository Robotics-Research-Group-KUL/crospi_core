/*
Copyright Rene Rivera 2008-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_COMPILER_PGI_H
#define CROSPI_BOOST_PREDEF_COMPILER_PGI_H

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_COMP_PGI`

http://en.wikipedia.org/wiki/The_Portland_Group[Portland Group C/{CPP}] compiler.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__PGI+` | {predef_detection}

| `+__PGIC__+`, `+__PGIC_MINOR__+`, `+__PGIC_PATCHLEVEL__+` | V.R.P
|===
*/ // end::reference[]

#define CROSPI_BOOST_COMP_PGI CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__PGI)
#   if !defined(CROSPI_BOOST_COMP_PGI_DETECTION) && (defined(__PGIC__) && defined(__PGIC_MINOR__) && defined(__PGIC_PATCHLEVEL__))
#       define CROSPI_BOOST_COMP_PGI_DETECTION CROSPI_BOOST_VERSION_NUMBER(__PGIC__,__PGIC_MINOR__,__PGIC_PATCHLEVEL__)
#   endif
#   if !defined(CROSPI_BOOST_COMP_PGI_DETECTION)
#       define CROSPI_BOOST_COMP_PGI_DETECTION CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#   endif
#endif

#ifdef CROSPI_BOOST_COMP_PGI_DETECTION
#   if defined(CROSPI_BOOST_PREDEF_DETAIL_COMP_DETECTED)
#       define CROSPI_BOOST_COMP_PGI_EMULATED CROSPI_BOOST_COMP_PGI_DETECTION
#   else
#       undef CROSPI_BOOST_COMP_PGI
#       define CROSPI_BOOST_COMP_PGI CROSPI_BOOST_COMP_PGI_DETECTION
#   endif
#   define CROSPI_BOOST_COMP_PGI_AVAILABLE
#   include <crospi_boost/predef/detail/comp_detected.h>
#endif

#define CROSPI_BOOST_COMP_PGI_NAME "Portland Group C/C++"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_COMP_PGI,CROSPI_BOOST_COMP_PGI_NAME)

#ifdef CROSPI_BOOST_COMP_PGI_EMULATED
#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_COMP_PGI_EMULATED,CROSPI_BOOST_COMP_PGI_NAME)
#endif
