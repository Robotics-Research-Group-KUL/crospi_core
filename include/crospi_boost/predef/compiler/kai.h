/*
Copyright Rene Rivera 2008-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_COMPILER_KAI_H
#define CROSPI_BOOST_PREDEF_COMPILER_KAI_H

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_COMP_KCC`

Kai {CPP} compiler.
Version number available as major, minor, and patch.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__KCC+` | {predef_detection}

| `+__KCC_VERSION+` | V.R.P
|===
*/ // end::reference[]

#define CROSPI_BOOST_COMP_KCC CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__KCC)
#   define CROSPI_BOOST_COMP_KCC_DETECTION CROSPI_BOOST_PREDEF_MAKE_0X_VRPP(__KCC_VERSION)
#endif

#ifdef CROSPI_BOOST_COMP_KCC_DETECTION
#   if defined(CROSPI_BOOST_PREDEF_DETAIL_COMP_DETECTED)
#       define CROSPI_BOOST_COMP_KCC_EMULATED CROSPI_BOOST_COMP_KCC_DETECTION
#   else
#       undef CROSPI_BOOST_COMP_KCC
#       define CROSPI_BOOST_COMP_KCC CROSPI_BOOST_COMP_KCC_DETECTION
#   endif
#   define CROSPI_BOOST_COMP_KCC_AVAILABLE
#   include <crospi_boost/predef/detail/comp_detected.h>
#endif

#define CROSPI_BOOST_COMP_KCC_NAME "Kai C++"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_COMP_KCC,CROSPI_BOOST_COMP_KCC_NAME)

#ifdef CROSPI_BOOST_COMP_KCC_EMULATED
#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_COMP_KCC_EMULATED,CROSPI_BOOST_COMP_KCC_NAME)
#endif
