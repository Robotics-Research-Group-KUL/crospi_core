/*
Copyright Rene Rivera 2008-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_COMPILER_MICROTEC_H
#define CROSPI_BOOST_PREDEF_COMPILER_MICROTEC_H

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_COMP_MRI`

http://www.mentor.com/microtec/[Microtec C/{CPP}] compiler.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+_MRI+` | {predef_detection}
|===
*/ // end::reference[]

#define CROSPI_BOOST_COMP_MRI CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(_MRI)
#   define CROSPI_BOOST_COMP_MRI_DETECTION CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#endif

#ifdef CROSPI_BOOST_COMP_MRI_DETECTION
#   if defined(CROSPI_BOOST_PREDEF_DETAIL_COMP_DETECTED)
#       define CROSPI_BOOST_COMP_MRI_EMULATED CROSPI_BOOST_COMP_MRI_DETECTION
#   else
#       undef CROSPI_BOOST_COMP_MRI
#       define CROSPI_BOOST_COMP_MRI CROSPI_BOOST_COMP_MRI_DETECTION
#   endif
#   define CROSPI_BOOST_COMP_MRI_AVAILABLE
#   include <crospi_boost/predef/detail/comp_detected.h>
#endif

#define CROSPI_BOOST_COMP_MRI_NAME "Microtec C/C++"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_COMP_MRI,CROSPI_BOOST_COMP_MRI_NAME)

#ifdef CROSPI_BOOST_COMP_MRI_EMULATED
#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_COMP_MRI_EMULATED,CROSPI_BOOST_COMP_MRI_NAME)
#endif
