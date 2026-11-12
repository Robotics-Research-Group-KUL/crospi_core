/*
Copyright Rene Rivera 2008-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_COMPILER_GCC_XML_H
#define CROSPI_BOOST_PREDEF_COMPILER_GCC_XML_H

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_COMP_GCCXML`

http://www.gccxml.org/[GCC XML] compiler.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__GCCXML__+` | {predef_detection}
|===
*/ // end::reference[]

#define CROSPI_BOOST_COMP_GCCXML CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__GCCXML__)
#   define CROSPI_BOOST_COMP_GCCXML_DETECTION CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#endif

#ifdef CROSPI_BOOST_COMP_GCCXML_DETECTION
#   if defined(CROSPI_BOOST_PREDEF_DETAIL_COMP_DETECTED)
#       define CROSPI_BOOST_COMP_GCCXML_EMULATED CROSPI_BOOST_COMP_GCCXML_DETECTION
#   else
#       undef CROSPI_BOOST_COMP_GCCXML
#       define CROSPI_BOOST_COMP_GCCXML CROSPI_BOOST_COMP_GCCXML_DETECTION
#   endif
#   define CROSPI_BOOST_COMP_GCCXML_AVAILABLE
#   include <crospi_boost/predef/detail/comp_detected.h>
#endif

#define CROSPI_BOOST_COMP_GCCXML_NAME "GCC XML"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_COMP_GCCXML,CROSPI_BOOST_COMP_GCCXML_NAME)

#ifdef CROSPI_BOOST_COMP_GCCXML_EMULATED
#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_COMP_GCCXML_EMULATED,CROSPI_BOOST_COMP_GCCXML_NAME)
#endif
