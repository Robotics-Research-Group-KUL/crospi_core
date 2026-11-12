/*
Copyright Rene Rivera 2008-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_LIBRARY_C_VMS_H
#define CROSPI_BOOST_PREDEF_LIBRARY_C_VMS_H

#include <crospi_boost/predef/library/c/_prefix.h>

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_LIB_C_VMS`

VMS libc Standard C library.
Version number available as major, minor, and patch.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__CRTL_VER+` | {predef_detection}

| `+__CRTL_VER+` | V.R.P
|===
*/ // end::reference[]

#define CROSPI_BOOST_LIB_C_VMS CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__CRTL_VER)
#   undef CROSPI_BOOST_LIB_C_VMS
#   define CROSPI_BOOST_LIB_C_VMS CROSPI_BOOST_PREDEF_MAKE_10_VVRR0PP00(__CRTL_VER)
#endif

#if CROSPI_BOOST_LIB_C_VMS
#   define CROSPI_BOOST_LIB_C_VMS_AVAILABLE
#endif

#define CROSPI_BOOST_LIB_C_VMS_NAME "VMS"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_LIB_C_VMS,CROSPI_BOOST_LIB_C_VMS_NAME)
