/*
Copyright Rene Rivera 2008-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_LIBRARY_STD_MSL_H
#define CROSPI_BOOST_PREDEF_LIBRARY_STD_MSL_H

#include <crospi_boost/predef/library/std/_prefix.h>

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_LIB_STD_MSL`

http://www.freescale.com/[Metrowerks] Standard {CPP} Library.
Version number available as major, minor, and patch.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__MSL_CPP__+` | {predef_detection}
| `+__MSL__+` | {predef_detection}

| `+__MSL_CPP__+` | V.R.P
| `+__MSL__+` | V.R.P
|===
*/ // end::reference[]

#define CROSPI_BOOST_LIB_STD_MSL CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__MSL_CPP__) || defined(__MSL__)
#   undef CROSPI_BOOST_LIB_STD_MSL
#   if defined(__MSL_CPP__)
#       define CROSPI_BOOST_LIB_STD_MSL CROSPI_BOOST_PREDEF_MAKE_0X_VRPP(__MSL_CPP__)
#   else
#       define CROSPI_BOOST_LIB_STD_MSL CROSPI_BOOST_PREDEF_MAKE_0X_VRPP(__MSL__)
#   endif
#endif

#if CROSPI_BOOST_LIB_STD_MSL
#   define CROSPI_BOOST_LIB_STD_MSL_AVAILABLE
#endif

#define CROSPI_BOOST_LIB_STD_MSL_NAME "Metrowerks"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_LIB_STD_MSL,CROSPI_BOOST_LIB_STD_MSL_NAME)
