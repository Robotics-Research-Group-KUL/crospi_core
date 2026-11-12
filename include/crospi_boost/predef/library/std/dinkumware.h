/*
Copyright Rene Rivera 2008-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_LIBRARY_STD_DINKUMWARE_H
#define CROSPI_BOOST_PREDEF_LIBRARY_STD_DINKUMWARE_H

#include <crospi_boost/predef/library/std/_prefix.h>

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_LIB_STD_DINKUMWARE`

http://en.wikipedia.org/wiki/Dinkumware[Dinkumware] Standard {CPP} Library.
If available version number as major, minor, and patch.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+_YVALS+`, `+__IBMCPP__+` | {predef_detection}
| `+_CPPLIB_VER+` | {predef_detection}

| `+_CPPLIB_VER+` | V.R.0
|===
*/ // end::reference[]

#define CROSPI_BOOST_LIB_STD_DINKUMWARE CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if (defined(_YVALS) && !defined(__IBMCPP__)) || defined(_CPPLIB_VER)
#   undef CROSPI_BOOST_LIB_STD_DINKUMWARE
#   if defined(_CPPLIB_VER)
#       define CROSPI_BOOST_LIB_STD_DINKUMWARE CROSPI_BOOST_PREDEF_MAKE_10_VVRR(_CPPLIB_VER)
#   else
#       define CROSPI_BOOST_LIB_STD_DINKUMWARE CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#   endif
#endif

#if CROSPI_BOOST_LIB_STD_DINKUMWARE
#   define CROSPI_BOOST_LIB_STD_DINKUMWARE_AVAILABLE
#endif

#define CROSPI_BOOST_LIB_STD_DINKUMWARE_NAME "Dinkumware"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_LIB_STD_DINKUMWARE,CROSPI_BOOST_LIB_STD_DINKUMWARE_NAME)
