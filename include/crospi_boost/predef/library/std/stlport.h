/*
Copyright Rene Rivera 2008-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_LIBRARY_STD_STLPORT_H
#define CROSPI_BOOST_PREDEF_LIBRARY_STD_STLPORT_H

#include <crospi_boost/predef/library/std/_prefix.h>

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_LIB_STD_STLPORT`

http://sourceforge.net/projects/stlport/[STLport Standard {CPP}] library.
Version number available as major, minor, and patch.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__SGI_STL_PORT+` | {predef_detection}
| `+_STLPORT_VERSION+` | {predef_detection}

| `+_STLPORT_MAJOR+`, `+_STLPORT_MINOR+`, `+_STLPORT_PATCHLEVEL+` | V.R.P
| `+_STLPORT_VERSION+` | V.R.P
| `+__SGI_STL_PORT+` | V.R.P
|===
*/ // end::reference[]

#define CROSPI_BOOST_LIB_STD_STLPORT CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__SGI_STL_PORT) || defined(_STLPORT_VERSION)
#   undef CROSPI_BOOST_LIB_STD_STLPORT
#   if !defined(CROSPI_BOOST_LIB_STD_STLPORT) && defined(_STLPORT_MAJOR)
#       define CROSPI_BOOST_LIB_STD_STLPORT \
            CROSPI_BOOST_VERSION_NUMBER(_STLPORT_MAJOR,_STLPORT_MINOR,_STLPORT_PATCHLEVEL)
#   endif
#   if !defined(CROSPI_BOOST_LIB_STD_STLPORT) && defined(_STLPORT_VERSION)
#       define CROSPI_BOOST_LIB_STD_STLPORT CROSPI_BOOST_PREDEF_MAKE_0X_VRP(_STLPORT_VERSION)
#   endif
#   if !defined(CROSPI_BOOST_LIB_STD_STLPORT)
#       define CROSPI_BOOST_LIB_STD_STLPORT CROSPI_BOOST_PREDEF_MAKE_0X_VRP(__SGI_STL_PORT)
#   endif
#endif

#if CROSPI_BOOST_LIB_STD_STLPORT
#   define CROSPI_BOOST_LIB_STD_STLPORT_AVAILABLE
#endif

#define CROSPI_BOOST_LIB_STD_STLPORT_NAME "STLport"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_LIB_STD_STLPORT,CROSPI_BOOST_LIB_STD_STLPORT_NAME)
