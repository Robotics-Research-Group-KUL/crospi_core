/*
Copyright Rene Rivera 2008-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_LIBRARY_STD_ROGUEWAVE_H
#define CROSPI_BOOST_PREDEF_LIBRARY_STD_ROGUEWAVE_H

#include <crospi_boost/predef/library/std/_prefix.h>

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_LIB_STD_RW`

http://stdcxx.apache.org/[Roguewave] Standard {CPP} library.
If available version number as major, minor, and patch.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__STD_RWCOMPILER_H__+` | {predef_detection}
| `+_RWSTD_VER+` | {predef_detection}

| `+_RWSTD_VER+` | V.R.P
|===
*/ // end::reference[]

#define CROSPI_BOOST_LIB_STD_RW CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__STD_RWCOMPILER_H__) || defined(_RWSTD_VER)
#   undef CROSPI_BOOST_LIB_STD_RW
#   if defined(_RWSTD_VER)
#       if _RWSTD_VER < 0x010000
#           define CROSPI_BOOST_LIB_STD_RW CROSPI_BOOST_PREDEF_MAKE_0X_VVRRP(_RWSTD_VER)
#       else
#           define CROSPI_BOOST_LIB_STD_RW CROSPI_BOOST_PREDEF_MAKE_0X_VVRRPP(_RWSTD_VER)
#       endif
#   else
#       define CROSPI_BOOST_LIB_STD_RW CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#   endif
#endif

#if CROSPI_BOOST_LIB_STD_RW
#   define CROSPI_BOOST_LIB_STD_RW_AVAILABLE
#endif

#define CROSPI_BOOST_LIB_STD_RW_NAME "Roguewave"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_LIB_STD_RW,CROSPI_BOOST_LIB_STD_RW_NAME)
