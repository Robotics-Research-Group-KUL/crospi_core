/*
Copyright Rene Rivera 2011-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_LIBRARY_STD_CXX_H
#define CROSPI_BOOST_PREDEF_LIBRARY_STD_CXX_H

#include <crospi_boost/predef/library/std/_prefix.h>

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_LIB_STD_CXX`

http://libcxx.llvm.org/[libc++] {CPP} Standard Library.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+_LIBCPP_VERSION+` | {predef_detection}

| `+_LIBCPP_VERSION+` | V.0.P
|===
*/ // end::reference[]

#define CROSPI_BOOST_LIB_STD_CXX CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(_LIBCPP_VERSION)
#   undef CROSPI_BOOST_LIB_STD_CXX
#   define CROSPI_BOOST_LIB_STD_CXX CROSPI_BOOST_PREDEF_MAKE_10_VVPPP(_LIBCPP_VERSION)
#endif

#if CROSPI_BOOST_LIB_STD_CXX
#   define CROSPI_BOOST_LIB_STD_CXX_AVAILABLE
#endif

#define CROSPI_BOOST_LIB_STD_CXX_NAME "libc++"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_LIB_STD_CXX,CROSPI_BOOST_LIB_STD_CXX_NAME)
