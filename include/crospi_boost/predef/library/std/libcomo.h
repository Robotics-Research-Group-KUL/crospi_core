/*
Copyright Rene Rivera 2008-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_LIBRARY_STD_LIBCOMO_H
#define CROSPI_BOOST_PREDEF_LIBRARY_STD_LIBCOMO_H

#include <crospi_boost/predef/library/std/_prefix.h>

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_LIB_STD_COMO`

http://www.comeaucomputing.com/libcomo/[Comeau Computing] Standard {CPP} Library.
Version number available as major.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__LIBCOMO__+` | {predef_detection}

| `+__LIBCOMO_VERSION__+` | V.0.0
|===
*/ // end::reference[]

#define CROSPI_BOOST_LIB_STD_COMO CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__LIBCOMO__)
#   undef CROSPI_BOOST_LIB_STD_COMO
#   define CROSPI_BOOST_LIB_STD_COMO CROSPI_BOOST_VERSION_NUMBER(__LIBCOMO_VERSION__,0,0)
#endif

#if CROSPI_BOOST_LIB_STD_COMO
#   define CROSPI_BOOST_LIB_STD_COMO_AVAILABLE
#endif

#define CROSPI_BOOST_LIB_STD_COMO_NAME "Comeau Computing"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_LIB_STD_COMO,CROSPI_BOOST_LIB_STD_COMO_NAME)
