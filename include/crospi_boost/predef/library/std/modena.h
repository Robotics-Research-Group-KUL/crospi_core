/*
Copyright Rene Rivera 2008-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_LIBRARY_STD_MODENA_H
#define CROSPI_BOOST_PREDEF_LIBRARY_STD_MODENA_H

#include <crospi_boost/predef/library/std/_prefix.h>

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_LIB_STD_MSIPL`

http://modena.us/[Modena Software Lib++] Standard {CPP} Library.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `MSIPL_COMPILE_H` | {predef_detection}
| `+__MSIPL_COMPILE_H+` | {predef_detection}
|===
*/ // end::reference[]

#define CROSPI_BOOST_LIB_STD_MSIPL CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(MSIPL_COMPILE_H) || defined(__MSIPL_COMPILE_H)
#   undef CROSPI_BOOST_LIB_STD_MSIPL
#   define CROSPI_BOOST_LIB_STD_MSIPL CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#endif

#if CROSPI_BOOST_LIB_STD_MSIPL
#   define CROSPI_BOOST_LIB_STD_MSIPL_AVAILABLE
#endif

#define CROSPI_BOOST_LIB_STD_MSIPL_NAME "Modena Software Lib++"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_LIB_STD_MSIPL,CROSPI_BOOST_LIB_STD_MSIPL_NAME)
