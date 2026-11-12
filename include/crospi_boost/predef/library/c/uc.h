/*
Copyright Rene Rivera 2008-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_LIBRARY_C_UC_H
#define CROSPI_BOOST_PREDEF_LIBRARY_C_UC_H

#include <crospi_boost/predef/library/c/_prefix.h>

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_LIB_C_UC`

http://en.wikipedia.org/wiki/Uclibc[uClibc] Standard C library.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__UCLIBC__+` | {predef_detection}

| `+__UCLIBC_MAJOR__+`, `+__UCLIBC_MINOR__+`, `+__UCLIBC_SUBLEVEL__+` | V.R.P
|===
*/ // end::reference[]

#define CROSPI_BOOST_LIB_C_UC CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__UCLIBC__)
#   undef CROSPI_BOOST_LIB_C_UC
#   define CROSPI_BOOST_LIB_C_UC CROSPI_BOOST_VERSION_NUMBER(\
        __UCLIBC_MAJOR__,__UCLIBC_MINOR__,__UCLIBC_SUBLEVEL__)
#endif

#if CROSPI_BOOST_LIB_C_UC
#   define CROSPI_BOOST_LIB_C_UC_AVAILABLE
#endif

#define CROSPI_BOOST_LIB_C_UC_NAME "uClibc"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_LIB_C_UC,CROSPI_BOOST_LIB_C_UC_NAME)
