/*
Copyright Rene Rivera 2008-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_LIBRARY_STD_STDCPP3_H
#define CROSPI_BOOST_PREDEF_LIBRARY_STD_STDCPP3_H

#include <crospi_boost/predef/library/std/_prefix.h>

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_LIB_STD_GNU`

https://gcc.gnu.org/onlinedocs/libstdc%2b%2b/[GNU libstdc++] Standard {CPP} library.
Version number available as year (from 1970), month, and day.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__GLIBCXX__+` | {predef_detection}
| `+__GLIBCPP__+` | {predef_detection}

| `+__GLIBCXX__+` | V.R.P
| `+__GLIBCPP__+` | V.R.P
|===
*/ // end::reference[]

#define CROSPI_BOOST_LIB_STD_GNU CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__GLIBCPP__) || defined(__GLIBCXX__)
#   undef CROSPI_BOOST_LIB_STD_GNU
#   if defined(__GLIBCXX__)
#       define CROSPI_BOOST_LIB_STD_GNU CROSPI_BOOST_PREDEF_MAKE_YYYYMMDD(__GLIBCXX__)
#   else
#       define CROSPI_BOOST_LIB_STD_GNU CROSPI_BOOST_PREDEF_MAKE_YYYYMMDD(__GLIBCPP__)
#   endif
#endif

#if CROSPI_BOOST_LIB_STD_GNU
#   define CROSPI_BOOST_LIB_STD_GNU_AVAILABLE
#endif

#define CROSPI_BOOST_LIB_STD_GNU_NAME "GNU"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_LIB_STD_GNU,CROSPI_BOOST_LIB_STD_GNU_NAME)
