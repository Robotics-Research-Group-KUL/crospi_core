/*
Copyright Rene Rivera 2011-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_LANGUAGE_OBJC_H
#define CROSPI_BOOST_PREDEF_LANGUAGE_OBJC_H

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_LANG_OBJC`

http://en.wikipedia.org/wiki/Objective-C[Objective-C] language.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__OBJC__+` | {predef_detection}
|===
*/ // end::reference[]

#define CROSPI_BOOST_LANG_OBJC CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__OBJC__)
#   undef CROSPI_BOOST_LANG_OBJC
#   define CROSPI_BOOST_LANG_OBJC CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#endif

#if CROSPI_BOOST_LANG_OBJC
#   define CROSPI_BOOST_LANG_OBJC_AVAILABLE
#endif

#define CROSPI_BOOST_LANG_OBJC_NAME "Objective-C"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_LANG_OBJC,CROSPI_BOOST_LANG_OBJC_NAME)
