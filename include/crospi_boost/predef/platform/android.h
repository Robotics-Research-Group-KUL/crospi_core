/*
Copyright Rene Rivera 2015-2019
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_PLAT_ANDROID_H
#define CROSPI_BOOST_PREDEF_PLAT_ANDROID_H

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_PLAT_ANDROID`

http://en.wikipedia.org/wiki/Android_%28operating_system%29[Android] platform.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__ANDROID__+` | {predef_detection}
|===
*/ // end::reference[]

#define CROSPI_BOOST_PLAT_ANDROID CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__ANDROID__)
#   undef CROSPI_BOOST_PLAT_ANDROID
#   define CROSPI_BOOST_PLAT_ANDROID CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#endif

#if CROSPI_BOOST_PLAT_ANDROID
#   define CROSPI_BOOST_PLAT_ANDROID_AVAILABLE
#   include <crospi_boost/predef/detail/platform_detected.h>
#endif

#define CROSPI_BOOST_PLAT_ANDROID_NAME "Android"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_PLAT_ANDROID,CROSPI_BOOST_PLAT_ANDROID_NAME)
