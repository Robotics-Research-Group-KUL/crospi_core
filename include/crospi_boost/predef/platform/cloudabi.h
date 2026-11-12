/*
  Copyright 2017 James E. King, III
  Distributed under the Boost Software License, Version 1.0.
  (See accompanying file LICENSE_1_0.txt or copy at
    http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_PLAT_CLOUDABI_H
#define CROSPI_BOOST_PREDEF_PLAT_CLOUDABI_H

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_PLAT_CLOUDABI`

https://github.com/NuxiNL/cloudabi[CloudABI] platform.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__CloudABI__+` | {predef_detection}
|===
*/ // end::reference[]

#define CROSPI_BOOST_PLAT_CLOUDABI CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__CloudABI__)
#   undef CROSPI_BOOST_PLAT_CLOUDABI
#   define CROSPI_BOOST_PLAT_CLOUDABI CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#endif

#if CROSPI_BOOST_PLAT_CLOUDABI
#   define CROSPI_BOOST_PLAT_CLOUDABI_AVAILABLE
#   include <crospi_boost/predef/detail/platform_detected.h>
#endif

#define CROSPI_BOOST_PLAT_CLOUDABI_NAME "CloudABI"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_PLAT_CLOUDABI,CROSPI_BOOST_PLAT_CLOUDABI_NAME)
