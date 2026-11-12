/*
 * Copyright (C) 2017 James E. King III
 *
 * Distributed under the Boost Software License, Version 1.0.
 * (See accompanying file LICENSE_1_0.txt or copy at
 *   http://www.boost.org/LICENSE_1_0.txt)
 */

#ifndef CROSPI_BOOST_PREDEF_LIBRARY_C_CLOUDABI_H
#define CROSPI_BOOST_PREDEF_LIBRARY_C_CLOUDABI_H

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

#include <crospi_boost/predef/library/c/_prefix.h>

#if defined(__CloudABI__)
#include <stddef.h>
#endif

/* tag::reference[]
= `CROSPI_BOOST_LIB_C_CLOUDABI`

https://github.com/NuxiNL/cloudlibc[cloudlibc] - CloudABI's standard C library.
Version number available as major, and minor.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__cloudlibc__+` | {predef_detection}

| `+__cloudlibc_major__+`, `+__cloudlibc_minor__+` | V.R.0
|===
*/ // end::reference[]

#define CROSPI_BOOST_LIB_C_CLOUDABI CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__cloudlibc__)
#   undef CROSPI_BOOST_LIB_C_CLOUDABI
#   define CROSPI_BOOST_LIB_C_CLOUDABI \
            CROSPI_BOOST_VERSION_NUMBER(__cloudlibc_major__,__cloudlibc_minor__,0)
#endif

#if CROSPI_BOOST_LIB_C_CLOUDABI
#   define CROSPI_BOOST_LIB_C_CLOUDABI_AVAILABLE
#endif

#define CROSPI_BOOST_LIB_C_CLOUDABI_NAME "cloudlibc"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_LIB_C_CLOUDABI,CROSPI_BOOST_LIB_C_CLOUDABI_NAME)
