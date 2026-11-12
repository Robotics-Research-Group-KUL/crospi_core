/*
Copyright (c) Microsoft Corporation 2014
Copyright Rene Rivera 2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_PLAT_WINDOWS_RUNTIME_H
#define CROSPI_BOOST_PREDEF_PLAT_WINDOWS_RUNTIME_H

#include <crospi_boost/predef/make.h>
#include <crospi_boost/predef/os/windows.h>
#include <crospi_boost/predef/platform/windows_phone.h>
#include <crospi_boost/predef/platform/windows_store.h>
#include <crospi_boost/predef/version_number.h>

/* tag::reference[]
= `CROSPI_BOOST_PLAT_WINDOWS_RUNTIME`

Deprecated.

https://docs.microsoft.com/en-us/windows/uwp/get-started/universal-application-platform-guide[UWP]
for Windows Phone or Store development.  This does not align to the existing development model for
UWP and is deprecated.  Use one of the other `CROSPI_BOOST_PLAT_WINDOWS_*`definitions instead.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `CROSPI_BOOST_PLAT_WINDOWS_PHONE` | {predef_detection}
| `CROSPI_BOOST_PLAT_WINDOWS_STORE` | {predef_detection}
|===
*/ // end::reference[]

#define CROSPI_BOOST_PLAT_WINDOWS_RUNTIME CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if CROSPI_BOOST_OS_WINDOWS && \
    (CROSPI_BOOST_PLAT_WINDOWS_STORE || CROSPI_BOOST_PLAT_WINDOWS_PHONE)
#   undef CROSPI_BOOST_PLAT_WINDOWS_RUNTIME
#   define CROSPI_BOOST_PLAT_WINDOWS_RUNTIME CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#endif
 
#if CROSPI_BOOST_PLAT_WINDOWS_RUNTIME
#   define CROSPI_BOOST_PLAT_WINDOWS_RUNTIME_AVAILABLE
#   include <crospi_boost/predef/detail/platform_detected.h>
#endif

#define CROSPI_BOOST_PLAT_WINDOWS_RUNTIME_NAME "Windows Runtime"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_PLAT_WINDOWS_RUNTIME,CROSPI_BOOST_PLAT_WINDOWS_RUNTIME_NAME)
