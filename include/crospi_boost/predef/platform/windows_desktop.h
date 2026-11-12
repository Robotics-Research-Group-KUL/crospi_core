/*
Copyright (c) Microsoft Corporation 2014
Copyright Rene Rivera 2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_PLAT_WINDOWS_DESKTOP_H
#define CROSPI_BOOST_PREDEF_PLAT_WINDOWS_DESKTOP_H

#include <crospi_boost/predef/make.h>
#include <crospi_boost/predef/os/windows.h>
#include <crospi_boost/predef/platform/windows_uwp.h>
#include <crospi_boost/predef/version_number.h>

/* tag::reference[]
= `CROSPI_BOOST_PLAT_WINDOWS_DESKTOP`

https://docs.microsoft.com/en-us/windows/uwp/get-started/universal-application-platform-guide[UWP]
for Windows Desktop development.  Also available if the Platform SDK is too
old to support UWP.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `WINAPI_FAMILY == WINAPI_FAMILY_DESKTOP_APP` | {predef_detection}
| `!CROSPI_BOOST_PLAT_WINDOWS_UWP` | {predef_detection}
|===
*/ // end::reference[]

#define CROSPI_BOOST_PLAT_WINDOWS_DESKTOP CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if CROSPI_BOOST_OS_WINDOWS && \
    ((defined(WINAPI_FAMILY_DESKTOP_APP) && WINAPI_FAMILY == WINAPI_FAMILY_DESKTOP_APP) || \
     !CROSPI_BOOST_PLAT_WINDOWS_UWP)
#   undef CROSPI_BOOST_PLAT_WINDOWS_DESKTOP
#   define CROSPI_BOOST_PLAT_WINDOWS_DESKTOP CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#endif
 
#if CROSPI_BOOST_PLAT_WINDOWS_DESKTOP
#   define CROSPI_BOOST_PLAT_WINDOWS_DESKTOP_AVAILABLE
#   include <crospi_boost/predef/detail/platform_detected.h>
#endif

#define CROSPI_BOOST_PLAT_WINDOWS_DESKTOP_NAME "Windows Desktop"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_PLAT_WINDOWS_DESKTOP,CROSPI_BOOST_PLAT_WINDOWS_DESKTOP_NAME)
