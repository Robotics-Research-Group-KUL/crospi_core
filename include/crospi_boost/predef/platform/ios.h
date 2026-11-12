/*
Copyright Ruslan Baratov 2017
Copyright Rene Rivera 2017
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_PLAT_IOS_H
#define CROSPI_BOOST_PREDEF_PLAT_IOS_H

#include <crospi_boost/predef/os/ios.h> // CROSPI_BOOST_OS_IOS
#include <crospi_boost/predef/version_number.h> // CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

/* tag::reference[]
= `CROSPI_BOOST_PLAT_IOS_DEVICE`
= `CROSPI_BOOST_PLAT_IOS_SIMULATOR`

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `TARGET_IPHONE_SIMULATOR` | {predef_detection}
| `TARGET_OS_SIMULATOR` | {predef_detection}
|===
*/ // end::reference[]

#define CROSPI_BOOST_PLAT_IOS_DEVICE CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE
#define CROSPI_BOOST_PLAT_IOS_SIMULATOR CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

// https://opensource.apple.com/source/CarbonHeaders/CarbonHeaders-18.1/TargetConditionals.h
#if CROSPI_BOOST_OS_IOS
#    include <TargetConditionals.h>
#    if defined(TARGET_OS_SIMULATOR) && (TARGET_OS_SIMULATOR == 1)
#        undef CROSPI_BOOST_PLAT_IOS_SIMULATOR
#        define CROSPI_BOOST_PLAT_IOS_SIMULATOR CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#    elif defined(TARGET_IPHONE_SIMULATOR) && (TARGET_IPHONE_SIMULATOR == 1)
#        undef CROSPI_BOOST_PLAT_IOS_SIMULATOR
#        define CROSPI_BOOST_PLAT_IOS_SIMULATOR CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#    else
#        undef CROSPI_BOOST_PLAT_IOS_DEVICE
#        define CROSPI_BOOST_PLAT_IOS_DEVICE CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#    endif
#endif

#if CROSPI_BOOST_PLAT_IOS_SIMULATOR
#    define CROSPI_BOOST_PLAT_IOS_SIMULATOR_AVAILABLE
#    include <crospi_boost/predef/detail/platform_detected.h>
#endif

#if CROSPI_BOOST_PLAT_IOS_DEVICE
#    define CROSPI_BOOST_PLAT_IOS_DEVICE_AVAILABLE
#    include <crospi_boost/predef/detail/platform_detected.h>
#endif

#define CROSPI_BOOST_PLAT_IOS_SIMULATOR_NAME "iOS Simulator"
#define CROSPI_BOOST_PLAT_IOS_DEVICE_NAME "iOS Device"

#endif // CROSPI_BOOST_PREDEF_PLAT_IOS_H

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_PLAT_IOS_SIMULATOR,CROSPI_BOOST_PLAT_IOS_SIMULATOR_NAME)
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_PLAT_IOS_DEVICE,CROSPI_BOOST_PLAT_IOS_DEVICE_NAME)
