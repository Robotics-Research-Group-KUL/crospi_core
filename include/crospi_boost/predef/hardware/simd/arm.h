/*
Copyright Charly Chevalier 2015
Copyright Joel Falcou 2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_HARDWARE_SIMD_ARM_H
#define CROSPI_BOOST_PREDEF_HARDWARE_SIMD_ARM_H

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/hardware/simd/arm/versions.h>

/* tag::reference[]
= `CROSPI_BOOST_HW_SIMD_ARM`

The SIMD extension for ARM (*if detected*).
Version number depends on the most recent detected extension.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__ARM_NEON__+` | {predef_detection}
| `+__aarch64__+` | {predef_detection}
| `+_M_ARM+` | {predef_detection}
| `+_M_ARM64+` | {predef_detection}
|===

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__ARM_NEON__+` | CROSPI_BOOST_HW_SIMD_ARM_NEON_VERSION
| `+__aarch64__+` | CROSPI_BOOST_HW_SIMD_ARM_NEON_VERSION
| `+_M_ARM+` | CROSPI_BOOST_HW_SIMD_ARM_NEON_VERSION
| `+_M_ARM64+` | CROSPI_BOOST_HW_SIMD_ARM_NEON_VERSION
|===

*/ // end::reference[]

#define CROSPI_BOOST_HW_SIMD_ARM CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#undef CROSPI_BOOST_HW_SIMD_ARM
#if !defined(CROSPI_BOOST_HW_SIMD_ARM) && (defined(__ARM_NEON__) || defined(__aarch64__) || defined (_M_ARM) || defined (_M_ARM64))
#   define CROSPI_BOOST_HW_SIMD_ARM CROSPI_BOOST_HW_SIMD_ARM_NEON_VERSION
#endif

#if !defined(CROSPI_BOOST_HW_SIMD_ARM)
#   define CROSPI_BOOST_HW_SIMD_ARM CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE
#else
#   define CROSPI_BOOST_HW_SIMD_ARM_AVAILABLE
#endif

#define CROSPI_BOOST_HW_SIMD_ARM_NAME "ARM SIMD"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_HW_SIMD_ARM, CROSPI_BOOST_HW_SIMD_ARM_NAME)
