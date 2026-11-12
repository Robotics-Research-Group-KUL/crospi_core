/*
Copyright Charly Chevalier 2015
Copyright Joel Falcou 2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_HARDWARE_SIMD_PPC_H
#define CROSPI_BOOST_PREDEF_HARDWARE_SIMD_PPC_H

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/hardware/simd/ppc/versions.h>

/* tag::reference[]
= `CROSPI_BOOST_HW_SIMD_PPC`

The SIMD extension for PowerPC (*if detected*).
Version number depends on the most recent detected extension.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__VECTOR4DOUBLE__+` | {predef_detection}

| `+__ALTIVEC__+` | {predef_detection}
| `+__VEC__+` | {predef_detection}

| `+__VSX__+` | {predef_detection}
|===

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__VECTOR4DOUBLE__+` | CROSPI_BOOST_HW_SIMD_PPC_QPX_VERSION

| `+__ALTIVEC__+` | CROSPI_BOOST_HW_SIMD_PPC_VMX_VERSION
| `+__VEC__+` | CROSPI_BOOST_HW_SIMD_PPC_VMX_VERSION

| `+__VSX__+` | CROSPI_BOOST_HW_SIMD_PPC_VSX_VERSION
|===

*/ // end::reference[]

#define CROSPI_BOOST_HW_SIMD_PPC CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#undef CROSPI_BOOST_HW_SIMD_PPC
#if !defined(CROSPI_BOOST_HW_SIMD_PPC) && defined(__VECTOR4DOUBLE__)
#   define CROSPI_BOOST_HW_SIMD_PPC CROSPI_BOOST_HW_SIMD_PPC_QPX_VERSION
#endif
#if !defined(CROSPI_BOOST_HW_SIMD_PPC) && defined(__VSX__)
#   define CROSPI_BOOST_HW_SIMD_PPC CROSPI_BOOST_HW_SIMD_PPC_VSX_VERSION
#endif
#if !defined(CROSPI_BOOST_HW_SIMD_PPC) && (defined(__ALTIVEC__) || defined(__VEC__))
#   define CROSPI_BOOST_HW_SIMD_PPC CROSPI_BOOST_HW_SIMD_PPC_VMX_VERSION
#endif

#if !defined(CROSPI_BOOST_HW_SIMD_PPC)
#   define CROSPI_BOOST_HW_SIMD_PPC CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE
#else
#   define CROSPI_BOOST_HW_SIMD_PPC_AVAILABLE
#endif

#define CROSPI_BOOST_HW_SIMD_PPC_NAME "PPC SIMD"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_HW_SIMD_PPC, CROSPI_BOOST_HW_SIMD_PPC_NAME)
