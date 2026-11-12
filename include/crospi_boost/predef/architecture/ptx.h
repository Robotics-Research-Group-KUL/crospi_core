/*
Copyright Benjamin Worpitz 2018
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_ARCHITECTURE_PTX_H
#define CROSPI_BOOST_PREDEF_ARCHITECTURE_PTX_H

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_ARCH_PTX`

https://en.wikipedia.org/wiki/Parallel_Thread_Execution[PTX] architecture.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__CUDA_ARCH__+` | {predef_detection}

| `+__CUDA_ARCH__+` | V.R.0
|===
*/ // end::reference[]

#define CROSPI_BOOST_ARCH_PTX CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__CUDA_ARCH__)
#   undef CROSPI_BOOST_ARCH_PTX
#   define CROSPI_BOOST_ARCH_PTX CROSPI_BOOST_PREDEF_MAKE_10_VR0(__CUDA_ARCH__)
#endif

#if CROSPI_BOOST_ARCH_PTX
#   define CROSPI_BOOST_ARCH_PTX_AVAILABLE
#endif

#if CROSPI_BOOST_ARCH_PTX
#   undef CROSPI_BOOST_ARCH_WORD_BITS_64
#   define CROSPI_BOOST_ARCH_WORD_BITS_64 CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#endif

#define CROSPI_BOOST_ARCH_PTX_NAME "PTX"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_ARCH_PTX,CROSPI_BOOST_ARCH_PTX_NAME)
