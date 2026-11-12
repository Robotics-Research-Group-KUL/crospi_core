/*
Copyright Rene Rivera 2008-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_COMPILER_LLVM_H
#define CROSPI_BOOST_PREDEF_COMPILER_LLVM_H

/* Other compilers that emulate this one need to be detected first. */

#include <crospi_boost/predef/compiler/clang.h>

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_COMP_LLVM`

http://en.wikipedia.org/wiki/LLVM[LLVM] compiler.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__llvm__+` | {predef_detection}
|===
*/ // end::reference[]

#define CROSPI_BOOST_COMP_LLVM CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__llvm__)
#   define CROSPI_BOOST_COMP_LLVM_DETECTION CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#endif

#ifdef CROSPI_BOOST_COMP_LLVM_DETECTION
#   if defined(CROSPI_BOOST_PREDEF_DETAIL_COMP_DETECTED)
#       define CROSPI_BOOST_COMP_LLVM_EMULATED CROSPI_BOOST_COMP_LLVM_DETECTION
#   else
#       undef CROSPI_BOOST_COMP_LLVM
#       define CROSPI_BOOST_COMP_LLVM CROSPI_BOOST_COMP_LLVM_DETECTION
#   endif
#   define CROSPI_BOOST_COMP_LLVM_AVAILABLE
#   include <crospi_boost/predef/detail/comp_detected.h>
#endif

#define CROSPI_BOOST_COMP_LLVM_NAME "LLVM"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_COMP_LLVM,CROSPI_BOOST_COMP_LLVM_NAME)

#ifdef CROSPI_BOOST_COMP_LLVM_EMULATED
#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_COMP_LLVM_EMULATED,CROSPI_BOOST_COMP_LLVM_NAME)
#endif
