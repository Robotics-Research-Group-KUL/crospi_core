/*
Copyright Rene Rivera 2008-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_COMPILER_HP_ACC_H
#define CROSPI_BOOST_PREDEF_COMPILER_HP_ACC_H

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_COMP_HPACC`

HP a{CPP} compiler.
Version number available as major, minor, and patch.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__HP_aCC+` | {predef_detection}

| `+__HP_aCC+` | V.R.P
|===
*/ // end::reference[]

#define CROSPI_BOOST_COMP_HPACC CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__HP_aCC)
#   if !defined(CROSPI_BOOST_COMP_HPACC_DETECTION) && (__HP_aCC > 1)
#       define CROSPI_BOOST_COMP_HPACC_DETECTION CROSPI_BOOST_PREDEF_MAKE_10_VVRRPP(__HP_aCC)
#   endif
#   if !defined(CROSPI_BOOST_COMP_HPACC_DETECTION)
#       define CROSPI_BOOST_COMP_HPACC_DETECTION CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#   endif
#endif

#ifdef CROSPI_BOOST_COMP_HPACC_DETECTION
#   if defined(CROSPI_BOOST_PREDEF_DETAIL_COMP_DETECTED)
#       define CROSPI_BOOST_COMP_HPACC_EMULATED CROSPI_BOOST_COMP_HPACC_DETECTION
#   else
#       undef CROSPI_BOOST_COMP_HPACC
#       define CROSPI_BOOST_COMP_HPACC CROSPI_BOOST_COMP_HPACC_DETECTION
#   endif
#   define CROSPI_BOOST_COMP_HPACC_AVAILABLE
#   include <crospi_boost/predef/detail/comp_detected.h>
#endif

#define CROSPI_BOOST_COMP_HPACC_NAME "HP aC++"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_COMP_HPACC,CROSPI_BOOST_COMP_HPACC_NAME)

#ifdef CROSPI_BOOST_COMP_HPACC_EMULATED
#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_COMP_HPACC_EMULATED,CROSPI_BOOST_COMP_HPACC_NAME)
#endif
