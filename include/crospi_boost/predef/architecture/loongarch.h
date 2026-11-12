/*
Copyright Zhang Na 2022
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_ARCHITECTURE_LOONGARCH_H
#define CROSPI_BOOST_PREDEF_ARCHITECTURE_LOONGARCH_H

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_ARCH_LOONGARCH`

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__loongarch__+` | {predef_detection}
|===
*/ // end::reference[]

#define CROSPI_BOOST_ARCH_LOONGARCH CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__loongarch__)
#   undef CROSPI_BOOST_ARCH_LOONGARCH
#   define CROSPI_BOOST_ARCH_LOONGARCH CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#endif

#if CROSPI_BOOST_ARCH_LOONGARCH
#   define CROSPI_BOOST_ARCH_LOONGARCH_AVAILABLE
#endif

#define CROSPI_BOOST_ARCH_LOONGARCH_NAME "LoongArch"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_ARCH_LOONGARCH,CROSPI_BOOST_ARCH_LOONGARCH_NAME)
