/*
Copyright Andreas Schwab 2019
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_ARCHITECTURE_RISCV_H
#define CROSPI_BOOST_PREDEF_ARCHITECTURE_RISCV_H

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_ARCH_RISCV`

http://en.wikipedia.org/wiki/RISC-V[RISC-V] architecture.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__riscv+` | {predef_detection}
|===
*/ // end::reference[]

#define CROSPI_BOOST_ARCH_RISCV CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__riscv)
#   undef CROSPI_BOOST_ARCH_RISCV
#   define CROSPI_BOOST_ARCH_RISCV CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#endif

#if CROSPI_BOOST_ARCH_RISCV
#   define CROSPI_BOOST_ARCH_RISCV_AVAILABLE
#endif

#if CROSPI_BOOST_ARCH_RISCV
#   undef CROSPI_BOOST_ARCH_WORD_BITS_32
#   define CROSPI_BOOST_ARCH_WORD_BITS_32 CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#endif

#define CROSPI_BOOST_ARCH_RISCV_NAME "RISC-V"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_ARCH_RISCV,CROSPI_BOOST_ARCH_RISCV_NAME)
