/*
Copyright Rene Ferdinand Rivera Morell 2020-2021
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_OTHER_WORD_SIZE_H
#define CROSPI_BOOST_PREDEF_OTHER_WORD_SIZE_H

#include <crospi_boost/predef/architecture.h>
#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_ARCH_WORD_BITS`

Detects the native word size, in bits, for the current architecture. There are
two types of macros for this detection:

* `CROSPI_BOOST_ARCH_WORD_BITS`, gives the number of word size bits
  (16, 32, 64).
* `CROSPI_BOOST_ARCH_WORD_BITS_16`, `CROSPI_BOOST_ARCH_WORD_BITS_32`, and
  `CROSPI_BOOST_ARCH_WORD_BITS_64`, indicate when the given word size is
  detected.

They allow for both single checks and direct use of the size in code.

NOTE: The word size is determined manually on each architecture. Hence use of
the `wordsize.h` header will also include all the architecture headers.

*/ // end::reference[]

#if !defined(CROSPI_BOOST_ARCH_WORD_BITS_64)
#   define CROSPI_BOOST_ARCH_WORD_BITS_64 CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE
#elif !defined(CROSPI_BOOST_ARCH_WORD_BITS)
#   define CROSPI_BOOST_ARCH_WORD_BITS 64
#endif

#if !defined(CROSPI_BOOST_ARCH_WORD_BITS_32)
#   define CROSPI_BOOST_ARCH_WORD_BITS_32 CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE
#elif !defined(CROSPI_BOOST_ARCH_WORD_BITS)
#	  define CROSPI_BOOST_ARCH_WORD_BITS 32
#endif

#if !defined(CROSPI_BOOST_ARCH_WORD_BITS_16)
#   define CROSPI_BOOST_ARCH_WORD_BITS_16 CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE
#elif !defined(CROSPI_BOOST_ARCH_WORD_BITS)
#   define CROSPI_BOOST_ARCH_WORD_BITS 16
#endif

#if !defined(CROSPI_BOOST_ARCH_WORD_BITS)
#   define CROSPI_BOOST_ARCH_WORD_BITS 0
#endif

#define CROSPI_BOOST_ARCH_WORD_BITS_NAME "Word Bits"
#define CROSPI_BOOST_ARCH_WORD_BITS_16_NAME "16-bit Word Size"
#define CROSPI_BOOST_ARCH_WORD_BITS_32_NAME "32-bit Word Size"
#define CROSPI_BOOST_ARCH_WORD_BITS_64_NAME "64-bit Word Size"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_ARCH_WORD_BITS,CROSPI_BOOST_ARCH_WORD_BITS_NAME)

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_ARCH_WORD_BITS_16,CROSPI_BOOST_ARCH_WORD_BITS_16_NAME)

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_ARCH_WORD_BITS_32,CROSPI_BOOST_ARCH_WORD_BITS_32_NAME)

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_ARCH_WORD_BITS_64,CROSPI_BOOST_ARCH_WORD_BITS_64_NAME)
