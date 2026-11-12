/*
Copyright Rene Rivera 2008-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#include <crospi_boost/predef/architecture/x86/32.h>
#include <crospi_boost/predef/architecture/x86/64.h>

#ifndef CROSPI_BOOST_PREDEF_ARCHITECTURE_X86_H
#define CROSPI_BOOST_PREDEF_ARCHITECTURE_X86_H

/* tag::reference[]
= `CROSPI_BOOST_ARCH_X86`

http://en.wikipedia.org/wiki/X86[Intel x86] architecture. This is
a category to indicate that either `CROSPI_BOOST_ARCH_X86_32` or
`CROSPI_BOOST_ARCH_X86_64` is detected.
*/ // end::reference[]

#define CROSPI_BOOST_ARCH_X86 CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if CROSPI_BOOST_ARCH_X86_32 || CROSPI_BOOST_ARCH_X86_64
#   undef CROSPI_BOOST_ARCH_X86
#   define CROSPI_BOOST_ARCH_X86 CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#endif

#if CROSPI_BOOST_ARCH_X86
#   define CROSPI_BOOST_ARCH_X86_AVAILABLE
#endif

#define CROSPI_BOOST_ARCH_X86_NAME "Intel x86"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_ARCH_X86,CROSPI_BOOST_ARCH_X86_NAME)
