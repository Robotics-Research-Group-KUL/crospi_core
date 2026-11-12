/*
Copyright Rene Rivera 2008-2015
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_LIBRARY_STD_VACPP_H
#define CROSPI_BOOST_PREDEF_LIBRARY_STD_VACPP_H

#include <crospi_boost/predef/library/std/_prefix.h>

#include <crospi_boost/predef/version_number.h>
#include <crospi_boost/predef/make.h>

/* tag::reference[]
= `CROSPI_BOOST_LIB_STD_IBM`

http://www.ibm.com/software/awdtools/xlcpp/[IBM VACPP Standard {CPP}] library.

[options="header"]
|===
| {predef_symbol} | {predef_version}

| `+__IBMCPP__+` | {predef_detection}
|===
*/ // end::reference[]

#define CROSPI_BOOST_LIB_STD_IBM CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE

#if defined(__IBMCPP__)
#   undef CROSPI_BOOST_LIB_STD_IBM
#   define CROSPI_BOOST_LIB_STD_IBM CROSPI_BOOST_VERSION_NUMBER_AVAILABLE
#endif

#if CROSPI_BOOST_LIB_STD_IBM
#   define CROSPI_BOOST_LIB_STD_IBM_AVAILABLE
#endif

#define CROSPI_BOOST_LIB_STD_IBM_NAME "IBM VACPP"

#endif

#include <crospi_boost/predef/detail/test.h>
CROSPI_BOOST_PREDEF_DECLARE_TEST(CROSPI_BOOST_LIB_STD_IBM,CROSPI_BOOST_LIB_STD_IBM_NAME)
