/*
Copyright Rene Rivera 2005-2016
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_VERSION_NUMBER_H
#define CROSPI_BOOST_PREDEF_VERSION_NUMBER_H

/* tag::reference[]
= `CROSPI_BOOST_VERSION_NUMBER`

[source]
----
CROSPI_BOOST_VERSION_NUMBER(major,minor,patch)
----

Defines standard version numbers, with these properties:

* Decimal base whole numbers in the range [0,1000000000).
  The number range is designed to allow for a (2,2,5) triplet.
  Which fits within a 32 bit value.
* The `major` number can be in the [0,99] range.
* The `minor` number can be in the [0,99] range.
* The `patch` number can be in the [0,99999] range.
* Values can be specified in any base. As the defined value
  is an constant expression.
* Value can be directly used in both preprocessor and compiler
  expressions for comparison to other similarly defined values.
* The implementation enforces the individual ranges for the
  major, minor, and patch numbers. And values over the ranges
  are truncated (modulo).

*/ // end::reference[]
#define CROSPI_BOOST_VERSION_NUMBER(major,minor,patch) \
    ( (((major)%100)*10000000) + (((minor)%100)*100000) + ((patch)%100000) )

#define CROSPI_BOOST_VERSION_NUMBER_MAX \
    CROSPI_BOOST_VERSION_NUMBER(99,99,99999)

#define CROSPI_BOOST_VERSION_NUMBER_ZERO \
    CROSPI_BOOST_VERSION_NUMBER(0,0,0)

#define CROSPI_BOOST_VERSION_NUMBER_MIN \
    CROSPI_BOOST_VERSION_NUMBER(0,0,1)

#define CROSPI_BOOST_VERSION_NUMBER_AVAILABLE \
    CROSPI_BOOST_VERSION_NUMBER_MIN

#define CROSPI_BOOST_VERSION_NUMBER_NOT_AVAILABLE \
    CROSPI_BOOST_VERSION_NUMBER_ZERO

/* tag::reference[]
[source]
----
CROSPI_BOOST_VERSION_NUMBER_MAJOR(N), CROSPI_BOOST_VERSION_NUMBER_MINOR(N), CROSPI_BOOST_VERSION_NUMBER_PATCH(N)
----

The macros extract the major, minor, and patch portion from a well formed
version number resulting in a preprocessor expression in the range of
[0,99] or [0,99999] for the major and minor, or patch numbers
respectively.
*/ // end::reference[]
#define CROSPI_BOOST_VERSION_NUMBER_MAJOR(N) \
    ( ((N)/10000000)%100 )

#define CROSPI_BOOST_VERSION_NUMBER_MINOR(N) \
    ( ((N)/100000)%100 )

#define CROSPI_BOOST_VERSION_NUMBER_PATCH(N) \
    ( (N)%100000 )

#endif
