/*
Copyright Rene Rivera 2017
Distributed under the Boost Software License, Version 1.0.
(See accompanying file LICENSE_1_0.txt or copy at
http://www.boost.org/LICENSE_1_0.txt)
*/

#ifndef CROSPI_BOOST_PREDEF_WORKAROUND_H
#define CROSPI_BOOST_PREDEF_WORKAROUND_H

/* tag::reference[]

= `CROSPI_BOOST_PREDEF_WORKAROUND`

[source]
----
CROSPI_BOOST_PREDEF_WORKAROUND(symbol,comp,major,minor,patch)
----

Usage:

[source]
----
#if CROSPI_BOOST_PREDEF_WORKAROUND(CROSPI_BOOST_COMP_CLANG,<,3,0,0)
    // Workaround for old clang compilers..
#endif
----

Defines a comparison against two version numbers that depends on the definion
of `CROSPI_BOOST_STRICT_CONFIG`. When `CROSPI_BOOST_STRICT_CONFIG` is defined this will expand
to a value convertible to `false`. Which has the effect of disabling all code
conditionally guarded by `CROSPI_BOOST_PREDEF_WORKAROUND`. When `CROSPI_BOOST_STRICT_CONFIG`
is undefine this expand to test the given `symbol` version value with the
`comp` comparison against `CROSPI_BOOST_VERSION_NUMBER(major,minor,patch)`.

*/ // end::reference[]
#ifdef CROSPI_BOOST_STRICT_CONFIG
#   define CROSPI_BOOST_PREDEF_WORKAROUND(symbol, comp, major, minor, patch) (0)
#else
#   include <crospi_boost/predef/version_number.h>
#   define CROSPI_BOOST_PREDEF_WORKAROUND(symbol, comp, major, minor, patch) \
        ( (symbol) != (0) ) && \
        ( (symbol) comp (CROSPI_BOOST_VERSION_NUMBER( (major) , (minor) , (patch) )) )
#endif

/* tag::reference[]

= `CROSPI_BOOST_PREDEF_TESTED_AT`

[source]
----
CROSPI_BOOST_PREDEF_TESTED_AT(symbol,major,minor,patch)
----

Usage:

[source]
----
#if CROSPI_BOOST_PREDEF_TESTED_AT(CROSPI_BOOST_COMP_CLANG,3,5,0)
    // Needed for clang, and last checked for 3.5.0.
#endif
----

Defines a comparison against two version numbers that depends on the definion
of `CROSPI_BOOST_STRICT_CONFIG` and `CROSPI_BOOST_DETECT_OUTDATED_WORKAROUNDS`.
When `CROSPI_BOOST_STRICT_CONFIG` is defined this will expand to a value convertible
to `false`. Which has the effect of disabling all code
conditionally guarded by `CROSPI_BOOST_PREDEF_TESTED_AT`. When `CROSPI_BOOST_STRICT_CONFIG`
is undefined this expand to either:

* A value convertible to `true` when `CROSPI_BOOST_DETECT_OUTDATED_WORKAROUNDS` is not
  defined.
* A value convertible `true` when the expansion of
  `CROSPI_BOOST_PREDEF_WORKAROUND(symbol, <=, major, minor, patch)` is `true` and
  `CROSPI_BOOST_DETECT_OUTDATED_WORKAROUNDS` is defined.
* A compile error when the expansion of
  `CROSPI_BOOST_PREDEF_WORKAROUND(symbol, >, major, minor, patch)` is true and
  `CROSPI_BOOST_DETECT_OUTDATED_WORKAROUNDS` is defined.

*/ // end::reference[]
#ifdef CROSPI_BOOST_STRICT_CONFIG
#   define CROSPI_BOOST_PREDEF_TESTED_AT(symbol, major, minor, patch) (0)
#else
#   ifdef CROSPI_BOOST_DETECT_OUTDATED_WORKAROUNDS
#       define CROSPI_BOOST_PREDEF_TESTED_AT(symbol, major, minor, patch) ( \
            CROSPI_BOOST_PREDEF_WORKAROUND(symbol, <=, major, minor, patch) \
            ? 1 \
            : (1%0) )
#   else
#       define CROSPI_BOOST_PREDEF_TESTED_AT(symbol, major, minor, patch) \
            ( (symbol) >= CROSPI_BOOST_VERSION_NUMBER_AVAILABLE )
#   endif
#endif

#endif
