// Copyright David Abrahams 2002.
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
#ifndef CROSPI_BOOST_CONFIG_WORKAROUND_HPP
#define CROSPI_BOOST_CONFIG_WORKAROUND_HPP

// Compiler/library version workaround macro
//
// Usage:
//
//   #if CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_MSVC, < 1300)
//      // workaround for eVC4 and VC6
//      ... // workaround code here
//   #endif
//
// When CROSPI_BOOST_STRICT_CONFIG is defined, expands to 0. Otherwise, the
// first argument must be undefined or expand to a numeric
// value. The above expands to:
//
//   (CROSPI_BOOST_MSVC) != 0 && (CROSPI_BOOST_MSVC) < 1300
//
// When used for workarounds that apply to the latest known version
// and all earlier versions of a compiler, the following convention
// should be observed:
//
//   #if CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_MSVC, CROSPI_BOOST_TESTED_AT(1301))
//
// The version number in this case corresponds to the last version in
// which the workaround was known to have been required. When
// CROSPI_BOOST_DETECT_OUTDATED_WORKAROUNDS is not the defined, the macro
// CROSPI_BOOST_TESTED_AT(x) expands to "!= 0", which effectively activates
// the workaround for any version of the compiler. When
// CROSPI_BOOST_DETECT_OUTDATED_WORKAROUNDS is defined, a compiler warning or
// error will be issued if the compiler version exceeds the argument
// to CROSPI_BOOST_TESTED_AT().  This can be used to locate workarounds which
// may be obsoleted by newer versions.

#ifndef CROSPI_BOOST_STRICT_CONFIG

#include <crospi_boost/config.hpp>

#ifndef __BORLANDC__
#define __BORLANDC___WORKAROUND_GUARD 1
#else
#define __BORLANDC___WORKAROUND_GUARD 0
#endif
#ifndef __CODEGEARC__
#define __CODEGEARC___WORKAROUND_GUARD 1
#else
#define __CODEGEARC___WORKAROUND_GUARD 0
#endif
#ifndef CROSPI_BOOST_BORLANDC
#define CROSPI_BOOST_BORLANDC_WORKAROUND_GUARD 1
#else
#define CROSPI_BOOST_BORLANDC_WORKAROUND_GUARD 0
#endif
#ifndef CROSPI_BOOST_CODEGEARC
#define CROSPI_BOOST_CODEGEARC_WORKAROUND_GUARD 1
#else
#define CROSPI_BOOST_CODEGEARC_WORKAROUND_GUARD 0
#endif
#ifndef CROSPI_BOOST_EMBTC
#define CROSPI_BOOST_EMBTC_WORKAROUND_GUARD 1
#else
#define CROSPI_BOOST_EMBTC_WORKAROUND_GUARD 0
#endif
#ifndef _MSC_VER
#define _MSC_VER_WORKAROUND_GUARD 1
#else
#define _MSC_VER_WORKAROUND_GUARD 0
#endif
#ifndef _MSC_FULL_VER
#define _MSC_FULL_VER_WORKAROUND_GUARD 1
#else
#define _MSC_FULL_VER_WORKAROUND_GUARD 0
#endif
#ifndef CROSPI_BOOST_MSVC
#define CROSPI_BOOST_MSVC_WORKAROUND_GUARD 1
#else
#define CROSPI_BOOST_MSVC_WORKAROUND_GUARD 0
#endif
#ifndef CROSPI_BOOST_MSVC_FULL_VER
#define CROSPI_BOOST_MSVC_FULL_VER_WORKAROUND_GUARD 1
#else
#define CROSPI_BOOST_MSVC_FULL_VER_WORKAROUND_GUARD 0
#endif
#ifndef __GNUC__
#define __GNUC___WORKAROUND_GUARD 1
#else
#define __GNUC___WORKAROUND_GUARD 0
#endif
#ifndef __GNUC_MINOR__
#define __GNUC_MINOR___WORKAROUND_GUARD 1
#else
#define __GNUC_MINOR___WORKAROUND_GUARD 0
#endif
#ifndef __GNUC_PATCHLEVEL__
#define __GNUC_PATCHLEVEL___WORKAROUND_GUARD 1
#else
#define __GNUC_PATCHLEVEL___WORKAROUND_GUARD 0
#endif
#ifndef CROSPI_BOOST_GCC
#define CROSPI_BOOST_GCC_WORKAROUND_GUARD 1
#define CROSPI_BOOST_GCC_VERSION_WORKAROUND_GUARD 1
#else
#define CROSPI_BOOST_GCC_WORKAROUND_GUARD 0
#define CROSPI_BOOST_GCC_VERSION_WORKAROUND_GUARD 0
#endif
#ifndef CROSPI_BOOST_XLCPP_ZOS
#define CROSPI_BOOST_XLCPP_ZOS_WORKAROUND_GUARD 1
#else
#define CROSPI_BOOST_XLCPP_ZOS_WORKAROUND_GUARD 0
#endif
#ifndef __IBMCPP__
#define __IBMCPP___WORKAROUND_GUARD 1
#else
#define __IBMCPP___WORKAROUND_GUARD 0
#endif
#ifndef __SUNPRO_CC
#define __SUNPRO_CC_WORKAROUND_GUARD 1
#else
#define __SUNPRO_CC_WORKAROUND_GUARD 0
#endif
#ifndef __DECCXX_VER
#define __DECCXX_VER_WORKAROUND_GUARD 1
#else
#define __DECCXX_VER_WORKAROUND_GUARD 0
#endif
#ifndef __MWERKS__
#define __MWERKS___WORKAROUND_GUARD 1
#else
#define __MWERKS___WORKAROUND_GUARD 0
#endif
#ifndef __EDG__
#define __EDG___WORKAROUND_GUARD 1
#else
#define __EDG___WORKAROUND_GUARD 0
#endif
#ifndef __EDG_VERSION__
#define __EDG_VERSION___WORKAROUND_GUARD 1
#else
#define __EDG_VERSION___WORKAROUND_GUARD 0
#endif
#ifndef __HP_aCC
#define __HP_aCC_WORKAROUND_GUARD 1
#else
#define __HP_aCC_WORKAROUND_GUARD 0
#endif
#ifndef __hpxstd98
#define __hpxstd98_WORKAROUND_GUARD 1
#else
#define __hpxstd98_WORKAROUND_GUARD 0
#endif
#ifndef _CRAYC
#define _CRAYC_WORKAROUND_GUARD 1
#else
#define _CRAYC_WORKAROUND_GUARD 0
#endif
#ifndef __DMC__
#define __DMC___WORKAROUND_GUARD 1
#else
#define __DMC___WORKAROUND_GUARD 0
#endif
#ifndef MPW_CPLUS
#define MPW_CPLUS_WORKAROUND_GUARD 1
#else
#define MPW_CPLUS_WORKAROUND_GUARD 0
#endif
#ifndef __COMO__
#define __COMO___WORKAROUND_GUARD 1
#else
#define __COMO___WORKAROUND_GUARD 0
#endif
#ifndef __COMO_VERSION__
#define __COMO_VERSION___WORKAROUND_GUARD 1
#else
#define __COMO_VERSION___WORKAROUND_GUARD 0
#endif
#ifndef __INTEL_COMPILER
#define __INTEL_COMPILER_WORKAROUND_GUARD 1
#else
#define __INTEL_COMPILER_WORKAROUND_GUARD 0
#endif
#ifndef __ICL
#define __ICL_WORKAROUND_GUARD 1
#else
#define __ICL_WORKAROUND_GUARD 0
#endif
#ifndef _COMPILER_VERSION
#define _COMPILER_VERSION_WORKAROUND_GUARD 1
#else
#define _COMPILER_VERSION_WORKAROUND_GUARD 0
#endif
#ifndef __clang_major__
#define __clang_major___WORKAROUND_GUARD 1
#else
#define __clang_major___WORKAROUND_GUARD 0
#endif

#ifndef _RWSTD_VER
#define _RWSTD_VER_WORKAROUND_GUARD 1
#else
#define _RWSTD_VER_WORKAROUND_GUARD 0
#endif
#ifndef CROSPI_BOOST_RWSTD_VER
#define CROSPI_BOOST_RWSTD_VER_WORKAROUND_GUARD 1
#else
#define CROSPI_BOOST_RWSTD_VER_WORKAROUND_GUARD 0
#endif
#ifndef __GLIBCPP__
#define __GLIBCPP___WORKAROUND_GUARD 1
#else
#define __GLIBCPP___WORKAROUND_GUARD 0
#endif
#ifndef _GLIBCXX_USE_C99_FP_MACROS_DYNAMIC
#define _GLIBCXX_USE_C99_FP_MACROS_DYNAMIC_WORKAROUND_GUARD 1
#else
#define _GLIBCXX_USE_C99_FP_MACROS_DYNAMIC_WORKAROUND_GUARD 0
#endif
#ifndef __SGI_STL_PORT
#define __SGI_STL_PORT_WORKAROUND_GUARD 1
#else
#define __SGI_STL_PORT_WORKAROUND_GUARD 0
#endif
#ifndef _STLPORT_VERSION
#define _STLPORT_VERSION_WORKAROUND_GUARD 1
#else
#define _STLPORT_VERSION_WORKAROUND_GUARD 0
#endif
#ifndef __LIBCOMO_VERSION__
#define __LIBCOMO_VERSION___WORKAROUND_GUARD 1
#else
#define __LIBCOMO_VERSION___WORKAROUND_GUARD 0
#endif
#ifndef _CPPLIB_VER
#define _CPPLIB_VER_WORKAROUND_GUARD 1
#else
#define _CPPLIB_VER_WORKAROUND_GUARD 0
#endif

#ifndef CROSPI_BOOST_INTEL_CXX_VERSION
#define CROSPI_BOOST_INTEL_CXX_VERSION_WORKAROUND_GUARD 1
#else
#define CROSPI_BOOST_INTEL_CXX_VERSION_WORKAROUND_GUARD 0
#endif
#ifndef CROSPI_BOOST_INTEL_WIN
#define CROSPI_BOOST_INTEL_WIN_WORKAROUND_GUARD 1
#else
#define CROSPI_BOOST_INTEL_WIN_WORKAROUND_GUARD 0
#endif
#ifndef CROSPI_BOOST_DINKUMWARE_STDLIB
#define CROSPI_BOOST_DINKUMWARE_STDLIB_WORKAROUND_GUARD 1
#else
#define CROSPI_BOOST_DINKUMWARE_STDLIB_WORKAROUND_GUARD 0
#endif
#ifndef CROSPI_BOOST_INTEL
#define CROSPI_BOOST_INTEL_WORKAROUND_GUARD 1
#else
#define CROSPI_BOOST_INTEL_WORKAROUND_GUARD 0
#endif
#ifndef CROSPI_BOOST_CLANG_VERSION
#define CROSPI_BOOST_CLANG_VERSION_WORKAROUND_GUARD 1
#else
#define CROSPI_BOOST_CLANG_VERSION_WORKAROUND_GUARD 0
#endif

// Always define to zero, if it's used it'll be defined my MPL:
#define CROSPI_BOOST_MPL_CFG_GCC_WORKAROUND_GUARD 0

#define CROSPI_BOOST_WORKAROUND(symbol, test)                \
       ((symbol ## _WORKAROUND_GUARD + 0 == 0) &&     \
       (symbol != 0) && (1 % (( (symbol test) ) + 1)))
//                              ^ ^           ^ ^
// The extra level of parenthesis nesting above, along with the
// CROSPI_BOOST_OPEN_PAREN indirection below, is required to satisfy the
// broken preprocessor in MWCW 8.3 and earlier.
//
// The basic mechanism works as follows:
//   (symbol test) + 1        =>   if (symbol test) then 2 else 1
//   1 % ((symbol test) + 1)  =>   if (symbol test) then 1 else 0
//
// The complication with % is for cooperation with CROSPI_BOOST_TESTED_AT().
// When "test" is CROSPI_BOOST_TESTED_AT(x) and
// CROSPI_BOOST_DETECT_OUTDATED_WORKAROUNDS is #defined,
//
//   symbol test              =>   if (symbol <= x) then 1 else -1
//   (symbol test) + 1        =>   if (symbol <= x) then 2 else 0
//   1 % ((symbol test) + 1)  =>   if (symbol <= x) then 1 else divide-by-zero
//

#ifdef CROSPI_BOOST_DETECT_OUTDATED_WORKAROUNDS
#  define CROSPI_BOOST_OPEN_PAREN (
#  define CROSPI_BOOST_TESTED_AT(value)  > value) ?(-1): CROSPI_BOOST_OPEN_PAREN 1
#else
#  define CROSPI_BOOST_TESTED_AT(value) != ((value)-(value))
#endif

#else

#define CROSPI_BOOST_WORKAROUND(symbol, test) 0

#endif

#endif // CROSPI_BOOST_CONFIG_WORKAROUND_HPP
