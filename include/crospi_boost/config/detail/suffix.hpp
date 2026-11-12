//  Boost config.hpp configuration header file  ------------------------------//
//  boostinspect:ndprecated_macros -- tell the inspect tool to ignore this file

//  Copyright (c) 2001-2003 John Maddock
//  Copyright (c) 2001 Darin Adler
//  Copyright (c) 2001 Peter Dimov
//  Copyright (c) 2002 Bill Kempf
//  Copyright (c) 2002 Jens Maurer
//  Copyright (c) 2002-2003 David Abrahams
//  Copyright (c) 2003 Gennaro Prota
//  Copyright (c) 2003 Eric Friedman
//  Copyright (c) 2010 Eric Jourdanneau, Joel Falcou
// Distributed under the Boost Software License, Version 1.0. (See
// accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)

//  See http://www.boost.org/ for most recent version.

//  Boost config.hpp policy and rationale documentation has been moved to
//  http://www.boost.org/libs/config/
//
//  This file is intended to be stable, and relatively unchanging.
//  It should contain boilerplate code only - no compiler specific
//  code unless it is unavoidable - no changes unless unavoidable.

#ifndef CROSPI_BOOST_CONFIG_SUFFIX_HPP
#define CROSPI_BOOST_CONFIG_SUFFIX_HPP

#if defined(__GNUC__) && (__GNUC__ >= 4)
//
// Some GCC-4.x versions issue warnings even when __extension__ is used,
// so use this as a workaround:
//
#pragma GCC system_header
#endif

//
// ensure that visibility macros are always defined, thus simplifying use
//
#ifndef CROSPI_BOOST_SYMBOL_EXPORT
# define CROSPI_BOOST_SYMBOL_EXPORT
#endif
#ifndef CROSPI_BOOST_SYMBOL_IMPORT
# define CROSPI_BOOST_SYMBOL_IMPORT
#endif
#ifndef CROSPI_BOOST_SYMBOL_VISIBLE
# define CROSPI_BOOST_SYMBOL_VISIBLE
#endif

//
// disable explicitly enforced visibility
//
#if defined(CROSPI_BOOST_DISABLE_EXPLICIT_SYMBOL_VISIBILITY)

#undef CROSPI_BOOST_SYMBOL_EXPORT
#define CROSPI_BOOST_SYMBOL_EXPORT

#undef CROSPI_BOOST_SYMBOL_IMPORT
#define CROSPI_BOOST_SYMBOL_IMPORT

#undef CROSPI_BOOST_SYMBOL_VISIBLE
#define CROSPI_BOOST_SYMBOL_VISIBLE

#endif

//
// look for long long by looking for the appropriate macros in <limits.h>.
// Note that we use limits.h rather than climits for maximal portability,
// remember that since these just declare a bunch of macros, there should be
// no namespace issues from this.
//
#if !defined(CROSPI_BOOST_HAS_LONG_LONG) && !defined(CROSPI_BOOST_NO_LONG_LONG)                                              \
   && !defined(CROSPI_BOOST_MSVC) && !defined(CROSPI_BOOST_BORLANDC)
# include <limits.h>
# if (defined(ULLONG_MAX) || defined(ULONG_LONG_MAX) || defined(ULONGLONG_MAX))
#   define CROSPI_BOOST_HAS_LONG_LONG
# else
#   define CROSPI_BOOST_NO_LONG_LONG
# endif
#endif

// GCC 3.x will clean up all of those nasty macro definitions that
// CROSPI_BOOST_NO_CTYPE_FUNCTIONS is intended to help work around, so undefine
// it under GCC 3.x.
#if defined(__GNUC__) && (__GNUC__ >= 3) && defined(CROSPI_BOOST_NO_CTYPE_FUNCTIONS)
#  undef CROSPI_BOOST_NO_CTYPE_FUNCTIONS
#endif

//
// Assume any extensions are in namespace std:: unless stated otherwise:
//
#  ifndef CROSPI_BOOST_STD_EXTENSION_NAMESPACE
#    define CROSPI_BOOST_STD_EXTENSION_NAMESPACE std
#  endif

//
// If cv-qualified specializations are not allowed, then neither are cv-void ones:
//
#  if defined(CROSPI_BOOST_NO_CV_SPECIALIZATIONS) \
      && !defined(CROSPI_BOOST_NO_CV_VOID_SPECIALIZATIONS)
#     define CROSPI_BOOST_NO_CV_VOID_SPECIALIZATIONS
#  endif

//
// If there is no numeric_limits template, then it can't have any compile time
// constants either!
//
#  if defined(CROSPI_BOOST_NO_LIMITS) \
      && !defined(CROSPI_BOOST_NO_LIMITS_COMPILE_TIME_CONSTANTS)
#     define CROSPI_BOOST_NO_LIMITS_COMPILE_TIME_CONSTANTS
#     define CROSPI_BOOST_NO_MS_INT64_NUMERIC_LIMITS
#     define CROSPI_BOOST_NO_LONG_LONG_NUMERIC_LIMITS
#  endif

//
// if there is no long long then there is no specialisation
// for numeric_limits<long long> either:
//
#if !defined(CROSPI_BOOST_HAS_LONG_LONG) && !defined(CROSPI_BOOST_NO_LONG_LONG_NUMERIC_LIMITS)
#  define CROSPI_BOOST_NO_LONG_LONG_NUMERIC_LIMITS
#endif

//
// if there is no __int64 then there is no specialisation
// for numeric_limits<__int64> either:
//
#if !defined(CROSPI_BOOST_HAS_MS_INT64) && !defined(CROSPI_BOOST_NO_MS_INT64_NUMERIC_LIMITS)
#  define CROSPI_BOOST_NO_MS_INT64_NUMERIC_LIMITS
#endif

//
// if member templates are supported then so is the
// VC6 subset of member templates:
//
#  if !defined(CROSPI_BOOST_NO_MEMBER_TEMPLATES) \
       && !defined(CROSPI_BOOST_MSVC6_MEMBER_TEMPLATES)
#     define CROSPI_BOOST_MSVC6_MEMBER_TEMPLATES
#  endif

//
// Without partial specialization, can't test for partial specialisation bugs:
//
#  if defined(CROSPI_BOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION) \
      && !defined(CROSPI_BOOST_BCB_PARTIAL_SPECIALIZATION_BUG)
#     define CROSPI_BOOST_BCB_PARTIAL_SPECIALIZATION_BUG
#  endif

//
// Without partial specialization, we can't have array-type partial specialisations:
//
#  if defined(CROSPI_BOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION) \
      && !defined(CROSPI_BOOST_NO_ARRAY_TYPE_SPECIALIZATIONS)
#     define CROSPI_BOOST_NO_ARRAY_TYPE_SPECIALIZATIONS
#  endif

//
// Without partial specialization, std::iterator_traits can't work:
//
#  if defined(CROSPI_BOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION) \
      && !defined(CROSPI_BOOST_NO_STD_ITERATOR_TRAITS)
#     define CROSPI_BOOST_NO_STD_ITERATOR_TRAITS
#  endif

//
// Without partial specialization, partial
// specialization with default args won't work either:
//
#  if defined(CROSPI_BOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION) \
      && !defined(CROSPI_BOOST_NO_PARTIAL_SPECIALIZATION_IMPLICIT_DEFAULT_ARGS)
#     define CROSPI_BOOST_NO_PARTIAL_SPECIALIZATION_IMPLICIT_DEFAULT_ARGS
#  endif

//
// Without member template support, we can't have template constructors
// in the standard library either:
//
#  if defined(CROSPI_BOOST_NO_MEMBER_TEMPLATES) \
      && !defined(CROSPI_BOOST_MSVC6_MEMBER_TEMPLATES) \
      && !defined(CROSPI_BOOST_NO_TEMPLATED_ITERATOR_CONSTRUCTORS)
#     define CROSPI_BOOST_NO_TEMPLATED_ITERATOR_CONSTRUCTORS
#  endif

//
// Without member template support, we can't have a conforming
// std::allocator template either:
//
#  if defined(CROSPI_BOOST_NO_MEMBER_TEMPLATES) \
      && !defined(CROSPI_BOOST_MSVC6_MEMBER_TEMPLATES) \
      && !defined(CROSPI_BOOST_NO_STD_ALLOCATOR)
#     define CROSPI_BOOST_NO_STD_ALLOCATOR
#  endif

//
// without ADL support then using declarations will break ADL as well:
//
#if defined(CROSPI_BOOST_NO_ARGUMENT_DEPENDENT_LOOKUP) && !defined(CROSPI_BOOST_FUNCTION_SCOPE_USING_DECLARATION_BREAKS_ADL)
#  define CROSPI_BOOST_FUNCTION_SCOPE_USING_DECLARATION_BREAKS_ADL
#endif

//
// Without typeid support we have no dynamic RTTI either:
//
#if defined(CROSPI_BOOST_NO_TYPEID) && !defined(CROSPI_BOOST_NO_RTTI)
#  define CROSPI_BOOST_NO_RTTI
#endif

//
// If we have a standard allocator, then we have a partial one as well:
//
#if !defined(CROSPI_BOOST_NO_STD_ALLOCATOR)
#  define CROSPI_BOOST_HAS_PARTIAL_STD_ALLOCATOR
#endif

//
// We can't have a working std::use_facet if there is no std::locale:
//
#  if defined(CROSPI_BOOST_NO_STD_LOCALE) && !defined(CROSPI_BOOST_NO_STD_USE_FACET)
#     define CROSPI_BOOST_NO_STD_USE_FACET
#  endif

//
// We can't have a std::messages facet if there is no std::locale:
//
#  if defined(CROSPI_BOOST_NO_STD_LOCALE) && !defined(CROSPI_BOOST_NO_STD_MESSAGES)
#     define CROSPI_BOOST_NO_STD_MESSAGES
#  endif

//
// We can't have a working std::wstreambuf if there is no std::locale:
//
#  if defined(CROSPI_BOOST_NO_STD_LOCALE) && !defined(CROSPI_BOOST_NO_STD_WSTREAMBUF)
#     define CROSPI_BOOST_NO_STD_WSTREAMBUF
#  endif

//
// We can't have a <cwctype> if there is no <cwchar>:
//
#  if defined(CROSPI_BOOST_NO_CWCHAR) && !defined(CROSPI_BOOST_NO_CWCTYPE)
#     define CROSPI_BOOST_NO_CWCTYPE
#  endif

//
// We can't have a swprintf if there is no <cwchar>:
//
#  if defined(CROSPI_BOOST_NO_CWCHAR) && !defined(CROSPI_BOOST_NO_SWPRINTF)
#     define CROSPI_BOOST_NO_SWPRINTF
#  endif

//
// If Win32 support is turned off, then we must turn off
// threading support also, unless there is some other
// thread API enabled:
//
#if defined(CROSPI_BOOST_DISABLE_WIN32) && defined(_WIN32) \
   && !defined(CROSPI_BOOST_DISABLE_THREADS) && !defined(CROSPI_BOOST_HAS_PTHREADS)
#  define CROSPI_BOOST_DISABLE_THREADS
#endif

//
// Turn on threading support if the compiler thinks that it's in
// multithreaded mode.  We put this here because there are only a
// limited number of macros that identify this (if there's any missing
// from here then add to the appropriate compiler section):
//
#if (defined(__MT__) || defined(_MT) || defined(_REENTRANT) \
    || defined(_PTHREADS) || defined(__APPLE__) || defined(__DragonFly__)) \
    && !defined(CROSPI_BOOST_HAS_THREADS)
#  define CROSPI_BOOST_HAS_THREADS
#endif

//
// Turn threading support off if CROSPI_BOOST_DISABLE_THREADS is defined:
//
#if defined(CROSPI_BOOST_DISABLE_THREADS) && defined(CROSPI_BOOST_HAS_THREADS)
#  undef CROSPI_BOOST_HAS_THREADS
#endif

//
// Turn threading support off if we don't recognise the threading API:
//
#if defined(CROSPI_BOOST_HAS_THREADS) && !defined(CROSPI_BOOST_HAS_PTHREADS)\
      && !defined(CROSPI_BOOST_HAS_WINTHREADS) && !defined(CROSPI_BOOST_HAS_BETHREADS)\
      && !defined(CROSPI_BOOST_HAS_MPTASKS)
#  undef CROSPI_BOOST_HAS_THREADS
#endif

//
// Turn threading detail macros off if we don't (want to) use threading
//
#ifndef CROSPI_BOOST_HAS_THREADS
#  undef CROSPI_BOOST_HAS_PTHREADS
#  undef CROSPI_BOOST_HAS_PTHREAD_MUTEXATTR_SETTYPE
#  undef CROSPI_BOOST_HAS_PTHREAD_YIELD
#  undef CROSPI_BOOST_HAS_PTHREAD_DELAY_NP
#  undef CROSPI_BOOST_HAS_WINTHREADS
#  undef CROSPI_BOOST_HAS_BETHREADS
#  undef CROSPI_BOOST_HAS_MPTASKS
#endif

//
// If the compiler claims to be C99 conformant, then it had better
// have a <stdint.h>:
//
#  if defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 199901)
#     define CROSPI_BOOST_HAS_STDINT_H
#     ifndef CROSPI_BOOST_HAS_LOG1P
#        define CROSPI_BOOST_HAS_LOG1P
#     endif
#     ifndef CROSPI_BOOST_HAS_EXPM1
#        define CROSPI_BOOST_HAS_EXPM1
#     endif
#  endif

//
// Define CROSPI_BOOST_NO_SLIST and CROSPI_BOOST_NO_HASH if required.
// Note that this is for backwards compatibility only.
//
#  if !defined(CROSPI_BOOST_HAS_SLIST) && !defined(CROSPI_BOOST_NO_SLIST)
#     define CROSPI_BOOST_NO_SLIST
#  endif

#  if !defined(CROSPI_BOOST_HAS_HASH) && !defined(CROSPI_BOOST_NO_HASH)
#     define CROSPI_BOOST_NO_HASH
#  endif

//
// Set CROSPI_BOOST_SLIST_HEADER if not set already:
//
#if defined(CROSPI_BOOST_HAS_SLIST) && !defined(CROSPI_BOOST_SLIST_HEADER)
#  define CROSPI_BOOST_SLIST_HEADER <slist>
#endif

//
// Set CROSPI_BOOST_HASH_SET_HEADER if not set already:
//
#if defined(CROSPI_BOOST_HAS_HASH) && !defined(CROSPI_BOOST_HASH_SET_HEADER)
#  define CROSPI_BOOST_HASH_SET_HEADER <hash_set>
#endif

//
// Set CROSPI_BOOST_HASH_MAP_HEADER if not set already:
//
#if defined(CROSPI_BOOST_HAS_HASH) && !defined(CROSPI_BOOST_HASH_MAP_HEADER)
#  define CROSPI_BOOST_HASH_MAP_HEADER <hash_map>
#endif

//  CROSPI_BOOST_HAS_ABI_HEADERS
//  This macro gets set if we have headers that fix the ABI,
//  and prevent ODR violations when linking to external libraries:
#if defined(CROSPI_BOOST_ABI_PREFIX) && defined(CROSPI_BOOST_ABI_SUFFIX) && !defined(CROSPI_BOOST_HAS_ABI_HEADERS)
#  define CROSPI_BOOST_HAS_ABI_HEADERS
#endif

#if defined(CROSPI_BOOST_HAS_ABI_HEADERS) && defined(CROSPI_BOOST_DISABLE_ABI_HEADERS)
#  undef CROSPI_BOOST_HAS_ABI_HEADERS
#endif

//  CROSPI_BOOST_NO_STDC_NAMESPACE workaround  --------------------------------------//
//  Because std::size_t usage is so common, even in boost headers which do not
//  otherwise use the C library, the <cstddef> workaround is included here so
//  that ugly workaround code need not appear in many other boost headers.
//  NOTE WELL: This is a workaround for non-conforming compilers; <cstddef>
//  must still be #included in the usual places so that <cstddef> inclusion
//  works as expected with standard conforming compilers.  The resulting
//  double inclusion of <cstddef> is harmless.

# if defined(CROSPI_BOOST_NO_STDC_NAMESPACE) && defined(__cplusplus)
#   include <cstddef>
    namespace std { using ::ptrdiff_t; using ::size_t; }
# endif

//  Workaround for the unfortunate min/max macros defined by some platform headers

#define CROSPI_BOOST_PREVENT_MACRO_SUBSTITUTION

#ifndef CROSPI_BOOST_USING_STD_MIN
#  define CROSPI_BOOST_USING_STD_MIN() using std::min
#endif

#ifndef CROSPI_BOOST_USING_STD_MAX
#  define CROSPI_BOOST_USING_STD_MAX() using std::max
#endif

//  CROSPI_BOOST_NO_STD_MIN_MAX workaround  -----------------------------------------//

#  if defined(CROSPI_BOOST_NO_STD_MIN_MAX) && defined(__cplusplus)

namespace std {
  template <class _Tp>
  inline const _Tp& min CROSPI_BOOST_PREVENT_MACRO_SUBSTITUTION (const _Tp& __a, const _Tp& __b) {
    return __b < __a ? __b : __a;
  }
  template <class _Tp>
  inline const _Tp& max CROSPI_BOOST_PREVENT_MACRO_SUBSTITUTION (const _Tp& __a, const _Tp& __b) {
    return  __a < __b ? __b : __a;
  }
}

#  endif

// CROSPI_BOOST_STATIC_CONSTANT workaround --------------------------------------- //
// On compilers which don't allow in-class initialization of static integral
// constant members, we must use enums as a workaround if we want the constants
// to be available at compile-time. This macro gives us a convenient way to
// declare such constants.

#  ifdef CROSPI_BOOST_NO_INCLASS_MEMBER_INITIALIZATION
#       define CROSPI_BOOST_STATIC_CONSTANT(type, assignment) enum { assignment }
#  else
#     define CROSPI_BOOST_STATIC_CONSTANT(type, assignment) static const type assignment
#  endif

// CROSPI_BOOST_USE_FACET / HAS_FACET workaround ----------------------------------//
// When the standard library does not have a conforming std::use_facet there
// are various workarounds available, but they differ from library to library.
// The same problem occurs with has_facet.
// These macros provide a consistent way to access a locale's facets.
// Usage:
//    replace
//       std::use_facet<Type>(loc);
//    with
//       CROSPI_BOOST_USE_FACET(Type, loc);
//    Note do not add a std:: prefix to the front of CROSPI_BOOST_USE_FACET!
//  Use for CROSPI_BOOST_HAS_FACET is analogous.

#if defined(CROSPI_BOOST_NO_STD_USE_FACET)
#  ifdef CROSPI_BOOST_HAS_TWO_ARG_USE_FACET
#     define CROSPI_BOOST_USE_FACET(Type, loc) std::use_facet(loc, static_cast<Type*>(0))
#     define CROSPI_BOOST_HAS_FACET(Type, loc) std::has_facet(loc, static_cast<Type*>(0))
#  elif defined(CROSPI_BOOST_HAS_MACRO_USE_FACET)
#     define CROSPI_BOOST_USE_FACET(Type, loc) std::_USE(loc, Type)
#     define CROSPI_BOOST_HAS_FACET(Type, loc) std::_HAS(loc, Type)
#  elif defined(CROSPI_BOOST_HAS_STLP_USE_FACET)
#     define CROSPI_BOOST_USE_FACET(Type, loc) (*std::_Use_facet<Type >(loc))
#     define CROSPI_BOOST_HAS_FACET(Type, loc) std::has_facet< Type >(loc)
#  endif
#else
#  define CROSPI_BOOST_USE_FACET(Type, loc) std::use_facet< Type >(loc)
#  define CROSPI_BOOST_HAS_FACET(Type, loc) std::has_facet< Type >(loc)
#endif

// CROSPI_BOOST_NESTED_TEMPLATE workaround ------------------------------------------//
// Member templates are supported by some compilers even though they can't use
// the A::template member<U> syntax, as a workaround replace:
//
// typedef typename A::template rebind<U> binder;
//
// with:
//
// typedef typename A::CROSPI_BOOST_NESTED_TEMPLATE rebind<U> binder;

#ifndef CROSPI_BOOST_NO_MEMBER_TEMPLATE_KEYWORD
#  define CROSPI_BOOST_NESTED_TEMPLATE template
#else
#  define CROSPI_BOOST_NESTED_TEMPLATE
#endif

// CROSPI_BOOST_UNREACHABLE_RETURN(x) workaround -------------------------------------//
// Normally evaluates to nothing, unless CROSPI_BOOST_NO_UNREACHABLE_RETURN_DETECTION
// is defined, in which case it evaluates to return x; Use when you have a return
// statement that can never be reached.

#ifndef CROSPI_BOOST_UNREACHABLE_RETURN
#  ifdef CROSPI_BOOST_NO_UNREACHABLE_RETURN_DETECTION
#     define CROSPI_BOOST_UNREACHABLE_RETURN(x) return x;
#  else
#     define CROSPI_BOOST_UNREACHABLE_RETURN(x)
#  endif
#endif

// CROSPI_BOOST_DEDUCED_TYPENAME workaround ------------------------------------------//
//
// Some compilers don't support the use of `typename' for dependent
// types in deduced contexts, e.g.
//
//     template <class T> void f(T, typename T::type);
//                                  ^^^^^^^^
// Replace these declarations with:
//
//     template <class T> void f(T, CROSPI_BOOST_DEDUCED_TYPENAME T::type);

#ifndef CROSPI_BOOST_NO_DEDUCED_TYPENAME
#  define CROSPI_BOOST_DEDUCED_TYPENAME typename
#else
#  define CROSPI_BOOST_DEDUCED_TYPENAME
#endif

#ifndef CROSPI_BOOST_NO_TYPENAME_WITH_CTOR
#  define CROSPI_BOOST_CTOR_TYPENAME typename
#else
#  define CROSPI_BOOST_CTOR_TYPENAME
#endif

//
// If we're on a CUDA device (note DEVICE not HOST, irrespective of compiler) then disable __int128 and __float128 support if present:
//
#if defined(__CUDA_ARCH__) && defined(CROSPI_BOOST_HAS_FLOAT128)
#  undef CROSPI_BOOST_HAS_FLOAT128
#endif
#if defined(__CUDA_ARCH__) && defined(CROSPI_BOOST_HAS_INT128)
#  undef CROSPI_BOOST_HAS_INT128
#endif

// long long workaround ------------------------------------------//
// On gcc (and maybe other compilers?) long long is alway supported
// but it's use may generate either warnings (with -ansi), or errors
// (with -pedantic -ansi) unless it's use is prefixed by __extension__
//
#if defined(CROSPI_BOOST_HAS_LONG_LONG) && defined(__cplusplus)
namespace crospi_boost{
#  ifdef __GNUC__
   __extension__ typedef long long long_long_type;
   __extension__ typedef unsigned long long ulong_long_type;
#  else
   typedef long long long_long_type;
   typedef unsigned long long ulong_long_type;
#  endif
}
#endif
// same again for __int128:
#if defined(CROSPI_BOOST_HAS_INT128) && defined(__cplusplus)
namespace crospi_boost{
#  ifdef __GNUC__
   __extension__ typedef __int128 int128_type;
   __extension__ typedef unsigned __int128 uint128_type;
#  else
   typedef __int128 int128_type;
   typedef unsigned __int128 uint128_type;
#  endif
}
#endif
// same again for __float128:
#if defined(CROSPI_BOOST_HAS_FLOAT128) && defined(__cplusplus)
namespace crospi_boost {
#  ifdef __GNUC__
   __extension__ typedef __float128 float128_type;
#  else
   typedef __float128 float128_type;
#  endif
}
#endif

// CROSPI_BOOST_[APPEND_]EXPLICIT_TEMPLATE_[NON_]TYPE macros --------------------------//

// These macros are obsolete. Port away and remove.

#  define CROSPI_BOOST_EXPLICIT_TEMPLATE_TYPE(t)
#  define CROSPI_BOOST_EXPLICIT_TEMPLATE_TYPE_SPEC(t)
#  define CROSPI_BOOST_EXPLICIT_TEMPLATE_NON_TYPE(t, v)
#  define CROSPI_BOOST_EXPLICIT_TEMPLATE_NON_TYPE_SPEC(t, v)

#  define CROSPI_BOOST_APPEND_EXPLICIT_TEMPLATE_TYPE(t)
#  define CROSPI_BOOST_APPEND_EXPLICIT_TEMPLATE_TYPE_SPEC(t)
#  define CROSPI_BOOST_APPEND_EXPLICIT_TEMPLATE_NON_TYPE(t, v)
#  define CROSPI_BOOST_APPEND_EXPLICIT_TEMPLATE_NON_TYPE_SPEC(t, v)

// When CROSPI_BOOST_NO_STD_TYPEINFO is defined, we can just import
// the global definition into std namespace, 
// see https://svn.boost.org/trac10/ticket/4115
#if defined(CROSPI_BOOST_NO_STD_TYPEINFO) && defined(__cplusplus) && defined(CROSPI_BOOST_MSVC)
#include <typeinfo>
namespace std{ using ::type_info; }
// Since we do now have typeinfo, undef the macro:
#undef CROSPI_BOOST_NO_STD_TYPEINFO
#endif

// ---------------------------------------------------------------------------//

// Helper macro CROSPI_BOOST_STRINGIZE:
// Helper macro CROSPI_BOOST_JOIN:

#include <crospi_boost/config/helper_macros.hpp>

//
// Set some default values for compiler/library/platform names.
// These are for debugging config setup only:
//
#  ifndef CROSPI_BOOST_COMPILER
#     define CROSPI_BOOST_COMPILER "Unknown ISO C++ Compiler"
#  endif
#  ifndef CROSPI_BOOST_STDLIB
#     define CROSPI_BOOST_STDLIB "Unknown ISO standard library"
#  endif
#  ifndef CROSPI_BOOST_PLATFORM
#     if defined(unix) || defined(__unix) || defined(_XOPEN_SOURCE) \
         || defined(_POSIX_SOURCE)
#        define CROSPI_BOOST_PLATFORM "Generic Unix"
#     else
#        define CROSPI_BOOST_PLATFORM "Unknown"
#     endif
#  endif

//
// Set some default values GPU support
//
#  ifndef CROSPI_BOOST_GPU_ENABLED
#  define CROSPI_BOOST_GPU_ENABLED
#  endif

// CROSPI_BOOST_RESTRICT ---------------------------------------------//
// Macro to use in place of 'restrict' keyword variants
#if !defined(CROSPI_BOOST_RESTRICT)
#  if defined(_MSC_VER)
#    define CROSPI_BOOST_RESTRICT __restrict
#    if !defined(CROSPI_BOOST_NO_RESTRICT_REFERENCES) && (_MSC_FULL_VER < 190023026)
#      define CROSPI_BOOST_NO_RESTRICT_REFERENCES
#    endif
#  elif defined(__GNUC__) && __GNUC__ > 3
     // Clang also defines __GNUC__ (as 4)
#    define CROSPI_BOOST_RESTRICT __restrict__
#  else
#    define CROSPI_BOOST_RESTRICT
#    if !defined(CROSPI_BOOST_NO_RESTRICT_REFERENCES)
#      define CROSPI_BOOST_NO_RESTRICT_REFERENCES
#    endif
#  endif
#endif

// CROSPI_BOOST_MAY_ALIAS -----------------------------------------------//
// The macro expands to an attribute to mark a type that is allowed to alias other types.
// The macro is defined in the compiler-specific headers.
#if !defined(CROSPI_BOOST_MAY_ALIAS)
#  define CROSPI_BOOST_NO_MAY_ALIAS
#  define CROSPI_BOOST_MAY_ALIAS
#endif

// CROSPI_BOOST_FORCEINLINE ---------------------------------------------//
// Macro to use in place of 'inline' to force a function to be inline
#if !defined(CROSPI_BOOST_FORCEINLINE)
#  if defined(_MSC_VER)
#    define CROSPI_BOOST_FORCEINLINE __forceinline
#  elif defined(__GNUC__) && __GNUC__ > 3
     // Clang also defines __GNUC__ (as 4)
#    define CROSPI_BOOST_FORCEINLINE inline __attribute__ ((__always_inline__))
#  else
#    define CROSPI_BOOST_FORCEINLINE inline
#  endif
#endif

// CROSPI_BOOST_NOINLINE ---------------------------------------------//
// Macro to use in place of 'inline' to prevent a function to be inlined
#if !defined(CROSPI_BOOST_NOINLINE)
#  if defined(_MSC_VER)
#    define CROSPI_BOOST_NOINLINE __declspec(noinline)
#  elif defined(__GNUC__) && __GNUC__ > 3
     // Clang also defines __GNUC__ (as 4)
#    if defined(__CUDACC__)
       // nvcc doesn't always parse __noinline__,
       // see: https://svn.boost.org/trac/crospi_boost/ticket/9392
#      define CROSPI_BOOST_NOINLINE __attribute__ ((noinline))
#    elif defined(__HIP__)
       // See https://github.com/boostorg/config/issues/392
#      define CROSPI_BOOST_NOINLINE __attribute__ ((noinline))
#    else
#      define CROSPI_BOOST_NOINLINE __attribute__ ((__noinline__))
#    endif
#  else
#    define CROSPI_BOOST_NOINLINE
#  endif
#endif

// CROSPI_BOOST_NORETURN ---------------------------------------------//
// Macro to use before a function declaration/definition to designate
// the function as not returning normally (i.e. with a return statement
// or by leaving the function scope, if the function return type is void).
#if !defined(CROSPI_BOOST_NORETURN)
#  if defined(_MSC_VER)
#    define CROSPI_BOOST_NORETURN __declspec(noreturn)
#  elif defined(__GNUC__) || defined(__CODEGEARC__) && defined(__clang__)
#    define CROSPI_BOOST_NORETURN __attribute__ ((__noreturn__))
#  elif defined(__has_attribute) && defined(__SUNPRO_CC) && (__SUNPRO_CC > 0x5130)
#    if __has_attribute(noreturn)
#      define CROSPI_BOOST_NORETURN [[noreturn]]
#    endif
#  elif defined(__has_cpp_attribute) 
#    if __has_cpp_attribute(noreturn)
#      define CROSPI_BOOST_NORETURN [[noreturn]]
#    endif
#  endif
#endif

#if !defined(CROSPI_BOOST_NORETURN)
#  define CROSPI_BOOST_NO_NORETURN
#  define CROSPI_BOOST_NORETURN
#endif

// CROSPI_BOOST_DEPRECATED -------------------------------------------//
// The macro can be used to mark deprecated symbols, such as functions, objects and types.
// Any code that uses these symbols will produce warnings, possibly with a message specified
// as an argument. The warnings can be suppressed by defining CROSPI_BOOST_ALLOW_DEPRECATED_SYMBOLS
// or CROSPI_BOOST_ALLOW_DEPRECATED.
#if !defined(CROSPI_BOOST_DEPRECATED) && __cplusplus >= 201402
#define CROSPI_BOOST_DEPRECATED(msg) [[deprecated(msg)]]
#endif

#if defined(CROSPI_BOOST_ALLOW_DEPRECATED_SYMBOLS) || defined(CROSPI_BOOST_ALLOW_DEPRECATED)
#undef CROSPI_BOOST_DEPRECATED
#endif

#if !defined(CROSPI_BOOST_DEPRECATED)
#define CROSPI_BOOST_DEPRECATED(msg)
#endif

// Branch prediction hints
// These macros are intended to wrap conditional expressions that yield true or false
//
//  if (CROSPI_BOOST_LIKELY(var == 10))
//  {
//     // the most probable code here
//  }
//
#if !defined(CROSPI_BOOST_LIKELY)
#  define CROSPI_BOOST_LIKELY(x) x
#endif
#if !defined(CROSPI_BOOST_UNLIKELY)
#  define CROSPI_BOOST_UNLIKELY(x) x
#endif

#if !defined(CROSPI_BOOST_NO_CXX11_OVERRIDE)
#  define CROSPI_BOOST_OVERRIDE override
#else
#  define CROSPI_BOOST_OVERRIDE
#endif

// Type and data alignment specification
//
#if !defined(CROSPI_BOOST_ALIGNMENT)
#  if !defined(CROSPI_BOOST_NO_CXX11_ALIGNAS)
#    define CROSPI_BOOST_ALIGNMENT(x) alignas(x)
#  elif defined(_MSC_VER)
#    define CROSPI_BOOST_ALIGNMENT(x) __declspec(align(x))
#  elif defined(__GNUC__)
#    define CROSPI_BOOST_ALIGNMENT(x) __attribute__ ((__aligned__(x)))
#  else
#    define CROSPI_BOOST_NO_ALIGNMENT
#    define CROSPI_BOOST_ALIGNMENT(x)
#  endif
#endif

// Lack of non-public defaulted functions is implied by the lack of any defaulted functions
#if !defined(CROSPI_BOOST_NO_CXX11_NON_PUBLIC_DEFAULTED_FUNCTIONS) && defined(CROSPI_BOOST_NO_CXX11_DEFAULTED_FUNCTIONS)
#  define CROSPI_BOOST_NO_CXX11_NON_PUBLIC_DEFAULTED_FUNCTIONS
#endif

// Lack of defaulted moves is implied by the lack of either rvalue references or any defaulted functions
#if !defined(CROSPI_BOOST_NO_CXX11_DEFAULTED_MOVES) && (defined(CROSPI_BOOST_NO_CXX11_DEFAULTED_FUNCTIONS) || defined(CROSPI_BOOST_NO_CXX11_RVALUE_REFERENCES))
#  define CROSPI_BOOST_NO_CXX11_DEFAULTED_MOVES
#endif

// Defaulted and deleted function declaration helpers
// These macros are intended to be inside a class definition.
// CROSPI_BOOST_DEFAULTED_FUNCTION accepts the function declaration and its
// body, which will be used if the compiler doesn't support defaulted functions.
// CROSPI_BOOST_DELETED_FUNCTION only accepts the function declaration. It
// will expand to a private function declaration, if the compiler doesn't support
// deleted functions. Because of this it is recommended to use CROSPI_BOOST_DELETED_FUNCTION
// in the end of the class definition.
//
//  class my_class
//  {
//  public:
//      // Default-constructible
//      CROSPI_BOOST_DEFAULTED_FUNCTION(my_class(), {})
//      // Copying prohibited
//      CROSPI_BOOST_DELETED_FUNCTION(my_class(my_class const&))
//      CROSPI_BOOST_DELETED_FUNCTION(my_class& operator= (my_class const&))
//  };
//
#if !(defined(CROSPI_BOOST_NO_CXX11_DEFAULTED_FUNCTIONS) || defined(CROSPI_BOOST_NO_CXX11_NON_PUBLIC_DEFAULTED_FUNCTIONS))
#   define CROSPI_BOOST_DEFAULTED_FUNCTION(fun, body) fun = default;
#else
#   define CROSPI_BOOST_DEFAULTED_FUNCTION(fun, body) fun body
#endif

#if !defined(CROSPI_BOOST_NO_CXX11_DELETED_FUNCTIONS)
#   define CROSPI_BOOST_DELETED_FUNCTION(fun) fun = delete;
#else
#   define CROSPI_BOOST_DELETED_FUNCTION(fun) private: fun;
#endif

//
// Set CROSPI_BOOST_NO_DECLTYPE_N3276 when CROSPI_BOOST_NO_DECLTYPE is defined
//
#if defined(CROSPI_BOOST_NO_CXX11_DECLTYPE) && !defined(CROSPI_BOOST_NO_CXX11_DECLTYPE_N3276)
#define CROSPI_BOOST_NO_CXX11_DECLTYPE_N3276 CROSPI_BOOST_NO_CXX11_DECLTYPE
#endif

//  -------------------- Deprecated macros for 1.50 ---------------------------
//  These will go away in a future release

//  Use CROSPI_BOOST_NO_CXX11_HDR_UNORDERED_SET or CROSPI_BOOST_NO_CXX11_HDR_UNORDERED_MAP
//           instead of CROSPI_BOOST_NO_STD_UNORDERED
#if defined(CROSPI_BOOST_NO_CXX11_HDR_UNORDERED_MAP) || defined (CROSPI_BOOST_NO_CXX11_HDR_UNORDERED_SET)
# ifndef CROSPI_BOOST_NO_CXX11_STD_UNORDERED
#  define CROSPI_BOOST_NO_CXX11_STD_UNORDERED
# endif
#endif

//  Use CROSPI_BOOST_NO_CXX11_HDR_INITIALIZER_LIST instead of CROSPI_BOOST_NO_INITIALIZER_LISTS
#if defined(CROSPI_BOOST_NO_CXX11_HDR_INITIALIZER_LIST) && !defined(CROSPI_BOOST_NO_INITIALIZER_LISTS)
#  define CROSPI_BOOST_NO_INITIALIZER_LISTS
#endif

//  Use CROSPI_BOOST_NO_CXX11_HDR_ARRAY instead of CROSPI_BOOST_NO_0X_HDR_ARRAY
#if defined(CROSPI_BOOST_NO_CXX11_HDR_ARRAY) && !defined(CROSPI_BOOST_NO_0X_HDR_ARRAY)
#  define CROSPI_BOOST_NO_0X_HDR_ARRAY
#endif
//  Use CROSPI_BOOST_NO_CXX11_HDR_CHRONO instead of CROSPI_BOOST_NO_0X_HDR_CHRONO
#if defined(CROSPI_BOOST_NO_CXX11_HDR_CHRONO) && !defined(CROSPI_BOOST_NO_0X_HDR_CHRONO)
#  define CROSPI_BOOST_NO_0X_HDR_CHRONO
#endif
//  Use CROSPI_BOOST_NO_CXX11_HDR_CODECVT instead of CROSPI_BOOST_NO_0X_HDR_CODECVT
#if defined(CROSPI_BOOST_NO_CXX11_HDR_CODECVT) && !defined(CROSPI_BOOST_NO_0X_HDR_CODECVT)
#  define CROSPI_BOOST_NO_0X_HDR_CODECVT
#endif
//  Use CROSPI_BOOST_NO_CXX11_HDR_CONDITION_VARIABLE instead of CROSPI_BOOST_NO_0X_HDR_CONDITION_VARIABLE
#if defined(CROSPI_BOOST_NO_CXX11_HDR_CONDITION_VARIABLE) && !defined(CROSPI_BOOST_NO_0X_HDR_CONDITION_VARIABLE)
#  define CROSPI_BOOST_NO_0X_HDR_CONDITION_VARIABLE
#endif
//  Use CROSPI_BOOST_NO_CXX11_HDR_FORWARD_LIST instead of CROSPI_BOOST_NO_0X_HDR_FORWARD_LIST
#if defined(CROSPI_BOOST_NO_CXX11_HDR_FORWARD_LIST) && !defined(CROSPI_BOOST_NO_0X_HDR_FORWARD_LIST)
#  define CROSPI_BOOST_NO_0X_HDR_FORWARD_LIST
#endif
//  Use CROSPI_BOOST_NO_CXX11_HDR_FUTURE instead of CROSPI_BOOST_NO_0X_HDR_FUTURE
#if defined(CROSPI_BOOST_NO_CXX11_HDR_FUTURE) && !defined(CROSPI_BOOST_NO_0X_HDR_FUTURE)
#  define CROSPI_BOOST_NO_0X_HDR_FUTURE
#endif

//  Use CROSPI_BOOST_NO_CXX11_HDR_INITIALIZER_LIST
//  instead of CROSPI_BOOST_NO_0X_HDR_INITIALIZER_LIST or CROSPI_BOOST_NO_INITIALIZER_LISTS
#ifdef CROSPI_BOOST_NO_CXX11_HDR_INITIALIZER_LIST
# ifndef CROSPI_BOOST_NO_0X_HDR_INITIALIZER_LIST
#  define CROSPI_BOOST_NO_0X_HDR_INITIALIZER_LIST
# endif
# ifndef CROSPI_BOOST_NO_INITIALIZER_LISTS
#  define CROSPI_BOOST_NO_INITIALIZER_LISTS
# endif
#endif

//  Use CROSPI_BOOST_NO_CXX11_HDR_MUTEX instead of CROSPI_BOOST_NO_0X_HDR_MUTEX
#if defined(CROSPI_BOOST_NO_CXX11_HDR_MUTEX) && !defined(CROSPI_BOOST_NO_0X_HDR_MUTEX)
#  define CROSPI_BOOST_NO_0X_HDR_MUTEX
#endif
//  Use CROSPI_BOOST_NO_CXX11_HDR_RANDOM instead of CROSPI_BOOST_NO_0X_HDR_RANDOM
#if defined(CROSPI_BOOST_NO_CXX11_HDR_RANDOM) && !defined(CROSPI_BOOST_NO_0X_HDR_RANDOM)
#  define CROSPI_BOOST_NO_0X_HDR_RANDOM
#endif
//  Use CROSPI_BOOST_NO_CXX11_HDR_RATIO instead of CROSPI_BOOST_NO_0X_HDR_RATIO
#if defined(CROSPI_BOOST_NO_CXX11_HDR_RATIO) && !defined(CROSPI_BOOST_NO_0X_HDR_RATIO)
#  define CROSPI_BOOST_NO_0X_HDR_RATIO
#endif
//  Use CROSPI_BOOST_NO_CXX11_HDR_REGEX instead of CROSPI_BOOST_NO_0X_HDR_REGEX
#if defined(CROSPI_BOOST_NO_CXX11_HDR_REGEX) && !defined(CROSPI_BOOST_NO_0X_HDR_REGEX)
#  define CROSPI_BOOST_NO_0X_HDR_REGEX
#endif
//  Use CROSPI_BOOST_NO_CXX11_HDR_SYSTEM_ERROR instead of CROSPI_BOOST_NO_0X_HDR_SYSTEM_ERROR
#if defined(CROSPI_BOOST_NO_CXX11_HDR_SYSTEM_ERROR) && !defined(CROSPI_BOOST_NO_0X_HDR_SYSTEM_ERROR)
#  define CROSPI_BOOST_NO_0X_HDR_SYSTEM_ERROR
#endif
//  Use CROSPI_BOOST_NO_CXX11_HDR_THREAD instead of CROSPI_BOOST_NO_0X_HDR_THREAD
#if defined(CROSPI_BOOST_NO_CXX11_HDR_THREAD) && !defined(CROSPI_BOOST_NO_0X_HDR_THREAD)
#  define CROSPI_BOOST_NO_0X_HDR_THREAD
#endif
//  Use CROSPI_BOOST_NO_CXX11_HDR_TUPLE instead of CROSPI_BOOST_NO_0X_HDR_TUPLE
#if defined(CROSPI_BOOST_NO_CXX11_HDR_TUPLE) && !defined(CROSPI_BOOST_NO_0X_HDR_TUPLE)
#  define CROSPI_BOOST_NO_0X_HDR_TUPLE
#endif
//  Use CROSPI_BOOST_NO_CXX11_HDR_TYPE_TRAITS instead of CROSPI_BOOST_NO_0X_HDR_TYPE_TRAITS
#if defined(CROSPI_BOOST_NO_CXX11_HDR_TYPE_TRAITS) && !defined(CROSPI_BOOST_NO_0X_HDR_TYPE_TRAITS)
#  define CROSPI_BOOST_NO_0X_HDR_TYPE_TRAITS
#endif
//  Use CROSPI_BOOST_NO_CXX11_HDR_TYPEINDEX instead of CROSPI_BOOST_NO_0X_HDR_TYPEINDEX
#if defined(CROSPI_BOOST_NO_CXX11_HDR_TYPEINDEX) && !defined(CROSPI_BOOST_NO_0X_HDR_TYPEINDEX)
#  define CROSPI_BOOST_NO_0X_HDR_TYPEINDEX
#endif
//  Use CROSPI_BOOST_NO_CXX11_HDR_UNORDERED_MAP instead of CROSPI_BOOST_NO_0X_HDR_UNORDERED_MAP
#if defined(CROSPI_BOOST_NO_CXX11_HDR_UNORDERED_MAP) && !defined(CROSPI_BOOST_NO_0X_HDR_UNORDERED_MAP)
#  define CROSPI_BOOST_NO_0X_HDR_UNORDERED_MAP
#endif
//  Use CROSPI_BOOST_NO_CXX11_HDR_UNORDERED_SET instead of CROSPI_BOOST_NO_0X_HDR_UNORDERED_SET
#if defined(CROSPI_BOOST_NO_CXX11_HDR_UNORDERED_SET) && !defined(CROSPI_BOOST_NO_0X_HDR_UNORDERED_SET)
#  define CROSPI_BOOST_NO_0X_HDR_UNORDERED_SET
#endif

//  ------------------ End of deprecated macros for 1.50 ---------------------------

//  -------------------- Deprecated macros for 1.51 ---------------------------
//  These will go away in a future release

//  Use     CROSPI_BOOST_NO_CXX11_AUTO_DECLARATIONS instead of   CROSPI_BOOST_NO_AUTO_DECLARATIONS
#if defined(CROSPI_BOOST_NO_CXX11_AUTO_DECLARATIONS) && !defined(CROSPI_BOOST_NO_AUTO_DECLARATIONS)
#  define CROSPI_BOOST_NO_AUTO_DECLARATIONS
#endif
//  Use     CROSPI_BOOST_NO_CXX11_AUTO_MULTIDECLARATIONS instead of   CROSPI_BOOST_NO_AUTO_MULTIDECLARATIONS
#if defined(CROSPI_BOOST_NO_CXX11_AUTO_MULTIDECLARATIONS) && !defined(CROSPI_BOOST_NO_AUTO_MULTIDECLARATIONS)
#  define CROSPI_BOOST_NO_AUTO_MULTIDECLARATIONS
#endif
//  Use     CROSPI_BOOST_NO_CXX11_CHAR16_T instead of   CROSPI_BOOST_NO_CHAR16_T
#if defined(CROSPI_BOOST_NO_CXX11_CHAR16_T) && !defined(CROSPI_BOOST_NO_CHAR16_T)
#  define CROSPI_BOOST_NO_CHAR16_T
#endif
//  Use     CROSPI_BOOST_NO_CXX11_CHAR32_T instead of   CROSPI_BOOST_NO_CHAR32_T
#if defined(CROSPI_BOOST_NO_CXX11_CHAR32_T) && !defined(CROSPI_BOOST_NO_CHAR32_T)
#  define CROSPI_BOOST_NO_CHAR32_T
#endif
//  Use     CROSPI_BOOST_NO_CXX11_TEMPLATE_ALIASES instead of   CROSPI_BOOST_NO_TEMPLATE_ALIASES
#if defined(CROSPI_BOOST_NO_CXX11_TEMPLATE_ALIASES) && !defined(CROSPI_BOOST_NO_TEMPLATE_ALIASES)
#  define CROSPI_BOOST_NO_TEMPLATE_ALIASES
#endif
//  Use     CROSPI_BOOST_NO_CXX11_CONSTEXPR instead of   CROSPI_BOOST_NO_CONSTEXPR
#if defined(CROSPI_BOOST_NO_CXX11_CONSTEXPR) && !defined(CROSPI_BOOST_NO_CONSTEXPR)
#  define CROSPI_BOOST_NO_CONSTEXPR
#endif
//  Use     CROSPI_BOOST_NO_CXX11_DECLTYPE_N3276 instead of   CROSPI_BOOST_NO_DECLTYPE_N3276
#if defined(CROSPI_BOOST_NO_CXX11_DECLTYPE_N3276) && !defined(CROSPI_BOOST_NO_DECLTYPE_N3276)
#  define CROSPI_BOOST_NO_DECLTYPE_N3276
#endif
//  Use     CROSPI_BOOST_NO_CXX11_DECLTYPE instead of   CROSPI_BOOST_NO_DECLTYPE
#if defined(CROSPI_BOOST_NO_CXX11_DECLTYPE) && !defined(CROSPI_BOOST_NO_DECLTYPE)
#  define CROSPI_BOOST_NO_DECLTYPE
#endif
//  Use     CROSPI_BOOST_NO_CXX11_DEFAULTED_FUNCTIONS instead of   CROSPI_BOOST_NO_DEFAULTED_FUNCTIONS
#if defined(CROSPI_BOOST_NO_CXX11_DEFAULTED_FUNCTIONS) && !defined(CROSPI_BOOST_NO_DEFAULTED_FUNCTIONS)
#  define CROSPI_BOOST_NO_DEFAULTED_FUNCTIONS
#endif
//  Use     CROSPI_BOOST_NO_CXX11_DELETED_FUNCTIONS instead of   CROSPI_BOOST_NO_DELETED_FUNCTIONS
#if defined(CROSPI_BOOST_NO_CXX11_DELETED_FUNCTIONS) && !defined(CROSPI_BOOST_NO_DELETED_FUNCTIONS)
#  define CROSPI_BOOST_NO_DELETED_FUNCTIONS
#endif
//  Use     CROSPI_BOOST_NO_CXX11_EXPLICIT_CONVERSION_OPERATORS instead of   CROSPI_BOOST_NO_EXPLICIT_CONVERSION_OPERATORS
#if defined(CROSPI_BOOST_NO_CXX11_EXPLICIT_CONVERSION_OPERATORS) && !defined(CROSPI_BOOST_NO_EXPLICIT_CONVERSION_OPERATORS)
#  define CROSPI_BOOST_NO_EXPLICIT_CONVERSION_OPERATORS
#endif
//  Use     CROSPI_BOOST_NO_CXX11_EXTERN_TEMPLATE instead of   CROSPI_BOOST_NO_EXTERN_TEMPLATE
#if defined(CROSPI_BOOST_NO_CXX11_EXTERN_TEMPLATE) && !defined(CROSPI_BOOST_NO_EXTERN_TEMPLATE)
#  define CROSPI_BOOST_NO_EXTERN_TEMPLATE
#endif
//  Use     CROSPI_BOOST_NO_CXX11_FUNCTION_TEMPLATE_DEFAULT_ARGS instead of   CROSPI_BOOST_NO_FUNCTION_TEMPLATE_DEFAULT_ARGS
#if defined(CROSPI_BOOST_NO_CXX11_FUNCTION_TEMPLATE_DEFAULT_ARGS) && !defined(CROSPI_BOOST_NO_FUNCTION_TEMPLATE_DEFAULT_ARGS)
#  define CROSPI_BOOST_NO_FUNCTION_TEMPLATE_DEFAULT_ARGS
#endif
//  Use     CROSPI_BOOST_NO_CXX11_LAMBDAS instead of   CROSPI_BOOST_NO_LAMBDAS
#if defined(CROSPI_BOOST_NO_CXX11_LAMBDAS) && !defined(CROSPI_BOOST_NO_LAMBDAS)
#  define CROSPI_BOOST_NO_LAMBDAS
#endif
//  Use     CROSPI_BOOST_NO_CXX11_LOCAL_CLASS_TEMPLATE_PARAMETERS instead of   CROSPI_BOOST_NO_LOCAL_CLASS_TEMPLATE_PARAMETERS
#if defined(CROSPI_BOOST_NO_CXX11_LOCAL_CLASS_TEMPLATE_PARAMETERS) && !defined(CROSPI_BOOST_NO_LOCAL_CLASS_TEMPLATE_PARAMETERS)
#  define CROSPI_BOOST_NO_LOCAL_CLASS_TEMPLATE_PARAMETERS
#endif
//  Use     CROSPI_BOOST_NO_CXX11_NOEXCEPT instead of   CROSPI_BOOST_NO_NOEXCEPT
#if defined(CROSPI_BOOST_NO_CXX11_NOEXCEPT) && !defined(CROSPI_BOOST_NO_NOEXCEPT)
#  define CROSPI_BOOST_NO_NOEXCEPT
#endif
//  Use     CROSPI_BOOST_NO_CXX11_NULLPTR instead of   CROSPI_BOOST_NO_NULLPTR
#if defined(CROSPI_BOOST_NO_CXX11_NULLPTR) && !defined(CROSPI_BOOST_NO_NULLPTR)
#  define CROSPI_BOOST_NO_NULLPTR
#endif
//  Use     CROSPI_BOOST_NO_CXX11_RAW_LITERALS instead of   CROSPI_BOOST_NO_RAW_LITERALS
#if defined(CROSPI_BOOST_NO_CXX11_RAW_LITERALS) && !defined(CROSPI_BOOST_NO_RAW_LITERALS)
#  define CROSPI_BOOST_NO_RAW_LITERALS
#endif
//  Use     CROSPI_BOOST_NO_CXX11_RVALUE_REFERENCES instead of   CROSPI_BOOST_NO_RVALUE_REFERENCES
#if defined(CROSPI_BOOST_NO_CXX11_RVALUE_REFERENCES) && !defined(CROSPI_BOOST_NO_RVALUE_REFERENCES)
#  define CROSPI_BOOST_NO_RVALUE_REFERENCES
#endif
//  Use     CROSPI_BOOST_NO_CXX11_SCOPED_ENUMS instead of   CROSPI_BOOST_NO_SCOPED_ENUMS
#if defined(CROSPI_BOOST_NO_CXX11_SCOPED_ENUMS) && !defined(CROSPI_BOOST_NO_SCOPED_ENUMS)
#  define CROSPI_BOOST_NO_SCOPED_ENUMS
#endif
//  Use     CROSPI_BOOST_NO_CXX11_STATIC_ASSERT instead of   CROSPI_BOOST_NO_STATIC_ASSERT
#if defined(CROSPI_BOOST_NO_CXX11_STATIC_ASSERT) && !defined(CROSPI_BOOST_NO_STATIC_ASSERT)
#  define CROSPI_BOOST_NO_STATIC_ASSERT
#endif
//  Use     CROSPI_BOOST_NO_CXX11_STD_UNORDERED instead of   CROSPI_BOOST_NO_STD_UNORDERED
#if defined(CROSPI_BOOST_NO_CXX11_STD_UNORDERED) && !defined(CROSPI_BOOST_NO_STD_UNORDERED)
#  define CROSPI_BOOST_NO_STD_UNORDERED
#endif
//  Use     CROSPI_BOOST_NO_CXX11_UNICODE_LITERALS instead of   CROSPI_BOOST_NO_UNICODE_LITERALS
#if defined(CROSPI_BOOST_NO_CXX11_UNICODE_LITERALS) && !defined(CROSPI_BOOST_NO_UNICODE_LITERALS)
#  define CROSPI_BOOST_NO_UNICODE_LITERALS
#endif
//  Use     CROSPI_BOOST_NO_CXX11_UNIFIED_INITIALIZATION_SYNTAX instead of   CROSPI_BOOST_NO_UNIFIED_INITIALIZATION_SYNTAX
#if defined(CROSPI_BOOST_NO_CXX11_UNIFIED_INITIALIZATION_SYNTAX) && !defined(CROSPI_BOOST_NO_UNIFIED_INITIALIZATION_SYNTAX)
#  define CROSPI_BOOST_NO_UNIFIED_INITIALIZATION_SYNTAX
#endif
//  Use     CROSPI_BOOST_NO_CXX11_VARIADIC_TEMPLATES instead of   CROSPI_BOOST_NO_VARIADIC_TEMPLATES
#if defined(CROSPI_BOOST_NO_CXX11_VARIADIC_TEMPLATES) && !defined(CROSPI_BOOST_NO_VARIADIC_TEMPLATES)
#  define CROSPI_BOOST_NO_VARIADIC_TEMPLATES
#endif
//  Use     CROSPI_BOOST_NO_CXX11_VARIADIC_MACROS instead of   CROSPI_BOOST_NO_VARIADIC_MACROS
#if defined(CROSPI_BOOST_NO_CXX11_VARIADIC_MACROS) && !defined(CROSPI_BOOST_NO_VARIADIC_MACROS)
#  define CROSPI_BOOST_NO_VARIADIC_MACROS
#endif
//  Use     CROSPI_BOOST_NO_CXX11_NUMERIC_LIMITS instead of   CROSPI_BOOST_NO_NUMERIC_LIMITS_LOWEST
#if defined(CROSPI_BOOST_NO_CXX11_NUMERIC_LIMITS) && !defined(CROSPI_BOOST_NO_NUMERIC_LIMITS_LOWEST)
#  define CROSPI_BOOST_NO_NUMERIC_LIMITS_LOWEST
#endif
//  ------------------ End of deprecated macros for 1.51 ---------------------------


//
// Helper macro for marking types and methods final
//
#if !defined(CROSPI_BOOST_NO_CXX11_FINAL)
#  define CROSPI_BOOST_FINAL final
#else
#  define CROSPI_BOOST_FINAL
#endif

//
// Helper macros CROSPI_BOOST_NOEXCEPT, CROSPI_BOOST_NOEXCEPT_IF, CROSPI_BOOST_NOEXCEPT_EXPR
// These aid the transition to C++11 while still supporting C++03 compilers
//
#ifdef CROSPI_BOOST_NO_CXX11_NOEXCEPT
#  define CROSPI_BOOST_NOEXCEPT
#  define CROSPI_BOOST_NOEXCEPT_OR_NOTHROW throw()
#  define CROSPI_BOOST_NOEXCEPT_IF(Predicate)
#  define CROSPI_BOOST_NOEXCEPT_EXPR(Expression) false
#else
#  define CROSPI_BOOST_NOEXCEPT noexcept
#  define CROSPI_BOOST_NOEXCEPT_OR_NOTHROW noexcept
#  define CROSPI_BOOST_NOEXCEPT_IF(Predicate) noexcept((Predicate))
#  define CROSPI_BOOST_NOEXCEPT_EXPR(Expression) noexcept((Expression))
#endif
//
// Helper macro CROSPI_BOOST_FALLTHROUGH
// Fallback definition of CROSPI_BOOST_FALLTHROUGH macro used to mark intended
// fall-through between case labels in a switch statement. We use a definition
// that requires a semicolon after it to avoid at least one type of misuse even
// on unsupported compilers.
//
#ifndef CROSPI_BOOST_FALLTHROUGH
#  define CROSPI_BOOST_FALLTHROUGH ((void)0)
#endif

//
// constexpr workarounds
//
#if defined(CROSPI_BOOST_NO_CXX11_CONSTEXPR)
#define CROSPI_BOOST_CONSTEXPR
#define CROSPI_BOOST_CONSTEXPR_OR_CONST const
#else
#define CROSPI_BOOST_CONSTEXPR constexpr
#define CROSPI_BOOST_CONSTEXPR_OR_CONST constexpr
#endif
#if defined(CROSPI_BOOST_NO_CXX14_CONSTEXPR)
#define CROSPI_BOOST_CXX14_CONSTEXPR
#else
#define CROSPI_BOOST_CXX14_CONSTEXPR constexpr
#endif
#if !defined(CROSPI_BOOST_NO_CXX17_STRUCTURED_BINDINGS) && defined(CROSPI_BOOST_NO_CXX11_HDR_TUPLE)
#  define CROSPI_BOOST_NO_CXX17_STRUCTURED_BINDINGS
#endif

//
// C++17 inline variables
//
#if !defined(CROSPI_BOOST_NO_CXX17_INLINE_VARIABLES)
#define CROSPI_BOOST_INLINE_VARIABLE inline
#else
#define CROSPI_BOOST_INLINE_VARIABLE
#endif
//
// C++17 if constexpr
//
#if !defined(CROSPI_BOOST_NO_CXX17_IF_CONSTEXPR)
#  define CROSPI_BOOST_IF_CONSTEXPR if constexpr
#else
#  define CROSPI_BOOST_IF_CONSTEXPR if
#endif

#define CROSPI_BOOST_INLINE_CONSTEXPR  CROSPI_BOOST_INLINE_VARIABLE CROSPI_BOOST_CONSTEXPR_OR_CONST

//
// Unused variable/typedef workarounds:
//
#ifndef CROSPI_BOOST_ATTRIBUTE_UNUSED
#  if defined(__has_attribute) && defined(__SUNPRO_CC) && (__SUNPRO_CC > 0x5130)
#    if __has_attribute(maybe_unused)
#       define CROSPI_BOOST_ATTRIBUTE_UNUSED [[maybe_unused]]
#    endif
#  elif defined(__has_cpp_attribute)
#    if __has_cpp_attribute(maybe_unused)
#      define CROSPI_BOOST_ATTRIBUTE_UNUSED [[maybe_unused]]
#    endif
#  endif
#endif

#ifndef CROSPI_BOOST_ATTRIBUTE_UNUSED
#  define CROSPI_BOOST_ATTRIBUTE_UNUSED
#endif

//
// [[nodiscard]]:
//
#if defined(__has_attribute) && defined(__SUNPRO_CC) && (__SUNPRO_CC > 0x5130)
#if __has_attribute(nodiscard)
# define CROSPI_BOOST_ATTRIBUTE_NODISCARD [[nodiscard]]
#endif
#if __has_attribute(no_unique_address)
# define CROSPI_BOOST_ATTRIBUTE_NO_UNIQUE_ADDRESS [[no_unique_address]]
#endif
#elif defined(__has_cpp_attribute)
// clang-6 accepts [[nodiscard]] with -std=c++14, but warns about it -pedantic
#if __has_cpp_attribute(nodiscard) && !(defined(__clang__) && (__cplusplus < 201703L)) && !(defined(__GNUC__) && (__cplusplus < 201100))
# define CROSPI_BOOST_ATTRIBUTE_NODISCARD [[nodiscard]]
#endif
#if __has_cpp_attribute(no_unique_address) && !(defined(__GNUC__) && (__cplusplus < 201100))
# define CROSPI_BOOST_ATTRIBUTE_NO_UNIQUE_ADDRESS [[no_unique_address]]
#endif
#endif
#ifndef CROSPI_BOOST_ATTRIBUTE_NODISCARD
# define CROSPI_BOOST_ATTRIBUTE_NODISCARD
#endif
#ifndef CROSPI_BOOST_ATTRIBUTE_NO_UNIQUE_ADDRESS
# define CROSPI_BOOST_ATTRIBUTE_NO_UNIQUE_ADDRESS
#endif

#define CROSPI_BOOST_STATIC_CONSTEXPR  static CROSPI_BOOST_CONSTEXPR_OR_CONST

#if !defined(CROSPI_BOOST_NO_CXX11_NULLPTR)
# define CROSPI_BOOST_NULLPTR nullptr
#else
# define CROSPI_BOOST_NULLPTR 0
#endif

//
// Set CROSPI_BOOST_HAS_STATIC_ASSERT when CROSPI_BOOST_NO_CXX11_STATIC_ASSERT is not defined
//
#if !defined(CROSPI_BOOST_NO_CXX11_STATIC_ASSERT) && !defined(CROSPI_BOOST_HAS_STATIC_ASSERT)
#  define CROSPI_BOOST_HAS_STATIC_ASSERT
#endif

//
// Set CROSPI_BOOST_HAS_RVALUE_REFS when CROSPI_BOOST_NO_CXX11_RVALUE_REFERENCES is not defined
//
#if !defined(CROSPI_BOOST_NO_CXX11_RVALUE_REFERENCES) && !defined(CROSPI_BOOST_HAS_RVALUE_REFS)
#define CROSPI_BOOST_HAS_RVALUE_REFS
#endif

//
// Set CROSPI_BOOST_HAS_VARIADIC_TMPL when CROSPI_BOOST_NO_CXX11_VARIADIC_TEMPLATES is not defined
//
#if !defined(CROSPI_BOOST_NO_CXX11_VARIADIC_TEMPLATES) && !defined(CROSPI_BOOST_HAS_VARIADIC_TMPL)
#define CROSPI_BOOST_HAS_VARIADIC_TMPL
#endif
//
// Set CROSPI_BOOST_NO_CXX11_FIXED_LENGTH_VARIADIC_TEMPLATE_EXPANSION_PACKS when
// CROSPI_BOOST_NO_CXX11_VARIADIC_TEMPLATES is set:
//
#if defined(CROSPI_BOOST_NO_CXX11_VARIADIC_TEMPLATES) && !defined(CROSPI_BOOST_NO_CXX11_FIXED_LENGTH_VARIADIC_TEMPLATE_EXPANSION_PACKS)
#  define CROSPI_BOOST_NO_CXX11_FIXED_LENGTH_VARIADIC_TEMPLATE_EXPANSION_PACKS
#endif

// This is a catch all case for obsolete compilers / std libs:
#if !defined(_YVALS) && !defined(_CPPLIB_VER)  // msvc std lib already configured
#if (!defined(__has_include) || (__cplusplus < 201700))
#  define CROSPI_BOOST_NO_CXX17_HDR_OPTIONAL
#  define CROSPI_BOOST_NO_CXX17_HDR_STRING_VIEW
#  define CROSPI_BOOST_NO_CXX17_HDR_VARIANT
#  define CROSPI_BOOST_NO_CXX17_HDR_ANY
#  define CROSPI_BOOST_NO_CXX17_HDR_MEMORY_RESOURCE
#  define CROSPI_BOOST_NO_CXX17_HDR_CHARCONV
#  define CROSPI_BOOST_NO_CXX17_HDR_EXECUTION
#  define CROSPI_BOOST_NO_CXX17_HDR_FILESYSTEM
#else
#if !__has_include(<optional>)
#  define CROSPI_BOOST_NO_CXX17_HDR_OPTIONAL
#endif
#if !__has_include(<string_view>)
#  define CROSPI_BOOST_NO_CXX17_HDR_STRING_VIEW
#endif
#if !__has_include(<variant>)
#  define CROSPI_BOOST_NO_CXX17_HDR_VARIANT
#endif
#if !__has_include(<any>)
#  define CROSPI_BOOST_NO_CXX17_HDR_ANY
#endif
#if !__has_include(<memory_resource>)
#  define CROSPI_BOOST_NO_CXX17_HDR_MEMORY_RESOURCE
#endif
#if !__has_include(<charconv>)
#  define CROSPI_BOOST_NO_CXX17_HDR_CHARCONV
#endif
#if !__has_include(<execution>)
#  define CROSPI_BOOST_NO_CXX17_HDR_EXECUTION
#endif
#if !__has_include(<filesystem>)
#  define CROSPI_BOOST_NO_CXX17_HDR_FILESYSTEM
#endif
#endif
#endif
//
// Define the std level that the compiler claims to support:
//
#ifndef CROSPI_BOOST_CXX_VERSION
#  define CROSPI_BOOST_CXX_VERSION __cplusplus
#endif

#if (!defined(__has_include) || (CROSPI_BOOST_CXX_VERSION < 201704))
#  define CROSPI_BOOST_NO_CXX20_HDR_BARRIER
#  define CROSPI_BOOST_NO_CXX20_HDR_FORMAT
#  define CROSPI_BOOST_NO_CXX20_HDR_SOURCE_LOCATION
#  define CROSPI_BOOST_NO_CXX20_HDR_BIT
#  define CROSPI_BOOST_NO_CXX20_HDR_LATCH
#  define CROSPI_BOOST_NO_CXX20_HDR_SPAN
#  define CROSPI_BOOST_NO_CXX20_HDR_COMPARE
#  define CROSPI_BOOST_NO_CXX20_HDR_NUMBERS
#  define CROSPI_BOOST_NO_CXX20_HDR_STOP_TOKEN
#  define CROSPI_BOOST_NO_CXX20_HDR_CONCEPTS
#  define CROSPI_BOOST_NO_CXX20_HDR_RANGES
#  define CROSPI_BOOST_NO_CXX20_HDR_SYNCSTREAM
#  define CROSPI_BOOST_NO_CXX20_HDR_COROUTINE
#  define CROSPI_BOOST_NO_CXX20_HDR_SEMAPHORE
#else
#if (!__has_include(<barrier>) || !defined(__cpp_lib_barrier) || (__cpp_lib_barrier < 201907L)) && !defined(CROSPI_BOOST_NO_CXX20_HDR_BARRIER)
#  define CROSPI_BOOST_NO_CXX20_HDR_BARRIER
#endif
#if (!__has_include(<format>) || !defined(__cpp_lib_format) || (__cpp_lib_format < 201907L)) && !defined(CROSPI_BOOST_NO_CXX20_HDR_FORMAT)
#  define CROSPI_BOOST_NO_CXX20_HDR_FORMAT
#endif
#if (!__has_include(<source_location>) || !defined(__cpp_lib_source_location) || (__cpp_lib_source_location < 201907L)) && !defined(CROSPI_BOOST_NO_CXX20_HDR_SOURCE_LOCATION)
#  define CROSPI_BOOST_NO_CXX20_HDR_SOURCE_LOCATION
#endif
#if (!__has_include(<bit>) || !defined(__cpp_lib_bit_cast) || (__cpp_lib_bit_cast < 201806L) || !defined(__cpp_lib_bitops) || (__cpp_lib_bitops < 201907L) || !defined(__cpp_lib_endian) || (__cpp_lib_endian < 201907L)) && !defined(CROSPI_BOOST_NO_CXX20_HDR_BIT)
#  define CROSPI_BOOST_NO_CXX20_HDR_BIT
#endif
#if (!__has_include(<latch>) || !defined(__cpp_lib_latch) || (__cpp_lib_latch < 201907L)) && !defined(CROSPI_BOOST_NO_CXX20_HDR_LATCH)
#  define CROSPI_BOOST_NO_CXX20_HDR_LATCH
#endif
#if (!__has_include(<span>) || !defined(__cpp_lib_span) || (__cpp_lib_span < 202002L)) && !defined(CROSPI_BOOST_NO_CXX20_HDR_SPAN)
#  define CROSPI_BOOST_NO_CXX20_HDR_SPAN
#endif
#if (!__has_include(<compare>) || !defined(__cpp_lib_three_way_comparison) || (__cpp_lib_three_way_comparison < 201907L)) && !defined(CROSPI_BOOST_NO_CXX20_HDR_COMPARE)
#  define CROSPI_BOOST_NO_CXX20_HDR_COMPARE
#endif
#if (!__has_include(<numbers>) || !defined(__cpp_lib_math_constants) || (__cpp_lib_math_constants < 201907L)) && !defined(CROSPI_BOOST_NO_CXX20_HDR_NUMBERS)
#  define CROSPI_BOOST_NO_CXX20_HDR_NUMBERS
#endif
#if (!__has_include(<stop_token>) || !defined(__cpp_lib_jthread) || (__cpp_lib_jthread < 201911L)) && !defined(CROSPI_BOOST_NO_CXX20_HDR_STOP_TOKEN)
#  define CROSPI_BOOST_NO_CXX20_HDR_STOP_TOKEN
#endif
#if (!__has_include(<concepts>) || !defined(__cpp_lib_concepts) || (__cpp_lib_concepts < 202002L)) && !defined(_YVALS) && !defined(_CPPLIB_VER) && !defined(CROSPI_BOOST_NO_CXX20_HDR_CONCEPTS)
#  define CROSPI_BOOST_NO_CXX20_HDR_CONCEPTS
#endif
#if (!__has_include(<ranges>) || !defined(__cpp_lib_ranges) || (__cpp_lib_ranges < 201911L)) && !defined(CROSPI_BOOST_NO_CXX20_HDR_RANGES)
#  define CROSPI_BOOST_NO_CXX20_HDR_RANGES
#endif
#if (!__has_include(<syncstream>) || !defined(__cpp_lib_syncbuf) || (__cpp_lib_syncbuf < 201803L)) && !defined(CROSPI_BOOST_NO_CXX20_HDR_SYNCSTREAM)
#  define CROSPI_BOOST_NO_CXX20_HDR_SYNCSTREAM
#endif
#if (!__has_include(<coroutine>) || !defined(__cpp_lib_coroutine) || (__cpp_lib_coroutine < 201902L)) && !defined(CROSPI_BOOST_NO_CXX20_HDR_COROUTINE)
#  define CROSPI_BOOST_NO_CXX20_HDR_COROUTINE
#endif
#if (!__has_include(<semaphore>) || !defined(__cpp_lib_semaphore) || (__cpp_lib_semaphore < 201907L)) && !defined(CROSPI_BOOST_NO_CXX20_HDR_SEMAPHORE)
#  define CROSPI_BOOST_NO_CXX20_HDR_SEMAPHORE
#endif
#endif

#if (!defined(__has_include) || (CROSPI_BOOST_CXX_VERSION < 202003L))
#  define CROSPI_BOOST_NO_CXX23_HDR_EXPECTED
#  define CROSPI_BOOST_NO_CXX23_HDR_FLAT_MAP
#  define CROSPI_BOOST_NO_CXX23_HDR_FLAT_SET
#  define CROSPI_BOOST_NO_CXX23_HDR_GENERATOR
#  define CROSPI_BOOST_NO_CXX23_HDR_MDSPAN
#  define CROSPI_BOOST_NO_CXX23_HDR_PRINT
#  define CROSPI_BOOST_NO_CXX23_HDR_SPANSTREAM
#  define CROSPI_BOOST_NO_CXX23_HDR_STACKTRACE
#  define CROSPI_BOOST_NO_CXX23_HDR_STDFLOAT
#else
#if (!__has_include(<expected>) || !defined(__cpp_lib_expected) || (__cpp_lib_expected < 202211L)) && !defined(CROSPI_BOOST_NO_CXX23_HDR_EXPECTED)
#  define CROSPI_BOOST_NO_CXX23_HDR_EXPECTED
#endif
#if (!__has_include(<flat_map>) || !defined(__cpp_lib_flat_map) || (__cpp_lib_flat_map < 202207L)) && !defined(CROSPI_BOOST_NO_CXX23_HDR_FLAT_MAP)
#  define CROSPI_BOOST_NO_CXX23_HDR_FLAT_MAP
#endif
#if (!__has_include(<flat_set>) || !defined(__cpp_lib_flat_set) || (__cpp_lib_flat_set < 202207L)) && !defined(CROSPI_BOOST_NO_CXX23_HDR_FLAT_SET)
#  define CROSPI_BOOST_NO_CXX23_HDR_FLAT_SET
#endif
#if (!__has_include(<generator>) || !defined(__cpp_lib_generator) || (__cpp_lib_generator < 202207L)) && !defined(CROSPI_BOOST_NO_CXX23_HDR_GENERATOR)
#  define CROSPI_BOOST_NO_CXX23_HDR_GENERATOR
#endif
#if (!__has_include(<mdspan>) || !defined(__cpp_lib_mdspan) || (__cpp_lib_mdspan < 202207L)) && !defined(CROSPI_BOOST_NO_CXX23_HDR_MDSPAN)
#  define CROSPI_BOOST_NO_CXX23_HDR_MDSPAN
#endif
#if (!__has_include(<print>) || !defined(__cpp_lib_print) || (__cpp_lib_print < 202207L)) && !defined(CROSPI_BOOST_NO_CXX23_HDR_PRINT)
#  define CROSPI_BOOST_NO_CXX23_HDR_PRINT
#endif
#if (!__has_include(<spanstream>) || !defined(__cpp_lib_spanstream) || (__cpp_lib_spanstream < 202106L)) && !defined(CROSPI_BOOST_NO_CXX23_HDR_SPANSTREAM)
#  define CROSPI_BOOST_NO_CXX23_HDR_SPANSTREAM
#endif
#if (!__has_include(<stacktrace>) || !defined(__cpp_lib_stacktrace) || (__cpp_lib_stacktrace < 202011L)) && !defined(CROSPI_BOOST_NO_CXX23_HDR_STACKTRACE)
#  define CROSPI_BOOST_NO_CXX23_HDR_STACKTRACE
#endif
#if !__has_include(<stdfloat>) && !defined(CROSPI_BOOST_NO_CXX23_HDR_STDFLOAT)
#  define CROSPI_BOOST_NO_CXX23_HDR_STDFLOAT
#endif
#endif

#if defined(__cplusplus) && defined(__has_include)
#if !__has_include(<version>)
#  define CROSPI_BOOST_NO_CXX20_HDR_VERSION
#else
// For convenience, this is always included:
#  include <version>
#endif
#else
#  define CROSPI_BOOST_NO_CXX20_HDR_VERSION
#endif

#if defined(CROSPI_BOOST_MSVC)
#if (CROSPI_BOOST_MSVC < 1914) || (_MSVC_LANG < 201703)
#  define CROSPI_BOOST_NO_CXX17_DEDUCTION_GUIDES
#endif
#elif !defined(__cpp_deduction_guides) || (__cpp_deduction_guides < 201606)
#  define CROSPI_BOOST_NO_CXX17_DEDUCTION_GUIDES
#endif

//
// Define composite agregate macros:
//
#include <crospi_boost/config/detail/cxx_composite.hpp>

//
// Finish off with checks for macros that are depricated / no longer supported,
// if any of these are set then it's very likely that much of Boost will no
// longer work.  So stop with a #error for now, but give the user a chance
// to continue at their own risk if they really want to:
//
#if defined(CROSPI_BOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION) && !defined(CROSPI_BOOST_CONFIG_ALLOW_DEPRECATED)
#  error "You are using a compiler which lacks features which are now a minimum requirement in order to use Boost, define CROSPI_BOOST_CONFIG_ALLOW_DEPRECATED if you want to continue at your own risk!!!"
#endif

#endif
