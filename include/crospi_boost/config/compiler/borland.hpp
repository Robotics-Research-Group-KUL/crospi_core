//  (C) Copyright John Maddock 2001 - 2003.
//  (C) Copyright David Abrahams 2002 - 2003.
//  (C) Copyright Aleksey Gurtovoy 2002.
//  Use, modification and distribution are subject to the
//  Boost Software License, Version 1.0. (See accompanying file
//  LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)

//  See http://www.boost.org for most recent version.

//  Borland C++ compiler setup:

//
// versions check:
// we don't support Borland prior to version 5.4:
#if __BORLANDC__ < 0x540
#  error "Compiler not supported or configured - please reconfigure"
#endif

// last known compiler version:
#if (__BORLANDC__ > 0x613)
//#  if defined(CROSPI_BOOST_ASSERT_CONFIG)
#     error "boost: Unknown compiler version - please run the configure tests and report the results"
//#  else
//#     pragma message( "boost: Unknown compiler version - please run the configure tests and report the results")
//#  endif
#elif (__BORLANDC__ == 0x600)
#  error "CBuilderX preview compiler is no longer supported"
#endif

//
// Support macros to help with standard library detection
#if (__BORLANDC__ < 0x560) || defined(_USE_OLD_RW_STL)
#  define CROSPI_BOOST_BCB_WITH_ROGUE_WAVE
#elif __BORLANDC__ < 0x570
#  define CROSPI_BOOST_BCB_WITH_STLPORT
#else
#  define CROSPI_BOOST_BCB_WITH_DINKUMWARE
#endif

//
// Version 5.0 and below:
#   if __BORLANDC__ <= 0x0550
// Borland C++Builder 4 and 5:
#     define CROSPI_BOOST_NO_MEMBER_TEMPLATE_FRIENDS
#     if __BORLANDC__ == 0x0550
// Borland C++Builder 5, command-line compiler 5.5:
#       define CROSPI_BOOST_NO_OPERATORS_IN_NAMESPACE
#     endif
// Variadic macros do not exist for C++ Builder versions 5 and below
#define CROSPI_BOOST_NO_CXX11_VARIADIC_MACROS
#   endif

// Version 5.51 and below:
#if (__BORLANDC__ <= 0x551)
#  define CROSPI_BOOST_NO_CV_SPECIALIZATIONS
#  define CROSPI_BOOST_NO_CV_VOID_SPECIALIZATIONS
#  define CROSPI_BOOST_NO_DEDUCED_TYPENAME
// workaround for missing WCHAR_MAX/WCHAR_MIN:
#ifdef __cplusplus
#include <climits>
#include <cwchar>
#else
#include <limits.h>
#include <wchar.h>
#endif // __cplusplus
#ifndef WCHAR_MAX
#  define WCHAR_MAX 0xffff
#endif
#ifndef WCHAR_MIN
#  define WCHAR_MIN 0
#endif
#endif

// Borland C++ Builder 6 and below:
#if (__BORLANDC__ <= 0x564)

#  if defined(NDEBUG) && defined(__cplusplus)
      // fix broken <cstring> so that Boost.test works:
#     include <cstring>
#     undef strcmp
#  endif
   // fix broken errno declaration:
#  include <errno.h>
#  ifndef errno
#     define errno errno
#  endif

#endif

//
// new bug in 5.61:
#if (__BORLANDC__ >= 0x561) && (__BORLANDC__ <= 0x580)
   // this seems to be needed by the command line compiler, but not the IDE:
#  define CROSPI_BOOST_NO_MEMBER_FUNCTION_SPECIALIZATIONS
#endif

// Borland C++ Builder 2006 Update 2 and below:
#if (__BORLANDC__ <= 0x582)
#  define CROSPI_BOOST_NO_SFINAE
#  define CROSPI_BOOST_BCB_PARTIAL_SPECIALIZATION_BUG
#  define CROSPI_BOOST_NO_TEMPLATE_TEMPLATES

#  define CROSPI_BOOST_NO_PRIVATE_IN_AGGREGATE

#  ifdef _WIN32
#     define CROSPI_BOOST_NO_SWPRINTF
#  elif defined(linux) || defined(__linux__) || defined(__linux)
      // we should really be able to do without this
      // but the wcs* functions aren't imported into std::
#     define CROSPI_BOOST_NO_STDC_NAMESPACE
      // _CPPUNWIND doesn't get automatically set for some reason:
#     pragma defineonoption CROSPI_BOOST_CPPUNWIND -x
#  endif
#endif

#if (__BORLANDC__ <= 0x613)  // Beman has asked Alisdair for more info
   // we shouldn't really need this - but too many things choke
   // without it, this needs more investigation:
#  define CROSPI_BOOST_NO_LIMITS_COMPILE_TIME_CONSTANTS
#  define CROSPI_BOOST_NO_IS_ABSTRACT
#  define CROSPI_BOOST_NO_FUNCTION_TYPE_SPECIALIZATIONS
#  define CROSPI_BOOST_NO_USING_TEMPLATE
#  define CROSPI_BOOST_SP_NO_SP_CONVERTIBLE

// Temporary workaround
#define CROSPI_BOOST_MPL_CFG_NO_PREPROCESSED_HEADERS
#endif

// Borland C++ Builder 2008 and below:
#  define CROSPI_BOOST_NO_INTEGRAL_INT64_T
#  define CROSPI_BOOST_FUNCTION_SCOPE_USING_DECLARATION_BREAKS_ADL
#  define CROSPI_BOOST_NO_DEPENDENT_NESTED_DERIVATIONS
#  define CROSPI_BOOST_NO_MEMBER_TEMPLATE_FRIENDS
#  define CROSPI_BOOST_NO_TWO_PHASE_NAME_LOOKUP
#  define CROSPI_BOOST_NO_USING_DECLARATION_OVERLOADS_FROM_TYPENAME_BASE
#  define CROSPI_BOOST_NO_NESTED_FRIENDSHIP
#  define CROSPI_BOOST_NO_TYPENAME_WITH_CTOR
#if (__BORLANDC__ < 0x600)
#  define CROSPI_BOOST_ILLEGAL_CV_REFERENCES
#endif

//
//  Positive Feature detection
//
// Borland C++ Builder 2008 and below:
#if (__BORLANDC__ >= 0x599)
#  pragma defineonoption CROSPI_BOOST_CODEGEAR_0X_SUPPORT -Ax
#endif
//
// C++0x Macros:
//
#if !defined( CROSPI_BOOST_CODEGEAR_0X_SUPPORT ) || (__BORLANDC__ < 0x610)
#  define CROSPI_BOOST_NO_CXX11_CHAR16_T
#  define CROSPI_BOOST_NO_CXX11_CHAR32_T
#  define CROSPI_BOOST_NO_CXX11_DECLTYPE
#  define CROSPI_BOOST_NO_CXX11_EXPLICIT_CONVERSION_OPERATORS
#  define CROSPI_BOOST_NO_CXX11_EXTERN_TEMPLATE
#  define CROSPI_BOOST_NO_CXX11_RVALUE_REFERENCES
#  define CROSPI_BOOST_NO_CXX11_SCOPED_ENUMS
#  define CROSPI_BOOST_NO_CXX11_STATIC_ASSERT
#else
#  define CROSPI_BOOST_HAS_ALIGNOF
#  define CROSPI_BOOST_HAS_CHAR16_T
#  define CROSPI_BOOST_HAS_CHAR32_T
#  define CROSPI_BOOST_HAS_DECLTYPE
#  define CROSPI_BOOST_HAS_EXPLICIT_CONVERSION_OPS
#  define CROSPI_BOOST_HAS_REF_QUALIFIER
#  define CROSPI_BOOST_HAS_RVALUE_REFS
#  define CROSPI_BOOST_HAS_STATIC_ASSERT
#endif

#define CROSPI_BOOST_NO_CXX11_AUTO_DECLARATIONS
#define CROSPI_BOOST_NO_CXX11_AUTO_MULTIDECLARATIONS
#define CROSPI_BOOST_NO_CXX11_CONSTEXPR
#define CROSPI_BOOST_NO_CXX11_DECLTYPE_N3276
#define CROSPI_BOOST_NO_CXX11_DEFAULTED_FUNCTIONS
#define CROSPI_BOOST_NO_CXX11_DEFAULTED_MOVES
#define CROSPI_BOOST_NO_CXX11_DELETED_FUNCTIONS
#define CROSPI_BOOST_NO_CXX11_FUNCTION_TEMPLATE_DEFAULT_ARGS
#define CROSPI_BOOST_NO_CXX11_HDR_INITIALIZER_LIST
#define CROSPI_BOOST_NO_CXX11_LAMBDAS
#define CROSPI_BOOST_NO_CXX11_LOCAL_CLASS_TEMPLATE_PARAMETERS
#define CROSPI_BOOST_NO_CXX11_NULLPTR
#define CROSPI_BOOST_NO_CXX11_RANGE_BASED_FOR
#define CROSPI_BOOST_NO_CXX11_RAW_LITERALS
#define CROSPI_BOOST_NO_CXX11_RVALUE_REFERENCES
#define CROSPI_BOOST_NO_CXX11_SCOPED_ENUMS
#define CROSPI_BOOST_NO_SFINAE_EXPR
#define CROSPI_BOOST_NO_CXX11_SFINAE_EXPR
#define CROSPI_BOOST_NO_CXX11_TEMPLATE_ALIASES
#define CROSPI_BOOST_NO_CXX11_UNICODE_LITERALS    // UTF-8 still not supported
#define CROSPI_BOOST_NO_CXX11_VARIADIC_TEMPLATES
#define CROSPI_BOOST_NO_CXX11_NOEXCEPT
#define CROSPI_BOOST_NO_CXX11_UNIFIED_INITIALIZATION_SYNTAX
#define CROSPI_BOOST_NO_CXX11_USER_DEFINED_LITERALS
#define CROSPI_BOOST_NO_CXX11_ALIGNAS
#define CROSPI_BOOST_NO_CXX11_ALIGNOF
#define CROSPI_BOOST_NO_CXX11_TRAILING_RESULT_TYPES
#define CROSPI_BOOST_NO_CXX11_INLINE_NAMESPACES
#define CROSPI_BOOST_NO_CXX11_REF_QUALIFIERS
#define CROSPI_BOOST_NO_CXX11_FINAL
#define CROSPI_BOOST_NO_CXX11_OVERRIDE
#define CROSPI_BOOST_NO_CXX11_THREAD_LOCAL
#define CROSPI_BOOST_NO_CXX11_UNRESTRICTED_UNION

// C++ 14:
#if !defined(__cpp_aggregate_nsdmi) || (__cpp_aggregate_nsdmi < 201304)
#  define CROSPI_BOOST_NO_CXX14_AGGREGATE_NSDMI
#endif
#if !defined(__cpp_binary_literals) || (__cpp_binary_literals < 201304)
#  define CROSPI_BOOST_NO_CXX14_BINARY_LITERALS
#endif
#if !defined(__cpp_constexpr) || (__cpp_constexpr < 201304)
#  define CROSPI_BOOST_NO_CXX14_CONSTEXPR
#endif
#if !defined(__cpp_decltype_auto) || (__cpp_decltype_auto < 201304)
#  define CROSPI_BOOST_NO_CXX14_DECLTYPE_AUTO
#endif
#if (__cplusplus < 201304) // There's no SD6 check for this....
#  define CROSPI_BOOST_NO_CXX14_DIGIT_SEPARATORS
#endif
#if !defined(__cpp_generic_lambdas) || (__cpp_generic_lambdas < 201304)
#  define CROSPI_BOOST_NO_CXX14_GENERIC_LAMBDAS
#endif
#if !defined(__cpp_init_captures) || (__cpp_init_captures < 201304)
#  define CROSPI_BOOST_NO_CXX14_INITIALIZED_LAMBDA_CAPTURES
#endif
#if !defined(__cpp_return_type_deduction) || (__cpp_return_type_deduction < 201304)
#  define CROSPI_BOOST_NO_CXX14_RETURN_TYPE_DEDUCTION
#endif
#if !defined(__cpp_variable_templates) || (__cpp_variable_templates < 201304)
#  define CROSPI_BOOST_NO_CXX14_VARIABLE_TEMPLATES
#endif

// C++17
#if !defined(__cpp_structured_bindings) || (__cpp_structured_bindings < 201606)
#  define CROSPI_BOOST_NO_CXX17_STRUCTURED_BINDINGS
#endif
#if !defined(__cpp_inline_variables) || (__cpp_inline_variables < 201606)
#  define CROSPI_BOOST_NO_CXX17_INLINE_VARIABLES
#endif
#if !defined(__cpp_fold_expressions) || (__cpp_fold_expressions < 201603)
#  define CROSPI_BOOST_NO_CXX17_FOLD_EXPRESSIONS
#endif
#if !defined(__cpp_if_constexpr) || (__cpp_if_constexpr < 201606)
#  define CROSPI_BOOST_NO_CXX17_IF_CONSTEXPR
#endif
#if !defined(__cpp_nontype_template_parameter_auto) || (__cpp_nontype_template_parameter_auto < 201606)
#  define CROSPI_BOOST_NO_CXX17_AUTO_NONTYPE_TEMPLATE_PARAMS
#endif

#if __BORLANDC__ >= 0x590
#  define CROSPI_BOOST_HAS_TR1_HASH

#  define CROSPI_BOOST_HAS_MACRO_USE_FACET
#endif

//
// Post 0x561 we have long long and stdint.h:
#if __BORLANDC__ >= 0x561
#  ifndef __NO_LONG_LONG
#     define CROSPI_BOOST_HAS_LONG_LONG
#  else
#     define CROSPI_BOOST_NO_LONG_LONG
#  endif
   // On non-Win32 platforms let the platform config figure this out:
#  ifdef _WIN32
#      define CROSPI_BOOST_HAS_STDINT_H
#  endif
#endif

// Borland C++Builder 6 defaults to using STLPort.  If _USE_OLD_RW_STL is
// defined, then we have 0x560 or greater with the Rogue Wave implementation
// which presumably has the std::DBL_MAX bug.
#if defined( CROSPI_BOOST_BCB_WITH_ROGUE_WAVE )
// <climits> is partly broken, some macros define symbols that are really in
// namespace std, so you end up having to use illegal constructs like
// std::DBL_MAX, as a fix we'll just include float.h and have done with:
#include <float.h>
#endif
//
// __int64:
//
#if (__BORLANDC__ >= 0x530) && !defined(__STRICT_ANSI__)
#  define CROSPI_BOOST_HAS_MS_INT64
#endif
//
// check for exception handling support:
//
#if !defined(_CPPUNWIND) && !defined(CROSPI_BOOST_CPPUNWIND) && !defined(__EXCEPTIONS) && !defined(CROSPI_BOOST_NO_EXCEPTIONS)
#  define CROSPI_BOOST_NO_EXCEPTIONS
#endif
//
// all versions have a <dirent.h>:
//
#ifndef __STRICT_ANSI__
#  define CROSPI_BOOST_HAS_DIRENT_H
#endif
//
// all versions support __declspec:
//
#if defined(__STRICT_ANSI__)
// config/platform/win32.hpp will define CROSPI_BOOST_SYMBOL_EXPORT, etc., unless already defined
#  define CROSPI_BOOST_SYMBOL_EXPORT
#endif
//
// ABI fixing headers:
//
#if __BORLANDC__ != 0x600 // not implemented for version 6 compiler yet
#ifndef CROSPI_BOOST_ABI_PREFIX
#  define CROSPI_BOOST_ABI_PREFIX "crospi_boost/config/abi/borland_prefix.hpp"
#endif
#ifndef CROSPI_BOOST_ABI_SUFFIX
#  define CROSPI_BOOST_ABI_SUFFIX "crospi_boost/config/abi/borland_suffix.hpp"
#endif
#endif
//
// Disable Win32 support in ANSI mode:
//
#if __BORLANDC__ < 0x600
#  pragma defineonoption CROSPI_BOOST_DISABLE_WIN32 -A
#elif defined(__STRICT_ANSI__)
#  define CROSPI_BOOST_DISABLE_WIN32
#endif
//
// MSVC compatibility mode does some nasty things:
// TODO: look up if this doesn't apply to the whole 12xx range
//
#if defined(_MSC_VER) && (_MSC_VER <= 1200)
#  define CROSPI_BOOST_NO_ARGUMENT_DEPENDENT_LOOKUP
#  define CROSPI_BOOST_NO_VOID_RETURNS
#endif

// Borland did not implement value-initialization completely, as I reported
// in 2007, Borland Report 51854, "Value-initialization: POD struct should be
// zero-initialized", http://qc.embarcadero.com/wc/qcmain.aspx?d=51854
// See also: http://www.boost.org/libs/utility/value_init.htm#compiler_issues
// (Niels Dekker, LKEB, April 2010)
#define CROSPI_BOOST_NO_COMPLETE_VALUE_INITIALIZATION

#define CROSPI_BOOST_BORLANDC __BORLANDC__
#define CROSPI_BOOST_COMPILER "Classic Borland C++ version " CROSPI_BOOST_STRINGIZE(__BORLANDC__)
