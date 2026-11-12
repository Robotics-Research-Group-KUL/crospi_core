//  (C) Copyright Bryce Lelbach 2011

//  Use, modification and distribution are subject to the
//  Boost Software License, Version 1.0. (See accompanying file
//  LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)

//  See http://www.boost.org for most recent version.

// PathScale EKOPath C++ Compiler

#ifndef CROSPI_BOOST_COMPILER
#  define CROSPI_BOOST_COMPILER "PathScale EKOPath C++ Compiler version " __PATHSCALE__
#endif

#if __PATHCC__ >= 6 
// PathCC is based on clang, and supports the __has_*() builtins used 
// to detect features in clang.hpp. Since the clang toolset is much 
// better maintained, it is more convenient to reuse its definitions. 
#  include "crospi_boost/config/compiler/clang.hpp"
#elif __PATHCC__ >= 4 
#  define CROSPI_BOOST_MSVC6_MEMBER_TEMPLATES
#  define CROSPI_BOOST_HAS_UNISTD_H
#  define CROSPI_BOOST_HAS_STDINT_H
#  define CROSPI_BOOST_HAS_SIGACTION
#  define CROSPI_BOOST_HAS_SCHED_YIELD
#  define CROSPI_BOOST_HAS_THREADS
#  define CROSPI_BOOST_HAS_PTHREADS
#  define CROSPI_BOOST_HAS_PTHREAD_YIELD
#  define CROSPI_BOOST_HAS_PTHREAD_MUTEXATTR_SETTYPE
#  define CROSPI_BOOST_HAS_PARTIAL_STD_ALLOCATOR
#  define CROSPI_BOOST_HAS_NRVO
#  define CROSPI_BOOST_HAS_NL_TYPES_H
#  define CROSPI_BOOST_HAS_NANOSLEEP
#  define CROSPI_BOOST_HAS_LONG_LONG
#  define CROSPI_BOOST_HAS_LOG1P
#  define CROSPI_BOOST_HAS_GETTIMEOFDAY
#  define CROSPI_BOOST_HAS_EXPM1
#  define CROSPI_BOOST_HAS_DIRENT_H
#  define CROSPI_BOOST_HAS_CLOCK_GETTIME
#  define CROSPI_BOOST_NO_CXX11_VARIADIC_TEMPLATES
#  define CROSPI_BOOST_NO_CXX11_UNICODE_LITERALS
#  define CROSPI_BOOST_NO_CXX11_TEMPLATE_ALIASES
#  define CROSPI_BOOST_NO_CXX11_STATIC_ASSERT
#  define CROSPI_BOOST_NO_SFINAE_EXPR
#  define CROSPI_BOOST_NO_CXX11_SFINAE_EXPR
#  define CROSPI_BOOST_NO_CXX11_SCOPED_ENUMS
#  define CROSPI_BOOST_NO_CXX11_RVALUE_REFERENCES
#  define CROSPI_BOOST_NO_CXX11_RANGE_BASED_FOR
#  define CROSPI_BOOST_NO_CXX11_RAW_LITERALS
#  define CROSPI_BOOST_NO_CXX11_NULLPTR
#  define CROSPI_BOOST_NO_CXX11_NUMERIC_LIMITS
#  define CROSPI_BOOST_NO_CXX11_NOEXCEPT
#  define CROSPI_BOOST_NO_CXX11_LAMBDAS
#  define CROSPI_BOOST_NO_CXX11_LOCAL_CLASS_TEMPLATE_PARAMETERS
#  define CROSPI_BOOST_NO_MS_INT64_NUMERIC_LIMITS
#  define CROSPI_BOOST_NO_CXX11_FUNCTION_TEMPLATE_DEFAULT_ARGS
#  define CROSPI_BOOST_NO_CXX11_EXPLICIT_CONVERSION_OPERATORS
#  define CROSPI_BOOST_NO_CXX11_DELETED_FUNCTIONS
#  define CROSPI_BOOST_NO_CXX11_DEFAULTED_FUNCTIONS
#  define CROSPI_BOOST_NO_CXX11_DECLTYPE
#  define CROSPI_BOOST_NO_CXX11_DECLTYPE_N3276
#  define CROSPI_BOOST_NO_CXX11_CONSTEXPR
#  define CROSPI_BOOST_NO_COMPLETE_VALUE_INITIALIZATION
#  define CROSPI_BOOST_NO_CXX11_CHAR32_T
#  define CROSPI_BOOST_NO_CXX11_CHAR16_T
#  define CROSPI_BOOST_NO_CXX11_AUTO_MULTIDECLARATIONS
#  define CROSPI_BOOST_NO_CXX11_AUTO_DECLARATIONS
#  define CROSPI_BOOST_NO_CXX11_UNIFIED_INITIALIZATION_SYNTAX
#  define CROSPI_BOOST_NO_CXX11_HDR_UNORDERED_SET
#  define CROSPI_BOOST_NO_CXX11_HDR_UNORDERED_MAP
#  define CROSPI_BOOST_NO_CXX11_HDR_TYPEINDEX
#  define CROSPI_BOOST_NO_CXX11_HDR_TUPLE
#  define CROSPI_BOOST_NO_CXX11_HDR_THREAD
#  define CROSPI_BOOST_NO_CXX11_HDR_SYSTEM_ERROR
#  define CROSPI_BOOST_NO_CXX11_HDR_REGEX
#  define CROSPI_BOOST_NO_CXX11_HDR_RATIO
#  define CROSPI_BOOST_NO_CXX11_HDR_RANDOM
#  define CROSPI_BOOST_NO_CXX11_HDR_MUTEX
#  define CROSPI_BOOST_NO_CXX11_HDR_INITIALIZER_LIST
#  define CROSPI_BOOST_NO_CXX11_HDR_FUTURE
#  define CROSPI_BOOST_NO_CXX11_HDR_FORWARD_LIST
#  define CROSPI_BOOST_NO_CXX11_HDR_CONDITION_VARIABLE
#  define CROSPI_BOOST_NO_CXX11_HDR_CODECVT
#  define CROSPI_BOOST_NO_CXX11_HDR_CHRONO
#  define CROSPI_BOOST_NO_CXX11_USER_DEFINED_LITERALS
#  define CROSPI_BOOST_NO_CXX11_ALIGNAS
#  define CROSPI_BOOST_NO_CXX11_ALIGNOF
#  define CROSPI_BOOST_NO_CXX11_TRAILING_RESULT_TYPES
#  define CROSPI_BOOST_NO_CXX11_INLINE_NAMESPACES
#  define CROSPI_BOOST_NO_CXX11_REF_QUALIFIERS
#  define CROSPI_BOOST_NO_CXX11_FINAL
#  define CROSPI_BOOST_NO_CXX11_OVERRIDE
#  define CROSPI_BOOST_NO_CXX11_THREAD_LOCAL
#  define CROSPI_BOOST_NO_CXX11_UNRESTRICTED_UNION

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
#endif
