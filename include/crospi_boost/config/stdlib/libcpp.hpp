//  (C) Copyright Christopher Jefferson 2011.
//  Use, modification and distribution are subject to the
//  Boost Software License, Version 1.0. (See accompanying file
//  LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)

//  See http://www.boost.org for most recent version.

//  config for libc++
//  Might need more in here later.

#if !defined(_LIBCPP_VERSION)
#  include <ciso646>
#  if !defined(_LIBCPP_VERSION)
#      error "This is not libc++!"
#  endif
#endif

#define CROSPI_BOOST_STDLIB "libc++ version " CROSPI_BOOST_STRINGIZE(_LIBCPP_VERSION)

#define CROSPI_BOOST_HAS_THREADS

#ifdef _LIBCPP_HAS_NO_VARIADICS
#    define CROSPI_BOOST_NO_CXX11_HDR_TUPLE
#endif

// CROSPI_BOOST_NO_CXX11_ALLOCATOR should imply no support for the C++11
// allocator model. The C++11 allocator model requires a conforming
// std::allocator_traits which is only possible with C++11 template
// aliases since members rebind_alloc and rebind_traits require it.
#if defined(_LIBCPP_HAS_NO_TEMPLATE_ALIASES)
#    define CROSPI_BOOST_NO_CXX11_ALLOCATOR
#    define CROSPI_BOOST_NO_CXX11_POINTER_TRAITS
#endif

#if __cplusplus < 201103
//
// These two appear to be somewhat useable in C++03 mode, there may be others...
//
//#  define CROSPI_BOOST_NO_CXX11_HDR_ARRAY
//#  define CROSPI_BOOST_NO_CXX11_HDR_FORWARD_LIST

#  define CROSPI_BOOST_NO_CXX11_HDR_CODECVT
#  define CROSPI_BOOST_NO_CXX11_HDR_CONDITION_VARIABLE
#  define CROSPI_BOOST_NO_CXX11_HDR_EXCEPTION
#  define CROSPI_BOOST_NO_CXX11_HDR_INITIALIZER_LIST
#  define CROSPI_BOOST_NO_CXX11_HDR_MUTEX
#  define CROSPI_BOOST_NO_CXX11_HDR_RANDOM
#  define CROSPI_BOOST_NO_CXX11_HDR_RATIO
#  define CROSPI_BOOST_NO_CXX11_HDR_REGEX
#  define CROSPI_BOOST_NO_CXX11_HDR_SYSTEM_ERROR
#  define CROSPI_BOOST_NO_CXX11_HDR_THREAD
#  define CROSPI_BOOST_NO_CXX11_HDR_TUPLE
#  define CROSPI_BOOST_NO_CXX11_HDR_TYPEINDEX
#  define CROSPI_BOOST_NO_CXX11_HDR_UNORDERED_MAP
#  define CROSPI_BOOST_NO_CXX11_HDR_UNORDERED_SET
#  define CROSPI_BOOST_NO_CXX11_NUMERIC_LIMITS
#  define CROSPI_BOOST_NO_CXX11_ALLOCATOR
#  define CROSPI_BOOST_NO_CXX11_POINTER_TRAITS
#  define CROSPI_BOOST_NO_CXX11_SMART_PTR
#  define CROSPI_BOOST_NO_CXX11_HDR_FUNCTIONAL
#  define CROSPI_BOOST_NO_CXX11_STD_ALIGN
#  define CROSPI_BOOST_NO_CXX11_ADDRESSOF
#  define CROSPI_BOOST_NO_CXX11_HDR_ATOMIC
#  define CROSPI_BOOST_NO_CXX11_ATOMIC_SMART_PTR
#  define CROSPI_BOOST_NO_CXX11_HDR_CHRONO
#  define CROSPI_BOOST_NO_CXX11_HDR_TYPE_TRAITS
#  define CROSPI_BOOST_NO_CXX11_HDR_FUTURE
#elif _LIBCPP_VERSION < 3700
//
// These appear to be unusable/incomplete so far:
//
#  define CROSPI_BOOST_NO_CXX11_HDR_ATOMIC
#  define CROSPI_BOOST_NO_CXX11_ATOMIC_SMART_PTR
#  define CROSPI_BOOST_NO_CXX11_HDR_CHRONO
#  define CROSPI_BOOST_NO_CXX11_HDR_TYPE_TRAITS
#  define CROSPI_BOOST_NO_CXX11_HDR_FUTURE
#endif


#if _LIBCPP_VERSION < 3700
// libc++ uses a non-standard messages_base
#define CROSPI_BOOST_NO_STD_MESSAGES
#endif

// C++14 features
#if (_LIBCPP_VERSION < 3700) || (__cplusplus <= 201402L)
#  define CROSPI_BOOST_NO_CXX14_STD_EXCHANGE
#endif

// C++17 features
#if (_LIBCPP_VERSION < 4000) || (__cplusplus <= 201402L)
#  define CROSPI_BOOST_NO_CXX17_STD_APPLY
#  define CROSPI_BOOST_NO_CXX17_HDR_OPTIONAL
#  define CROSPI_BOOST_NO_CXX17_HDR_STRING_VIEW
#  define CROSPI_BOOST_NO_CXX17_HDR_VARIANT
#endif
#if (_LIBCPP_VERSION > 4000) && (__cplusplus > 201402L) && !defined(_LIBCPP_ENABLE_CXX17_REMOVED_AUTO_PTR)
#  define CROSPI_BOOST_NO_AUTO_PTR
#endif
#if (_LIBCPP_VERSION > 4000) && (__cplusplus > 201402L) && !defined(_LIBCPP_ENABLE_CXX17_REMOVED_RANDOM_SHUFFLE)
#  define CROSPI_BOOST_NO_CXX98_RANDOM_SHUFFLE
#endif
#if (_LIBCPP_VERSION > 4000) && (__cplusplus > 201402L) && !defined(_LIBCPP_ENABLE_CXX17_REMOVED_BINDERS)
#  define CROSPI_BOOST_NO_CXX98_BINDERS
#endif

#if defined(__cplusplus) && defined(__has_include)
#if __has_include(<version>)
#include <version>

#if !defined(__cpp_lib_execution) || (__cpp_lib_execution < 201603L)
#  define CROSPI_BOOST_NO_CXX17_HDR_EXECUTION
#endif
#if !defined(__cpp_lib_invoke) || (__cpp_lib_invoke < 201411L)
#define CROSPI_BOOST_NO_CXX17_STD_INVOKE
#endif

#if(_LIBCPP_VERSION < 9000)
// as_writable_bytes is missing.
#  define CROSPI_BOOST_NO_CXX20_HDR_SPAN
#endif

#else
#define CROSPI_BOOST_NO_CXX17_STD_INVOKE      // Invoke support is incomplete (no invoke_result)
#define CROSPI_BOOST_NO_CXX17_HDR_EXECUTION
#endif
#else
#define CROSPI_BOOST_NO_CXX17_STD_INVOKE      // Invoke support is incomplete (no invoke_result)
#define CROSPI_BOOST_NO_CXX17_HDR_EXECUTION
#endif

#if _LIBCPP_VERSION < 10000  // What's the correct version check here?
#define CROSPI_BOOST_NO_CXX17_ITERATOR_TRAITS
#endif

#if (_LIBCPP_VERSION <= 1101) && !defined(CROSPI_BOOST_NO_CXX11_THREAD_LOCAL)
// This is a bit of a sledgehammer, because really it's just libc++abi that has no
// support for thread_local, leading to linker errors such as
// "undefined reference to `__cxa_thread_atexit'".  It is fixed in the
// most recent releases of libc++abi though...
#  define CROSPI_BOOST_NO_CXX11_THREAD_LOCAL
#endif

#if defined(__linux__) && (_LIBCPP_VERSION < 6000) && !defined(CROSPI_BOOST_NO_CXX11_THREAD_LOCAL)
// After libc++-dev is installed on Trusty, clang++-libc++ almost works,
// except uses of `thread_local` fail with undefined reference to
// `__cxa_thread_atexit`.
//
// clang's libc++abi provides an implementation by deferring to the glibc
// implementation, which may or may not be available (it is not on Trusty).
// clang 4's libc++abi will provide an implementation if one is not in glibc
// though, so thread local support should work with clang 4 and above as long
// as libc++abi is linked in.
#  define CROSPI_BOOST_NO_CXX11_THREAD_LOCAL
#endif

#if defined(__has_include)
#if !__has_include(<shared_mutex>)
#  define CROSPI_BOOST_NO_CXX14_HDR_SHARED_MUTEX
#elif __cplusplus <= 201103
#  define CROSPI_BOOST_NO_CXX14_HDR_SHARED_MUTEX
#endif
#elif __cplusplus < 201402
#  define CROSPI_BOOST_NO_CXX14_HDR_SHARED_MUTEX
#endif

#if !defined(CROSPI_BOOST_NO_CXX14_HDR_SHARED_MUTEX) && (_LIBCPP_VERSION < 5000)
#  define CROSPI_BOOST_NO_CXX14_HDR_SHARED_MUTEX
#endif

#if _LIBCPP_VERSION >= 15000
//
// Unary function is now deprecated in C++11 and later:
//
#if __cplusplus >= 201103L
#define CROSPI_BOOST_NO_CXX98_FUNCTION_BASE
#endif
#endif

#if _LIBCPP_VERSION <= 170006
// no std::ranges::join_view
#  define CROSPI_BOOST_NO_CXX20_HDR_RANGES
#endif

//  --- end ---
