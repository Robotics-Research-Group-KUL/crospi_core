//  (C) Copyright John Maddock 2001 - 2003. 
//  Use, modification and distribution are subject to the 
//  Boost Software License, Version 1.0. (See accompanying file 
//  LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)

//  See http://www.boost.org for most recent version.

//  cygwin specific config options:

#define CROSPI_BOOST_PLATFORM "Cygwin"
#define CROSPI_BOOST_HAS_DIRENT_H
#define CROSPI_BOOST_HAS_LOG1P
#define CROSPI_BOOST_HAS_EXPM1

//
// Threading API:
// See if we have POSIX threads, if we do use them, otherwise
// revert to native Win threads.
#define CROSPI_BOOST_HAS_UNISTD_H
#include <unistd.h>
#if defined(_POSIX_THREADS) && (_POSIX_THREADS+0 >= 0) && !defined(CROSPI_BOOST_HAS_WINTHREADS)
#  define CROSPI_BOOST_HAS_PTHREADS
#  define CROSPI_BOOST_HAS_SCHED_YIELD
#  define CROSPI_BOOST_HAS_GETTIMEOFDAY
#  define CROSPI_BOOST_HAS_PTHREAD_MUTEXATTR_SETTYPE
//#  define CROSPI_BOOST_HAS_SIGACTION
#else
#  if !defined(CROSPI_BOOST_HAS_WINTHREADS)
#     define CROSPI_BOOST_HAS_WINTHREADS
#  endif
#  define CROSPI_BOOST_HAS_FTIME
#endif

//
// find out if we have a stdint.h, there should be a better way to do this:
//
#include <sys/types.h>
#ifdef _STDINT_H
#define CROSPI_BOOST_HAS_STDINT_H
#endif
#if __GNUC__ > 5 && !defined(CROSPI_BOOST_HAS_STDINT_H)
#   define CROSPI_BOOST_HAS_STDINT_H
#endif

#include <cygwin/version.h>
#if (CYGWIN_VERSION_API_MAJOR == 0 && CYGWIN_VERSION_API_MINOR < 231)
/// Cygwin has no fenv.h
#define CROSPI_BOOST_NO_FENV_H
#endif

// Cygwin has it's own <pthread.h> which breaks <shared_mutex> unless the correct compiler flags are used:
#ifndef CROSPI_BOOST_NO_CXX14_HDR_SHARED_MUTEX
#include <pthread.h>
#if !(__XSI_VISIBLE >= 500 || __POSIX_VISIBLE >= 200112)
#  define CROSPI_BOOST_NO_CXX14_HDR_SHARED_MUTEX
#endif
#endif

// boilerplate code:
#include <crospi_boost/config/detail/posix_features.hpp>

//
// Cygwin lies about XSI conformance, there is no nl_types.h:
//
#ifdef CROSPI_BOOST_HAS_NL_TYPES_H
#  undef CROSPI_BOOST_HAS_NL_TYPES_H
#endif




