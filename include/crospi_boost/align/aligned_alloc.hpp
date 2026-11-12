/*
Copyright 2014-2015 Glen Joseph Fernandes
(glenjofe@gmail.com)

Distributed under the Boost Software License, Version 1.0.
(http://www.boost.org/LICENSE_1_0.txt)
*/
#ifndef CROSPI_BOOST_ALIGN_ALIGNED_ALLOC_HPP
#define CROSPI_BOOST_ALIGN_ALIGNED_ALLOC_HPP

#include <crospi_boost/config.hpp>

#if defined(CROSPI_BOOST_HAS_UNISTD_H)
#include <unistd.h>
#endif

#if defined(__APPLE__) || defined(__APPLE_CC__) || defined(macintosh)
#include <AvailabilityMacros.h>
#endif

#if defined(CROSPI_BOOST_ALIGN_USE_ALIGN)
#include <crospi_boost/align/detail/aligned_alloc.hpp>
#elif defined(CROSPI_BOOST_ALIGN_USE_NEW)
#include <crospi_boost/align/detail/aligned_alloc_new.hpp>
#elif defined(_MSC_VER) && !defined(UNDER_CE)
#include <crospi_boost/align/detail/aligned_alloc_msvc.hpp>
#elif defined(__MINGW32__) && (__MSVCRT_VERSION__ >= 0x0700)
#include <crospi_boost/align/detail/aligned_alloc_msvc.hpp>
#elif defined(__MINGW32__)
#include <crospi_boost/align/detail/aligned_alloc_mingw.hpp>
#elif MAC_OS_X_VERSION_MIN_REQUIRED >= 1090
#include <crospi_boost/align/detail/aligned_alloc_posix.hpp>
#elif MAC_OS_X_VERSION_MIN_REQUIRED >= 1060
#include <crospi_boost/align/detail/aligned_alloc_macos.hpp>
#elif defined(__ANDROID__)
#include <crospi_boost/align/detail/aligned_alloc_android.hpp>
#elif defined(__SunOS_5_11) || defined(__SunOS_5_12)
#include <crospi_boost/align/detail/aligned_alloc_posix.hpp>
#elif defined(sun) || defined(__sun)
#include <crospi_boost/align/detail/aligned_alloc_sunos.hpp>
#elif defined(_POSIX_VERSION)
#include <crospi_boost/align/detail/aligned_alloc_posix.hpp>
#else
#include <crospi_boost/align/detail/aligned_alloc.hpp>
#endif

#endif
