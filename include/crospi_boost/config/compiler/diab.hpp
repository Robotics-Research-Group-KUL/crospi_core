//  (C) Copyright Brian Kuhl 2016.
//  Use, modification and distribution are subject to the
//  Boost Software License, Version 1.0. (See accompanying file
//  LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)

// Check this is a recent EDG based compiler, otherwise we don't support it here:


#ifndef __EDG_VERSION__
#     error "Unknown Diab compiler version - please run the configure tests and report the results"
#endif

#include "crospi_boost/config/compiler/common_edg.hpp"

#define CROSPI_BOOST_NO_TWO_PHASE_NAME_LOOKUP
#define CROSPI_BOOST_BUGGY_INTEGRAL_CONSTANT_EXPRESSIONS

#define CROSPI_BOOST_MPL_CFG_NO_HAS_XXX_TEMPLATE
#define CROSPI_BOOST_LOG_NO_MEMBER_TEMPLATE_FRIENDS
#define CROSPI_BOOST_REGEX_NO_EXTERNAL_TEMPLATES

#define CROSPI_BOOST_NO_CXX11_HDR_INITIALIZER_LIST
#define CROSPI_BOOST_NO_CXX11_HDR_CODECVT
#define CROSPI_BOOST_NO_CXX11_NUMERIC_LIMITS 

#define CROSPI_BOOST_COMPILER "Wind River Diab " CROSPI_BOOST_STRINGIZE(__VERSION_NUMBER__)
