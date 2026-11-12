
#ifndef CROSPI_BOOST_MPL_AUX_ARG_TYPEDEF_HPP_INCLUDED
#define CROSPI_BOOST_MPL_AUX_ARG_TYPEDEF_HPP_INCLUDED

// Copyright Aleksey Gurtovoy 2001-2004
//
// Distributed under the Boost Software License, Version 1.0. 
// (See accompanying file LICENSE_1_0.txt or copy at 
// http://www.boost.org/LICENSE_1_0.txt)
//
// See http://www.boost.org/libs/mpl for documentation.

// $Id$
// $Date$
// $Revision$

#include <crospi_boost/mpl/aux_/config/lambda.hpp>
#include <crospi_boost/mpl/aux_/config/workaround.hpp>

#if defined(CROSPI_BOOST_MPL_CFG_NO_FULL_LAMBDA_SUPPORT) \
    || CROSPI_BOOST_WORKAROUND(__DMC__, CROSPI_BOOST_TESTED_AT(0x840))
    
#   define CROSPI_BOOST_MPL_AUX_ARG_TYPEDEF(T, name) typedef T name;

#else

#   define CROSPI_BOOST_MPL_AUX_ARG_TYPEDEF(T, name) /**/

#endif

#endif // CROSPI_BOOST_MPL_AUX_ARG_TYPEDEF_HPP_INCLUDED
