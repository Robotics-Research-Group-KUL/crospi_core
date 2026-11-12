
#ifndef CROSPI_BOOST_MPL_AUX_CONFIG_PP_COUNTER_HPP_INCLUDED
#define CROSPI_BOOST_MPL_AUX_CONFIG_PP_COUNTER_HPP_INCLUDED

// Copyright Aleksey Gurtovoy 2006
//
// Distributed under the Boost Software License, Version 1.0. 
// (See accompanying file LICENSE_1_0.txt or copy at 
// http://www.boost.org/LICENSE_1_0.txt)
//
// See http://www.boost.org/libs/mpl for documentation.

// $Id$
// $Date$
// $Revision$

#if !defined(CROSPI_BOOST_MPL_AUX_PP_COUNTER)
#   include <crospi_boost/mpl/aux_/config/msvc.hpp>
#   if CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_MSVC, >= 1300)
#       define CROSPI_BOOST_MPL_AUX_PP_COUNTER() __COUNTER__
#   else
#       define CROSPI_BOOST_MPL_AUX_PP_COUNTER() __LINE__
#   endif
#endif

#endif // CROSPI_BOOST_MPL_AUX_CONFIG_PP_COUNTER_HPP_INCLUDED
