
#ifndef CROSPI_BOOST_MPL_AUX_CONFIG_TTP_HPP_INCLUDED
#define CROSPI_BOOST_MPL_AUX_CONFIG_TTP_HPP_INCLUDED

// Copyright Aleksey Gurtovoy 2000-2004
//
// Distributed under the Boost Software License, Version 1.0. 
// (See accompanying file LICENSE_1_0.txt or copy at 
// http://www.boost.org/LICENSE_1_0.txt)
//
// See http://www.boost.org/libs/mpl for documentation.

// $Id$
// $Date$
// $Revision$

#include <crospi_boost/mpl/aux_/config/msvc.hpp>
#include <crospi_boost/mpl/aux_/config/gcc.hpp>
#include <crospi_boost/mpl/aux_/config/workaround.hpp>

#if !defined(CROSPI_BOOST_MPL_CFG_NO_TEMPLATE_TEMPLATE_PARAMETERS) \
    && ( defined(CROSPI_BOOST_NO_TEMPLATE_TEMPLATES) \
      || CROSPI_BOOST_WORKAROUND( CROSPI_BOOST_BORLANDC, CROSPI_BOOST_TESTED_AT( 0x590) ) \
       )

#   define CROSPI_BOOST_MPL_CFG_NO_TEMPLATE_TEMPLATE_PARAMETERS

#endif


#if    !defined(CROSPI_BOOST_MPL_CFG_EXTENDED_TEMPLATE_PARAMETERS_MATCHING) \
    && !defined(CROSPI_BOOST_MPL_PREPROCESSING_MODE) \
    && (   CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_MPL_CFG_GCC, CROSPI_BOOST_TESTED_AT(0x0302)) \
        || CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_BORLANDC, CROSPI_BOOST_TESTED_AT(0x610)) \
        )

#   define CROSPI_BOOST_MPL_CFG_EXTENDED_TEMPLATE_PARAMETERS_MATCHING

#endif

#endif // CROSPI_BOOST_MPL_AUX_CONFIG_TTP_HPP_INCLUDED
