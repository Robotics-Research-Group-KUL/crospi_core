
#ifndef CROSPI_BOOST_MPL_AUX_CONFIG_BIND_HPP_INCLUDED
#define CROSPI_BOOST_MPL_AUX_CONFIG_BIND_HPP_INCLUDED

// Copyright David Abrahams 2002
// Copyright Aleksey Gurtovoy 2002-2004
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
#include <crospi_boost/mpl/aux_/config/workaround.hpp>

#if    !defined(CROSPI_BOOST_MPL_CFG_NO_BIND_TEMPLATE) \
    && !defined(CROSPI_BOOST_MPL_PREPROCESSING_MODE) \
    && (   CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_MSVC, <= 1300) \
        || CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_BORLANDC, CROSPI_BOOST_TESTED_AT(0x610)) \
        )

#   define CROSPI_BOOST_MPL_CFG_NO_BIND_TEMPLATE

#endif

//#define CROSPI_BOOST_MPL_CFG_NO_UNNAMED_PLACEHOLDER_SUPPORT

#endif // CROSPI_BOOST_MPL_AUX_CONFIG_BIND_HPP_INCLUDED
