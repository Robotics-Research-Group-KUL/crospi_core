
#ifndef CROSPI_BOOST_MPL_AUX_CONFIG_HAS_XXX_HPP_INCLUDED
#define CROSPI_BOOST_MPL_AUX_CONFIG_HAS_XXX_HPP_INCLUDED

// Copyright Aleksey Gurtovoy 2002-2004
// Copyright David Abrahams 2002-2003
//
// Distributed under the Boost Software License, Version 1.0. 
// (See accompanying file LICENSE_1_0.txt or copy at 
// http://www.boost.org/LICENSE_1_0.txt)
//
// See http://www.boost.org/libs/mpl for documentation.

// $Id$
// $Date$
// $Revision$

#include <crospi_boost/mpl/aux_/config/overload_resolution.hpp>
#include <crospi_boost/mpl/aux_/config/workaround.hpp>

// agurt, 11/jan/03: signals a stub-only 'has_xxx' implementation

#if !defined(CROSPI_BOOST_MPL_CFG_NO_HAS_XXX) \
    && (   defined(CROSPI_BOOST_MPL_CFG_BROKEN_OVERLOAD_RESOLUTION) \
        || CROSPI_BOOST_WORKAROUND(__GNUC__, <= 2) \
        || CROSPI_BOOST_WORKAROUND(__DMC__, CROSPI_BOOST_TESTED_AT(0x840)) \
        )

#   define CROSPI_BOOST_MPL_CFG_NO_HAS_XXX
#   define CROSPI_BOOST_MPL_CFG_NO_HAS_XXX_TEMPLATE

#endif

#endif // CROSPI_BOOST_MPL_AUX_CONFIG_HAS_XXX_HPP_INCLUDED
