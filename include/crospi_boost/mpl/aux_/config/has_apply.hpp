
#ifndef CROSPI_BOOST_MPL_AUX_CONFIG_HAS_APPLY_HPP_INCLUDED
#define CROSPI_BOOST_MPL_AUX_CONFIG_HAS_APPLY_HPP_INCLUDED

// Copyright Aleksey Gurtovoy 2004
//
// Distributed under the Boost Software License, Version 1.0. 
// (See accompanying file LICENSE_1_0.txt or copy at 
// http://www.boost.org/LICENSE_1_0.txt)
//
// See http://www.boost.org/libs/mpl for documentation.

// $Id$
// $Date$
// $Revision$

#include <crospi_boost/mpl/aux_/config/has_xxx.hpp>
#include <crospi_boost/mpl/aux_/config/msvc.hpp>
#include <crospi_boost/mpl/aux_/config/workaround.hpp>

#if !defined(CROSPI_BOOST_MPL_CFG_NO_HAS_APPLY) \
    && (   defined(CROSPI_BOOST_MPL_CFG_NO_HAS_XXX) \
        || CROSPI_BOOST_WORKAROUND(__EDG_VERSION__, < 300) \
        || CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_MSVC, <= 1300) \
        || CROSPI_BOOST_WORKAROUND(__MWERKS__, CROSPI_BOOST_TESTED_AT(0x3202)) \
        )

#   define CROSPI_BOOST_MPL_CFG_NO_HAS_APPLY

#endif

#endif // CROSPI_BOOST_MPL_AUX_CONFIG_HAS_APPLY_HPP_INCLUDED
