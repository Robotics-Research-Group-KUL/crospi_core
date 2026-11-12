
#ifndef CROSPI_BOOST_MPL_AUX_CONFIG_PREPROCESSOR_HPP_INCLUDED
#define CROSPI_BOOST_MPL_AUX_CONFIG_PREPROCESSOR_HPP_INCLUDED

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

#include <crospi_boost/mpl/aux_/config/workaround.hpp>

#if !defined(CROSPI_BOOST_MPL_CFG_BROKEN_PP_MACRO_EXPANSION) \
    && (   CROSPI_BOOST_WORKAROUND(__MWERKS__, <= 0x3003) \
        || CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_BORLANDC, < 0x582) \
        || CROSPI_BOOST_WORKAROUND(__IBMCPP__, CROSPI_BOOST_TESTED_AT(502)) \
        )

#   define CROSPI_BOOST_MPL_CFG_BROKEN_PP_MACRO_EXPANSION

#endif

#if !defined(CROSPI_BOOST_MPL_CFG_NO_OWN_PP_PRIMITIVES)
#   define CROSPI_BOOST_MPL_CFG_NO_OWN_PP_PRIMITIVES
#endif

#if !defined(CROSPI_BOOST_NEEDS_TOKEN_PASTING_OP_FOR_TOKENS_JUXTAPOSING) \
    && CROSPI_BOOST_WORKAROUND(__DMC__, CROSPI_BOOST_TESTED_AT(0x840))
#   define CROSPI_BOOST_NEEDS_TOKEN_PASTING_OP_FOR_TOKENS_JUXTAPOSING
#endif


#endif // CROSPI_BOOST_MPL_AUX_CONFIG_PREPROCESSOR_HPP_INCLUDED
