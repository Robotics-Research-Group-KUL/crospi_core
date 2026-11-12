
#ifndef CROSPI_BOOST_MPL_AUX_CONFIG_ADL_HPP_INCLUDED
#define CROSPI_BOOST_MPL_AUX_CONFIG_ADL_HPP_INCLUDED

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
#include <crospi_boost/mpl/aux_/config/intel.hpp>
#include <crospi_boost/mpl/aux_/config/gcc.hpp>
#include <crospi_boost/mpl/aux_/config/workaround.hpp>

// agurt, 25/apr/04: technically, the ADL workaround is only needed for GCC,
// but putting everything expect public, user-specializable metafunctions into
// a separate global namespace has a nice side effect of reducing the length 
// of template instantiation symbols, so we apply the workaround on all 
// platforms that can handle it

#if !defined(CROSPI_BOOST_MPL_CFG_NO_ADL_BARRIER_NAMESPACE) \
    && (   CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_MSVC, CROSPI_BOOST_TESTED_AT(1400)) \
        || CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_BORLANDC, CROSPI_BOOST_TESTED_AT(0x610)) \
        || CROSPI_BOOST_WORKAROUND(__DMC__, CROSPI_BOOST_TESTED_AT(0x840)) \
        || CROSPI_BOOST_WORKAROUND(__MWERKS__, CROSPI_BOOST_TESTED_AT(0x3202)) \
        || CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_INTEL_CXX_VERSION, CROSPI_BOOST_TESTED_AT(810)) \
        )

#   define CROSPI_BOOST_MPL_CFG_NO_ADL_BARRIER_NAMESPACE

#endif

#endif // CROSPI_BOOST_MPL_AUX_CONFIG_ADL_HPP_INCLUDED
