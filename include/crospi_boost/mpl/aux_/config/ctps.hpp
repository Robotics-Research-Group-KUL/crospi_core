
#ifndef CROSPI_BOOST_MPL_AUX_CONFIG_CTPS_HPP_INCLUDED
#define CROSPI_BOOST_MPL_AUX_CONFIG_CTPS_HPP_INCLUDED

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
#include <crospi_boost/config.hpp>

#if    !defined(CROSPI_BOOST_MPL_CFG_NO_NONTYPE_TEMPLATE_PARTIAL_SPEC) \
    && !defined(CROSPI_BOOST_MPL_PREPROCESSING_MODE) \
    && CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_BORLANDC, < 0x582)

#   define CROSPI_BOOST_MPL_CFG_NO_NONTYPE_TEMPLATE_PARTIAL_SPEC

#endif

// CROSPI_BOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION is defined in <crospi_boost/config.hpp>

#endif // CROSPI_BOOST_MPL_AUX_CONFIG_CTPS_HPP_INCLUDED
