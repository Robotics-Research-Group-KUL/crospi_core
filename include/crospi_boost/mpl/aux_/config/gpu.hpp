
#ifndef CROSPI_BOOST_MPL_AUX_CONFIG_GPU_HPP_INCLUDED
#define CROSPI_BOOST_MPL_AUX_CONFIG_GPU_HPP_INCLUDED

// Copyright Eric Niebler 2014
//
// Distributed under the Boost Software License, Version 1.0.
// (See accompanying file LICENSE_1_0.txt or copy at
// http://www.boost.org/LICENSE_1_0.txt)
//
// See http://www.boost.org/libs/mpl for documentation.

// $Id$
// $Date$
// $Revision$

#include <crospi_boost/config.hpp>

#if !defined(CROSPI_BOOST_MPL_CFG_GPU_ENABLED) \

#   define CROSPI_BOOST_MPL_CFG_GPU_ENABLED CROSPI_BOOST_GPU_ENABLED

#endif

#if defined __CUDACC__

#    define CROSPI_BOOST_MPL_CFG_GPU 1

#else

#    define CROSPI_BOOST_MPL_CFG_GPU 0

#endif

#endif // CROSPI_BOOST_MPL_AUX_CONFIG_GPU_HPP_INCLUDED
