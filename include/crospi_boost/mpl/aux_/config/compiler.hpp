
#ifndef CROSPI_BOOST_MPL_AUX_CONFIG_COMPILER_HPP_INCLUDED
#define CROSPI_BOOST_MPL_AUX_CONFIG_COMPILER_HPP_INCLUDED

// Copyright Aleksey Gurtovoy 2001-2008
//
// Distributed under the Boost Software License, Version 1.0. 
// (See accompanying file LICENSE_1_0.txt or copy at 
// http://www.boost.org/LICENSE_1_0.txt)
//
// See http://www.boost.org/libs/mpl for documentation.

// $Id$
// $Date$
// $Revision$

#if !defined(CROSPI_BOOST_MPL_CFG_COMPILER_DIR)

#   include <crospi_boost/mpl/aux_/config/dtp.hpp>
#   include <crospi_boost/mpl/aux_/config/ttp.hpp>
#   include <crospi_boost/mpl/aux_/config/ctps.hpp>
#   include <crospi_boost/mpl/aux_/config/msvc.hpp>
#   include <crospi_boost/mpl/aux_/config/gcc.hpp>
#   include <crospi_boost/mpl/aux_/config/workaround.hpp>

#   if CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_MSVC, < 1300)
#       define CROSPI_BOOST_MPL_CFG_COMPILER_DIR msvc60

#   elif CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_MSVC, == 1300)
#       define CROSPI_BOOST_MPL_CFG_COMPILER_DIR msvc70

#   elif CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_MPL_CFG_GCC, CROSPI_BOOST_TESTED_AT(0x0304))
#       define CROSPI_BOOST_MPL_CFG_COMPILER_DIR gcc

#   elif CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_BORLANDC, CROSPI_BOOST_TESTED_AT(0x610))
#       if !defined(CROSPI_BOOST_MPL_CFG_NO_DEFAULT_PARAMETERS_IN_NESTED_TEMPLATES)
#           define CROSPI_BOOST_MPL_CFG_COMPILER_DIR bcc551
#       elif CROSPI_BOOST_WORKAROUND(CROSPI_BOOST_BORLANDC, >= 0x590)
#           define CROSPI_BOOST_MPL_CFG_COMPILER_DIR bcc
#       else
#           define CROSPI_BOOST_MPL_CFG_COMPILER_DIR bcc_pre590
#       endif

#   elif CROSPI_BOOST_WORKAROUND(__DMC__, CROSPI_BOOST_TESTED_AT(0x840))
#       define CROSPI_BOOST_MPL_CFG_COMPILER_DIR dmc

#   elif defined(__MWERKS__)
#       if defined(CROSPI_BOOST_MPL_CFG_BROKEN_DEFAULT_PARAMETERS_IN_NESTED_TEMPLATES)
#           define CROSPI_BOOST_MPL_CFG_COMPILER_DIR mwcw
#       else
#           define CROSPI_BOOST_MPL_CFG_COMPILER_DIR plain
#       endif

#   elif defined(CROSPI_BOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION)
#       define CROSPI_BOOST_MPL_CFG_COMPILER_DIR no_ctps

#   elif defined(CROSPI_BOOST_MPL_CFG_NO_TEMPLATE_TEMPLATE_PARAMETERS)
#       define CROSPI_BOOST_MPL_CFG_COMPILER_DIR no_ttp

#   else
#       define CROSPI_BOOST_MPL_CFG_COMPILER_DIR plain
#   endif

#endif // CROSPI_BOOST_MPL_CFG_COMPILER_DIR

#endif // CROSPI_BOOST_MPL_AUX_CONFIG_COMPILER_HPP_INCLUDED
