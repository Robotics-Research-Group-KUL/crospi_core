
// NO INCLUDE GUARDS, THE HEADER IS INTENDED FOR MULTIPLE INCLUSION

// Copyright Aleksey Gurtovoy 2000-2006
//
// Distributed under the Boost Software License, Version 1.0. 
// (See accompanying file LICENSE_1_0.txt or copy at 
// http://www.boost.org/LICENSE_1_0.txt)
//
// See http://www.boost.org/libs/mpl for documentation.

// $Id$
// $Date$
// $Revision$

#include <crospi_boost/mpl/aux_/config/compiler.hpp>
#include <crospi_boost/mpl/aux_/config/preprocessor.hpp>
#include <crospi_boost/mpl/aux_/config/workaround.hpp>
#include <crospi_boost/preprocessor/cat.hpp>
#include <crospi_boost/preprocessor/stringize.hpp>

#if !defined(CROSPI_BOOST_NEEDS_TOKEN_PASTING_OP_FOR_TOKENS_JUXTAPOSING)
#   define AUX778076_PREPROCESSED_HEADER \
    CROSPI_BOOST_MPL_CFG_COMPILER_DIR/CROSPI_BOOST_MPL_PREPROCESSED_HEADER \
/**/
#else
#   define AUX778076_PREPROCESSED_HEADER \
    CROSPI_BOOST_PP_CAT(CROSPI_BOOST_MPL_CFG_COMPILER_DIR,/)##CROSPI_BOOST_MPL_PREPROCESSED_HEADER \
/**/
#endif

#if CROSPI_BOOST_WORKAROUND(__IBMCPP__, CROSPI_BOOST_TESTED_AT(700))
#   define AUX778076_INCLUDE_STRING CROSPI_BOOST_PP_STRINGIZE(crospi_boost/mpl/aux_/preprocessed/AUX778076_PREPROCESSED_HEADER)
#   include AUX778076_INCLUDE_STRING
#   undef AUX778076_INCLUDE_STRING
#else
#   include CROSPI_BOOST_PP_STRINGIZE(crospi_boost/mpl/aux_/preprocessed/AUX778076_PREPROCESSED_HEADER)
#endif

#   undef AUX778076_PREPROCESSED_HEADER

#undef CROSPI_BOOST_MPL_PREPROCESSED_HEADER
